#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String # Added String for fleet/turn
import cv2
import numpy as np
import math
import json
from collections import deque
import requests
import time
import threading 

class GaugeMonitorNode(Node):
    def __init__(self):
        super().__init__('gauge_monitor_node')
        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.Lock() 

        # 1. Load Config
        try:
            with open('gauge_template_final.json', 'r') as f:
                self.config = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Config Load Error: {e}')
            self.config = {"min_angle": 45, "max_angle": 315, "min_val": 0, "max_val": 10}

        # 2. State Variables
        self.current_ns = "robot2" # Default namespace
        self.monitoring_active = False  
        self.is_calibrated = False      
        self.matrix = None              
        self.image_received = False     
        self.last_sent_status = None    
        self.current_gauge_label = "Gauge_01"
        
        # 3. Filter & Timing
        self.value_history = deque(maxlen=10)
        self.stability_threshold = 0.5  
        self.arrival_time = 0.0
        self.stabilization_delay = 2.0  
        self.calibration_start_time = 0.0
        self.retry_timeout = 10.0       

        # 4. Fleet & Camera Setup
        self.image_sub = None # Initialize as None
        from rclpy.qos import qos_profile_sensor_data
        self.qos = qos_profile_sensor_data
        
        # Initial Camera Subscription
        self.update_camera_subscription(self.current_ns)

        # Fleet Awareness: Listen to which robot is active
        self.turn_sub = self.create_subscription(
            String, '/fleet/turn', self.turn_cb, 10, callback_group=self.callback_group)

        # Global Signals (Shared across both robots)
        self.goal_sub = self.create_subscription(
            Bool, '/waypoint_arrived', self.arrival_callback, 10, 
            callback_group=self.callback_group)
        self.ready_pub = self.create_publisher(
            Bool, '/gauge_safe_status', 10, callback_group=self.callback_group)

        self.status_timer = self.create_timer(2.0, self.report_status)
        self.get_logger().info(f'=== Fleet Gauge Node: Starting with {self.current_ns} ===')

    def turn_cb(self, msg):
        """Dynamic namespace switching logic"""
        new_ns = msg.data.strip()
        if new_ns != self.current_ns:
            self.get_logger().warn(f'MANAGER SIGNAL: Switching monitoring from {self.current_ns} to {new_ns}')
            with self.lock:
                self.current_ns = new_ns
                self.monitoring_active = False # Stop current check
                self.is_calibrated = False     # Must re-calibrate for new robot camera
                self.update_camera_subscription(new_ns)

    def update_camera_subscription(self, ns):
        """Rebinds camera subscriber to a new namespace"""
        if self.image_sub:
            self.destroy_subscription(self.image_sub)
        
        topic = f'/{ns}/oakd/rgb/image_raw/compressed'
        self.image_sub = self.create_subscription(
            CompressedImage, topic, self.image_callback, 
            self.qos, callback_group=self.callback_group)
        self.image_received = False
        self.get_logger().info(f'Subscribed to: {topic}')

    def report_status(self):
        if not self.image_received:
            self.get_logger().warn('Waiting for camera topic...')
        else:
            status = "MONITORING" if self.monitoring_active else "IDLE"
            self.get_logger().info(f'Node Status: {status}')

    def arrival_callback(self, msg):
        if msg.data:
            self.get_logger().info('Arrival signal: Starting Detection.')
            with self.lock:
                self.value_history.clear()
                self.is_calibrated = False
                self.arrival_time = time.time()
                self.calibration_start_time = time.time()
                self.monitoring_active = True
                self.last_sent_status = None # Reset status for new waypoint
        else:
            self.monitoring_active = False
            cv2.destroyAllWindows()

    def image_callback(self, msg):
        self.image_received = True
        if not self.monitoring_active:
            return

        if time.time() - self.arrival_time < self.stabilization_delay:
            return

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if not self.is_calibrated:
                self.auto_calibrate(frame)
                if self.monitoring_active and not self.is_calibrated:
                    cv2.imshow("Analysis Window", frame)
                    cv2.waitKey(1)
                    if time.time() - self.calibration_start_time > self.retry_timeout:
                        self.process_logic(is_safe=True, value=0.0) # Timeout bypass
            else:
                value, debug_img = self.read_gauge(frame)
                if value is not None:
                    self.value_history.append(value)
                    if len(self.value_history) == 10:
                        diff = max(self.value_history) - min(self.value_history)
                        if diff < self.stability_threshold:
                            avg_value = sum(self.value_history) / 10
                            is_safe = 1.0 < avg_value < 8.0
                            self.process_logic(is_safe, avg_value)
                
                if self.monitoring_active:
                    cv2.imshow("Analysis Window", debug_img)
                    cv2.waitKey(1)
                    
        except Exception as e:
            self.get_logger().error(f'Image Pipeline Error: {e}')

    def auto_calibrate(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(cv2.medianBlur(gray, 5), cv2.HOUGH_GRADIENT, 1, 100, 
                                   param1=100, param2=50, minRadius=50, maxRadius=300)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            cx, cy, r = circles[0, 0]
            margin = int(r * 1.15) 
            src_pts = np.float32([[cx-margin, cy-margin], [cx+margin, cy-margin], 
                                  [cx+margin, cy+margin], [cx-margin, cy+margin]])
            dst_pts = np.float32([[0, 0], [500, 0], [500, 500], [0, 500]])
            self.matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
            self.is_calibrated = True

    def read_gauge(self, frame):
        warped = cv2.warpPerspective(frame, self.matrix, (500, 500), flags=cv2.INTER_CUBIC)
        debug_img = warped.copy()
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=40, minLineLength=60, maxLineGap=10)
        center = (250, 250)
        if lines is not None:
            valid_needles = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if math.dist(center, (x1, y1)) < 40 or math.dist(center, (x2, y2)) < 40:
                    valid_needles.append((x1, y1) if math.dist(center, (x1, y1)) > math.dist(center, (x2, y2)) else (x2, y2))
            if valid_needles:
                best_pt = max(valid_needles, key=lambda p: math.dist(center, p))
                cv2.line(debug_img, center, best_pt, (0, 255, 0), 3)
                dx, dy = best_pt[0] - center[0], best_pt[1] - center[1]
                angle = np.degrees(np.arctan2(dx, -dy)) % 360
                total_sweep = (self.config["max_angle"] - self.config["min_angle"]) % 360
                rel_angle = (angle - self.config["min_angle"]) % 360
                ratio = np.clip(rel_angle / total_sweep, 0.0, 1.0)
                value = self.config["min_val"] + (ratio * (self.config["max_val"] - self.config["min_val"]))
                return value, debug_img
        return None, debug_img
    
    def process_logic(self, is_safe, value):
        """Modified logic: Same as yours, but ensures results are clearly logged for current robot."""
        safe_bool = bool(is_safe)
        
        msg = Bool()
        msg.data = safe_bool
        self.ready_pub.publish(msg)

        if safe_bool != self.last_sent_status:
            try:
                log_data = {
                    "robot": self.current_ns, # Added robot ID to logs
                    "label": self.current_gauge_label, 
                    "value": round(float(value), 2), 
                    "status": "정상" if safe_bool else "비정상"
                }
                requests.post("http://192.168.108.21:5000/log", json=log_data, timeout=0.5)
            except: pass
            
            if not safe_bool:
                self.get_logger().error(f'[{self.current_ns}] GAUGE UNSAFE: {value:.2f}. Waiting recovery.')
            else:
                self.get_logger().info(f'[{self.current_ns}] GAUGE SAFE: {value:.2f}. Resume patrol.')
            
            self.last_sent_status = safe_bool

        if safe_bool:
            with self.lock:
                self.monitoring_active = False
                cv2.destroyAllWindows()