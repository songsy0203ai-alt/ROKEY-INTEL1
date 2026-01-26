#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String
import cv2
import numpy as np
import math
from collections import deque
import requests
import time
import threading 
from cv_bridge import CvBridge

class FleetGaugeMonitor(Node):
    def __init__(self):
        super().__init__('fleet_gauge_monitor')
        self.callback_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()
        self.lock = threading.Lock() 

        # 1. State Variables
        self.current_ns = "robot2"
        self.monitoring_active = False  
        self.is_calibrated = False      
        self.matrix = None              
        self.last_sent_status = None    
        
        # 2. Timing & Filters
        self.arrival_time = 0.0
        self.stabilization_delay = 2.0  
        self.value_history = deque(maxlen=10)

        # 3. Config (Initialized with defaults, including center)
        self.config = {
            "min_angle": 223,
            "max_angle": 315,
            "min_val": 0.0,
            "max_val": 10.0,
            "center_x": 250,  # Default center for 500x500 warp
            "center_y": 250
        }

        # 4. Fleet & Camera Setup
        self.image_sub = None
        from rclpy.qos import qos_profile_sensor_data
        self.qos = qos_profile_sensor_data
        
        self.update_camera_subscription(self.current_ns)

        self.turn_sub = self.create_subscription(
            String, '/fleet/turn', self.turn_cb, 10, callback_group=self.callback_group)

        self.goal_sub = self.create_subscription(
            Bool, '/waypoint_arrived', self.arrival_callback, 10, callback_group=self.callback_group)
        self.ready_pub = self.create_publisher(
            Bool, '/gauge_safe_status', 10, callback_group=self.callback_group)

        self.get_logger().info(f'=== PC3 Gauge Monitor: Watching {self.current_ns} (Compressed) ===')

    def turn_cb(self, msg):
        new_ns = msg.data.strip()
        if new_ns in ["robot2", "robot3"] and new_ns != self.current_ns:
            self.get_logger().warn(f'SWITCHING CAMERA TO: {new_ns}')
            with self.lock:
                self.current_ns = new_ns
                self.monitoring_active = False
                self.is_calibrated = False 
                self.matrix = None # Reset warp matrix for new camera
                self.update_camera_subscription(new_ns)

    def update_camera_subscription(self, ns):
        if self.image_sub:
            self.destroy_subscription(self.image_sub)
        topic = f'/{ns}/oakd/rgb/preview/image_raw/compressed'
        self.image_sub = self.create_subscription(
            CompressedImage, topic, self.image_callback, self.qos, callback_group=self.callback_group)

    def arrival_callback(self, msg):
        if msg.data:
            self.get_logger().info(f'[{self.current_ns}] Arrived at Gauge. Analyzing...')
            with self.lock:
                self.value_history.clear()
                self.is_calibrated = False
                self.arrival_time = time.time()
                self.monitoring_active = True
                self.last_sent_status = None 
        else:
            self.monitoring_active = False

    def image_callback(self, msg):
        if not self.monitoring_active:
            return

        # Wait for robot to stop moving
        if time.time() - self.arrival_time < self.stabilization_delay:
            return

        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            display_img = frame.copy()

            if not self.is_calibrated:
                # Still calibrating: show raw frame and attempt to find gauge
                success, debug_frame = self.auto_calibrate(frame)
                display_img = debug_frame
                if success:
                    self.get_logger().info("Calibration Success!")
            else:
                # Calibrated: read and show the warped gauge
                value, debug_img = self.read_gauge(frame)
                if debug_img is not None:
                    display_img = debug_img
                
                if value is not None:
                    self.value_history.append(value)
                    if len(self.value_history) == 10:
                        avg_value = sum(self.value_history) / 10
                        is_safe = 1.0 < avg_value < 8.0 
                        self.publish_result(is_safe, avg_value)
            
            # Show the window every frame so it's not blank
            cv2.imshow("PC3 Gauge Monitor", display_img)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'CV Error: {e}')

    def auto_calibrate(self, frame):
        """Find the gauge circle and generate the warp matrix."""
        debug_frame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(cv2.medianBlur(gray, 5), cv2.HOUGH_GRADIENT, 1, 100, 
                                   param1=100, param2=50, minRadius=50, maxRadius=300)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            cx, cy, r = circles[0, 0]
            
            # Draw for user feedback
            cv2.circle(debug_frame, (cx, cy), r, (0, 255, 0), 2)
            cv2.circle(debug_frame, (cx, cy), 2, (0, 0, 255), 3)

            offset = 10
            src_pts = np.float32([[cx-r-offset, cy-r-offset], [cx+r+offset, cy-r-offset], 
                                  [cx+r+offset, cy+r+offset], [cx-r-offset, cy+r+offset]])
            dst_pts = np.float32([[0, 0], [500, 0], [500, 500], [0, 500]])
            
            self.matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
            self.is_calibrated = True
            return True, debug_frame
        
        cv2.putText(debug_frame, "Searching for Gauge...", (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return False, debug_frame
        
    def read_gauge(self, frame):
        # FIX: Added warp dependency check
        if self.matrix is None:
            return None, frame

        warped = cv2.warpPerspective(frame, self.matrix, (500, 500), flags=cv2.INTER_CUBIC)
        debug_img = warped.copy()
        
        # FIX: Use center from config as requested
        center = (int(self.config.get("center_x", 250)), 
                  int(self.config.get("center_y", 250)))
        cv2.circle(debug_img, center, 5, (0, 0, 255), -1)

        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=30, minLineLength=45, maxLineGap=7)
        
        if lines is not None:
            valid_needles = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                pt1, pt2 = (x1, y1), (x2, y2)
                dist1 = math.dist(center, pt1)
                dist2 = math.dist(center, pt2)
                
                start_pt, start_dist = (pt1, dist1) if dist1 < dist2 else (pt2, dist2)
                end_pt = pt2 if dist1 < dist2 else pt1

                if start_dist <= 30:
                    valid_needles.append((start_pt, end_pt))

            if valid_needles:
                best_needle = max(valid_needles, key=lambda n: math.dist(n[0], n[1]))
                start_node, end_node = best_needle
                cv2.line(debug_img, center, end_node, (0, 255, 0), 3)
                
                dx, dy = end_node[0] - center[0], end_node[1] - center[1]
                angle = np.degrees(np.arctan2(dx, -dy)) % 360
                
                total_sweep = (self.config["max_angle"] - self.config["min_angle"]) % 360
                rel_angle = (angle - self.config["min_angle"]) % 360
                ratio = np.clip(rel_angle / total_sweep, 0.0, 1.0)
                value = self.config["min_val"] + (ratio * (self.config["max_val"] - self.config["min_val"]))
                
                cv2.putText(debug_img, f"Val: {value:.2f}", (20, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                return value, debug_img
        
        return None, debug_img

    def publish_result(self, is_safe, value):
        msg = Bool()
        msg.data = bool(is_safe)
        self.ready_pub.publish(msg)

        if msg.data != self.last_sent_status:
            try:
                requests.post("http://192.168.108.21:5000/log", 
                              json={"robot": self.current_ns, "value": round(value, 2), 
                                    "status": "OK" if msg.data else "WARN"}, 
                              timeout=0.5)
            except: pass
            self.last_sent_status = msg.data

        if msg.data:
            self.monitoring_active = False 

def main():
    rclpy.init()
    node = FleetGaugeMonitor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
