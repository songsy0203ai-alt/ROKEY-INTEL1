#!/usr/bin/env python3
"""
- [코드 기능]: 로봇의 카메라 피드에서 아날로그 게이지를 인식하고, 바늘의 각도를 계산하여 현재 수치를 측정 및 안전 여부를 판별하는 ROS 2 노드
- [입력(Input)]: 로봇 네임스페이스(String), 목적지 도착 여부(Bool), 압축 이미지 데이터(CompressedImage)
- [출력(Output)]: 게이지 안전 상태(Bool), 외부 로그 서버 데이터(JSON via HTTP POST)
"""

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
        """
        [인풋]: 없음 (클래스 생성자)
        [아웃풋]: FleetGaugeMonitor 인스턴스 초기화
        """
        super().__init__('fleet_gauge_monitor')
        # 멀티스레드 실행을 위한 콜백 그룹 및 동기화 락 설정
        self.callback_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()
        self.lock = threading.Lock() 

        # 1. 상태 변수 초기화
        self.current_ns = "robot2" # 현재 감시 대상 로봇
        self.monitoring_active = False  # 분석 활성화 여부
        self.is_calibrated = False      # 게이지 위치 보정 완료 여부
        self.matrix = None              # 원근 변환(Warp) 행렬
        self.last_sent_status = None    # 중복 전송 방지를 위한 이전 상태 저장
        
        # 2. 타이밍 및 필터 설정
        self.arrival_time = 0.0
        self.stabilization_delay = 2.0  # 정지 후 안정화를 위한 대기 시간 (초)
        self.value_history = deque(maxlen=10) # 수치 평균 필터링을 위한 큐

        # 3. 게이지 설정값 (중심점 및 각도 범위)
        self.config = {
            "min_angle": 223,
            "max_angle": 315,
            "min_val": 0.0,
            "max_val": 10.0,
            "center_x": 250,  # 500x500 변환 이미지 기준 중심점
            "center_y": 250
        }

        # 4. 카메라 및 통신 설정
        self.image_sub = None
        from rclpy.qos import qos_profile_sensor_data
        self.qos = qos_profile_sensor_data
        
        # 초기 카메라 구독 설정
        self.update_camera_subscription(self.current_ns)

        # 로봇 전환 및 도착 신호 구독자 설정
        self.turn_sub = self.create_subscription(
            String, '/fleet/turn', self.turn_cb, 10, callback_group=self.callback_group)

        self.goal_sub = self.create_subscription(
            Bool, '/waypoint_arrived', self.arrival_callback, 10, callback_group=self.callback_group)
            
        # 결과 발행자 설정
        self.ready_pub = self.create_publisher(
            Bool, '/gauge_safe_status', 10, callback_group=self.callback_group)

        self.get_logger().info(f'=== PC3 Gauge Monitor: Watching {self.current_ns} (Compressed) ===')

    def turn_cb(self, msg):
        """
        [인풋]: msg (String - 새로운 로봇 네임스페이스)
        [아웃풋]: 없음 (내부 상태 업데이트 및 구독 갱신)
        """
        new_ns = msg.data.strip()
        # 로봇이 변경될 경우 기존 상태 초기화 및 구독 교체
        if new_ns in ["robot2", "robot3"] and new_ns != self.current_ns:
            self.get_logger().warn(f'SWITCHING CAMERA TO: {new_ns}')
            with self.lock:
                self.current_ns = new_ns
                self.monitoring_active = False
                self.is_calibrated = False 
                self.matrix = None # 새로운 카메라에 대한 보정 행렬 초기화
                self.update_camera_subscription(new_ns)

    def update_camera_subscription(self, ns):
        """
        [인풋]: ns (String - 로봇 네임스페이스)
        [아웃풋]: 없음 (기존 구독 파괴 및 신규 구독 생성)
        """
        if self.image_sub:
            self.destroy_subscription(self.image_sub)
        # 네임스페이스를 기반으로 동적 토픽 경로 생성
        topic = f'/{ns}/oakd/rgb/preview/image_raw/compressed'
        self.image_sub = self.create_subscription(
            CompressedImage, topic, self.image_callback, self.qos, callback_group=self.callback_group)

    def arrival_callback(self, msg):
        """
        [인풋]: msg (Bool - 게이지 앞 도착 여부)
        [아웃풋]: 없음 (모니터링 활성화 상태 제어)
        """
        if msg.data:
            self.get_logger().info(f'[{self.current_ns}] Arrived at Gauge. Analyzing...')
            with self.lock:
                self.value_history.clear()
                self.is_calibrated = False # 재보정 유도
                self.arrival_time = time.time()
                self.monitoring_active = True
                self.last_sent_status = None 
        else:
            self.monitoring_active = False

    def image_callback(self, msg):
        """
        [인풋]: msg (CompressedImage - 압축 이미지 메시지)
        [아웃풋]: 없음 (이미지 처리 및 결과 발행)
        """
        if not self.monitoring_active:
            return

        # 로봇 정지 후 흔들림 방지를 위해 설정된 시간만큼 대기
        if time.time() - self.arrival_time < self.stabilization_delay:
            return

        try:
            # ROS CompressedImage 메시지를 OpenCV 형식으로 변환
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            display_img = frame.copy()

            if not self.is_calibrated:
                # 보정 전: 게이지 외곽 원 탐색 시도
                success, debug_frame = self.auto_calibrate(frame)
                display_img = debug_frame
                if success:
                    self.get_logger().info("Calibration Success!")
            else:
                # 보정 후: 바늘 인식 및 수치 읽기
                value, debug_img = self.read_gauge(frame)
                if debug_img is not None:
                    display_img = debug_img
                
                if value is not None:
                    self.value_history.append(value)
                    # 10개 데이터 수집 후 평균값으로 안전 여부 판단
                    if len(self.value_history) == 10:
                        avg_value = sum(self.value_history) / 10
                        # 1.0 ~ 8.0 범위를 안전(Safe)으로 간주
                        is_safe = 1.0 < avg_value < 8.0 
                        self.publish_result(is_safe, avg_value)
            
            # 실시간 결과 화면 출력
            cv2.imshow("PC3 Gauge Monitor", display_img)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'CV Error: {e}')

    def auto_calibrate(self, frame):
        """
        [인풋]: frame (numpy.ndarray - 원본 이미지)
        [아웃풋]: success (Bool), debug_frame (numpy.ndarray - 결과 시각화 이미지)
        """
        debug_frame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 허프 변환을 사용하여 원형 게이지 탐색
        circles = cv2.HoughCircles(cv2.medianBlur(gray, 5), cv2.HOUGH_GRADIENT, 1, 100, 
                                   param1=100, param2=50, minRadius=50, maxRadius=300)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            cx, cy, r = circles[0, 0]
            
            # 감지된 게이지 영역 표시
            cv2.circle(debug_frame, (cx, cy), r, (0, 255, 0), 2)
            cv2.circle(debug_frame, (cx, cy), 2, (0, 0, 255), 3)

            # 게이지 영역을 정사각형(500x500)으로 펴기 위한 원근 변환 행렬 계산
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
        """
        [인풋]: frame (numpy.ndarray - 원본 이미지)
        [아웃풋]: value (Float - 계산된 수치), debug_img (numpy.ndarray - 시각화 이미지)
        """
        if self.matrix is None:
            return None, frame

        # 사전에 계산된 행렬을 사용하여 게이지 영역만 정면으로 Warp
        warped = cv2.warpPerspective(frame, self.matrix, (500, 500), flags=cv2.INTER_CUBIC)
        debug_img = warped.copy()
        
        # 게이지 중심점 설정 (Config 기반)
        center = (int(self.config.get("center_x", 250)), 
                  int(self.config.get("center_y", 250)))
        cv2.circle(debug_img, center, 5, (0, 0, 255), -1)

        # 바늘 추출을 위한 전처리 및 캐니 에지 검출
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # 확률적 허프 변환으로 바늘(직선) 후보군 탐색
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=30, minLineLength=45, maxLineGap=7)
        
        if lines is not None:
            valid_needles = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                pt1, pt2 = (x1, y1), (x2, y2)
                dist1 = math.dist(center, pt1)
                dist2 = math.dist(center, pt2)
                
                # 중심점 근처에서 시작하는 선분만 바늘로 필터링
                start_pt, start_dist = (pt1, dist1) if dist1 < dist2 else (pt2, dist2)
                end_pt = pt2 if dist1 < dist2 else pt1

                if start_dist <= 30: # 중심점으로부터 30픽셀 이내 시작
                    valid_needles.append((start_pt, end_pt))

            if valid_needles:
                # 가장 긴 선분을 최종 바늘로 선정
                best_needle = max(valid_needles, key=lambda n: math.dist(n[0], n[1]))
                start_node, end_node = best_needle
                cv2.line(debug_img, center, end_node, (0, 255, 0), 3)
                
                # 바늘의 각도($\theta$) 계산: $angle = \text{atan2}(dx, -dy) \times \frac{180}{\pi}$
                dx, dy = end_node[0] - center[0], end_node[1] - center[1]
                angle = np.degrees(np.arctan2(dx, -dy)) % 360
                
                # 설정된 각도 범위 내에서 상대적 비율 계산
                total_sweep = (self.config["max_angle"] - self.config["min_angle"]) % 360
                rel_angle = (angle - self.config["min_angle"]) % 360
                ratio = np.clip(rel_angle / total_sweep, 0.0, 1.0)
                
                # 수치 변환 공식: $V = V_{min} + (\text{ratio} \times (V_{max} - V_{min}))$
                value = self.config["min_val"] + (ratio * (self.config["max_val"] - self.config["min_val"]))
                
                cv2.putText(debug_img, f"Val: {value:.2f}", (20, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                return value, debug_img
        
        return None, debug_img

    def publish_result(self, is_safe, value):
        """
        [인풋]: is_safe (Bool - 안전 여부), value (Float - 측정 수치)
        [아웃풋]: 없음 (ROS 토픽 발행 및 외부 서버 전송)
        """
        msg = Bool()
        msg.data = bool(is_safe)
        self.ready_pub.publish(msg)

        # 상태가 변할 때만 외부 서버로 데이터 전송 (HTTP POST)
        if msg.data != self.last_sent_status:
            try:
                requests.post("http://192.168.108.21:5000/log", 
                              json={"robot": self.current_ns, "value": round(value, 2), 
                                    "status": "OK" if msg.data else "WARN"}, 
                              timeout=0.5)
            except: pass
            self.last_sent_status = msg.data

        # 안전이 확인되면 해당 로봇 모니터링 종료
        if msg.data:
            self.monitoring_active = False 

def main():
    """
    [인풋]: 없음
    [아웃풋]: 노드 실행
    """
    rclpy.init()
    node = FleetGaugeMonitor()
    # 멀티스레드 실행기를 통해 콜백 병렬 처리
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