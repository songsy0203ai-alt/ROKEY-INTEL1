#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
import cv2
import numpy as np
import math
import json
from collections import deque
import requests
import time
import threading  # 중복 로그 및 윈도우 제어를 위한 락(Lock)


class GaugeMonitorNode(Node):
    def __init__(self):
        super().__init__('gauge_monitor_node')

        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()  # 쓰레드 간 동기화

        # 1. 설정 로드
        try:
            with open('gauge_template_final.json', 'r') as f:
                self.config = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Config Load Error: {e}')

        # 2. 상태 변수
        self.monitoring_active = False
        self.is_calibrated = False
        self.matrix = None
        self.image_received = False
        self.last_sent_status = None
        self.current_gauge_label = "Gauge_01"

        # 3. 타이밍 변수
        self.arrival_time = 0.0
        self.stabilization_delay = 2.0  # 2초 진동 방지
        self.calibration_start_time = 0.0
        self.retry_timeout = 10.0       # 10초 보정 시도

        # 4. 필터링
        self.value_history = deque(maxlen=10)
        self.stability_threshold = 0.5

        # 5. QoS 설정 (이미지 큐를 1로 제한하여 최신 프레임만 유지)
        from rclpy.qos import qos_profile_sensor_data

        # 구독자 및 발행자 (메시지 이름 유지)
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/robot2/oakd/rgb/image_raw/compressed',
            self.image_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group
        )

        self.goal_sub = self.create_subscription(
            Bool,
            '/waypoint_arrived',
            self.arrival_callback,
            10,
            callback_group=self.callback_group
        )

        self.ready_pub = self.create_publisher(
            Bool,
            '/gauge_safe_status',
            10,
            callback_group=self.callback_group
        )

        # 6. 카메라 상태 정보 타이머 (요청 사항 1)
        self.status_timer = self.create_timer(2.0, self.report_status)

        self.get_logger().info('=== Fixed OG Gauge Monitor Node Initialized ===')

    def report_status(self):
        """카메라 토픽 수신 여부 및 현재 상태 모니터링"""
        if not self.image_received:
            self.get_logger().warn(
                'Waiting for camera topic (/robot2/oakd/rgb/image_raw/compressed)...'
            )
        else:
            status = "ACTIVE (Analyzing)" if self.monitoring_active else "IDLE (Standby)"
            self.get_logger().info(f'Node Status: {status}')

    def arrival_callback(self, msg):
        if msg.data:
            self.get_logger().info('Arrival signal: Initializing logic.')
            with self.lock:
                self.value_history.clear()
                self.is_calibrated = False
                self.arrival_time = time.time()
                self.calibration_start_time = time.time()
                self.monitoring_active = True
        else:
            # Navigator가 False를 보내면 즉시 중단
            self.monitoring_active = False
            cv2.destroyAllWindows()

    def image_callback(self, msg):
        self.image_received = True

        if not self.monitoring_active:
            return

        # 로봇 정지 후 안정화 대기
        if time.time() - self.arrival_time < self.stabilization_delay:
            return

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if not self.is_calibrated:
                self.auto_calibrate(frame)

                # 보정 중 화면 표시 (쓰레드 게이트 적용)
                if self.monitoring_active and not self.is_calibrated:
                    cv2.putText(
                        frame,
                        "Searching Gauge...",
                        (30, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 0, 255),
                        2
                    )
                    cv2.imshow("Analysis Window", frame)
                    cv2.waitKey(1)

                    if time.time() - self.calibration_start_time > self.retry_timeout:
                        self.get_logger().error("Calibration Timeout.")
                        self.finish_process(is_safe=True)

            else:
                value, debug_img = self.read_gauge(frame)

                if value is not None:
                    self.value_history.append(value)
                    if len(self.value_history) == 10:
                        diff = max(self.value_history) - min(self.value_history)
                        if diff < self.stability_threshold:
                            avg_value = sum(self.value_history) / 10
                            is_safe = 1.0 < avg_value < 8.0
                            self.finish_process(is_safe, avg_value)

                # 결과 확인 중 화면 표시
                if self.monitoring_active:
                    cv2.imshow("Analysis Window", debug_img)
                    cv2.waitKey(1)
                else:
                    cv2.destroyAllWindows()

        except Exception as e:
            self.get_logger().error(f'Image Pipeline Error: {e}')

    def auto_calibrate(self, frame):
        """정확한 중심점 보정 (Centering)"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(
            cv2.medianBlur(gray, 5),
            cv2.HOUGH_GRADIENT,
            1,
            100,
            param1=100,
            param2=50,
            minRadius=50,
            maxRadius=300
        )

        if circles is not None:
            circles = np.uint16(np.around(circles))
            cx, cy, r = circles[0, 0]
            margin = int(r * 1.15)

            src_pts = np.float32([
                [cx - margin, cy - margin],
                [cx + margin, cy - margin],
                [cx + margin, cy + margin],
                [cx - margin, cy + margin]
            ])

            dst_pts = np.float32([
                [0, 0],
                [500, 0],
                [500, 500],
                [0, 500]
            ])

            self.matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
            self.is_calibrated = True
            self.get_logger().info('Calibration Success.')

    def read_gauge(self, frame):
        warped = cv2.warpPerspective(
            frame,
            self.matrix,
            (500, 500),
            flags=cv2.INTER_CUBIC
        )

        debug_img = warped.copy()
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(
            cv2.GaussianBlur(gray, (5, 5), 0),
            50,
            150
        )

        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180,
            threshold=40,
            minLineLength=60,
            maxLineGap=10
        )

        center = (250, 250)

        if lines is not None:
            valid_needles = []

            for line in lines:
                x1, y1, x2, y2 = line[0]
                if (
                    math.dist(center, (x1, y1)) < 40
                    or math.dist(center, (x2, y2)) < 40
                ):
                    valid_needles.append(
                        (x1, y1)
                        if math.dist(center, (x1, y1))
                        > math.dist(center, (x2, y2))
                        else (x2, y2)
                    )

            if valid_needles:
                best_pt = max(
                    valid_needles,
                    key=lambda p: math.dist(center, p)
                )

                cv2.line(debug_img, center, best_pt, (0, 255, 0), 3)

                dx, dy = best_pt[0] - center[0], best_pt[1] - center[1]
                angle = np.degrees(np.arctan2(dx, -dy)) % 360

                total_sweep = (
                    self.config["max_angle"]
                    - self.config["min_angle"]
                ) % 360

                rel_angle = (angle - self.config["min_angle"]) % 360
                ratio = np.clip(rel_angle / total_sweep, 0.0, 1.0)

                value = self.config["min_val"] + (
                    ratio * (self.config["max_val"] - self.config["min_val"])
                )

                return value, debug_img

        return None, debug_img

    def finish_process(self, is_safe, value=0.0):
        """Thread-Safe하게 결과를 전송하고 모든 프로세스를 종료"""
        with self.lock:
            if not self.monitoring_active:
                return

            self.monitoring_active = False  # 다른 쓰레드 즉시 차단

            # Flask 전송 (상태 변화 시)
            if is_safe != self.last_sent_status:
                try:
                    log_data = {
                        "label": self.current_gauge_label,
                        "value": round(value, 2),
                        "status": "정상" if is_safe else "비정상"
                    }
                    requests.post(
                        "http://192.168.108.21:5000/log",
                        json=log_data,
                        timeout=0.5
                    )
                    self.last_sent_status = is_safe
                except:
                    pass

            # 결과 발행 (True/False 분기 - 요청 사항 반영)
            self.ready_pub.publish(Bool(data=is_safe))
            self.get_logger().info(
                f'Final Result: {"SAFE" if is_safe else "UNSAFE"} ({value:.2f})'
            )

            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = GaugeMonitorNode()
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
