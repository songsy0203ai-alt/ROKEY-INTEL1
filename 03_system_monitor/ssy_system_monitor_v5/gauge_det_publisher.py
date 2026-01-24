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

class GaugeMonitorNode(Node):
    def __init__(self):
        super().__init__('gauge_monitor_node')
        
        # [Intermediate Point] 병렬 처리를 위해 ReentrantCallbackGroup 사용
        # 단일 스레드 데드락을 방지하고 영상 처리 중에도 토픽 수신을 허용함
        self.callback_group = ReentrantCallbackGroup()

        # JSON 설정 로드: 게이지의 최소/최대 각도 및 물리적 수치 매핑 데이터
        try:
            with open('gauge_template_final.json', 'r') as f:
                self.config = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')

        # 상태 제어 변수 (State Variables)
        self.monitoring_active = False  # 웨이포인트 도착 시에만 True로 전환 (자원 최적화)
        self.is_calibrated = False      # 게이지 외곽선(Circle) 검출 완료 여부
        self.matrix = None              # 시점 변환(Perspective Transform)을 위한 행렬
        self.image_received = False     # 하트비트 체크용 이미지 수신 플래그

        # [Logic] 노이즈 제거를 위한 10프레임 Moving Average 필터 (Sliding Window)
        self.value_history = deque(maxlen=10)
        self.stability_threshold = 0.5  # 수치 안정화 판정 임계값

        # QoS 및 Callback Group을 설정하여 구독자(Subscriber) 정의
        self.image_sub = self.create_subscription(
            CompressedImage, '/robot2/oakd/rgb/image_raw/compressed', 
            self.image_callback, 10, callback_group=self.callback_group)
        
        self.goal_sub = self.create_subscription(
            Bool, '/waypoint_arrived', self.arrival_callback, 10, 
            callback_group=self.callback_group)
        
        self.ready_pub = self.create_publisher(
            Bool, '/gauge_safe_status', 10, callback_group=self.callback_group)

        # 시스템 헬스 체크를 위한 주기적 상태 타이머
        self.status_timer = self.create_timer(2.0, self.report_status)
        
        self.get_logger().info('=== Gauge Monitor Node Initialized ===')

    def report_status(self):
        """노드 생존 확인 및 토픽 연결 상태 모니터링 로그"""
        if not self.image_received:
            self.get_logger().warn('Waiting for camera topic...')
        else:
            status = "ACTIVE" if self.monitoring_active else "IDLE"
            self.get_logger().info(f'Node Status: {status}')

    def arrival_callback(self, msg):
        """내비게이션 스크립트로부터 도착 신호를 수신하면 판독 루틴 활성화"""
        if msg.data:
            self.get_logger().info('Arrival signal received. Starting AGD pipeline.')
            self.value_history.clear()
            self.monitoring_active = True
            self.is_calibrated = False 

    def image_callback(self, msg):
        """메인 영상 처리 파이프라인 (Compressed -> CV2 -> Calibration -> Reading)"""
        self.image_received = True
        if not self.monitoring_active:
            return

        try:
            # 압축된 JPEG 데이터를 넘파이 배열을 거쳐 OpenCV 매트릭스로 디코딩
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if not self.is_calibrated:
                # 1단계: Hough Circle 기반 게이지 위치 자동 보정
                self.auto_calibrate(frame)
                cv2.imshow("Calibration Mode", frame)
                cv2.waitKey(1)
            else:
                # 2단계: 바늘 검출 및 물리 수치 변환
                value, debug_img = self.read_gauge(frame)
                
                if value is not None:
                    self.value_history.append(value)
                    
                    # 3단계: 데이터 안정화 검증 (10프레임 연속성 확인)
                    if len(self.value_history) == 10:
                        diff = max(self.value_history) - min(self.value_history)
                        if diff < self.stability_threshold:
                            # 4단계: 최종 판단 후 결과 발행 (Publish)
                            avg_value = sum(self.value_history) / 10
                            self.check_safety(avg_value)
                
                cv2.imshow("Live Analysis", debug_img)
                cv2.waitKey(1)
                    
        except Exception as e:
            self.get_logger().error(f'Pipeline Error: {e}')

    def auto_calibrate(self, frame):
        """Hough Circles 알고리즘을 사용하여 원형 게이지를 찾고 Perspective 변환 수행"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(cv2.medianBlur(gray, 5), cv2.HOUGH_GRADIENT, 1, 100, 
                                   param1=100, param2=50, minRadius=50, maxRadius=300)

        if circles is not None:
            # 검출된 원의 좌표를 바탕으로 정면 시점(Bird's Eye View) 변환 행렬 생성
            circles = np.uint16(np.around(circles))
            cx, cy, r = circles[0, 0]
            offset = 10
            src_pts = np.float32([[cx-r-offset, cy-r-offset], [cx+r+offset, cy-r-offset], 
                                  [cx+r+offset, cy+r+offset], [cx-r-offset, cy+r+offset]])
            dst_pts = np.float32([[0, 0], [500, 0], [500, 500], [0, 500]])
            self.matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
            self.is_calibrated = True
            self.get_logger().info('Calibration Success: Warp matrix generated.')

    def read_gauge(self, frame):
        # [Intermediate Point] Cubic Interpolation 적용
        # 단순 선형 보간보다 주변 16픽셀을 활용하여 고주파 성분(Edge)을 더 선명하게 보존함
        warped = cv2.warpPerspective(frame, self.matrix, (500, 500), flags=cv2.INTER_CUBIC)
        debug_img = warped.copy()
        
        # 가우시안 블러로 미세한 압축 노이즈(Artifacts) 제거
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # 바늘 후보군 검출 (파라미터 튜닝: minLineLength를 높여 잔상을 제거)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=80, maxLineGap=5)
        
        center = (250, 250) # Warp 설계상 고정된 게이지 회전 중심축
        cv2.circle(debug_img, center, 5, (0, 0, 255), -1)

        if lines is not None:
            valid_needles = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                pt1, pt2 = (x1, y1), (x2, y2)

                # [Spatial Filtering] 양 끝점 중 중심과 더 가까운 점을 시작점으로 설정
                dist1 = math.dist(center, pt1)
                dist2 = math.dist(center, pt2)
                
                start_pt, start_dist = (pt1, dist1) if dist1 < dist2 else (pt2, dist2)
                end_pt = pt2 if dist1 < dist2 else pt1

                # 제안 로직: 시작점이 중심 반경 10px 이내인 경우만 "바늘"로 인정
                if start_dist <= 10:
                    valid_needles.append((start_pt, end_pt))

            if valid_needles:
                # 필터링된 후보 중 가장 긴 선분을 최종 바늘로 확정
                best_needle = max(valid_needles, key=lambda n: math.dist(n[0], n[1]))
                start_node, end_node = best_needle
                
                # 시각화: 초록색 선으로 검출 결과 표시
                cv2.line(debug_img, center, end_node, (0, 255, 0), 3)
                
                # 각도 산출 및 0-360도 정규화
                dx, dy = end_node[0] - center[0], end_node[1] - center[1]
                angle = np.degrees(np.arctan2(dx, -dy)) % 360
                
                # Calibration 데이터를 기반으로 물리 수치 변환 (Linear Interpolation)
                total_sweep = (self.config["max_angle"] - self.config["min_angle"]) % 360
                rel_angle = (angle - self.config["min_angle"]) % 360
                ratio = np.clip(rel_angle / total_sweep, 0.0, 1.0)
                value = self.config["min_val"] + (ratio * (self.config["max_val"] - self.config["min_val"]))
                
                cv2.putText(debug_img, f"Val: {value:.2f}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                return value, debug_img
        
        return None, debug_img

    def check_safety(self, value):
        """최종 안정화된 값이 임계 범위 내에 있는지 판단하여 결과 발행"""
        msg = Bool()
        if 1.0 < value < 8.0:
            self.get_logger().info(f'SAFE RANGE: {value:.2f}. Publishing True.')
            msg.data = True
            self.ready_pub.publish(msg)
            self.monitoring_active = False # 태스크 완료 후 비활성화 (자원 회수)
            cv2.destroyAllWindows()
        else:
            self.get_logger().warn(f'DANGER RANGE: {value:.2f}. Waiting for recovery.')
            msg.data = False
            self.ready_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GaugeMonitorNode()
    # [Intermediate Point] 멀티스레드 실행기를 사용하여 콜백 간 병목 현상 제거
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()