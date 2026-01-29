import os
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
import cv2 

from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from ultralytics import YOLO

"""
- [코드 기능]: 카메라 영상을 실시간으로 분석하여 '화재(fire)'를 감지하고, 상태 안정화 알고리즘을 거쳐 신뢰할 수 있는 화재 발생 신호를 ROS 2 토픽으로 발행함.
- [입력(Input)]: 로컬 카메라 영상 스트림(cv2.VideoCapture), YOLOv8 학습 모델 파일(.pt)
- [출력(Output)]: 화재 감지 여부(std_msgs/Bool, /fire_detected_signal)
"""

class YOLOFireStartPublisher(Node):
    def __init__(
        self,
        model_path: str,
        camera_index: int = 2,
        conf_thres: float = 0.5,
        stable_frames: int = 5,
        scan_max_index: int = 6,
        max_read_failures: int = 10,
        auto_recover: bool = True,
        recover_cooldown_frames: int = 15,
    ):
        """
        [인풋]: 모델 경로, 카메라 인덱스, 신뢰도 임계값, 안정화 프레임 수 등 설정 파라미터
        [아웃풋]: 클래스 인스턴스 초기화
        """
        super().__init__('yolo_fire_start_publisher')

        # YOLO 모델 로드 및 클래스 이름 지정
        self.model = YOLO(model_path)
        self.class_names = self.model.names
        self.conf_thres = float(conf_thres)

        # 큐(deque)를 이용한 상태 안정화 변수: 모든 요소가 True여야 감지로 인정
        self.stable_frames = max(1, int(stable_frames))
        self.fire_hist = deque(maxlen=self.stable_frames)
        self.fire_state = False # 현재 최종 화재 상태 (Latching)

        # 카메라 장치 관리 인덱스 설정
        self.scan_indices = list(range(0, int(scan_max_index)))
        self.camera_index = int(camera_index)
        self.cap = None

        # 카메라 연결 실패 시 자동 복구를 위한 변수들
        self.auto_recover = bool(auto_recover)
        self.max_read_failures = max(1, int(max_read_failures))
        self.read_fail_count = 0
        self.recover_cooldown_frames = max(0, int(recover_cooldown_frames))
        self._recover_cooldown = 0

        # 초기 카메라 스트림 오픈
        self._open_camera(self.camera_index)

        # ROS 2 퍼블리셔 설정 (신뢰성 모드: RELIABLE, 내구성: TRANSIENT_LOCAL)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.fire_pub = self.create_publisher(Bool, '/fire_detected_signal', qos)

        # 0.1초(10Hz) 간격으로 메인 로직(process_frame) 실행
        self.timer = self.create_timer(0.1, self.process_frame)

        self.get_logger().info("🔥 YOLO Fire Publisher (Headless Mode) started")

    def _open_camera(self, idx: int) -> bool:
        """
        [인풋]: idx (int - 열고자 하는 카메라 인덱스)
        [아웃풋]: 성공 여부 (bool)
        """
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            return False

        # 카메라가 정상적으로 프레임을 읽어오는지 첫 프레임 테스트
        ret, _ = cap.read()
        if not ret:
            cap.release()
            return False

        self.cap = cap
        self.camera_index = idx
        self.read_fail_count = 0
        return True

    def _attempt_auto_recover(self):
        """
        [인풋]: 없음
        [아웃풋]: 복구 성공 여부 (bool)
        """
        self.get_logger().warn("🔄 Auto-recover triggered...")
        # 현재 인덱스 및 스캔 가능 범위 내의 모든 인덱스 순회하며 재연결 시도
        for idx in [self.camera_index] + self.scan_indices:
            if self._open_camera(idx):
                self.get_logger().info(f"✅ Recovered on camera index={idx}")
                self._recover_cooldown = self.recover_cooldown_frames
                return True
        self._recover_cooldown = self.recover_cooldown_frames
        return False

    def _publish_bool(self, val: bool):
        """
        [인풋]: val (bool - 전송할 논리값)
        [아웃풋]: 없음 (ROS 토픽 발행)
        """
        msg = Bool()
        msg.data = bool(val)
        self.fire_pub.publish(msg)

    def process_frame(self):
        """
        [인풋]: 없음 (타이머 콜백)
        [아웃풋]: 없음 (추론 결과에 따른 상태 업데이트 및 토픽 발행)
        """
        # 복구 쿨다운 진행 중이면 대기
        if self._recover_cooldown > 0:
            self._recover_cooldown -= 1

        # 카메라 객체가 없을 경우 복구 시도
        if self.cap is None:
            if self.auto_recover and self._recover_cooldown == 0:
                self._attempt_auto_recover()
            return

        # 프레임 캡처 및 읽기 실패 처리
        ret, img = self.cap.read()
        if not ret:
            self.read_fail_count += 1
            if self.auto_recover and self.read_fail_count >= self.max_read_failures and self._recover_cooldown == 0:
                self._attempt_auto_recover()
            return

        self.read_fail_count = 0

        # YOLOv8 추론 실행 (스트리밍 모드, 로그 출력 억제)
        results = self.model(img, stream=True, verbose=False)
        detected_now = False

        # 추론 결과에서 'fire' 클래스 탐색
        for r in results:
            if r.boxes is None: continue
            for box in r.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = str(self.class_names[cls]).lower()

                # 조건: 라벨이 fire이고 $conf \ge \text{conf\_thres}$ 인 경우
                if label == 'fire' and conf >= self.conf_thres:
                    detected_now = True
                    break
            if detected_now: break

        # 상태 안정화 로직: 윈도우 내의 모든 프레임이 일치해야 상태 전환
        self.fire_hist.append(detected_now)
        # $\text{stable\_true} \iff \forall x \in \text{fire\_hist}, x = True$
        stable_true = (len(self.fire_hist) == self.stable_frames) and all(self.fire_hist)
        # $\text{stable\_false} \iff \forall x \in \text{fire\_hist}, x = False$
        stable_false = (len(self.fire_hist) == self.stable_frames) and not any(self.fire_hist)

        if not self.fire_state:
            # 꺼진 상태에서 켜짐으로 전환 시도
            if stable_true:
                self._publish_bool(True)
                self.fire_state = True
                self.get_logger().info("🔥 [ALERT] FIRE DETECTED!")
            else:
                self._publish_bool(False)
        else:
            # 켜진 상태에서 모든 프레임이 깨끗하면 꺼짐으로 전환
            if stable_false:
                self.fire_state = False
                self.get_logger().info("✅ [CLEAR] Fire status cleared.")

    def shutdown(self):
        """
        [인풋]: 없음
        [아웃풋]: 자원 해제
        """
        if self.cap is not None:
            self.cap.release()
        self.destroy_node()

def main():
    """
    [인풋]: 없음
    [아웃풋]: 노드 실행 및 종료 관리
    """
    model_path = "/home/rokey/ssy_ws/03_system_monitor/ssy_system_monitor_v8/fire.pt"
    
    # 모델 파일 존재 여부 선행 확인
    if not os.path.exists(model_path):
        print(f"❌ Model not found: {model_path}")
        sys.exit(1)

    rclpy.init()
    node = YOLOFireStartPublisher(model_path=model_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()