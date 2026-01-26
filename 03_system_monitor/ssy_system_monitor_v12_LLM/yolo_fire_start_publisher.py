import os
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
import cv2  # 이미지 캡처를 위해 cv2 자체는 필요하지만 imshow/waitKey는 제거

from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from ultralytics import YOLO

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
        super().__init__('yolo_fire_start_publisher')

        # YOLO 모델 초기화
        self.model = YOLO(model_path)
        self.class_names = self.model.names
        self.conf_thres = float(conf_thres)

        # 상태 안정화 알고리즘 (deque)
        self.stable_frames = max(1, int(stable_frames))
        self.fire_hist = deque(maxlen=self.stable_frames)
        self.fire_state = False

        # 카메라 관리
        self.scan_indices = list(range(0, int(scan_max_index)))
        self.camera_index = int(camera_index)
        self.cap = None

        # 자동 복구 로직
        self.auto_recover = bool(auto_recover)
        self.max_read_failures = max(1, int(max_read_failures))
        self.read_fail_count = 0
        self.recover_cooldown_frames = max(0, int(recover_cooldown_frames))
        self._recover_cooldown = 0

        self._open_camera(self.camera_index)

        # ROS 2 Publisher 설정
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.fire_pub = self.create_publisher(Bool, '/fire_detected_signal', qos)

        # 메인 루프 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.process_frame)

        self.get_logger().info("🔥 YOLO Fire Publisher (Headless Mode) started")
        self.get_logger().info(f"📷 Camera: {self.camera_index} | 🎯 Conf: {self.conf_thres} | 🧱 Stable: {self.stable_frames}")

    def _open_camera(self, idx: int) -> bool:
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            self.get_logger().warn(f"❌ Camera open failed: index={idx}")
            return False

        ret, _ = cap.read()
        if not ret:
            self.get_logger().warn(f"❌ Camera read failed: index={idx}")
            cap.release()
            return False

        self.cap = cap
        self.camera_index = idx
        self.read_fail_count = 0
        return True

    def _attempt_auto_recover(self):
        self.get_logger().warn("🔄 Auto-recover triggered...")
        # 현재 카메라 재시도 후 실패 시 전체 스캔
        for idx in [self.camera_index] + self.scan_indices:
            if self._open_camera(idx):
                self.get_logger().info(f"✅ Recovered on camera index={idx}")
                self._recover_cooldown = self.recover_cooldown_frames
                return True
        self._recover_cooldown = self.recover_cooldown_frames
        return False

    def _publish_bool(self, val: bool):
        msg = Bool()
        msg.data = bool(val)
        self.fire_pub.publish(msg)

    def process_frame(self):
        if self._recover_cooldown > 0:
            self._recover_cooldown -= 1

        if self.cap is None:
            if self.auto_recover and self._recover_cooldown == 0:
                self._attempt_auto_recover()
            return

        ret, img = self.cap.read()
        if not ret:
            self.read_fail_count += 1
            if self.auto_recover and self.read_fail_count >= self.max_read_failures and self._recover_cooldown == 0:
                self._attempt_auto_recover()
            return

        self.read_fail_count = 0

        # YOLO 추론 (Visuals 제외)
        results = self.model(img, stream=True, verbose=False)
        detected_now = False

        for r in results:
            if r.boxes is None: continue
            for box in r.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = str(self.class_names[cls]).lower()

                if label == 'fire' and conf >= self.conf_thres:
                    detected_now = True
                    break
            if detected_now: break

        # 상태 안정화 로직
        self.fire_hist.append(detected_now)
        stable_true = (len(self.fire_hist) == self.stable_frames) and all(self.fire_hist)
        stable_false = (len(self.fire_hist) == self.stable_frames) and not any(self.fire_hist)

        if not self.fire_state:
            if stable_true:
                self._publish_bool(True)
                self.fire_state = True
                self.get_logger().info("🔥 [ALERT] FIRE DETECTED!")
            else:
                self._publish_bool(False)
        else:
            if stable_false:
                self.fire_state = False
                self.get_logger().info("✅ [CLEAR] Fire status cleared.")

    def shutdown(self):
        if self.cap is not None:
            self.cap.release()
        self.destroy_node()

def main():
    # 경로 설정
    model_path = "/home/rokey/ssy_ws/03_system_monitor/ssy_system_monitor_v8/fire.pt"
    
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