#!/usr/bin/env python3
import os
import sys
import cv2
from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from ultralytics import YOLO


class YOLOFireStartPublisher(Node):
    def __init__(
        self,
        model_path: str,
        camera_index: int = 0,
        conf_thres: float = 0.5,
        stable_frames: int = 5,
        draw_only_fire: bool = True,
        scan_max_index: int = 6,   # 0~5 스캔
    ):
        super().__init__('yolo_fire_start_publisher')

        self.model = YOLO(model_path)
        self.class_names = self.model.names
        self.conf_thres = float(conf_thres)

        # ✅ deque 기반 안정화
        self.stable_frames = max(1, int(stable_frames))
        self.fire_hist = deque(maxlen=self.stable_frames)

        # ✅ fire 상태 기억(초기 False)
        self.fire_state = False
        self.draw_only_fire = bool(draw_only_fire)

        # ✅ 카메라 인덱스 관리
        self.scan_indices = list(range(0, int(scan_max_index)))
        self.camera_index = int(camera_index)
        self.cap = None
        self._open_camera(self.camera_index)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.fire_pub = self.create_publisher(Bool, '/fire_detected_signal', qos)

        # 10Hz
        self.timer = self.create_timer(0.1, self.process_frame)

        self.get_logger().info("🔥 YOLO Fire Publisher running")
        self.get_logger().info(f"📷 Camera index: {self.camera_index}")
        self.get_logger().info(f"🎯 Confidence threshold: {self.conf_thres}")
        self.get_logger().info(f"🧱 Stable frames (deque): {self.stable_frames}")
        self.get_logger().info("⌨️ Keys: [0-9]=switch cam, n=next, p=prev, l=list, q=quit")

    # -------------------------
    # Camera helpers
    # -------------------------
    def _open_camera(self, idx: int) -> bool:
        # 기존 cap 정리
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            self.get_logger().warn(f"❌ Camera open failed: index={idx}")
            return False

        # 첫 프레임 확인
        ret, _ = cap.read()
        if not ret:
            self.get_logger().warn(f"❌ Camera read failed right after open: index={idx}")
            cap.release()
            return False

        self.cap = cap
        self.camera_index = idx
        self.get_logger().info(f"✅ Camera switched: index={idx}")
        return True

    def _list_available_cameras(self):
        ok_list = []
        for idx in self.scan_indices:
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    ok_list.append(idx)
            cap.release()
        self.get_logger().info(f"📷 Available camera indices: {ok_list if ok_list else 'None'}")

    def _switch_next(self):
        if not self.scan_indices:
            return
        cur_pos = self.scan_indices.index(self.camera_index) if self.camera_index in self.scan_indices else 0
        for step in range(1, len(self.scan_indices) + 1):
            nxt = self.scan_indices[(cur_pos + step) % len(self.scan_indices)]
            if self._open_camera(nxt):
                return
        self.get_logger().warn("❌ No camera could be opened (next)")

    def _switch_prev(self):
        if not self.scan_indices:
            return
        cur_pos = self.scan_indices.index(self.camera_index) if self.camera_index in self.scan_indices else 0
        for step in range(1, len(self.scan_indices) + 1):
            prv = self.scan_indices[(cur_pos - step) % len(self.scan_indices)]
            if self._open_camera(prv):
                return
        self.get_logger().warn("❌ No camera could be opened (prev)")

    # -------------------------
    # ROS publish helper
    # -------------------------
    def _publish_bool(self, val: bool):
        msg = Bool()
        msg.data = bool(val)
        self.fire_pub.publish(msg)

    # -------------------------
    # Main loop
    # -------------------------
    def process_frame(self):
        if self.cap is None:
            self.get_logger().warn("No camera opened. Press 'l' to list, or '0-9'/'n'/'p' to switch.")
            return

        ret, img = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame (camera might be disconnected). Try switching camera.")
            return

        # YOLO 추론
        results = self.model(img, stream=True)

        detected_now = False

        for r in results:
            if r.boxes is None:
                continue

            for box in r.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = str(self.class_names[cls]).lower()

                if label == 'fire' and conf >= self.conf_thres:
                    detected_now = True

                if self.draw_only_fire and label != 'fire':
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                text = f"{label} {conf:.2f}"
                cv2.putText(img, text, (x1, max(20, y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # deque 안정화
        self.fire_hist.append(bool(detected_now))
        stable_true = (len(self.fire_hist) == self.stable_frames) and all(self.fire_hist)
        stable_false = (len(self.fire_hist) == self.stable_frames) and (not any(self.fire_hist))

        next_state = self.fire_state
        if stable_true:
            next_state = True
        elif stable_false:
            next_state = False

        # Publish 정책 (그대로)
        if not self.fire_state:
            if next_state:
                self._publish_bool(True)
                self.fire_state = True
                self.get_logger().info(f"🔥 FIRE=True published ONCE (stable {self.stable_frames} frames)")
            else:
                self._publish_bool(False)
        else:
            if not next_state:
                self.fire_state = False
                self.get_logger().info(f"✅ FIRE back to False (stable {self.stable_frames} frames)")

        # 화면 표시 + 키 입력 처리
        hist_str = ''.join(['1' if x else '0' for x in self.fire_hist])
        status_text = f"Cam:{self.camera_index}  State:{'FIRE' if self.fire_state else 'SAFE'}  hist={hist_str}"
        cv2.putText(img, status_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.imshow("YOLO Fire Detection", img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.shutdown()
            return
        elif key == ord('n'):
            self._switch_next()
        elif key == ord('p'):
            self._switch_prev()
        elif key == ord('l'):
            self._list_available_cameras()
        elif ord('0') <= key <= ord('9'):
            idx = key - ord('0')
            self._open_camera(idx)

    def shutdown(self):
        self.get_logger().info("🛑 Shutting down YOLO publisher")
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        self.destroy_node()


def main():
    model_path = "/home/rokey/ssy_ws/03_system_monitor/ssy_system_monitor_v7/fire.pt"
    camera_index = 1

    if not os.path.exists(model_path):
        print(f"❌ Model not found: {model_path}")
        sys.exit(1)

    rclpy.init()
    node = YOLOFireStartPublisher(
        model_path,
        camera_index=camera_index,
        conf_thres=0.5,
        stable_frames=5,
        draw_only_fire=True,
        scan_max_index=6
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()
        print("✅ YOLO publisher shutdown")


if __name__ == "__main__":
    main()

