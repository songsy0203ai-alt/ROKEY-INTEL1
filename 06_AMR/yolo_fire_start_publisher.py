import os
import sys
import cv2
import time
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
        scan_max_index: int = 6,        # 0~(scan_max_index-1) 스캔
        max_read_failures: int = 10,    # ✅ 연속 프레임 read 실패 허용치
        auto_recover: bool = True,      # ✅ 자동복구 on/off
        recover_cooldown_frames: int = 15,  # ✅ 복구 연속시도 방지
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

        # ✅ 자동복구 관련
        self.auto_recover = bool(auto_recover)
        self.max_read_failures = max(1, int(max_read_failures))
        self.read_fail_count = 0
        self.recover_cooldown_frames = max(0, int(recover_cooldown_frames))
        self._recover_cooldown = 0

        # ✅ HUD/경고 UI
        self.show_hud = True
        self.show_help = True
        self.banner_until = 0.0          # 🔔 배너 표시 종료 시각
        self.banner_duration = 1.0       # 🔔 True 발생 순간 1초 표시

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
        self.get_logger().info(f"🔁 Auto recover: {self.auto_recover} (max_failures={self.max_read_failures})")
        self.get_logger().info("⌨️ Keys: [0-9]=switch cam, n=next, p=prev, l=list, a=auto_recover toggle, h=help, q=quit")

    # -------------------------
    # Camera helpers
    # -------------------------
    def _open_camera(self, idx: int) -> bool:
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

        ret, _ = cap.read()
        if not ret:
            self.get_logger().warn(f"❌ Camera read failed right after open: index={idx}")
            cap.release()
            return False

        self.cap = cap
        self.camera_index = idx
        self.read_fail_count = 0
        self.get_logger().info(f"✅ Camera switched/opened: index={idx}")
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

    def _attempt_auto_recover(self):
        self.get_logger().warn("🔄 Auto-recover triggered: trying to reopen current camera...")
        cur = self.camera_index

        if self._open_camera(cur):
            self.get_logger().info("✅ Auto-recover: reopened current camera successfully")
            self._recover_cooldown = self.recover_cooldown_frames
            return True

        self.get_logger().warn("🔎 Auto-recover: scanning other camera indices...")
        for idx in self.scan_indices:
            if idx == cur:
                continue
            if self._open_camera(idx):
                self.get_logger().info(f"✅ Auto-recover: switched to camera index={idx}")
                self._recover_cooldown = self.recover_cooldown_frames
                return True

        self.get_logger().error("❌ Auto-recover failed: no camera available")
        self._recover_cooldown = self.recover_cooldown_frames
        return False

    # -------------------------
    # Publish helper
    # -------------------------
    def _publish_bool(self, val: bool):
        msg = Bool()
        msg.data = bool(val)
        self.fire_pub.publish(msg)

    # -------------------------
    # UI helpers
    # -------------------------
    def _draw_hud(self, img, lines_left, lines_right=None, danger=False):
        if not self.show_hud:
            return img

        h, w = img.shape[:2]

        # danger면 붉은 톤, safe면 검정 톤
        bg_color = (20, 20, 160) if danger else (0, 0, 0)  # BGR
        bar_h = 70 if (lines_right and self.show_help) else 52

        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (w, bar_h), bg_color, -1)

        alpha = 0.35
        img = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)

        x = 12
        y = 26
        for i, t in enumerate(lines_left[:3]):
            cv2.putText(img, t, (x, y + i * 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)

        if lines_right and self.show_help:
            y2 = 26
            for i, t in enumerate(lines_right[:3]):
                (tw, _), _ = cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
                cv2.putText(img, t, (w - tw - 12, y2 + i * 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 2)

        # bar 아래 얇은 라인
        line_color = (0, 0, 255) if danger else (160, 160, 160)
        cv2.line(img, (0, bar_h), (w, bar_h), line_color, 2)

        return img

    def _draw_fire_banner(self, img):
        """🔔 True 상승엣지 순간 1초 동안 큰 배너 표시"""
        now = time.time()
        if now > self.banner_until:
            return img

        h, w = img.shape[:2]

        # 중앙 배너 영역
        banner_h = int(h * 0.22)
        y1 = int(h * 0.39)
        y2 = y1 + banner_h

        overlay = img.copy()
        cv2.rectangle(overlay, (0, y1), (w, y2), (0, 0, 255), -1)  # 빨간 배너
        img = cv2.addWeighted(overlay, 0.35, img, 0.65, 0)

        text = "🔥  FIRE DETECTED  🔥"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.6, 4)
        cv2.putText(img, text, ((w - tw) // 2, y1 + banner_h // 2 + th // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.6, (255, 255, 255), 4)

        sub = "TAKE ACTION IMMEDIATELY"
        (tw2, th2), _ = cv2.getTextSize(sub, cv2.FONT_HERSHEY_SIMPLEX, 0.85, 2)
        cv2.putText(img, sub, ((w - tw2) // 2, y2 - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2)

        return img

    # -------------------------
    # Keyboard handling
    # -------------------------
    def _handle_key(self, key: int):
        if key == ord('q'):
            self.shutdown()
            return True

        elif key == ord('n'):
            self._switch_next()

        elif key == ord('p'):
            self._switch_prev()

        elif key == ord('l'):
            self._list_available_cameras()

        elif key == ord('a'):
            self.auto_recover = not self.auto_recover
            self.get_logger().info(f"🔁 Auto-recover toggled -> {self.auto_recover}")
            self.read_fail_count = 0
            self._recover_cooldown = 0

        elif key == ord('h'):
            self.show_help = not self.show_help
            self.get_logger().info(f"🛈 Help overlay toggled -> {self.show_help}")

        elif ord('0') <= key <= ord('9'):
            idx = key - ord('0')
            self._open_camera(idx)

        return False

    # -------------------------
    # Main loop
    # -------------------------
    def process_frame(self):
        if self._recover_cooldown > 0:
            self._recover_cooldown -= 1

        if self.cap is None:
            self.get_logger().warn("No camera opened. Press 'l' to list, or '0-9'/'n'/'p' to switch.")
            if self.auto_recover and self._recover_cooldown == 0:
                self._attempt_auto_recover()
            return

        ret, img = self.cap.read()
        if not ret:
            self.read_fail_count += 1
            self.get_logger().warn(
                f"Failed to read frame (count={self.read_fail_count}/{self.max_read_failures}). "
                f"Camera might be disconnected."
            )
            if self.auto_recover and self.read_fail_count >= self.max_read_failures and self._recover_cooldown == 0:
                self._attempt_auto_recover()
            return

        self.read_fail_count = 0

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

        # Publish 정책 (그대로) + 🔔 상승엣지 배너 트리거
        if not self.fire_state:
            if next_state:
                self._publish_bool(True)
                self.fire_state = True
                self.banner_until = time.time() + self.banner_duration  # 🔔 1초 배너
                self.get_logger().info(f"🔥 FIRE=True published ONCE (stable {self.stable_frames} frames)")
            else:
                self._publish_bool(False)
        else:
            if not next_state:
                self.fire_state = False
                self.get_logger().info(f"✅ FIRE back to False (stable {self.stable_frames} frames)")

        # HUD (FIRE면 상단바 붉은 톤)
        hist_str = ''.join(['1' if x else '0' for x in self.fire_hist])

        left_lines = [
            f"Cam: {self.camera_index}   State: {'FIRE' if self.fire_state else 'SAFE'}",
            f"hist[{self.stable_frames}]: {hist_str}   conf_thres: {self.conf_thres:.2f}",
            f"AR: {'ON' if self.auto_recover else 'OFF'}   fails: {self.read_fail_count}/{self.max_read_failures}",
        ]

        right_lines = [
            "Keys: q quit | l list | a AR toggle | h help",
            "Keys: n/p next/prev | 0-9 switch cam",
        ]

        img = self._draw_hud(img, left_lines, right_lines, danger=self.fire_state)

        # 🔔 상승엣지 배너 (1초)
        img = self._draw_fire_banner(img)

        cv2.imshow("YOLO Fire Detection", img)

        key = cv2.waitKey(1) & 0xFF
        if key != 255:
            if self._handle_key(key):
                return

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
    model_path = "/home/rokey/fire_bool_ws/src/fire_detector/fire_detector/fire.pt"
    camera_index = 0

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
        scan_max_index=6,
        max_read_failures=10,
        auto_recover=True,
        recover_cooldown_frames=15,
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
