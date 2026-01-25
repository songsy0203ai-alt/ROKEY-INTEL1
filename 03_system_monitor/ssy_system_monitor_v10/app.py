#!/usr/bin/env python3
"""
[코드 기능]
- PC3(중앙 관제 서버)에서 USB 웹캠을 이용해 화재를 실시간 탐지(YOLOv8)하고,
  네트워크를 통해 연결된 두 대의 AMR(Robot2, Robot3)의 상태 및 영상을 모니터링함.
- 화재 발생 시 ROS2 토픽을 통해 시스템 전체에 알람 신호를 전파하며, 모든 이상 징후를 DB에 기록함.
- ✅ 추가: 화재 발생 시 PC3에서 직접(브라우저 무관) 알람 사운드를 재생함.

[입력(Input)]
- Central_Fire: PC3에 직접 연결된 USB 웹캠 영상 프레임
- AMR1/AMR2 Image: 각 로봇에서 송신하는 '/robotX/oakd/rgb/preview/image_raw' (sensor_msgs/Image)
- Gauge Data:
  - (기존) '/robotX/gauge_safe_status' (std_msgs/Bool)
  - (추가) Flask '/log' (POST JSON)

[출력(Output)]
- Web Dashboard: Flask 기반 실시간 MJPEG 스트리밍 및 상태 조회 API
- Fire Signal: '/fire_detected_signal' (std_msgs/Bool) 토픽 발행
- Database: 'aprs_system.db' 내 이상 징후 로그 저장
- ✅ PC3 Speaker: 화재 상태 ON일 때 반복 알람 재생
"""

# =========================
# [1] 기본 라이브러리 / 유틸
# =========================
import os
import cv2
import time
import json
import datetime
import sqlite3
import numpy as np
import threading
from typing import Optional, Dict, Any

# ✅ PC3 사운드 재생용
import subprocess
import shutil

# =========================
# [2] Flask (Web Dashboard 서버)
# =========================
from flask import Flask, render_template, Response, jsonify, request
from flask import Flask, render_template, Response, jsonify, request, send_from_directory

# =========================
# [3] ROS2 (PC3에서 실행되는 중앙 브리지 노드)
# =========================
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge

# =========================
# [4] YOLO (중앙웹캠 기반 화재 탐지)
# =========================
from ultralytics import YOLO


# =========================
# Flask App 설정
# =========================
app = Flask(__name__)

# =========================
# [공유] 글로벌 버퍼 및 상태 변수
# =========================
last_frames = {
    "Central_Fire": None,
    "AMR1": None,
    "AMR2": None,
}

alarm_state = {
    "fire": False,
    "fire_last_ts": None,
    "fire_conf": 0.0,
}

latest_gauges = {
    "AMR1": None,
    "AMR2": None,
}

gauge_logs = []
GAUGE_LOG_MAXLEN = 200

DB_PATH = '/home/rokey/ssy_ws/03_system_monitor/ssy_system_monitor_v8/aprs_system.db'


# =========================
# ✅ PC3 알람 사운드 플레이어
# =========================
class AlarmSoundPlayer:
    """
    PC3에서 직접 소리를 내는 플레이어(브라우저 무관).
    - start(): 반복 재생 시작
    - stop(): 재생 중지
    - paplay(우선) / aplay(대체) 사용
    """
    def __init__(self, wav_path: str = None, interval_sec: float = 1.0):
        self.interval_sec = float(interval_sec)
        self._stop_evt = threading.Event()
        self._thread = None
        self._proc = None

        self._paplay = shutil.which("paplay")
        self._aplay = shutil.which("aplay")

        self.wav_path = wav_path or self._find_default_sound()

        if self.wav_path:
            print(f"[SOUND] ✅ sound file: {self.wav_path}")
        else:
            print("[SOUND] ⚠️ sound file not found. Using terminal bell only (may be silent).")

        if self._paplay:
            print("[SOUND] ✅ player: paplay")
        elif self._aplay:
            print("[SOUND] ✅ player: aplay")
        else:
            print("[SOUND] ⚠️ player not found (paplay/aplay).")

    def _find_default_sound(self):
        # paplay는 .oga도 가능, aplay는 wav가 안정적
        candidates = [
            "/usr/share/sounds/alsa/Front_Center.wav",
            "/usr/share/sounds/alsa/Front_Left.wav",
            "/usr/share/sounds/alsa/Noise.wav",
            "/usr/share/sounds/freedesktop/stereo/alarm-clock-elapsed.oga",
            "/usr/share/sounds/freedesktop/stereo/message.oga",
        ]
        for p in candidates:
            if os.path.exists(p):
                # aplay만 있는 환경이면 oga는 피하는게 안전(자동으로 wav 우선)
                if self._aplay and (p.endswith(".oga") or p.endswith(".ogg")):
                    continue
                return p
        # 그래도 없으면 oga라도 선택(paplay 환경이라면 OK)
        for p in candidates:
            if os.path.exists(p):
                return p
        return None

    def _kill_proc(self):
        if self._proc is not None:
            try:
                if self._proc.poll() is None:
                    self._proc.terminate()
            except Exception:
                pass
            self._proc = None

    def _play_once(self):
        self._kill_proc()

        # 사운드 파일 없으면 bell(환경에 따라 안 들릴 수 있음)
        if not self.wav_path:
            print("\a", end="", flush=True)
            return

        try:
            if self._paplay:
                self._proc = subprocess.Popen(
                    [self._paplay, self.wav_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
            elif self._aplay:
                self._proc = subprocess.Popen(
                    [self._aplay, "-q", self.wav_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
            else:
                print("\a", end="", flush=True)
        except Exception as e:
            print(f"[SOUND] play failed: {e}")

    def _loop(self):
        while not self._stop_evt.is_set():
            self._play_once()
            if self._stop_evt.wait(self.interval_sec):
                break
        self._kill_proc()

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("[SOUND] 🔥 START")

    def stop(self):
        self._stop_evt.set()
        self._kill_proc()
        print("[SOUND] ✅ STOP")


# =========================
# DB 헬퍼 함수
# =========================
def init_db():
    try:
        conn = sqlite3.connect(DB_PATH)
        c = conn.cursor()
        c.execute(
            """CREATE TABLE IF NOT EXISTS anomaly_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT,
                sensor_id TEXT,
                value REAL
            )"""
        )
        conn.commit()
        conn.close()
    except sqlite3.Error as e:
        print(f"[DB 오류] 초기화 실패: {e}")


def db_log(sensor_id: str, value: float):
    try:
        ts = datetime.datetime.now().isoformat(timespec="seconds")
        conn = sqlite3.connect(DB_PATH)
        c = conn.cursor()
        c.execute(
            "INSERT INTO anomaly_logs(timestamp, sensor_id, value) VALUES (?,?,?)",
            (ts, sensor_id, float(value)),
        )
        conn.commit()
        conn.close()
    except Exception as e:
        print(f"[DB 오류] 로그 기록 실패: {e}")


# =========================
# ROS Bridge Node (PC3)
# =========================
class CentralBridge(Node):
    def __init__(
        self,
        amr1_img_topic="/robot2/oakd/rgb/preview/image_raw",
        amr2_img_topic="/robot3/oakd/rgb/preview/image_raw",
        amr1_gauge_topic="/robot2/gauge_safe_status",
        amr2_gauge_topic="/robot3/gauge_safe_status",
    ):
        super().__init__("aprs_central_bridge")
        self.bridge = CvBridge()

        self.sub_amr1 = self.create_subscription(Image, amr1_img_topic, self.amr1_cb, 10)
        self.sub_amr2 = self.create_subscription(Image, amr2_img_topic, self.amr2_cb, 10)

        self.sub_gauge1 = self.create_subscription(Bool, amr1_gauge_topic, self.gauge1_cb, 10)
        self.sub_gauge2 = self.create_subscription(Bool, amr2_gauge_topic, self.gauge2_cb, 10)

        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.fire_pub = self.create_publisher(Bool, "/fire_detected_signal", qos_latched)

        self.get_logger().info("[PC3] CentralBridge 노드 기동 완료.")

    def amr1_cb(self, msg: Image):
        last_frames["AMR1"] = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def amr2_cb(self, msg: Image):
        last_frames["AMR2"] = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def gauge1_cb(self, msg: Bool):
        payload = {
            "source": "topic",
            "safe": bool(msg.data),
            "ts": datetime.datetime.now().isoformat(timespec="seconds"),
        }
        latest_gauges["AMR1"] = payload
        db_log("AMR1_GAUGE_SAFE", 1.0 if msg.data else 0.0)

    def gauge2_cb(self, msg: Bool):
        payload = {
            "source": "topic",
            "safe": bool(msg.data),
            "ts": datetime.datetime.now().isoformat(timespec="seconds"),
        }
        latest_gauges["AMR2"] = payload
        db_log("AMR2_GAUGE_SAFE", 1.0 if msg.data else 0.0)

    def publish_fire(self, on: bool):
        b = Bool()
        b.data = bool(on)
        self.fire_pub.publish(b)


def _safe_json(s: str):
    try:
        return json.loads(s)
    except Exception:
        return {"raw": s}


# =========================
# Central_Fire capture + YOLO
# =========================
class FireCameraWorker:
    def __init__(
        self,
        ros_node: CentralBridge,
        model_path: str = "fire.pt",
        cam_index: int = 1,
        infer_fps: float = 5.0,
        fire_conf_th: float = 0.55,
        hold_seconds: float = 5.0,
        sound_player: AlarmSoundPlayer = "/home/rokey/ssy_ws/03_system_monitor/ssy_system_monitor_v8/Fire.wav",   # ✅ 추가
    ):
        self.ros_node = ros_node
        self.model_path = model_path
        self.cam_index = cam_index

        self.infer_period = 1.0 / max(infer_fps, 0.1)
        self.fire_conf_th = fire_conf_th
        self.hold_seconds = hold_seconds

        self.sound_player = sound_player        # ✅ 추가

        self._stop = False
        self._thread = None

        self.model = YOLO(model_path)

        self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            print(f"[화재 감시] 카메라 인덱스 {cam_index}번을 열 수 없습니다.")

        self._last_infer_t = 0.0
        self._last_fire_t = 0.0

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop = True
        try:
            if self.sound_player:
                self.sound_player.stop()        # ✅ 안전 정지
        except Exception:
            pass
        try:
            if self.cap:
                self.cap.release()
        except Exception:
            pass

    def _run(self):
        while not self._stop:
            ok, frame = (False, None)
            if self.cap and self.cap.isOpened():
                ok, frame = self.cap.read()

            if ok and frame is not None:
                last_frames["Central_Fire"] = frame

                now = time.time()
                if now - self._last_infer_t >= self.infer_period:
                    self._last_infer_t = now
                    fire_on, conf = self._infer_fire(frame)
                    self._update_fire_state(fire_on, conf)
            else:
                last_frames["Central_Fire"] = None

            time.sleep(0.01)

    def _infer_fire(self, frame_bgr) -> (bool, float):
        try:
            results = self.model.predict(frame_bgr, verbose=False, save=False, exist_ok=True)
            if not results:
                return False, 0.0

            r = results[0]
            if r.boxes is None or len(r.boxes) == 0:
                return False, 0.0

            confs = r.boxes.conf.detach().cpu().numpy().tolist()
            max_conf = float(max(confs)) if confs else 0.0

            fire = max_conf >= self.fire_conf_th
            if fire:
                print(f"[탐지 중] 화재 감지됨! 확률: {max_conf:.2f}")

            return fire, max_conf
        except Exception as e:
            print(f"[화재 감시 오류] YOLO 추론 실패: {e}")
            return False, 0.0

    def _update_fire_state(self, fire_now: bool, conf: float):
        now = time.time()
        if fire_now:
            self._last_fire_t = now

        fire_latched = (now - self._last_fire_t) <= self.hold_seconds

        prev = alarm_state["fire"]
        alarm_state["fire"] = bool(fire_latched)
        alarm_state["fire_conf"] = float(conf)

        if alarm_state["fire"]:
            alarm_state["fire_last_ts"] = datetime.datetime.now().isoformat(timespec="seconds")

        if prev != alarm_state["fire"]:
            status_msg = "발생" if alarm_state["fire"] else "해제"
            print("\n" + "=" * 50)
            print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] 화재 알람 {status_msg}!")
            print(f" - 현재 확률: {conf:.2f}")
            print(f" - ROS 전송: /fire_detected_signal -> {alarm_state['fire']}")
            print("=" * 50 + "\n")

            # (1) ROS 알람 전파
            self.ros_node.publish_fire(alarm_state["fire"])

            # ✅ (2) PC3 사운드 ON/OFF
            if self.sound_player:
                if alarm_state["fire"]:
                    self.sound_player.start()
                else:
                    self.sound_player.stop()

            # (3) DB 기록
            db_log("CENTRAL_FIRE", 1.0 if alarm_state["fire"] else 0.0)


# =========================
# MJPEG streaming generator
# =========================
def gen_frames(camera_name: str):
    while True:
        frame = last_frames.get(camera_name)

        if frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, f"WAITING SOURCE: {camera_name}", (40, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            cv2.putText(frame, f"LIVE: {camera_name}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, datetime.datetime.now().strftime("%H:%M:%S"), (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            if camera_name == "Central_Fire" and alarm_state["fire"]:
                cv2.putText(frame, "!!! FIRE DETECTED !!!", (40, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

        ok, buffer = cv2.imencode(".jpg", frame)
        if not ok:
            continue

        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n")


# =========================
# ✅ 오디오 파일 서비스 라우트 추가
# =========================
@app.route('/audio/fire.wav')
def serve_audio():
    # 파일이 있는 디렉토리와 파일명을 지정
    audio_path = "/home/rokey/ssy_ws/03_system_monitor/ssy_system_monitor_v8"
    return send_from_directory(audio_path, 'Fire.wav')

# =========================
# Flask 라우팅 (통합본)
# =========================

@app.route("/")
def index():
    """메인 메뉴 페이지"""
    return render_template("index.html")

@app.route("/alarm")
def alarm():
    """화재 경보 전용 페이지"""
    return render_template("alarm.html")

# --- 실시간 개별 모니터링 페이지 ---
@app.route("/view/fire")
def view_fire():
    return render_template("cam_view.html", cam_id="Central_Fire", title="Central Fire Monitor")

@app.route("/view/amr1")
def view_amr1():
    return render_template("cam_view.html", cam_id="AMR1", title="AMR 01 Monitor")

@app.route("/view/amr2")
def view_amr2():
    return render_template("cam_view.html", cam_id="AMR2", title="AMR 02 Monitor")

@app.route("/logs")
def view_logs():
    """로그 전용 페이지"""
    return render_template("log_view.html")

# --- 데이터 및 스트리밍 API (필수 유지) ---
@app.route("/video_feed/<cam_id>")
def video_feed(cam_id):
    return Response(gen_frames(cam_id), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/api/status")
def api_status():
    res = alarm_state.copy()
    res["server_time"] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return jsonify(res)

@app.route("/api/logs")
def api_logs():
    try:
        conn = sqlite3.connect(DB_PATH)
        c = conn.cursor()
        c.execute("SELECT timestamp, sensor_id, value FROM anomaly_logs ORDER BY id DESC LIMIT 20")
        rows = c.fetchall()
        conn.close()
        logs = [{"time": r[0], "sensor": r[1], "type": "ALARM", "value": r[2]} for r in rows]
        return jsonify(logs)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route("/log", methods=["POST"])
def receive_log():
    # ... (기존 로그 수신 로직 유지)
    pass


# =========================
# main
# =========================
def main():
    init_db()
    rclpy.init()

    bridge = CentralBridge()

    exec_ = MultiThreadedExecutor(num_threads=2)
    exec_.add_node(bridge)

    ros_thread = threading.Thread(target=exec_.spin, daemon=True)
    ros_thread.start()

    # ✅ PC3 사운드 플레이어 준비

    # - wav_path를 지정 안 하면 시스템 기본 사운드(wav)를 자동으로 찾아 씀
    # - 네가 원하는 사이렌 wav 파일이 있으면 wav_path="/home/rokey/.../siren.wav" 처럼 지정
    sound_player = AlarmSoundPlayer(
        wav_path="/home/rokey/ssy_ws/03_system_monitor/ssy_system_monitor_v8/Fire.wav",
        interval_sec=22.0
    )

    model_path = '/home/rokey/ssy_ws/03_system_monitor/ssy_system_monitor_v8/fire.pt'
    cam_index = 1

    fire_worker = FireCameraWorker(
        ros_node=bridge,
        model_path=model_path,
        cam_index=cam_index,
        sound_player=sound_player,   # ✅ 주입
    )
    fire_worker.start()

    print("\n[알림] APRS 중앙 서버가 기동되었습니다.")

    try:
        app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
    except Exception as e:
        print(f"[서버 오류] Flask 실행 실패: {e}")
    finally:
        # ✅ 안전 정지
        try:
            sound_player.stop()
        except Exception:
            pass
        fire_worker.stop()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
