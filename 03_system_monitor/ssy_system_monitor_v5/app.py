#!/usr/bin/env python3
"""
[코드 기능]
- PC3(중앙 관제 서버)에서 USB 웹캠을 이용해 화재를 실시간 탐지(YOLOv8)하고, 
  네트워크를 통해 연결된 두 대의 AMR(Robot2, Robot3)의 상태 및 영상을 모니터링함.
- 화재 발생 시 ROS2 토픽을 통해 시스템 전체에 알람 신호를 전파하며, 모든 이상 징후를 DB에 기록함.

[입력(Input)]
- Central_Fire: PC3에 직접 연결된 USB 웹캠 영상 프레임
- AMR1/AMR2 Image: 각 로봇에서 송신하는 '/robotX/oakd/rgb/preview/image_raw' (sensor_msgs/Image)
- Gauge Data: 각 로봇에서 송신하는 '/robotX/gauge/result' (std_msgs/String, JSON 포맷)

[출력(Output)]
- Web Dashboard: Flask 기반 실시간 MJPEG 스트리밍 및 상태 조회 API
- Fire Signal: '/fire_detected_signal' (std_msgs/Bool) 토픽 발행
- Database: 'aprs_system.db' 내 이상 징후 로그 저장
"""

import os
import cv2
import time
import json
import datetime
import sqlite3
import numpy as np
import threading
from typing import Optional, Dict, Any

from flask import Flask, render_template, Response, jsonify

# ROS2 관련 라이브러리
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge

# YOLO 객체 탐지 모델
from ultralytics import YOLO


# =========================
# Flask App 설정
# =========================
app = Flask(__name__)

# =========================
# 글로벌 버퍼 및 상태 변수
# =========================
last_frames = {
    "Central_Fire": None,  # PC3 웹캠 프레임 저장
    "AMR1": None,          # Robot2 수신 프레임 저장
    "AMR2": None,          # Robot3 수신 프레임 저장
}

alarm_state = {
    "fire": False,         # 현재 화재 알람 상태
    "fire_last_ts": None,  # 마지막 화재 감지 시각
    "fire_conf": 0.0,      # 화재 탐지 확률
}

latest_gauges = {
    "AMR1": None,          # Robot2 게이지 분석 결과(JSON)
    "AMR2": None,          # Robot3 게이지 분석 결과(JSON)
}

DB_PATH = "/home/rokey/ssy_ws/03_system_monitor/ssy_system_monitor_v5/aprs_system.db"


# =========================
# DB 헬퍼 함수
# =========================
def init_db():
    """
    [기능] 시스템 로그 저장을 위한 SQLite 데이터베이스 초기화
    [입력] 없음
    [출력] 없음 (테이블 생성 수행)
    """
    try:
        conn = sqlite3.connect(DB_PATH)
        c = conn.cursor()
        # anomaly_logs 테이블이 없을 경우 생성
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
        # 데이터베이스 파일 쓰기 권한이나 SQL 문법 오류 시 발생
        print(f"[DB 오류] 초기화 실패: {e}")

def db_log(sensor_id: str, value: float):
    """
    [기능] 감지된 이상 데이터를 DB에 기록
    [입력] sensor_id(센서 식별자), value(감지된 값/수치)
    [출력] 없음
    """
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
        # DB 연결 끊김이나 파일 잠김(Lock) 발생 시 예외 처리
        print(f"[DB 오류] 로그 기록 실패: {e}")


# =========================
# ROS Bridge Node (PC3)
# =========================
class CentralBridge(Node):
    """
    [기능] 로봇 데이터 수신 및 중앙 관제 신호 발행을 위한 ROS2 노드
    [입력] AMR 영상 및 게이지 데이터 토픽
    [출력] 화재 감지 여부 토픽(/fire_detected_signal)
    """
    def __init__(
        self,
        amr1_img_topic="/robot2/oakd/rgb/preview/image_raw",
        amr2_img_topic="/robot3/oakd/rgb/preview/image_raw",
        amr1_gauge_topic="/robot2/gauge/result",
        amr2_gauge_topic="/robot3/gauge/result",
    ):
        super().__init__("aprs_central_bridge")
        self.bridge = CvBridge()

        # AMR 영상 구독자 설정
        self.sub_amr1 = self.create_subscription(Image, amr1_img_topic, self.amr1_cb, 10)
        self.sub_amr2 = self.create_subscription(Image, amr2_img_topic, self.amr2_cb, 10)

        # 게이지 분석 결과 구독자 설정
        self.sub_gauge1 = self.create_subscription(String, amr1_gauge_topic, self.gauge1_cb, 10)
        self.sub_gauge2 = self.create_subscription(String, amr2_gauge_topic, self.gauge2_cb, 10)

        # 화재 발생 신호 발행자 설정 (Transient Local 설정으로 늦게 킨 노드도 수신 가능)
        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.fire_pub = self.create_publisher(Bool, "/fire_detected_signal", qos_latched)

        self.get_logger().info("[PC3] CentralBridge 노드 기동 완료.")

    def amr1_cb(self, msg: Image):
        # ROS Image 메시지를 OpenCV 포맷으로 변환하여 버퍼에 저장
        last_frames["AMR1"] = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def amr2_cb(self, msg: Image):
        # ROS Image 메시지를 OpenCV 포맷으로 변환하여 버퍼에 저장
        last_frames["AMR2"] = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def gauge1_cb(self, msg: String):
        # 수신된 JSON 문자열 파싱 후 최신 데이터 갱신 및 DB 로그 기록
        payload = _safe_json(msg.data)
        latest_gauges["AMR1"] = payload
        if isinstance(payload, dict) and "value" in payload:
            db_log("AMR1_GAUGE", float(payload["value"]))

    def gauge2_cb(self, msg: String):
        payload = _safe_json(msg.data)
        latest_gauges["AMR2"] = payload
        if isinstance(payload, dict) and "value" in payload:
            db_log("AMR2_GAUGE", float(payload["value"]))

    def publish_fire(self, on: bool):
        """
        [기능] 화재 감지 여부 신호를 발행
        [입력] on (감지 여부 Bool 값)
        [출력] 없음
        """
        b = Bool()
        b.data = bool(on)
        self.fire_pub.publish(b)


def _safe_json(s: str):
    """
    [기능] 문자열을 JSON으로 안전하게 파싱
    [입력] s(문자열)
    [출력] 파싱된 딕셔너리 또는 원본 포함 객체
    """
    try:
        return json.loads(s)
    except Exception:
        # JSON 형식이 아닐 경우 에러 방지를 위해 원본 문자열 반환
        return {"raw": s}


# =========================
# Central_Fire capture + YOLO
# =========================
class FireCameraWorker:
    """
    [기능] USB 웹캠으로부터 실시간 영상을 읽고 YOLO 모델로 화재를 탐지하는 워커
    [입력] USB 카메라 프레임
    [출력] 화재 상태 갱신 및 콘솔 로그 출력
    """
    def __init__(
        self,
        ros_node: CentralBridge,
        model_path: str = "fire.pt",
        cam_index: int = 1,
        infer_fps: float = 5.0,
        fire_conf_th: float = 0.55,
        hold_seconds: float = 5.0,
    ):
        self.ros_node = ros_node
        self.model_path = model_path
        self.cam_index = cam_index
        self.infer_period = 1.0 / max(infer_fps, 0.1)
        self.fire_conf_th = fire_conf_th
        self.hold_seconds = hold_seconds

        self._stop = False
        self._thread = None

        # YOLO 모델 초기화
        self.model = YOLO(model_path)
        self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            print(f"[화재 감시] 카메라 인덱스 {cam_index}번을 열 수 없습니다.")

        self._last_infer_t = 0.0
        self._last_fire_t = 0.0 

    def start(self):
        # 별도 스레드에서 무한 루프 실행
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop = True
        try:
            if self.cap:
                self.cap.release()
        except Exception as e:
            # 카메라 리소스 해제 실패 시 무시
            pass

    def _run(self):
        while not self._stop:
            ok, frame = (False, None)
            if self.cap and self.cap.isOpened():
                ok, frame = self.cap.read()

            if ok and frame is not None:
                last_frames["Central_Fire"] = frame

                now = time.time()
                # 설정된 추론 FPS 주기에 맞춰 YOLO 실행
                if now - self._last_infer_t >= self.infer_period:
                    self._last_infer_t = now
                    fire_on, conf = self._infer_fire(frame)
                    self._update_fire_state(fire_on, conf)
            else:
                last_frames["Central_Fire"] = None
            time.sleep(0.01)

    def _infer_fire(self, frame_bgr) -> (bool, float):
        """
        [기능] YOLO 모델을 이용한 객체 탐지 수행
        [입력] frame_bgr (이미지 프레임)
        [출력] (fire_detected_bool, max_confidence)
        """
        try:
            # save=False 옵션으로 권한 문제 예방
            results = self.model.predict(frame_bgr, verbose=False, save=False, exist_ok=True)
            if not results:
                return False, 0.0

            r = results[0]
            if r.boxes is None or len(r.boxes) == 0:
                return False, 0.0

            # 가장 높은 Confidence 값을 가진 박스 확인
            confs = r.boxes.conf.detach().cpu().numpy().tolist()
            max_conf = float(max(confs)) if confs else 0.0
            fire = max_conf >= self.fire_conf_th
            
            if fire:
                print(f"[탐지 중] 화재 감지됨! 확률: {max_conf:.2f}")
                
            return fire, max_conf
        except Exception as e:
            # YOLO 엔진 오류 또는 라이브러리 충돌 시 예외 처리
            print(f"[화재 감시 오류] YOLO 추론 실패: {e}")
            return False, 0.0

    def _update_fire_state(self, fire_now: bool, conf: float):
        """
        [기능] 감지 결과를 바탕으로 알람 상태를 갱신하고 로그 출력
        [입력] fire_now(현재 탐지 여부), conf(확률)
        [출력] 없음
        """
        now = time.time()
        if fire_now:
            self._last_fire_t = now

        # 설정된 hold_seconds 초 동안은 화재가 깜빡여도 상태를 유지함
        fire_latched = (now - self._last_fire_t) <= self.hold_seconds

        prev = alarm_state["fire"]
        alarm_state["fire"] = bool(fire_latched)
        alarm_state["fire_conf"] = float(conf)
        
        if alarm_state["fire"]:
            alarm_state["fire_last_ts"] = datetime.datetime.now().isoformat(timespec="seconds")

        # 상태가 변할 때만 한국어 로그 출력 및 ROS 전파
        if prev != alarm_state["fire"]:
            status_msg = "발생" if alarm_state["fire"] else "해제"
            print("\n" + "="*50)
            print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] 화재 알람 {status_msg}!")
            print(f" - 현재 확률: {conf:.2f}")
            print(f" - ROS 전송: /fire_detected_signal -> {alarm_state['fire']}")
            print("="*50 + "\n")
            
            self.ros_node.publish_fire(alarm_state["fire"])
            db_log("CENTRAL_FIRE", 1.0 if alarm_state["fire"] else 0.0)


# =========================
# MJPEG streaming generator
# =========================
def gen_frames(camera_name: str):
    """
    [기능] 웹 브라우저 전송을 위한 MJPEG 비디오 프레임 생성기
    [입력] camera_name (Central_Fire / AMR1 / AMR2)
    [출력] MJPEG 스트림 바이트 데이터
    """
    while True:
        frame = last_frames.get(camera_name)

        if frame is None:
            # 수신되지 않을 경우 검은 화면에 안내 텍스트 출력
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, f"WAITING SOURCE: {camera_name}", (40, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            # 실시간 시각 및 카메라 이름 오버레이
            cv2.putText(frame, f"LIVE: {camera_name}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, datetime.datetime.now().strftime("%H:%M:%S"), (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # 화재 감지 시 빨간색 경고 문구 오버레이
            if camera_name == "Central_Fire" and alarm_state["fire"]:
                cv2.putText(frame, "!!! FIRE DETECTED !!!", (40, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

        ok, buffer = cv2.imencode(".jpg", frame)
        if not ok:
            continue

        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n")


# =========================
# Flask 라우팅 설정
# =========================
@app.route("/")
def index():
    return render_template("index.html")

@app.route("/alarm")
def alarm():
    return render_template("alarm.html")

@app.route("/video_feed/<cam_id>")
def video_feed(cam_id):
    # 각 카메라 ID에 맞는 스트리밍 응답 반환
    return Response(gen_frames(cam_id), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/api/status")
def api_status():
    return jsonify(alarm_state)

@app.route("/api/gauges")
def api_gauges():
    return jsonify(latest_gauges)

@app.route("/api/logs")
def api_logs():
    """
    [기능] DB에 저장된 최근 이상 로그 20개를 반환
    [입력] 없음
    [출력] JSON 형식의 로그 리스트
    """
    try:
        conn = sqlite3.connect(DB_PATH)
        c = conn.cursor()
        c.execute("SELECT timestamp, sensor_id, value FROM anomaly_logs ORDER BY id DESC LIMIT 20")
        rows = c.fetchall()
        conn.close()
        logs = [{"time": r[0], "id": r[1], "val": r[2]} for r in rows]
        return jsonify(logs)
    except Exception as e:
        # DB 읽기 실패 시 500 에러와 사유 반환
        return jsonify({"error": str(e)}), 500


# =========================
# 메인 실행 루프
# =========================
def main():
    # 1. DB 초기화
    init_db()

    # 2. ROS2 시스템 초기화
    rclpy.init()

    # 3. 중앙 관제 브리지 노드 기동
    bridge = CentralBridge()
    exec_ = MultiThreadedExecutor(num_threads=2)
    exec_.add_node(bridge)

    # ROS 멀티스레딩 스핀 시작 (백그라운드)
    ros_thread = threading.Thread(target=exec_.spin, daemon=True)
    ros_thread.start()

    # 4. 화재 감시 워커 기동
    model_path = '/home/rokey/ssy_ws/03_system_monitor/ssy_system_monitor_v5/fire.pt'
    cam_index = 1 # 웹캠 포트 번호
    fire_worker = FireCameraWorker(
        ros_node=bridge,
        model_path=model_path,
        cam_index=cam_index
    )
    fire_worker.start()

    print("\n[알림] APRS 중앙 서버가 기동되었습니다.")

    # 5. Flask 웹 서버 실행
    try:
        app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
    except Exception as e:
        print(f"[서버 오류] Flask 실행 실패: {e}")
    finally:
        # 프로그램 종료 시 리소스 정리
        fire_worker.stop()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()