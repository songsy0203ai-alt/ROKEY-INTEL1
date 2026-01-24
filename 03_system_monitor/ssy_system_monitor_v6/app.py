import os
import cv2
import time
import datetime
import sqlite3
import numpy as np
import threading
from flask import Flask, render_template, Response, jsonify
from PIL import ImageFont, ImageDraw, Image

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO

app = Flask(__name__)

# 글로벌 상태 관리
last_frames = {"Central_Fire": None, "AMR1": None, "AMR2": None}
last_update_time = {"AMR1": 0, "AMR2": 0} # 도킹 판별용
fire_status = False
DB_PATH = 'aprs_system.db'

def init_db():
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute("CREATE TABLE IF NOT EXISTS anomaly_logs (id INTEGER PRIMARY KEY AUTOINCREMENT, timestamp TEXT, sensor_id TEXT, status TEXT)")
    conn.commit()
    conn.close()

def db_log(sensor_id, status):
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    c.execute("INSERT INTO anomaly_logs(timestamp, sensor_id, status) VALUES (?,?,?)", (ts, sensor_id, status))
    conn.commit()
    conn.close()

class CentralBridge(Node):
    def __init__(self):
        super().__init__("central_monitor_bridge")
        self.bridge = CvBridge()
        
        # 구독 설정
        self.create_subscription(Image, "/robot2/oakd/rgb/preview/image_raw", self.amr1_cb, 10)
        self.create_subscription(Image, "/robot3/oakd/rgb/preview/image_raw", self.amr2_cb, 10)
        self.create_subscription(Bool, "/robot2/gauge_safe_status", lambda msg: self.gauge_cb(msg, "AMR1"), 10)
        self.create_subscription(Bool, "/robot3/gauge_safe_status", lambda msg: self.gauge_cb(msg, "AMR2"), 10)
        self.create_subscription(Bool, "/fire_detected_signal", self.fire_signal_cb, 10)

    def amr1_cb(self, msg):
        last_frames["AMR1"] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        last_update_time["AMR1"] = time.time()

    def amr2_cb(self, msg):
        last_frames["AMR2"] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        last_update_time["AMR2"] = time.time()

    def gauge_cb(self, msg, sensor_id):
        if msg.data: # True일 때 이상치 발생으로 기록
            db_log(sensor_id, "GAUGE_ANOMALY")

    def fire_signal_cb(self, msg):
        global fire_status
        fire_status = msg.data


# 한글 렌더링을 위한 헬퍼 함수
def draw_korean_text(img, text, position, font_size, color):
    img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img_pil)
    # 리눅스 시스템의 경우 보통 아래 경로에 폰트가 있습니다. 없으면 설치된 폰트 경로로 변경하세요.
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf", font_size)
    except:
        font = ImageFont.load_default()
        
    draw.text(position, text, font=font, fill=color)
    return cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)

def gen_frames(camera_name):
    while True:
        now = time.time()
        frame = last_frames.get(camera_name)
        
        # AMR 도킹/순찰 판별
        if camera_name in ["AMR1", "AMR2"]:
            if now - last_update_time[camera_name] > 2.0:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                # 한글 텍스트 적용
                frame = draw_korean_text(frame, "순찰봇이 쉬고 있습니다.", (150, 200), 30, (255, 255, 255))
                frame = draw_korean_text(frame, "(도킹 상태)", (250, 250), 25, (200, 200, 200))
        
        # 화재 알림 레이어
        if camera_name == "Central_Fire":
            if frame is None: frame = np.zeros((480, 640, 3), dtype=np.uint8)
            if fire_status:
                cv2.rectangle(frame, (0,0), (640, 480), (0,0,255), 15)
                frame = draw_korean_text(frame, "⚠️ 화재 발생 감지!", (180, 50), 40, (255, 255, 0))

        if frame is not None:
            ret, buffer = cv2.imencode('.jpg', frame)
            if ret:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.05)

@app.route('/')
def index(): return render_template('index.html')

@app.route('/video_feed/<cam_id>')
def video_feed(cam_id): return Response(gen_frames(cam_id), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/logs')
def get_logs():
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute("SELECT timestamp, sensor_id, status FROM anomaly_logs ORDER BY id DESC LIMIT 10")
    rows = c.fetchall()
    conn.close()
    return jsonify([{"time": r[0], "id": r[1], "status": r[2]} for r in rows])

def main():
    init_db()
    rclpy.init()
    bridge = CentralBridge()
    
    # YOLO 웹캠 스레드
    def run_yolo():
        global fire_status
        model = YOLO("fire.pt")
        cap = cv2.VideoCapture(0)
        while rclpy.ok():
            ret, frame = cap.read()
            if ret:
                last_frames["Central_Fire"] = frame
                # 자체 탐지 로직 (토픽과 별개로 보완)
                results = model.predict(frame, verbose=False)
                if len(results[0].boxes) > 0: fire_status = True
            time.sleep(0.1)
    
    threading.Thread(target=run_yolo, daemon=True).start()
    
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    threading.Thread(target=executor.spin, daemon=True).start()
    
    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    main()