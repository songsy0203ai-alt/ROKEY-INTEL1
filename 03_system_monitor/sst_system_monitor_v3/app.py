import cv2
import datetime
import sqlite3
import numpy as np
import threading
from flask import Flask, render_template, Response, jsonify

# ROS 2 및 이미지 변환 라이브러리
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

app = Flask(__name__)

# 전역 프레임 버퍼: 수신된 최신 영상을 저장하는 공간
last_frames = {
    'Central_Fire': None,
    'AMR1': None,
    'AMR2': None
}

# --- 데이터베이스 초기화 ---
def init_db():
    try:
        conn = sqlite3.connect('aprs_system.db')
        c = conn.cursor()
        c.execute('''CREATE TABLE IF NOT EXISTS anomaly_logs
                     (id INTEGER PRIMARY KEY AUTOINCREMENT, 
                      timestamp TEXT, 
                      sensor_id TEXT, 
                      value REAL)''')
        conn.commit()
        conn.close()
    except sqlite3.Error as e:
        print(f"DB 초기화 에러: {e}")

# --- ROS 2 이미지 수신 노드 ---
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('aprs_central_subscriber')
        self.bridge = CvBridge()
        
        # ★ 수정된 토픽 경로 반영 (AMR1: robot2, AMR2: robot3)
        self.sub_amr1 = self.create_subscription(
            Image,
            '/robot2/oakd/rgb/preview/image_raw',
            self.amr1_callback,
            10)
            
        self.sub_amr2 = self.create_subscription(
            Image,
            '/robot3/oakd/rgb/preview/image_raw',
            self.amr2_callback,
            10)

    def amr1_callback(self, msg):
        # ROS Image 메시지를 OpenCV 포맷으로 변환하여 버퍼 저장
        last_frames['AMR1'] = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def amr2_callback(self, msg):
        last_frames['AMR2'] = self.bridge.imgmsg_to_cv2(msg, "bgr8")

# --- 영상 스트리밍 엔진 (Generator) ---
def gen_frames(camera_name):
    # PC3 직결 USB 웹캠 초기화 (Central_Fire 전용)
    cap = None
    if camera_name == 'Central_Fire':
        cap = cv2.VideoCapture(1) # USB 웹캠 장치 인덱스 2번

    while True:
        frame = None
        
        if camera_name == 'Central_Fire' and cap:
            success, frame = cap.read()
        else:
            # AMR1, AMR2는 ROS Subscriber 쓰레드가 업데이트하는 전역 변수에서 읽음
            frame = last_frames.get(camera_name)

        # 영상 데이터가 없는 경우 검은색 화면 및 상태 메시지 출력
        if frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, f"WAITING TOPIC: {camera_name}", (50, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            # 정상 수신 시 오버레이 (시간 및 소스 표시)
            cv2.putText(frame, f"LIVE: {camera_name}", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, datetime.datetime.now().strftime("%H:%M:%S"), (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # JPEG 인코딩 및 스트림 전송
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
            
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    
    if cap:
        cap.release()

# --- 라우팅 설정 ---
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed/<cam_id>')
def video_feed(cam_id):
    return Response(gen_frames(cam_id), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/logs')
def get_logs():
    try:
        conn = sqlite3.connect('aprs_system.db')
        c = conn.cursor()
        c.execute("SELECT timestamp, sensor_id, value FROM anomaly_logs ORDER BY id DESC LIMIT 10")
        logs = [{"time": row[0], "id": row[1], "val": row[2]} for row in c.fetchall()]
        conn.close()
        return jsonify(logs)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

# --- 메인 실행부 (멀티쓰레딩 적용) ---
if __name__ == '__main__':
    init_db()
    
    # ROS 2 초기화
    rclpy.init()
    image_subscriber = ImageSubscriber()
    
    # ROS 2 스핀(수신 대기)을 별도 쓰레드에서 실행하여 Flask와 병렬 구동
    ros_thread = threading.Thread(target=lambda: rclpy.spin(image_subscriber), daemon=True)
    ros_thread.start()
    
    print("\n[INFO] APRS Central Server Started (PC3)")
    print("[INFO] Subscribing to: /robot2/oakd/rgb/preview/image_raw")
    print("[INFO] Subscribing to: /robot3/oakd/rgb/preview/image_raw")
    
    try:
        # Flask 서버 실행
        app.run(host='0.0.0.0', port=5000, debug=False)
    finally:
        rclpy.shutdown()