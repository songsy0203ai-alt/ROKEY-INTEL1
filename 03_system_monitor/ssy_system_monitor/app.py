"""
[코드 기능]
- Flask 웹 서버를 활용하여 로봇 또는 센서의 상태를 모니터링하는 APRS(Abnormal Process Reporting System) 웹 기반 대시보드를 구현함.
- AMR과 Webcam 상의 실시간 영상 스트리밍(OpenCV) 및 SQLite3 데이터베이스를 이용한 센서 게이지 이상치 로그 저장 및 조회 기능을 제공함.

[입력(Input)]
- 하드웨어: 시스템 웹캠 영상 데이터 (cv2.VideoCapture)
- API 호출: 센서 ID 및 이상치 값 (URL 파라미터를 통한 GET 요청)
- 데이터베이스: 'aprs_system.db' 내 저장된 로그 레코드

[출력(Output)]
- 웹 인터페이스: 'index.html'을 통한 대시보드 화면
- 비디오 스트림: MJPEG 포맷의 실시간 영상 데이터
- JSON 데이터: 최근 10개의 이상 징후 로그 데이터 (/api/logs)
"""

import cv2
import datetime
import sqlite3
import numpy as np # 예외 처리 및 더미 프레임 생성을 위해 명시적 추가
from flask import Flask, render_template, Response, jsonify

app = Flask(__name__)

# --- 데이터베이스 초기화 ---
def init_db():
    """
    기능: 시스템 가동 전 필요한 SQLite 데이터베이스 및 테이블을 생성함.
    입력(Input): 없음
    출력(Output): 없음 (파일 시스템에 'aprs_system.db' 생성)
    """
    try:
        conn = sqlite3.connect('aprs_system.db')
        c = conn.cursor()
        # 게이지 이상치 로그를 저장하기 위한 테이블 생성 (ID, 시간, 센서ID, 측정값)
        c.execute('''CREATE TABLE IF NOT EXISTS anomaly_logs
                     (id INTEGER PRIMARY KEY AUTOINCREMENT, 
                      timestamp TEXT, 
                      sensor_id TEXT, 
                      value REAL)''')
        conn.commit()
        conn.close()
    except sqlite3.Error as e:
        # 예외 처리: 데이터베이스 파일 접근 권한 문제나 생성 실패 시 에러 출력
        print(f"DB 초기화 에러: {e}")

def add_log(sensor_id, value):
    """
    기능: 탐지된 이상 징후 데이터를 DB에 기록함.
    입력(Input): sensor_id (str), value (float)
    출력(Output): 없음
    """
    try:
        conn = sqlite3.connect('aprs_system.db')
        c = conn.cursor()
        # 현재 시간을 "YYYY-MM-DD HH:MM:SS" 형식으로 생성
        now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        # 데이터베이스에 로그 삽입 쿼리 실행
        c.execute("INSERT INTO anomaly_logs (timestamp, sensor_id, value) VALUES (?, ?, ?)",
                  (now, sensor_id, value))
        conn.commit()
        conn.close()
    except sqlite3.OperationalError as e:
        # 예외 처리: DB 파일이 잠겨 있거나 경로가 잘못된 경우 처리
        print(f"데이터 삽입 중 오류 발생: {e}")

# --- 영상 스트리밍 시뮬레이션 ---
def gen_frames(camera_name):
    """
    기능: 카메라로부터 프레임을 읽어와 웹으로 전송 가능한 MJPEG 스트림으로 변환함.
    입력(Input): camera_name (str) - 카메라 식별자
    출력(Output): Generator (byte 단위의 JPEG 이미지 프레임)
    """
    # 시스템의 기본 웹캠(Index 0) 연결 시도
    cap = cv2.VideoCapture(0)
    
    while True:
        success, frame = cap.read()
        
        # 예외 처리: 카메라 하드웨어 연결 실패 시 검은색 더미 영상으로 대체
        if not success:
            # 480x640 해상도의 검은색 배경 생성
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            # 영상 내에 카메라 연결 실패 메시지 출력
            cv2.putText(frame, f"CAM ERROR: {camera_name}", (50, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:
            # 카메라 정상 작동 시 화면 중앙에 LIVE 상태 표시
            cv2.putText(frame, f"LIVE: {camera_name}", (50, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 모든 프레임 상단에 현재 시간(시:분:초) 오버레이 레이아웃 추가
        cv2.putText(frame, datetime.datetime.now().strftime("%H:%M:%S"), (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 이미지를 JPEG 포맷으로 인코딩하여 메모리 효율화
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue # 인코딩 실패 시 다음 프레임으로 건너뜀
            
        frame_bytes = buffer.tobytes()
        # multipart/x-mixed-replace 규격에 맞는 Generator yield 형식으로 반환
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# --- 라우팅 ---
@app.route('/')
def index():
    """메인 대시보드 페이지 렌더링"""
    return render_template('index.html')

@app.route('/video_feed/<cam_id>')
def video_feed(cam_id):
    """
    기능: 카메라 스트리밍 데이터를 Response 객체로 감싸 전송함.
    입력(Input): cam_id (str)
    출력(Output): Flask Response 객체 (mimetype 설정 포함)
    """
    # gen_frames 제너레이터를 호출하여 스트리밍 데이터 유지
    return Response(gen_frames(cam_id), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/logs')
def get_logs():
    """
    기능: 데이터베이스에서 최근 로그 10개를 조회하여 JSON으로 반환함.
    입력(Input): 없음
    출력(Output): JSON 응답 (최근 10개 로그 데이터)
    """
    try:
        conn = sqlite3.connect('aprs_system.db')
        c = conn.cursor()
        # 최신 데이터 순으로 10개만 추출하는 SQL 쿼리
        c.execute("SELECT timestamp, sensor_id, value FROM anomaly_logs ORDER BY id DESC LIMIT 10")
        logs = [{"time": row[0], "id": row[1], "val": row[2]} for row in c.fetchall()]
        conn.close()
        return jsonify(logs)
    except Exception as e:
        # 예외 처리: 데이터베이스 조회 실패 시 빈 리스트 및 에러 메시지 반환
        return jsonify({"error": str(e)}), 500

@app.route('/simulate_anomaly/<sensor_id>/<float:val>')
def simulate(sensor_id, val):
    """
    기능: 강제로 이상 데이터를 입력하여 시스템 작동을 테스트함.
    입력(Input): sensor_id (str), val (float)
    출력(Output): str (처리 결과 메시지)
    """
    add_log(sensor_id, val)
    return f"Logged: {sensor_id} - {val}"

if __name__ == '__main__':
    # 시스템 구동 시 DB 초기화 함수 호출
    init_db()
    
    # 서버 실행 안내 문구 출력
    print("\n--- APRS 시스템을 시작합니다 ---")
    # Flask 앱 실행 (0.0.0.0으로 설정하여 외부 접속 허용)
    app.run(host='0.0.0.0', port=5000, debug=True)