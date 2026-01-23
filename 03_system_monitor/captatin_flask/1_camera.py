from flask import Flask, render_template, Response
import cv2

app = Flask(__name__)

# 카메라를 전역 객체로 선언하여 중복 오픈 방지
camera = cv2.VideoCapture(0)

def generate_frames():
    while True:
        # 카메라 프레임 읽기
        success, frame = camera.read()
        if not success:
            break
        else:
            # JPEG 인코딩
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            # MJPEG 스트리밍 규격에 맞춘 yield
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    return render_template('cam_index_flex.html')

@app.route('/video_feed')
def video_feed():
    # Response 객체를 통해 스트리밍 데이터 전송
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    # threaded=True를 명시하여 멀티 세션 허용
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False, threaded=True)