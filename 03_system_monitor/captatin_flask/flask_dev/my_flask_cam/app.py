import os
import time
import cv2
from flask import Flask, render_template, Response, redirect, url_for, send_from_directory
from models import db, ImageRecord

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CAPTURE_DIR = os.path.join(BASE_DIR, "static", "captures")
os.makedirs(CAPTURE_DIR, exist_ok=True)

app = Flask(__name__)
app.config["SQLALCHEMY_DATABASE_URI"] = "sqlite:///app.db"
app.config["SQLALCHEMY_TRACK_MODIFICATIONS"] = False
db.init_app(app)

# ---- 웹캠 설정 ----
# 보통 USB 웹캠은 0, 안 잡히면 1,2로 바꿔보기
CAM_INDEX = 0
cap = cv2.VideoCapture(CAM_INDEX)

def gen_frames():
    # MJPEG 스트리밍
    while True:
        if not cap.isOpened():
            time.sleep(0.2)
            continue

        success, frame = cap.read()
        if not success:
            time.sleep(0.05)
            continue

        # 필요하면 해상도/회전/텍스트 등 처리 가능
        ret, buffer = cv2.imencode(".jpg", frame)
        if not ret:
            continue

        frame_bytes = buffer.tobytes()
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n")

@app.before_first_request
def init_db():
    db.create_all()

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/capture")
def capture():
    if not cap.isOpened():
        return "Camera not opened", 500

    success, frame = cap.read()
    if not success:
        return "Failed to read frame", 500

    ts = time.strftime("%Y%m%d_%H%M%S")
    filename = f"cap_{ts}.jpg"
    filepath = os.path.join(CAPTURE_DIR, filename)

    cv2.imwrite(filepath, frame)

    rec = ImageRecord(filename=filename, source="usb_cam")
    db.session.add(rec)
    db.session.commit()

    return redirect(url_for("gallery"))

@app.route("/gallery")
def gallery():
    images = ImageRecord.query.order_by(ImageRecord.created_at.desc()).all()
    return render_template("gallery.html", images=images)

@app.route("/static/captures/<path:filename>")
def serve_capture(filename):
    return send_from_directory(CAPTURE_DIR, filename)

if __name__ == "__main__":
    # 외부 접속 필요하면 host="0.0.0.0"
    app.run(debug=True, host="127.0.0.1", port=5000)
