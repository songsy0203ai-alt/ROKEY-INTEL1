from flask_sqlalchemy import SQLAlchemy
from datetime import datetime

db = SQLAlchemy()

class ImageRecord(db.Model):
    __tablename__ = "images"

    id = db.Column(db.Integer, primary_key=True)
    filename = db.Column(db.String(255), nullable=False, unique=True)
    source = db.Column(db.String(50), default="usb_cam")  # usb_cam / turtlebot4 등
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
