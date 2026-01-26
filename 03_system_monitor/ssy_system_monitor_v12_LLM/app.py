#!/usr/bin/env python3
"""
[코드 기능]
- PC3(중앙 관제 서버)에서 USB 웹캠을 이용해 화재를 실시간 탐지(YOLOv8)하고,
  네트워크를 통해 연결된 두 대의 AMR(Robot2, Robot3)의 상태 및 영상을 모니터링함.
- 화재 발생 시 ROS2 토픽을 통해 시스템 전체에 알람 신호를 전파하며, 모든 이상 징후를 DB에 기록함.
- ✅ 추가: 화재 발생 시 PC3에서 직접(브라우저 무관) 알람 사운드를 재생함.
- ✅ 추가: 네트워크 부하 감소를 위해 AMR 영상을 CompressedImage 형식으로 수신하고, OpenCV(imdecode)를 통해 실시간 디코딩하여 대시보드에 스트리밍함.

[입력(Input)]
- Central_Fire: PC3에 직접 연결된 USB 웹캠 영상 프레임
- AMR1/AMR2 Image: 각 로봇에서 송신하는 '/robotX/oakd/rgb/image_raw/compressed' (sensor_msgs/CompressedImage)
- Gauge Data:
  - (기존) '/robotX/gauge_safe_status' (std_msgs/Bool)
  - (추가) Flask '/log' (POST JSON)

[출력(Output)]
- Web Dashboard: Flask 기반 실시간 MJPEG 스트리밍 및 상태 조회 API
- Fire Signal: '/fire_detected_signal' (std_msgs/Bool) 토픽 발행
- Database: anomaly_logs 테이블에 이상 징후 로그 저장
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
from typing import Optional, Dict, Any, Tuple

# ✅ PC3 사운드 재생용
import subprocess
import shutil

# =========================
# [2] Flask (Web Dashboard 서버)
# =========================
# ✅ FIX: 중복 import 제거
from flask import Flask, render_template, Response, jsonify, request, send_from_directory

# =========================
# [3] ROS2 (PC3에서 실행되는 중앙 브리지 노드)
# =========================
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge

# =========================
# [4] YOLO (중앙웹캠 기반 화재 탐지)
# =========================
from ultralytics import YOLO

# =========================
# [5] Gemini API 사용
# =========================
import google.generativeai as genai


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

# ✅ FIX: 실시간 로그 캐시 + 락(Flask/ROS 스레드 동시 접근 보호)
gauge_logs = []
GAUGE_LOG_MAXLEN = 200
gauge_lock = threading.Lock()

# ✅ DB 경로 (기존 경로 유지. 실제 파일명 다르면 여기만 맞추면 됨)
DB_PATH = '/home/rokey/ssy_ws/03_system_monitor/ssy_system_monitor_v12_LLM/amrs_system.db'


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
        self._thread: Optional[threading.Thread] = None
        self._proc: Optional[subprocess.Popen] = None

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
