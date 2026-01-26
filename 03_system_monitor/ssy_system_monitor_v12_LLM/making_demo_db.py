import sqlite3
import datetime
import os

# [수정] 현재 파일이 있는 디렉토리를 자동으로 찾아서 DB 경로를 설정합니다.
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.path.join(BASE_DIR, 'aprs_system.db')

def create_demo_db():
    # 기존 DB가 있다면 삭제하고 초기화
    if os.path.exists(DB_PATH):
        try:
            os.remove(DB_PATH)
            print(f"기존 DB 삭제 완료: {DB_PATH}")
        except PermissionError:
            print("오류: DB 파일이 다른 프로그램(app.py 등)에서 사용 중입니다. 종료 후 다시 시도하세요.")
            return

    try:
        conn = sqlite3.connect(DB_PATH)
        c = conn.cursor()

        # 1. 테이블 생성
        c.execute("""
            CREATE TABLE IF NOT EXISTS anomaly_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT,
                value REAL,
                outlier TEXT
            )
        """)

        # 2. 테스트용 데이터 생성 (오늘 날짜 기준)
        now = datetime.datetime.now()
        
        # 실제 데이터가 들어가는 것처럼 보이게 시계열 데이터 구성
        test_data = [
            (now.replace(hour=9, minute=0).isoformat(timespec='seconds'), 4.5, "정상치"),
            (now.replace(hour=10, minute=30).isoformat(timespec='seconds'), 5.2, "정상치"),
            (now.replace(hour=13, minute=15).isoformat(timespec='seconds'), 9.2, "이상치"), # 고압
            (now.replace(hour=15, minute=40).isoformat(timespec='seconds'), 4.8, "정상치"),
            (now.replace(hour=17, minute=20).isoformat(timespec='seconds'), 0.5, "이상치"), # 저압
            (now.replace(hour=18, minute=0).isoformat(timespec='seconds'), 3.9, "정상치"),
        ]

        # 3. 데이터 삽입
        c.executemany("INSERT INTO anomaly_logs (timestamp, value, outlier) VALUES (?, ?, ?)", test_data)

        conn.commit()
        conn.close()
        print("-" * 50)
        print(f"✅ 데모 DB 생성 성공!")
        print(f"📍 경로: {DB_PATH}")
        print(f"📊 데이터: {len(test_data)}개의 로그 기록 완료")
        print("-" * 50)

    except Exception as e:
        print(f"❌ DB 생성 중 예외 발생: {e}")

if __name__ == "__main__":
    create_demo_db()