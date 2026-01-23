"""
[코드 기능]
SQLite3를 사용하여 로컬 데이터베이스 파일('mydatabase.db')을 생성하고, 
객체 탐지 및 위반 기록을 저장하기 위한 두 개의 테이블 구조를 정의합니다.
이미 테이블이 존재할 경우 새로 생성하지 않으며, 데이터베이스 연결 후 설정을 확정(Commit)합니다.

[입력(Input)]
1. 데이터베이스 파일: 로컬 경로의 'mydatabase.db' (파일이 없을 경우 자동 생성)
2. 테이블 정의(Schema): 
   - detection_table: id(PK), name(TEXT)
   - violation_detected: id(PK), name(TEXT), time(TIMESTAMP)

[출력(Output)]
1. 로컬 파일 시스템: 테이블 구조가 생성/업데이트된 'mydatabase.db' 파일
2. 콘솔(터미널) 출력: "Database and table created successfully." 메시지 출력
"""

import sqlite3

# Connect to SQLite database (or create it if it doesn't exist)
connection = sqlite3.connect('mydatabase.db') # mydatabase.db 라는 db와 연결(없으면 생성)

# Create a cursor object to interact with the database
cursor = connection.cursor() # 커서 = 데이터 베이스를 돌아다니면서 어느 셀에 접근할지 결정하는 포인트

# SQL command to create the table
# 아래 여러줄 주석은 sqlite 명령어
# detection_table : 테이블 이름
# id : 1행 이름 - 정수이고 Primary 키다
# name : 2행 이름 - 텍스트이고 공백이다
create_detection_table = """
CREATE TABLE IF NOT EXISTS detection_table (
    id INTEGER PRIMARY KEY,
    name TEXT NOT NULL
);
"""
# SQL command to create the table
create_violation_table = """
CREATE TABLE IF NOT EXISTS violation_detected (
    id INTEGER PRIMARY KEY,
    name TEXT NOT NULL,
    time TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
"""

# Execute the command
# cursor로 괄호 안 인자를 바로 실행
cursor.execute(create_detection_table)
cursor.execute(create_violation_table)

# Commit the changes and close the connection
connection.commit()
connection.close()

print("Database and table created successfully.")
