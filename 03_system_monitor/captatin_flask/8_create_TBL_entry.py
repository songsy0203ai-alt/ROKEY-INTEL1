'''
[코드 기능]
SQLite 데이터베이스(mydatabase.db)를 생성하고, 
두 개의 테이블(detection_table, violation_detected)을 정의한 뒤 초기화합니다. 
이후 detection_table에 트럭 및 더미 데이터를 삽입하고 저장된 내용을 콘솔에 출력하는 기능을 수행합니다.

[입력(Input)]
1. 내부 정의 데이터: detection_entries 리스트에 하드코딩된 데이터 (0, 'Truck'), (1, 'Dummy').
2. 데이터베이스 파일: 로컬 경로의 mydatabase.db (파일이 없을 경우 자동 생성).

[출력(Output)]
1. 로컬 파일 시스템: 데이터베이스 테이블 및 데이터가 저장된 mydatabase.db 파일 생성 및 갱신.
2. 콘솔(터미널) 출력:
    - 테이블 생성 및 초기화 성공 메시지 ("Tables created and emptied successfully.")
    - detection_table에 저장된 전체 행(Row) 데이터 출력.
'''

import sqlite3

# Connect to SQLite database (or create it if it doesn't exist)
connection = sqlite3.connect('mydatabase.db')

# Create a cursor object to interact with the database
cursor = connection.cursor()

def create_tables():
    # SQL command to create the table
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
    cursor.execute(create_detection_table)
    cursor.execute(create_violation_table)

    # SQL command to delete all entries in the detection_table
    delete_all_entries_query = "DELETE FROM detection_table;"

    # Execute the command
    cursor.execute(delete_all_entries_query)

        # SQL command to delete all entries in the detection_table
    delete_all_entries_query = "DELETE FROM violation_detected;"

    # Execute the command
    cursor.execute(delete_all_entries_query)

    print("Tables created and emptied successfully.")

def create_Detection_entries():
    # Data to insert (a list of tuples, where each tuple represents a row)
    detection_entries = [ # 더미 데이터 2개 만들기
        (0,'Truck'),
        (1,'Dummy'),
    ]

    # SQL command to insert data
    # 만든 더미 데이터를 db에 넣기
    insert_query = """
    INSERT INTO detection_table (id, name) VALUES (?, ?);
    """
    cursor.executemany(insert_query,detection_entries)

    # SQL command to select all data from the table
    select_query = "SELECT * FROM detection_table;"

    # Execute the command and fetch all results
    cursor.execute(select_query)
    rows = cursor.fetchall()

    # Print each row
    for row in rows:
        print(row)

def main():
    create_tables()
    create_Detection_entries()
    
if __name__ == "__main__":

    main()

    # Commit the changes and close the connection
    connection.commit()
    connection.close()


