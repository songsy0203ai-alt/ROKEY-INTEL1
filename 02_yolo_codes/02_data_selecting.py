"""
[코드 기능]
- 원본 디렉토리(src_dir)에서 이미지 파일들을 탐색하여 사용자가 지정한 수량만큼 무작위로 추출합니다.
- 추출된 파일들은 대상 디렉토리(dest_dir)로 복사되어, 라벨링(Annotation) 작업을 위한 데이터셋 준비 과정을 자동화합니다.

[입력(Input)]
1. 소스 경로(string): 이미지 파일들이 저장된 원본 폴더 경로
2. 대상 경로(string): 선택된 이미지를 복사할 새로운 폴더 경로
3. 추출 개수(int): 무작위로 샘플링할 이미지의 수 (기본값: 100)

[출력(Output)]
1. 로컬 파일 시스템: 대상 경로에 선택된 이미지 파일들의 물리적 복사본 생성
2. 터미널 로그: 검색된 파일 현황, 복사 진행 상황 및 성공 여부 메시지 출력
"""

import os
import random
import shutil

def select_random_images(src_dir, dest_dir, sample_count=100):
    """
    [인풋] 
    - src_dir (str): 원본 이미지 디렉토리 경로
    - dest_dir (str): 복사본이 저장될 목적지 디렉토리 경로
    - sample_count (int): 추출할 이미지 파일의 개수
    
    [아웃풋] 
    - 없음 (함수 실행 결과로 파일 시스템에 파일이 복사됨)
    """
    
    # 1. 예외 처리: 원본 디렉토리 존재 여부 확인
    if not os.path.exists(src_dir):
        print(f"[오류] 원본 디렉토리를 찾을 수 없습니다: {src_dir}")
        return

    # 2. 대상 디렉토리가 존재하지 않을 경우 안전하게 자동 생성
    if not os.path.exists(dest_dir):
        try:
            os.makedirs(dest_dir)
            print(f">> 디렉토리 신규 생성됨: {dest_dir}")
        except Exception as e:
            print(f"[오류] 디렉토리 생성 실패: {e}")
            return

    # 3. 처리 가능한 이미지 파일 확장자 리스트 정의 (대소문자 무관하게 처리)
    valid_extensions = ('.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.webp')
    
    # 4. 리스트 컴프리헨션을 통한 이미지 파일 필터링 추출
    # os.listdir로 전체 파일을 읽은 후 확장자를 체크하여 이미지 파일만 선별
    try:
        all_files = [f for f in os.listdir(src_dir) 
                     if f.lower().endswith(valid_extensions)]
    except PermissionError:
        print(f"[오류] 원본 경로에 대한 접근 권한이 없습니다.")
        return
    
    # 5. 검색된 유효 이미지 파일의 총 개수 파악
    file_count = len(all_files)
    print(f">> 원본 경로 내 유효 이미지 수: {file_count}")

    # 6. 예외 처리: 요청한 샘플 수가 원본 데이터보다 많을 경우
    if file_count < sample_count:
        print(f"[주의] 원본 파일 수({file_count})가 요청 수량({sample_count})보다 적습니다.")
        print(">> 가용 가능한 모든 파일을 복사합니다.")
        sample_count = file_count # 가용한 최대치로 조정

    # 7. random.sample을 이용한 중복 없는 무작위 추출 실행
    # 인덱스가 아닌 파일명 리스트에서 직접 샘플링 수행
    selected_files = random.sample(all_files, sample_count)

    # 8. 파일 복사 프로세스 진행
    print(f">> 복사 시작: {sample_count}장의 이미지...")
    for file_name in selected_files:
        # 파일명과 디렉토리 경로를 결합하여 절대 경로 생성
        src_path = os.path.join(src_dir, file_name)
        dest_path = os.path.join(dest_dir, file_name)
        
        try:
            # shutil.copy2: 파일 내용뿐만 아니라 메타데이터(수정 시간 등)까지 보존하며 복사
            shutil.copy2(src_path, dest_path)
        except Exception as e:
            print(f"[오류] 파일 복사 중 문제 발생 ({file_name}): {e}")

    print(f"--- 작업 완료: {len(selected_files)}장의 이미지가 {dest_dir}로 안전하게 복사되었습니다. ---")

# 스크립트 직접 실행 시 진입점
if __name__ == "__main__":
    """
    [인풋] 소스 및 대상 경로 설정값
    [아웃풋] 프로그램 실행 결과 반환
    """
    # AMR 데이터 수집 경로 설정 (사용자 환경에 맞게 수정됨)
    SOURCE_PATH = "/home/rokey/ssy_ws/amr_person/"
    DEST_PATH = "/home/rokey/ssy_ws/amr_person_selected/"
    
    # 추출 실행: 100장을 무작위로 선별
    select_random_images(SOURCE_PATH, DEST_PATH, 100)