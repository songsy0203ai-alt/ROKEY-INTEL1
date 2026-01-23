"""
[코드 기능]
- 원본 이미지와 라벨 데이터를 지정된 비율(7:2:1)로 무작위 분할하여 학습용 데이터셋 구조를 자동 구축합니다.
- YOLO 등 객체 탐지 모델 학습에 표준적으로 사용되는 `train/valid/test` 폴더 구조를 생성합니다.

[입력(Input)]
1. 소스 이미지 경로: `/home/rokey/ssy_ws/amr_person_selected_labeled/images/`
2. 소스 라벨 경로: `/home/rokey/ssy_ws/amr_person_selected_labeled/labels/`
3. 작업 최상위 경로: `/home/rokey/ssy_ws/`
4. 분할 비율: Train(0.7), Valid(0.2), Test(0.1)

[출력(Output)]
1. 로컬 파일 시스템: `my_data` 폴더 하위에 이미지와 라벨이 분리된 6개의 서브 디렉토리 생성 및 파일 복사
2. 터미널 로그: 각 단계별 디렉토리 생성, 이미지 분할 개수, 라벨 매칭 결과 출력
"""

import os
import shutil
import time
import random

def setup_directories(base_path):
    """
    [인풋] base_path (str): 데이터셋이 생성될 최상위 작업 디렉토리 경로
    [아웃풋] target_root (str): 최종 생성된 'my_data' 폴더의 절대 경로
    """
    target_root = os.path.join(base_path, "my_data")
    
    # 1. 예외 처리 및 초기화: 기존 데이터와의 혼선 방지를 위해 기존 폴더 삭제 후 재생성
    try:
        if os.path.exists(target_root):
            shutil.rmtree(target_root)
            print(f">> 기존 디렉토리 초기화 완료: {target_root}")
    except PermissionError:
        print(f"[오류] {target_root} 삭제 권한이 없습니다.")
        return None
    
    # 2. 모델 학습용 표준 하위 디렉토리(Standard Structure) 리스트 정의
    sub_dirs = [
        "train/images", "train/labels",
        "valid/images", "valid/labels",
        "test/images", "test/labels"
    ]
    
    # 3. 모든 하위 폴더 생성
    for sub_dir in sub_dirs:
        os.makedirs(os.path.join(target_root, sub_dir), exist_ok=True)
    
    print(f">> 데이터셋 구조 생성 완료: {target_root}")
    return target_root

def split_images(source_img_dir, target_root, train_ratio=0.7, valid_ratio=0.2):
    """
    [인풋] 
    - source_img_dir (str): 원본 이미지 경로
    - target_root (str): 목적지 루트 경로
    - train_ratio/valid_ratio (float): 데이터 분할 비율
    [아웃풋] splits (dict): 분할된 파일명들이 담긴 딕셔너리
    """
    # 1. 예외 처리: 원본 이미지 디렉토리 존재 확인
    if not os.path.exists(source_img_dir):
        print(f"[오류] 원본 이미지 경로를 찾을 수 없습니다: {source_img_dir}")
        return None

    # 2. 이미지 확장자 필터링 (.jpg, .jpeg, .png)
    all_files = [f for f in os.listdir(source_img_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
    
    # 3. 무작위성 확보: 현재 시스템 시간을 시드(Seed)로 셔플
    random.seed(time.time())
    random.shuffle(all_files)

    # 4. 데이터 분할 지점(Index) 계산
    total = len(all_files)
    if total == 0:
        print("[주의] 분할할 이미지 파일이 없습니다.")
        return None

    train_end = int(total * train_ratio)
    valid_end = train_end + int(total * valid_ratio)

    # 5. 리스트 슬라이싱을 통한 그룹화
    splits = {
        "train": all_files[:train_end],
        "valid": all_files[train_end:valid_end],
        "test": all_files[valid_end:]
    }

    # 6. 물리적 파일 복사 수행
    for split_name, files in splits.items():
        dest_dir = os.path.join(target_root, split_name, "images")
        for f in files:
            shutil.copy(os.path.join(source_img_dir, f), os.path.join(dest_dir, f))
    
    print(f">> 이미지 분할: Train {len(splits['train'])} / Valid {len(splits['valid'])} / Test {len(splits['test'])}")
    return splits

def match_labels(source_label_dir, target_root, split_dict):
    """
    [인풋] 
    - source_label_dir (str): 원본 라벨(.txt) 경로
    - target_root (str): 목적지 루트 경로
    - split_dict (dict): 이미지 분할 정보 딕셔너리
    [아웃풋] 없음 (라벨 파일 복사 수행)
    """
    # 1. 예외 처리: 입력받은 이미지 분할 정보가 비어있는지 확인
    if split_dict is None:
        print("[오류] 이미지 분할 정보가 없어 라벨 매칭을 중단합니다.")
        return

    # 2. 이미지 파일명을 기반으로 짝이 맞는 라벨 파일 복사
    for split_name, files in split_dict.items():
        dest_label_dir = os.path.join(target_root, split_name, "labels")
        count = 0
        
        for img_file in files:
            # 확장자를 제거하고 .txt를 붙여 라벨 파일명 생성
            base_name = os.path.splitext(img_file)[0]
            txt_file = base_name + ".txt"
            src_txt_path = os.path.join(source_label_dir, txt_file)
            
            # 3. 예외 처리: 이미지에 해당하는 라벨이 없는 경우 건너뜀 (데이터 불일치 방지)
            if os.path.exists(src_txt_path):
                shutil.copy(src_txt_path, os.path.join(dest_label_dir, txt_file))
                count += 1
            else:
                print(f"[경고] 라벨 없음: {img_file}")
        
        print(f">> {split_name} 라벨 매칭: {count}개 복사 완료.")

def main():
    """
    [인풋] 없음
    [아웃풋] 전체 프로세스 실행
    """
    # 환경 변수 및 경로 설정
    BASE_WORKSPACE = "/home/rokey/ssy_ws/"
    SOURCE_IMAGES = "/home/rokey/ssy_ws/amr_person_selected_labeled/images/"
    SOURCE_LABELS = "/home/rokey/ssy_ws/amr_person_selected_labeled/labels/"

    # 단계 1: 폴더 구조 초기화
    target_root = setup_directories(BASE_WORKSPACE)
    if target_root is None: return
    
    # 단계 2: 이미지 무작위 분할 (7:2:1)
    split_info = split_images(SOURCE_IMAGES, target_root)
    if split_info is None: return
    
    # 단계 3: 이미지 리스트에 맞춘 라벨 파일 자동 매칭 및 복사
    match_labels(SOURCE_LABELS, target_root, split_info)

    print(f"\n--- 데이터셋 구축 완료 ---")
    print(f"위치: {target_root}")

if __name__ == "__main__":
    main()