"""
[코드 기능]
- 학습된 YOLOv8/v11 모델 가중치(.pt)를 로드하여 실시간 웹캠 영상 내 객체를 추론(Inference)합니다.
- 설정된 신뢰도 임계값(0.8) 이상인 객체에 대해서만 선별적으로 바운딩 박스와 클래스 정보를 시각화합니다.
- 탐지된 객체의 총 개수를 화면 좌상단에 실시간으로 표시합니다.

[입력(Input)]
1. 모델 가중치: `/home/rokey/ssy_ws/trained_pt/trained_pt_v2.pt` (PyTorch 가중치 파일)
2. 비디오 소스: 로컬 웹캠(인덱스 0) 영상 데이터 스트림
3. 임계값(Threshold): 0.8 (80% 이상의 확신을 가진 탐지만 허용)

[출력(Output)]
1. GUI 윈도우: 실시간 탐지 결과가 렌더링된 'Real-time Detection' 창 (2배 확대 출력)
2. 터미널 로그: 모델 로드 상태 및 프로그램 종료 메시지
"""

import math
import os
import sys
from ultralytics import YOLO
from pathlib import Path
import cv2

class YOLOWebcamViewer:
    """
    YOLO 추론 및 실시간 시각화를 관리하는 클래스
    """
    def __init__(self, model, threshold=0.8):
        """
        [인풋] 
        - model (YOLO): 로드된 YOLO 모델 인스턴스
        - threshold (float): 객체 탐지 인정을 위한 최소 신뢰도 (0.0 ~ 1.0)
        [아웃풋] 없음
        """
        self.model = model
        self.threshold = threshold  
        # 클래스 인덱스와 라벨 매핑 (프로젝트 목적에 맞게 수정 가능)
        self.classNames = {0: 'person'}  
        self.should_shutdown = False

    def run(self):
        """
        [인풋] 없음 (내부 멤버 변수 및 카메라 스트림 활용)
        [아웃풋] 없음 (실시간 창 출력 및 키 인터럽트 처리)
        """
        # 1. 비디오 캡처 객체 초기화
        cap = cv2.VideoCapture(0)
        
        # 2. 예외 처리: 카메라 하드웨어 접근 불가 시 종료
        if not cap.isOpened():
            print("[오류] 웹캠을 열 수 없습니다. 장치 연결 상태를 확인하세요.")
            return

        while not self.should_shutdown:
            # 카메라로부터 한 프레임을 읽음
            ret, img = cap.read()
            if not ret:
                print("[주의] 영상 프레임을 수신할 수 없습니다.")
                continue

            # 3. 모델 추론: stream=True 옵션으로 메모리 효율 최적화
            results = self.model(img, stream=True)

            object_count = 0
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    # 4. 신뢰도(Confidence) 기반 필터링
                    confidence = float(box.conf[0])
                    
                    # 설정된 임계값(0.8)보다 낮은 탐지 결과는 무시
                    if confidence >= self.threshold:
                        # 좌표 추출 (xmin, ymin, xmax, ymax)
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        
                        # 5. 시각화 요소 렌더링 (박스 및 텍스트)
                        # BGR: (0, 0, 255) -> 빨간색 박스
                        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

                        conf_display = math.ceil((confidence * 100)) / 100
                        cls = int(box.cls[0])
                        label = self.classNames.get(cls, f"class_{cls}")
                        
                        # 라벨 텍스트: (255, 0, 0) -> 파란색
                        cv2.putText(img, f"{label}: {conf_display}", (x1, y1 - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                        
                        object_count += 1

            # 6. 통계 정보 표시: 현재 프레임에서 탐지된 인원수
            cv2.putText(img, f"Detected: {object_count}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # 7. 화면 출력 최적화: 시인성을 위해 2배 확대
            try:
                display_img = cv2.resize(img, (img.shape[1]*2, img.shape[0]*2))
                cv2.imshow("Real-time Detection (Conf >= 0.8)", display_img)
            except Exception as e:
                print(f"[오류] 영상 리사이즈 실패: {e}")

            # 8. 키보드 입력 감지: 'q' 키 입력 시 루프 탈출
            if cv2.waitKey(1) == ord('q'):
                self.should_shutdown = True

        # 리소스 해제
        cap.release()
        cv2.destroyAllWindows()

def main():
    """
    [인풋] 없음
    [아웃풋] 프로그램 진입 및 모델 로드
    """
    # 추론에 사용할 가중치 파일 경로
    model_path = "/home/rokey/ssy_ws/trained_pt/trained_pt_v2.pt"

    # 1. 예외 처리: 모델 파일 존재 여부 확인
    if not os.path.exists(model_path):
        print(f"[오류] 가중치 파일을 찾을 수 없습니다: {model_path}")
        exit(1)

    # 2. 파일 확장자에 따른 YOLO 모델 로드 프로세스
    suffix = Path(model_path).suffix.lower()
    try:
        if suffix == '.pt':
            model = YOLO(model_path)
        elif suffix in ['.onnx', '.engine']:
            # ONNX나 TensorRT 엔진 형식인 경우 task를 명시적으로 지정
            model = YOLO(model_path, task='detect')
        else:
            print(f"[오류] 지원하지 않는 모델 형식입니다: {suffix}")
            exit(1)
    except Exception as e:
        print(f"[오류] 모델 로드 중 예외 발생: {e}")
        exit(1)

    # 3. 객체 탐지 뷰어 실행
    print(">> 실시간 탐지 시작 (종료하시려면 'q'를 누르세요)")
    viewer = YOLOWebcamViewer(model, threshold=0.8)
    viewer.run()
    
    print(">> 프로그램이 정상 종료되었습니다.")
    sys.exit(0)

if __name__ == '__main__':
    main()