"""
[코드 기능]
- YOLO 모델을 사용하여 웹캠 영상에서 사람(person)을 실시간으로 탐지합니다.
- 탐지된 객체의 수를 ROS2 토픽으로 발행하고, 시각화된 화면을 출력합니다.

[입력(Input)]
- 하드웨어: 시스템에 연결된 카메라(시연 때는 AMR1, 2에 연결)의 비디오 스트림
- 모델: 지정된 경로의 YOLO 학습 모델 파일 (.pt)

[출력(Output)]
- ROS2 Topic: '/person_event' (std_msgs/Int32 타입, 탐지된 사람 수)
- GUI: 탐지 결과(Bounding Box, Class Label, Confidence)가 표시된 실시간 영상 창
- Terminal: 객체 탐지 시 탐지 되었다는 메시지 및 탐지 수 출력
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import cv2
import os
import math
from ultralytics import YOLO
from pathlib import Path

class PersonDetPublisher(Node):
    def __init__(self):
        """
        [함수 기능] PersonDetPublisher 노드 초기화 및 리소스 설정
        [Input] 없음
        [Output] PersonDetPublisher 인스턴스 초기화
        """
        super().__init__('person_det_publisher')
        
        # 1. ROS2 퍼블리셔 설정 (토픽명 : /person_event, 큐 사이즈: 10)
        self.publisher_ = self.create_publisher(Int32, '/person_event', 10)
        
        # 2. YOLO 모델 로드
        model_path = "/home/rokey/ssy_ws/trained_pt/trained_pt_v2.pt"
        
        # [Exception] 모델 파일 존재 여부 확인 후 예외 처리
        if not os.path.exists(model_path):
            self.get_logger().error(f"모델 파일을 찾을 수 없습니다: {model_path}")
            raise FileNotFoundError(f"Missing model file: {model_path}")

        self.model = YOLO(model_path)
        self.threshold = 0.8 # 탐지 신뢰도 임계값 설정
        
        # 학습 시 'object'로 잘못 기입된 라벨을 'person'으로 강제 매핑하기 위한 사전
        self.classNames = {0: 'person'}
        
        # 3. 카메라 설정
        self.cap = cv2.VideoCapture(0) # 카메라 포트 번호 개별 입력해야 함
        
        # [Exception] 웹캠 장치 연결 실패 시 예외 처리
        if not self.cap.isOpened():
            self.get_logger().error("웹캠을 열 수 없습니다.")
            raise RuntimeError("Could not open video device")

        # 4. 주기적 실행을 위한 타이머 설정 (0.033초 간격으로 callback 실행, 약 30 FPS)
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        """
        [함수 기능] 주기적으로 카메라 프레임을 읽어 객체 탐지 및 결과 발행
        [Input] 없음 (클래스 내부 self.cap 스트림 사용)
        [Output] ROS2 메시지 발행 및 시각화 윈도우 갱신
        """
        # 프레임 읽기
        ret, img = self.cap.read()
        
        # [Exception] 프레임 읽기 실패 시 콜백 조기 종료
        if not ret:
            return

        # YOLO 모델을 통한 객체 추론 (stream=True로 메모리 효율화, verbose=False로 로그 억제)
        results = self.model(img, stream=True, verbose=False)
        object_count = 0 # 현재 프레임에서 탐지된 객체 수 초기화

        for r in results:
            boxes = r.boxes
            for box in boxes:
                # 탐지 신뢰도 추출
                confidence = float(box.conf[0])
                
                # 설정한 임계값(0.8) 이상인 경우에만 처리
                if confidence >= self.threshold:
                    # Bounding Box 좌표 및 클래스 인덱스 추출
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls = int(box.cls[0])
                    
                    # 딕셔너리를 이용한 라벨 수정 매핑
                    label = self.classNames.get(cls, f"class_{cls}")
                    
                    # OpenCV 시각화: 사각형 및 텍스트(라벨+신뢰도) 그리기
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    conf_display = math.ceil((confidence * 100)) / 100
                    cv2.putText(img, f"{label}: {conf_display}", (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    
                    object_count += 1

        # 탐지된 객체 수를 Int32 메시지에 담아 퍼블리싱
        msg = Int32()
        msg.data = object_count
        self.publisher_.publish(msg)

        # [조건부 로직] 객체 탐지 시에만 터미널에 로그 및 경고 출력
        if object_count > 0:
            self.get_logger().info("-" * 30)
            self.get_logger().info("person 객체가 탐지되었습니다.")
            self.get_logger().info(f"탐지된 person 객체 수: {object_count}")
            self.get_logger().warn("신원 미상자가 탐지 되었습니다.")

        # GUI 화면 상단에 탐지된 총 개수 표시
        cv2.putText(img, f"Detected: {object_count}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # 시인성을 위해 화면 크기를 2배로 확대하여 표시
        display_img = cv2.resize(img, (img.shape[1]*2, img.shape[0]*2))
        cv2.imshow("ROS2 Person Detection Publisher", display_img)
        
        # 'q' 키 입력 시 노드 종료 처리
        if cv2.waitKey(1) == ord('q'):
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    """
    [함수 기능] ROS2 노드 실행 및 예외 상황 관리
    [Input] args: 명령줄 인자
    [Output] 없음
    """
    rclpy.init(args=args)
    try:
        node = PersonDetPublisher()
        # 노드가 명시적으로 종료될 때까지 이벤트 루프 실행
        rclpy.spin(node)
    except Exception as e:
        # [Exception] 실행 중 발생하는 예기치 못한 에러 출력
        print(f"종료 사유: {e}")
    finally:
        # 리소스 해제: 창 닫기 및 노드 종료 확인
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()