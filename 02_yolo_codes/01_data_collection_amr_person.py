"""
[코드 기능]
- AMR(Autonomous Mobile Robot)의 RGB 카메라 토픽을 구독하여 실시간 영상을 확인합니다.
- 사용자가 설정한 시간 간격(capture_interval)에 따라 자동으로 이미지를 캡처하여 저장합니다.
- 수집된 데이터는 신원미상자 객체 탐지 모델의 학습 데이터셋으로 활용 가능합니다.

[입력(Input)]
1. 사용자 키보드 입력: 저장 경로(string), 파일명 접두사(string), 캡처 간격(float, 초 단위)
2. ROS 2 Topic: '/robot3/oakd/rgb/preview/image_raw' (sensor_msgs/Image 타입)
3. 제어 키: 's' (자동 촬영 시작), 'q' (프로그램 종료)

[출력(Output)]
1. 로컬 파일 시스템: 지정된 경로에 '.jpg' 형식의 이미지 파일 연속 저장
2. GUI 화면: 'Live Feed' 윈도우를 통한 실시간 영상 스트리밍 및 촬영 상태 표시
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class ImageCaptureNode(Node):
    """
    ROS 2 노드 클래스: 카메라 데이터를 수신하고 저장 정보를 관리함
    """
    def __init__(self, save_directory, file_prefix):
        """
        [인풋] 
        - save_directory (str): 이미지가 저장될 디렉토리 경로
        - file_prefix (str): 저장될 파일명의 접두사
        [아웃풋] 없음
        """
        super().__init__('image_capture_node')
        
        # AMR의 RGB 카메라 영상 토픽 구독 설정 (QoS depth: 10)
        # 주석: 입력 설명서와 코드 내 토픽명('/robot3/...')을 일치시켰습니다.
        self.subscription = self.create_subscription(
            Image,
            '/robot3/oakd/rgb/preview/image_raw',
            self.listener_callback,
            10)
        
        # ROS Image 메시지를 OpenCV 포맷으로 변환하기 위한 브릿지 객체 생성
        self.bridge = CvBridge()
        self.frame = None
        self.save_directory = save_directory
        self.file_prefix = f"{file_prefix}_"
        self.image_count = 0
        
        # 촬영 활성화 상태를 제어하는 플래그
        self.is_capturing = False 
        
        # 저장 디렉토리가 존재하지 않을 경우 생성 (예외 방지)
        os.makedirs(self.save_directory, exist_ok=True)

    def listener_callback(self, msg):
        """
        [인풋] msg (sensor_msgs/Image): 구독한 카메라 이미지 메시지
        [아웃풋] 없음 (변환된 이미지를 클래스 멤버 변수 self.frame에 갱신)
        """
        try:
            # ROS 이미지 메시지를 BGR 형태의 OpenCV 이미지로 변환
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            # 변환 과정에서 발생할 수 있는 데이터 손상 등 예외 처리
            self.get_logger().error(f'이미지 변환 실패: {e}')

def main():
    """
    [인풋] 없음 (사용자로부터 CLI 입력을 받음)
    [아웃풋] 없음
    """
    # 1. 초기 사용자 환경 설정값 입력
    try:
        save_directory = input("이미지를 저장할 디렉토리 이름을 입력하세요: ")
        file_prefix = input("파일 이름 앞에 붙일 접두사(prefix)를 입력하세요: ")
        capture_interval = float(input("촬영 간격을 초 단위로 입력하세요 (예: 1.0): "))
    except ValueError as e:
        # 숫자 입력이 필요한 곳에 문자를 입력한 경우 등 형식 오류 처리
        print(f"입력 형식이 잘못되었습니다: {e}")
        return

    # 2. ROS 2 통신 및 노드 초기화
    rclpy.init()
    node = ImageCaptureNode(save_directory, file_prefix)
    last_capture_time = time.time()

    print("--- 준비 완료: 's'를 누르면 자동 촬영을 시작합니다. 'q'는 종료입니다. ---")

    try:
        # rclpy.ok()가 True인 동안 루프 실행 (노드가 살아있는 동안)
        while rclpy.ok():
            # ROS 콜백 대기열 처리 (비차단 방식으로 0.01초간 수행)
            rclpy.spin_once(node, timeout_sec=0.01)

            # 수신된 영상 데이터가 있는 경우에만 로직 처리
            if node.frame is not None:
                # 3. 실시간 모니터링용 프레임 생성 (데이터 로깅용 텍스트 추가)
                display_frame = node.frame.copy()
                
                # 촬영 상태에 따른 텍스트 및 색상 설정 (BGR 순서)
                status_text = "Status: CAPTURING..." if node.is_capturing else "Status: READY (Press 's')"
                color = (0, 0, 255) if node.is_capturing else (0, 255, 0)
                
                # 시각화 윈도우에 상태 정보 렌더링
                cv2.putText(display_frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                
                # 사용자가 확인 가능한 윈도우 창 출력
                cv2.imshow("Live Feed", display_frame)
                
                # 4. 키보드 인터럽트 처리 (1ms 대기)
                key = cv2.waitKey(1) & 0xFF

                # 's' 키: 자동 촬영 모드 활성화 (최초 1회 누름)
                if key == ord('s'):
                    if not node.is_capturing:
                        node.is_capturing = True
                        last_capture_time = time.time() # 시작 시점 기록
                        print(">> 자동 촬영 시작!")

                # 5. 시간 간격 기반 자동 이미지 저장 로직
                if node.is_capturing:
                    now = time.time()
                    # 현재 시간과 마지막 저장 시간의 차이가 설정한 간격보다 크면 저장
                    if now - last_capture_time >= capture_interval:
                        file_name = os.path.join(
                            node.save_directory,
                            f"{node.file_prefix}img_{node.image_count}.jpg"
                        )
                        # 중요: 텍스트가 없는 원본 프레임(node.frame)을 저장하여 학습 데이터 품질 유지
                        cv2.imwrite(file_name, node.frame)
                        print(f"Image saved: {file_name}")
                        node.image_count += 1
                        last_capture_time = now # 마지막 저장 시간 갱신

                # 'q' 키: 루프 탈출 및 프로그램 종료
                if key == ord('q'):
                    print(">> 사용자 요청에 의한 프로그램 종료")
                    break

    except KeyboardInterrupt:
        # Ctrl+C 입력 시 안전하게 종료하기 위한 예외 처리
        print("\n>> 인터럽트 감지: 프로그램을 종료합니다.")
    except Exception as e:
        # 실행 중 발생하는 예기치 못한 런타임 에러 출력
        print(f">> 런타임 에러 발생: {e}")
    finally:
        # 6. 리소스 해제 및 정리
        node.destroy_node() # ROS 노드 소멸
        rclpy.shutdown()    # ROS 통신 종료
        cv2.destroyAllWindows() # OpenCV 윈도우 닫기

if __name__ == '__main__':
    main()