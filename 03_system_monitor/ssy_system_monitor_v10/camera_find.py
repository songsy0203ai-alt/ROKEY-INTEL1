import cv2
import sys

def test_camera(index):
    cap = cv2.VideoCapture(index)
    
    if not cap.isOpened():
        print(f"Error: 인덱스 {index} 카메라를 열 수 없습니다.")
        return

    print(f"인덱스 {index} 테스트 중... (종료하려면 'q' 입력)")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break
            
        cv2.imshow(f"Camera Index {index}", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # 실행 시 인자를 주지 않으면 기본적으로 0번 테스트
    idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    test_camera(idx)
