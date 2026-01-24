#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ImageRelayNode(Node):
    def __init__(self):
        super().__init__('pc2_image_relay_node')

        # 1. QoS 설정: 영상 데이터이므로 성능을 위해 최신 데이터 위주로 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # PC3의 구독 설정에 맞춤
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 2. 구독자(Subscriber) 설정: 로봇(AMR2)이 실제로 내뱉는 원본 토픽명 확인 필요
        # 만약 로봇 자체에서 이미 /robot3/... 로 내보내고 있다면 이 노드는 필요 없습니다.
        # 보통 로봇 로컬에서는 /oakd/rgb/preview/image_raw 등으로 나옵니다.
        self.subscription = self.create_subscription(
            Image,
            '/robot3/oakd/rgb/preview/image_raw',  # <--- 로봇이 실제로 발행하는 토픽명으로 수정
            self.image_callback,
            qos_profile)

        # 3. 발행자(Publisher) 설정: PC3가 기다리는 토픽명으로 발행
        self.publisher = self.create_publisher(
            Image,
            '/robot3/oakd/rgb/preview/image_raw', 
            qos_profile)

        self.get_logger().info('PC2 Image Relay Node가 시작되었습니다.')
        self.get_logger().info('구독 중: /robot3/oakd/rgb/preview/image_raw')
        self.get_logger().info('발행 중: /robot3/oakd/rgb/preview/image_raw')

    def image_callback(self, msg):
        # 받은 메시지를 그대로 다시 발행 (내용 변경 없음, 토픽명만 변경하는 효과)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()