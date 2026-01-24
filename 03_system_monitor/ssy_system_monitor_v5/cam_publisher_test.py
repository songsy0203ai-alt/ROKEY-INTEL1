#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
class ImageRelay(Node):
    def __init__(self):
        super().__init__('pc2_image_relay') # 추정... pc1 ,2에서 amr 카메라 토픽 받아오는 노드 이름!!!!!!!!!!
        # ===== 원본 토픽 (TurtleBot4에서 이미 받고 있는 토픽) =====
        self.src_topic = "/robot3/oakd/rgb/preview/compressed"
        # ===== PC3로 보낼 토픽 =====
        self.dst_topic = "/robot3/oakd/rgb/preview/compressed" # 이걸 수정함 !!!!!!!!!!!!!!
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.sub = self.create_subscription(
            Image,
            self.src_topic,
            self.image_cb,
            qos
        )
        self.pub = self.create_publisher(
            Image,
            self.dst_topic,
            qos
        )
        self.bridge = CvBridge()
        self.get_logger().info("📡 PC2 Image Relay started")
        self.get_logger().info(f"SUB: {self.src_topic}")
        self.get_logger().info(f"PUB: {self.dst_topic}")
    def image_cb(self, msg: Image):
        # 그대로 전달 (재인코딩 없음)
        self.pub.publish(msg)
def main():
    rclpy.init()
    node = ImageRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()