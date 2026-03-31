#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class QosBridge(Node):
    def __init__(self):
        super().__init__('camera_qos_bridge')
        
        # 受信（サブスクライバ）用のQoS：BEST_EFFORTを指定
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 送信（パブリッシャ）用のQoS：RELIABLEを指定
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Front Camera
        self.pub_front = self.create_publisher(CompressedImage, '/camera_front/image_reliable/compressed', reliable_qos)
        self.sub_front = self.create_subscription(CompressedImage, '/camera_front/image/compressed', self.front_callback, best_effort_qos)
        
        # Back Camera
        self.pub_back = self.create_publisher(CompressedImage, '/camera_back/image_reliable/compressed', reliable_qos)
        self.sub_back = self.create_subscription(CompressedImage, '/camera_back/image/compressed', self.back_callback, best_effort_qos)
        
        self.get_logger().info('QoS Bridge is running. Translating BEST_EFFORT to RELIABLE.')

    def front_callback(self, msg):
        self.pub_front.publish(msg)

    def back_callback(self, msg):
        self.pub_back.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
