
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')

        self.subscriber = self.create_subscription(
            Float32MultiArray,
            '/imu/data',   
            self.callback,
            10
        )

        self.get_logger().info('IMU Subscriber Node has started')

    def callback(self, msg: Float32MultiArray):
        if len(msg.data) != 3:
            self.get_logger().warn('IMU data length is not 3')
            return

        heading = msg.data[0]
        roll = msg.data[1]
        pitch = msg.data[2]

        self.get_logger().info(
            f'IMU received → Heading: {heading:.2f}, Roll: {roll:.2f}, Pitch: {pitch:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ImuSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

