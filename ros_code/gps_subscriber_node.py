
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GpsSubscriber(Node):
    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',     # 使用しているトピック名に応じて変更
            self.listener_callback,
            10
        )
        self.subscription  # 未使用警告回避
        self.get_logger().info('GPS Subscriber Initialized – waiting for data...')

    def listener_callback(self, msg: NavSatFix):
        self.get_logger().info(
            f'Received GPS → Latitude: {msg.latitude:.6f}, '
            f'Longitude: {msg.longitude:.6f}, Altitude: {msg.altitude:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = GpsSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

