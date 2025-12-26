import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, '/goal_fix', 10)
        self.get_logger().info('Goal Publisher Initialized!')

    def send_goal(self, lat, lon, alt):
        msg = NavSatFix()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        self.publisher_.publish(msg)
        self.get_logger().info(f'Goal Published → Latitude: {lat}, Longitude: {lon}, Altitude: {alt}')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()

    try:
        while True:
            # ユーザー入力
            lat = float(input("Latitude: "))
            lon = float(input("Longitude: "))
            alt = float(input("Altitude: "))
            node.send_goal(lat, lon, alt)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()