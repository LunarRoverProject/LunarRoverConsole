import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class GoalReachedSubscriber(Node):
    def __init__(self):
        super().__init__('goal_reached_subscriber')
        self.subscription = self.create_subscription(
            Bool,
            '/goal_reached',
            self.callback,
            10
        )
        self.get_logger().info('Waiting for goal reached signal...')

    def callback(self, msg):
        if msg.data:
            self.get_logger().info('Goal reached!')
        else:
            self.get_logger().info('Goal not reached yet.')


def main():
    rclpy.init()
    node = GoalReachedSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

