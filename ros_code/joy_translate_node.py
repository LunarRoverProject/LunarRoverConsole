
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyScalerLinearAngular(Node):
    def __init__(self):
        super().__init__('joy_scaler_lin_ang')
        self.lin_scale = 1.0
        self.ang_scale = 1.0
        self.factor = 1.1

        self.x_button_idx = 2  # Xで並進ダウン
        self.a_button_idx = 0  # Aで回転ダウン
        self.b_button_idx = 1  # Bで回転アップ
        self.y_button_idx = 3  # Yで並進アップ

        self.prev_buttons = []

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.get_logger().info('JoyScalerLinearAngular node started.')

    def joy_callback(self, msg: Joy):
        if not self.prev_buttons:
            self.prev_buttons = msg.buttons

        # 並進速度調整
        if msg.buttons[self.y_button_idx] == 1 and self.prev_buttons[self.y_button_idx] == 0:
            self.lin_scale *= self.factor
            self.get_logger().info(f'Y pressed → linear scale = {self.lin_scale:.2f}')
        if msg.buttons[self.x_button_idx] == 1 and self.prev_buttons[self.x_button_idx] == 0:
            self.lin_scale /= self.factor
            self.get_logger().info(f'X pressed → linear scale = {self.lin_scale:.2f}')

        # 回転速度調整
        if msg.buttons[self.b_button_idx] == 1 and self.prev_buttons[self.b_button_idx] == 0:
            self.ang_scale *= self.factor
            self.get_logger().info(f'B pressed → angular scale = {self.ang_scale:.2f}')
        if msg.buttons[self.a_button_idx] == 1 and self.prev_buttons[self.a_button_idx] == 0:
            self.ang_scale /= self.factor
            self.get_logger().info(f'A pressed → angular scale = {self.ang_scale:.2f}')

        self.prev_buttons = list(msg.buttons)

        twist = Twist()
        twist.linear.x = self.lin_scale * msg.axes[1]
        twist.angular.z = self.ang_scale * msg.axes[0]
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyScalerLinearAngular()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

