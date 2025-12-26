#ros2 node
#subscrive actual wheel velocity target and save it to csv file with timestamp
#actual wheel vel: /C620/actual_vel (std_msgs/Float64MultiArray)
#target wheel vel: /rover/targets (std_msgs/Float64MultiArray)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import csv
import time
from datetime import datetime

###

class RaspiWheelVelSub(Node):
    def __init__(self):
        super().__init__('raspi_wheel_vel_sub')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/C620/actual_rad',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.target_subscription = self.create_subscription(
            Float64MultiArray,
            '/rover/targets',
            self.target_listener_callback,
            10)
        self.target_subscription  # prevent unused variable warning
        
        self.twist_subscription=self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_listener_callback,
            10)
        
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.actual_vel = [0, 0, 0, 0]
        self.target_vel = [0, 0, 0 ,0]
        self.lin_vel=[0,0,0]
        self.ang_vel=[0,0,0]
        self.time = time.time()
        self.file = open('wheel_vel_{}.csv'.format(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')), 'w', newline='')
        self.get_logger().info('saved in wheel_vel_{}.csv'.format(datetime.now().strftime('%Y_%m_%d_%H_%M_%S')))
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time', "lf_target", "rf_target", "lb_target", "rb_target", "lf_actual", "rf_actual", "lb_actual", "rb_actual","linvel_x","linvel_y","linvel_z","angvel_x","angvel_y","angvel_z"])

    def listener_callback(self, msg):
        self.actual_vel = msg.data
    

    def target_listener_callback(self, msg):
        self.target_vel = msg.data
        
    def twist_listener_callback(self,msg):
        self.lin_vel=[msg.linear.x,msg.linear.y,msg.linear.z]
        self.ang_vel=[msg.angular.x,msg.angular.y,msg.angular.z]

    def timer_callback(self):
        self.time = time.time()
        self.writer.writerow([self.time, self.target_vel[0], self.target_vel[1], self.target_vel[2], self.target_vel[3], self.actual_vel[0], self.actual_vel[1], self.actual_vel[2], self.actual_vel[3],self.lin_vel[0],self.lin_vel[1],self.lin_vel[2],self.ang_vel[0],self.ang_vel[1],self.ang_vel[2]])

def main(args=None):
    rclpy.init(args=args)

    raspi_wheel_vel_sub = RaspiWheelVelSub()

    rclpy.spin(raspi_wheel_vel_sub)

    raspi_wheel_vel_sub.file.close()
    raspi_wheel_vel_sub.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    