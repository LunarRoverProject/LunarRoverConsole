#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import json
import threading
import time
import os

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32, Float64MultiArray
from std_msgs.msg import Bool as MsgBool

class XBeeRoverBridge(Node):
    """
    ROS 2 Bridge for XBee telemetry. Runs on the Rover (Ubuntu).
    Subscribes to local ROS 2 topics and forwards them over XBee Serial port as JSON.
    Listens to the XBee Serial port for incoming JSON commands and publishes them to ROS.
    """
    def __init__(self):
        super().__init__('xbee_rover_bridge')
        
        # --- Serial Setup ---
        self.port = os.getenv('XBEE_PORT', '/dev/ttyUSB0')
        self.baud = int(os.getenv('XBEE_BAUD_RATE', '115200'))
        
        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=1)
            self.get_logger().info(f"Connected to XBee on {self.port} at {self.baud} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            self.serial = None
            
        # --- ROS Publishers (Incoming from XBee) ---
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_goal = self.create_publisher(NavSatFix, 'goal/fix', 10)
        
        # --- ROS Subscribers (Outgoing to XBee) ---
        self.sub_gps = self.create_subscription(NavSatFix, 'gps/fix', self.gps_cb, 10)
        self.sub_imu = self.create_subscription(Imu, 'bno055/imu', self.imu_cb, 10)
        self.sub_speed = self.create_subscription(Float32, '/rover/speed', self.speed_cb, 10)
        self.sub_rpm = self.create_subscription(Float32, '/rover/rpm', self.rpm_cb, 10)
        self.sub_actual_rad = self.create_subscription(Float64MultiArray, '/C620/actual_rad', self.actual_rad_cb, 10)
        self.sub_targets = self.create_subscription(Float64MultiArray, '/rover/targets', self.targets_cb, 10)
        self.sub_goal_reached = self.create_subscription(MsgBool, '/goal_reached', self.goal_reached_cb, 10)
        self.sub_pose = self.create_subscription(PoseStamped, '/rover/pose', self.pose_cb, 10)

        # Start Serial Reader Thread
        if self.serial:
            self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
            self.read_thread.start()

    def send_to_xbee(self, data_dict):
        if not self.serial:
            return
        try:
            # Flatten to a single line JSON string followed by newline (\n)
            msg_str = json.dumps(data_dict) + '\n'
            self.serial.write(msg_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Error writing to XBee: {e}")

    # --- Topic Callbacks ---
    def gps_cb(self, msg):
        self.send_to_xbee({
            "type": "gps", "data": {
                "latitude": msg.latitude, 
                "longitude": msg.longitude, 
                "altitude": msg.altitude,
                "position_covariance": list(msg.position_covariance)
            }
        })

    def imu_cb(self, msg):
        self.send_to_xbee({
            "type": "imu",
            "data": {
                "orientation": {
                    "x": msg.orientation.x, "y": msg.orientation.y, 
                    "z": msg.orientation.z, "w": msg.orientation.w
                }
            }
        })

    def speed_cb(self, msg):
        self.send_to_xbee({"type": "speed", "data": msg.data})

    def rpm_cb(self, msg):
        self.send_to_xbee({"type": "rpm", "data": msg.data})

    def actual_rad_cb(self, msg):
        self.send_to_xbee({"type": "actual_rad", "data": list(msg.data)})

    def targets_cb(self, msg):
        self.send_to_xbee({"type": "targets", "data": list(msg.data)})

    def goal_reached_cb(self, msg):
        self.send_to_xbee({"type": "goal_reached", "data": msg.data})

    def pose_cb(self, msg):
        self.send_to_xbee({
            "type": "pose",
            "data": {
                "header": {"stamp": {"sec": msg.header.stamp.sec}},
                "pose": {
                    "position": {"x": msg.pose.position.x, "y": msg.pose.position.y, "z": msg.pose.position.z},
                    "orientation": {"x": msg.pose.orientation.x, "y": msg.pose.orientation.y, "z": msg.pose.orientation.z, "w": msg.pose.orientation.w}
                }
            }
        })

    # --- Read Thread & Processing ---
    def serial_read_loop(self):
        while rclpy.ok():
            try:
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8').strip()
                    if line:
                        self.process_incoming_command(line)
            except Exception as e:
                self.get_logger().error(f"Error reading XBee: {e}")
            time.sleep(0.01)

    def process_incoming_command(self, line):
        try:
            data = json.loads(line)
            msg_type = data.get("type")
            
            if msg_type == "cmd_vel":
                twist = Twist()
                twist.linear.x = float(data.get("linear", 0.0))
                twist.angular.z = float(data.get("angular", 0.0))
                self.pub_cmd_vel.publish(twist)
                self.get_logger().info(f"Published cmd_vel: {twist.linear.x}, {twist.angular.z}")
                
            elif msg_type == "goal_fix":
                fix = NavSatFix()
                fix.latitude = float(data.get("latitude", 0.0))
                fix.longitude = float(data.get("longitude", 0.0))
                fix.altitude = float(data.get("altitude", 0.0))
                self.pub_goal.publish(fix)
                self.get_logger().info(f"Published goal_fix: {fix.latitude}, {fix.longitude}")
                
        except json.JSONDecodeError:
            pass # Ignore corrupted lines over serial radio
        except Exception as e:
            self.get_logger().error(f"Error processing command {line}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = XBeeRoverBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial:
            node.serial.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
