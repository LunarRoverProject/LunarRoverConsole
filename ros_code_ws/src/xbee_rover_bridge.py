#!/usr/bin/env python3
"""
xbee_rover_bridge.py  —  ローバー側 XBee AT モード ブリッジ (ROS 2)

動作:
  - ROS 2 トピックを購読し、JSON 形式で XBee 経由(シリアル)へ送信
  - XBee 経由で受信した JSON コマンドを ROS 2 トピックへ publish

設定 (環境変数):
  XBEE_PORT       シリアルポート (デフォルト: /dev/ttyUSB0)
  XBEE_BAUD_RATE  ボーレート     (デフォルト: 115200)
"""

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

    def __init__(self):
        super().__init__('xbee_rover_bridge')

        # --- 設定 ---
        self.port = os.getenv('XBEE_PORT',      '/dev/ttyUSB0')
        self.baud = int(os.getenv('XBEE_BAUD_RATE', '115200'))

        # --- 状態 ---
        self._serial: serial.Serial | None = None
        self._serial_lock = threading.Lock()
        self._connected = False

        # --- ROS Publishers (XBee → ROS) ---
        self.pub_cmd_vel = self.create_publisher(Twist,     'cmd_vel',  10)
        self.pub_goal    = self.create_publisher(NavSatFix, 'goal/fix', 10)

        # --- ROS Subscribers (ROS → XBee) ---
        self.create_subscription(NavSatFix,        'gps/fix',          self.gps_cb,        10)
        self.create_subscription(Imu,              'bno055/imu',       self.imu_cb,        10)
        self.create_subscription(Float32,          '/rover/speed',     self.speed_cb,      10)
        self.create_subscription(Float32,          '/rover/rpm',       self.rpm_cb,        10)
        self.create_subscription(Float64MultiArray,'/C620/actual_rad', self.actual_rad_cb, 10)
        self.create_subscription(Float64MultiArray,'/rover/targets',   self.targets_cb,    10)
        self.create_subscription(MsgBool,          '/goal_reached',    self.goal_reached_cb, 10)
        self.create_subscription(PoseStamped,      '/rover/pose',      self.pose_cb,       10)

        # --- シリアル接続 & 読み込みスレッド (常時起動・自動再接続) ---
        self._serial_thread = threading.Thread(
            target=self._serial_worker, daemon=True, name='xbee_serial'
        )
        self._serial_thread.start()

        self.get_logger().info(f'XBee bridge started. Port={self.port} Baud={self.baud}')

    # =========================================================
    # シリアル接続管理 (自動再接続ループ)
    # =========================================================
    def _serial_worker(self):
        """シリアルポートへの接続・再接続と受信を一つのスレッドで管理する。"""
        while rclpy.ok():
            if not self._connected:
                self._try_connect()
                if not self._connected:
                    time.sleep(3.0)
                    continue

            # 受信処理
            try:
                with self._serial_lock:
                    waiting = self._serial.in_waiting if self._serial else 0

                if waiting > 0:
                    with self._serial_lock:
                        line = self._serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self._process_incoming(line)
                else:
                    time.sleep(0.005)

            except serial.SerialException as e:
                self.get_logger().warn(f'[XBee] Serial error: {e}  → 再接続を試みます')
                self._disconnect()
            except Exception as e:
                self.get_logger().error(f'[XBee] Unexpected error in read loop: {e}')
                time.sleep(0.1)

    def _try_connect(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
            with self._serial_lock:
                self._serial = ser
            self._connected = True
            self.get_logger().info(f'[XBee] Connected: {self.port} @ {self.baud} baud')
        except serial.SerialException as e:
            self.get_logger().warn(f'[XBee] Cannot open {self.port}: {e}')

    def _disconnect(self):
        with self._serial_lock:
            if self._serial:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
        self._connected = False

    # =========================================================
    # 送信 (ROS → XBee)
    # =========================================================
    def _send(self, data_dict: dict):
        """JSON を 1 行にして XBee へ書き込む。接続していない場合は無視。"""
        if not self._connected:
            return
        try:
            line = json.dumps(data_dict, separators=(',', ':')) + '\n'
            with self._serial_lock:
                if self._serial:
                    self._serial.write(line.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().warn(f'[XBee] Write error: {e}')
            self._disconnect()
        except Exception as e:
            self.get_logger().error(f'[XBee] Unexpected write error: {e}')

    # =========================================================
    # ROS サブスクライバ コールバック
    # =========================================================
    def gps_cb(self, msg: NavSatFix):
        self._send({
            'type': 'gps',
            'data': {
                'latitude':  msg.latitude,
                'longitude': msg.longitude,
                'altitude':  msg.altitude,
                'position_covariance': list(msg.position_covariance),
            }
        })

    def imu_cb(self, msg: Imu):
        o = msg.orientation
        self._send({
            'type': 'imu',
            'data': {
                'orientation': {'x': o.x, 'y': o.y, 'z': o.z, 'w': o.w}
            }
        })

    def speed_cb(self, msg: Float32):
        self._send({'type': 'speed', 'data': msg.data})

    def rpm_cb(self, msg: Float32):
        self._send({'type': 'rpm', 'data': msg.data})

    def actual_rad_cb(self, msg: Float64MultiArray):
        self._send({'type': 'actual_rad', 'data': list(msg.data)})

    def targets_cb(self, msg: Float64MultiArray):
        self._send({'type': 'targets', 'data': list(msg.data)})

    def goal_reached_cb(self, msg: MsgBool):
        self._send({'type': 'goal_reached', 'data': msg.data})

    def pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        self._send({
            'type': 'pose',
            'data': {
                'header': {'stamp': {'sec': msg.header.stamp.sec}},
                'pose': {
                    'position':    {'x': p.x, 'y': p.y, 'z': p.z},
                    'orientation': {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w},
                }
            }
        })

    # =========================================================
    # 受信処理 (XBee → ROS)
    # =========================================================
    def _process_incoming(self, line: str):
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            return  # 電波ノイズ等による化けは無視

        msg_type = data.get('type')

        if msg_type == 'cmd_vel':
            twist = Twist()
            twist.linear.x  = float(data.get('linear',  0.0))
            twist.angular.z = float(data.get('angular', 0.0))
            self.pub_cmd_vel.publish(twist)
            self.get_logger().info(
                f'[XBee] cmd_vel → linear={twist.linear.x:.2f}  angular={twist.angular.z:.2f}'
            )

        elif msg_type == 'goal_fix':
            fix = NavSatFix()
            fix.latitude  = float(data.get('latitude',  0.0))
            fix.longitude = float(data.get('longitude', 0.0))
            fix.altitude  = float(data.get('altitude',  0.0))
            self.pub_goal.publish(fix)
            self.get_logger().info(
                f'[XBee] goal_fix → lat={fix.latitude:.6f}  lng={fix.longitude:.6f}'
            )

        else:
            self.get_logger().debug(f'[XBee] Unknown message type: {msg_type}')


# =============================================================
# エントリーポイント
# =============================================================
def main(args=None):
    rclpy.init(args=args)
    node = XBeeRoverBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
