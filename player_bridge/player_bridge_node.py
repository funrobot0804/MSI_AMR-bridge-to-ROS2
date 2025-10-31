#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math
import numpy as np
import time
import ctypes
import os
import sys
from ament_index_python.packages import get_package_share_directory





# 取得安裝後的 share 目錄
try:
    pkg_share_dir = get_package_share_directory('player_bridge')
    lib_path = os.path.join(pkg_share_dir, 'client_lib')
except Exception:
    # 若直接執行（非安裝狀態）
    pkg_dir = os.path.dirname(os.path.realpath(__file__))
    lib_path = os.path.join(os.path.dirname(pkg_dir), 'client_lib')

# 加入到 sys.path 讓 python 能 import
if os.path.isdir(lib_path) and lib_path not in sys.path:
    sys.path.insert(0, lib_path)

# 加入動態連結庫搜尋路徑
os.environ['LD_LIBRARY_PATH'] = os.environ.get('LD_LIBRARY_PATH', '') + ':' + lib_path

# 確認目前路徑
print("✅ client_lib path:", lib_path)

# 匯入 Player binding
from playercpp import *
from playerc import *



###################################################################################################
def euler_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll/2)
    sr = math.sin(roll/2)
    cp = math.cos(pitch/2)
    sp = math.sin(pitch/2)
    cy = math.cos(yaw/2)
    sy = math.sin(yaw/2)

    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy

    return x, y, z, w
###################################################################################################
class PlayerBridgeNode(Node):
    def __init__(self):
        super().__init__('player_bridge_node')

        # --- ROS 參數 ---
        self.declare_parameter('player_host', '192.168.0.1')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate', 30.0)

        # --- Player 連線 ---
        host = self.get_parameter('player_host').get_parameter_value().string_value
        self.get_logger().info(f'Connecting to Player server at {host}')

        self.robot = PlayerClient(host, 6665)
        self.p2d = Position2dProxy(self.robot, 0)
        self.dis = DispatcherProxy(self.robot,0);


        # --- ROS Publisher / Subscriber ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)


        # --- 速度命令 ---
        self.cmd_v = 0.0
        self.cmd_w = 0.0

        #not allow MSI AI 控制底盤
        self.dis.SetAiCmd(ctypes.c_uint(PLAYER_DISPATCH_INIT_DEST_RD).value,ctypes.c_uint(PLAYER_DISPATCH_AI_NOT_CONTROL_CHASSIS).value) 
        
        # --- Timer ---
        rate = self.get_parameter('publish_rate').value  # 直接取 value
        self.timer = self.create_timer(1.0 / rate, self.loop)
        #self.timer = self.create_timer(1.0 / self.get_parameter('publish_rate').get_parameter_value().double_value(), self.loop)

        self.get_logger().info("Player ROS2 bridge started.")

    # -------------------------------------------------------------------------
    def cmd_callback(self, msg: Twist):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        # 發送到 Player
        self.p2d.SetSpeed(self.cmd_v, self.cmd_w, 0.0, 0.0)

    # -------------------------------------------------------------------------
    def loop(self):
        if not self.robot.Peek(100):
            return
        self.robot.Read()

        # --- Odometry ---
        if self.p2d.IsFresh():
            self.p2d.NotFresh()
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.get_parameter('odom_frame').get_parameter_value().string_value
            odom_msg.child_frame_id = self.get_parameter('base_frame').get_parameter_value().string_value

            x = self.p2d.GetXPos()
            y = self.p2d.GetYPos()
            yaw = self.p2d.GetYaw()

            quat = euler_to_quaternion(0, 0, yaw)
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]

            odom_msg.twist.twist.linear.x = self.p2d.GetXSpeed()
            odom_msg.twist.twist.linear.y = self.p2d.GetYSpeed()
            odom_msg.twist.twist.angular.z = self.p2d.GetYawSpeed()
            self.odom_pub.publish(odom_msg)


    # -------------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info("Stopping robot...")
        try:
            self.p2d.SetSpeed(0.0, 0.0, 0.0, 0.0)
            # allow MSI AI 控制底盤
            self.dis.SetAiCmd(ctypes.c_uint(PLAYER_DISPATCH_INIT_DEST_RD).value,ctypes.c_uint(PLAYER_DISPATCH_AI_CONTROL_CHASSIS).value) 
        except Exception as e:
            self.get_logger().warn(str(e))

        del self.dis
        del self.p2d
        del self.robot
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlayerBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
