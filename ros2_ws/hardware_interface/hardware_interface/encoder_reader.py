import rclpy
import serial
import numpy as np
from math import pi, sqrt, cos, sin
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class encoder_reader(Node):
    """
    ROS2 node that reads encoder ticks from STM32 over serial (UART-USB Bridge).

    Published Topics:
    /odom : nav_msgs/Odometry containing estimated (x, y, theta) and body velocities.

    TF Broadcast:
    odom-base_link transform is broadcast into the TF2 tree

    Assumptions / Placeholders:
    - 4 wheels sampled every 'dt' seconds
    - Encoders send delta ticks per wheel every cycle framed as `s<t1>,<t2>,<t3>,<t4>e`.
    - Robot half-width = 0.2, half-length = 0.3, and wheel-radius = 0.03
    - Encoder resolution ticks-per-rev = 500
    """

    def __init__(self):
        # Initialize ROS publishers, serial connection, TF broadcaster, and pose state.
        super().__init__("encoder_reader")
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self.serial_buffer = ""
        self.dt = 0.05  # 50ms
        self.stm32 = serial.Serial('<port-name>', 115200, timeout=0.05)  # 11.52Kbps baud rate of STM32

        self.timer = self.create_timer(self.dt, self.publish_msg)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.ticks_per_rev = 500  # Encoder resolution
        self.wheel_radius = 0.03  # Wheel radius

        # half width and half length (centre to axle line)
        self.w = 0.2
        self.l = 0.3

        # init dead-reckoning
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.get_logger().info("Encoder odometry node <encoder_reader.py> initialized.")

    def publish_msg(self):
        # Read raw bytes from STM32 serial.
        data = self.stm32.read(self.stm32.in_waiting or 1).decode("utf-8", errors="ignore")
        self.serial_buffer += data

        while 's' in self.serial_buffer:
            # find start and end indices of packets
            start = self.serial_buffer.find('s')
            end = self.serial_buffer.find('e', start)
            if end == -1:  # if end isn't read yet, wait until one is found.
                break

            message = self.serial_buffer[start + 1: end]
            self.serial_buffer = self.serial_buffer[end + 1:]

            ticks = message.split(',')
            if len(ticks) != 4:  # if ticks aren't in 4 motor encoder format, go to next packet
                self.get_logger().warn(f"Malformed packet ignored: {message}")
                continue

            d_ticks = np.array([int(t) for t in ticks])
            wheel_dist = (d_ticks / self.ticks_per_rev) * 2 * pi * self.wheel_radius  # linear dist of wheel
            v = wheel_dist / self.dt  # linear vel of wheel

            # coeffs assume 45deg mounted wheels in regular alignment
            vx = (1 / sqrt(2)) * np.sum(v)
            vy = (1 / sqrt(2)) * (-v[0] + v[1] + v[2] - v[3])
            wz = (1 / sqrt(2)) * ((1 / self.w) + (1 / self.l)) * (v[0] - v[1] + v[2] - v[3])

            prev_theta = self.theta
            self.theta += wz * self.dt
            self.theta = (self.theta + pi) % (2 * pi) - pi

            dx = vx * cos(prev_theta) - vy * sin(prev_theta)
            dy = vx * sin(prev_theta) + vy * cos(prev_theta)

            self.x += dx * self.dt
            self.y += dy * self.dt

            # Publish Odometry
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"
            msg.pose.pose.position.x = self.x
            msg.pose.pose.position.y = self.y
            msg.pose.pose.position.z = 0.0

            msg.pose.pose.orientation.z = sin(self.theta / 2)
            msg.pose.pose.orientation.w = cos(self.theta / 2)

            self.odom_publisher.publish(msg)

            # Publish Transform
            transform = TransformStamped()
            transform.header.stamp = msg.header.stamp  # keep TF and odom time-aligned
            transform.header.frame_id = "odom"
            transform.child_frame_id = "base_link"
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = 0.0

            transform.transform.rotation.w = cos(self.theta / 2)
            transform.transform.rotation.z = sin(self.theta / 2)

            self.tf_broadcaster.sendTransform(transform)

            # self.get_logger().info(f"Pose → x: {self.x:.4f}, y: {self.y:.4f}, theta: {self.theta:.4f} rad")


def main():
    rclpy.init()
    node = encoder_reader()
    rclpy.spin(node)
    rclpy.shutdown()
