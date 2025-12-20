import rclpy
import serial
import numpy as np
from math import sqrt
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class motor_controller(Node):
    def __init__(self):
        super().__init__("motor_controller")
        self.wheel_vel = self.create_subscription(Twist, '/cmd_vel', self.callback_wheel_vel, 10)
        self.scan_vel = self.create_subscription(Float32, '/scan_height', self.callback_scan_height, 10)
        self.w = 0.4
        self.l = 0.6
        self.wheel_speeds = np.array([0.0, 0.0, 0.0, 0.0])
        self.z_height = 0.0

        self.stm32 = serial.Serial('<port-name>', 115200, timeout=0.05)
        self.timer = self.create_timer(0.05, self.send_msg)
        
    def callback_wheel_vel(self, msg: Twist):
        self.wheel_speeds = (msg.linear.x * (sqrt(2) / 4) * np.array([1.0, 1.0, 1.0, 1.0])) + (msg.linear.y * (sqrt(2) / 4) * np.array([-1.0, 1.0, 1.0, -1.0])) + (msg.angular.z * (1 / (4 * sqrt(2) * ((1 / self.w) - (1 / self.l)))) * np.array([1.0, -1.0, 1.0, -1.0]))
    
    def callback_scan_height(self, msg: Float32):
        self.z_height = msg.data
    
    def timer(self):
        message = "s{:.3f},{:.3f},{:.3f},{:.3f},".format(*self.wheel_speeds) + "{:.3f}e".format(self.z_height)
        self.stm32.write(message.encode("utf-8"))

         

def main():
    rclpy.init()
    node = motor_controller()
    rclpy.spin(node)
    rclpy.shutdown()