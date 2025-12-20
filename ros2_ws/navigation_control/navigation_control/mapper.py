import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import atan2, sin, cos, sqrt, pi

class mapper(Node):
    def __init__(self):
        super().__init__("mapper")
        self.slam_vel = self.create_publisher(TwistStamped, "/mecanum_drive_controller/cmd_vel", 10)
        self.lidar_scan = self.create_subscription(LaserScan, "/scan", self.callback_lidar, 10)
        self.estimated_pos = self.create_subscription(PoseWithCovarianceStamped, "/pose", self.get_position, 10)
        self.timer = self.create_timer(0.01, self.publish_vel)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.kp_lin = 2.0
        self.kp_ang = 5.0
        self.ranges = np.zeros(360)
        self.target_position = np.zeros(3)

    def callback_lidar(self, msg : LaserScan):
        ranges = np.array(msg.ranges)
        self.ranges = ranges[180 : 541]

    def get_position(self, msg : PoseWithCovarianceStamped):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        if(sqrt((self.target_position[0] - self.x)**2 + (self.target_position[1] - self.y)**2) < 1 and abs((self.theta - self.target_position[2] + pi) % (2 * pi) - pi) < np.radians(10)):
            discrete_indices = np.arange(0, 365, 10)
            discrete_sectors = np.zeros(36)
            target_index = 0
            for i in range(len(discrete_indices) - 1):
                discrete_sectors[i] = np.mean(self.ranges[discrete_indices[i] : discrete_indices[i + 1]])
                if(discrete_sectors[i] > discrete_sectors[target_index]):
                    target_index = i  
            target_angle = np.radians(5 * target_index + 2.5)  
            target_distance = discrete_sectors[target_index]
            self.target_position[0] = target_distance * cos(self.theta + target_angle - pi / 2) + self.x
            self.target_position[1] = target_distance * sin(self.theta + target_angle - pi / 2) + self.y
            self.target_position[2] = self.theta + target_angle - pi / 2
            
    
    def publish_vel(self):
        vel = TwistStamped()
        vel.header.stamp = self.get_clock().now().to_msg()
        dist = sqrt((self.target_position[0] - self.x)**2 + (self.target_position[1] - self.y)**2)
        vel.twist.linear.x = self.kp_lin * dist + 0.001
        vel.twist.angular.z = self.kp_ang * (self.target_position[2] - self.theta)
        self.slam_vel.publish(vel)


def main():
    rclpy.init()
    node = mapper()
    rclpy.spin(node)
    rclpy.shutdown()