import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from math import sin, cos, radians


class GoalSetter(Node):
    def __init__(self):
        super().__init__("goal_setter")
        self.racks_pos = np.zeros((15, 3))
        with open("/home/ysiddhanth/ros2_ws/src/navigation_control/navigation_control/rack_positions.txt", "r") as file:
            i=0
            for i, line in enumerate(file):
                self.racks_pos[i] = list(map(float, line.strip().split(',')))
                i += 1
        
        self.navigator = BasicNavigator()
        self.initial_pose = self.add_waypoint([0.0, 0.0, 0.0])
        self.navigator.setInitialPose(self.initial_pose)
        self.navigator.waitUntilNav2Active()

        for i in range(15):
            waypoint = self.add_waypoint(self.racks_pos[i])
            self.get_logger().info(f"Navigating to Rack {i}")
            self.navigator.goToPose(waypoint)
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
                feedback : NavigateToPose.Feedback = self.navigator.getFeedback()
                if feedback is not None:
                    self.get_logger().info(str(feedback.distance_remaining))
                    if self.goal_reached(feedback):
                        self.get_logger().info("Goal is under tolerance limit")
            
            result = self.navigator.getResult()
            
            if result == 0:
                self.get_logger().info("Succeeded")
            else:
                self.get_logger().info("Failed")


    def add_waypoint(self, position):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = position[0]
        goal.pose.position.y = position[1]
        position[2] = radians(position[2])
        goal.pose.orientation.z = sin(position[2] / 2)
        goal.pose.orientation.w = cos(position[2] / 2)
        return goal

    def goal_reached(self,feedback:NavigateToPose.Feedback):
        if feedback.distance_remaining<0.1:
            return True
        return False

def main():
    rclpy.init()
    node = GoalSetter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
