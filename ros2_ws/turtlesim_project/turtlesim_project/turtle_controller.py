import rclpy 
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
from my_robot_interfaces.srv import CatchTurtle
from my_robot_interfaces.msg import Turtle,TurtleArray
from functools import partial

class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.turtle_to_catch:Turtle=None
        self.pose:Pose=None
        self.pose_sub=self.create_subscription(Pose,"/turtle1/pose",self.pose_cb,10)
        self.twistpub=self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.alive_turtles=self.create_subscription(TurtleArray,"alive_turtles",self.cb_alive_turtles,10)
        self.control_timer=self.create_timer(0.01,self.control_cb)
        self.catch_turtle_client=self.create_client(CatchTurtle,"catch_turtle")


    def cb_alive_turtles(self,msg:TurtleArray):
        if len(msg.turtles)>0:
            self.turtle_to_catch=msg.turtles[0]


    def pose_cb(self,pose):
        self.pose=pose

    def control_cb(self):
        if self.pose is None or self.turtle_to_catch is None:
            return

        dx = self.turtle_to_catch.x - self.pose.x
        dy = self.turtle_to_catch.y - self.pose.y
        dist = math.sqrt(dx**2 + dy**2)

        cmd = Twist()

        if dist > 0.5:
            goal_theta = math.atan2(dy, dx)
            diff = goal_theta - self.pose.theta
            # normalize angle
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi

            # proportional control
            cmd.linear.x = float(min(2.0 * dist, 2.0))   # speed capped
            cmd.angular.z = float(6.0 * diff)

        else:
            # Stop and catch the turtle
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.twistpub.publish(cmd)   # <--- immediately stop
            self.call_catch_turtle_service(self.turtle_to_catch.name)
            self.turtle_to_catch = None
            return  # avoid publishing old command below

        self.twistpub.publish(cmd)

    def call_catch_turtle_service(self,turtle_name):
        while not self.catch_turtle_client.wait_for_service(1.0):
            self.get_logger().info("Waiting for server...")
        req=CatchTurtle.Request()
        req.name=turtle_name
        future=self.catch_turtle_client.call_async(req)
        future.add_done_callback(partial(self.catch_turtle_cb,turtle_name=turtle_name))
    
    def catch_turtle_cb(self,future,turtle_name):
        response:CatchTurtle.Response=future.result()
        if not response.success:
            self.get_logger().error("Turtle " + turtle_name+" couldnt be captured!")


def main(args=None):
    rclpy.init(args=args)
    node=TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()
