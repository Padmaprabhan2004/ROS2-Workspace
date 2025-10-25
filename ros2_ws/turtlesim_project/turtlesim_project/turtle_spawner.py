import rclpy
from rclpy.node import Node
import random
from turtlesim.srv import Spawn
from functools import partial
import math
from turtlesim.srv import Kill
from my_robot_interfaces.msg import Turtle,TurtleArray 
from my_robot_interfaces.srv import CatchTurtle

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawn")
        self.pre="turtle_"
        self.ctr=2
        self.alive_turtles_pub=self.create_publisher(TurtleArray,"alive_turtles",10)
        self.alive_turtles=[]
        self.spawn_client=self.create_client(Spawn,"/spawn")
        self.spawn_timer=self.create_timer(2.0,self.spawn_new_turtle)
        self.catch_turtle_service=self.create_service(CatchTurtle,"catch_turtle",self.cb_catch_turtle)
        self.kill_client=self.create_client(Kill,"/kill")

    def cb_catch_turtle(self,request:CatchTurtle.Request,response:CatchTurtle.Response):
        #call kill service
        self.call_kill_service(request.name)
        response.success=True
        return response
    
    
    def publish_alive_turtles(self):
        msg=TurtleArray()
        msg.turtles=self.alive_turtles
        self.alive_turtles_pub.publish(msg)

    def spawn_new_turtle(self):
        name=self.pre+str(self.ctr)
        self.ctr+=1
        x=random.uniform(0,11.0)
        y=random.uniform(0.0,11.0)
        theta=random.uniform(0.0,2*math.pi)
        self.call_service(name,x,y,theta)


    def call_kill_service(self,turtle_name):
        while not self.kill_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for kill server!")
        request=Kill.Request()
        request.name=turtle_name
        future=self.kill_client.call_async(request)
        future.add_done_callback(partial(self.cb_call_kill_service,turtle_name=turtle_name))
    def cb_call_kill_service(self,future,turtle_name):
        for (i,turtle) in enumerate(self.alive_turtles):
            if turtle.name==turtle_name:
                del self.alive_turtles[i]
                self.publish_alive_turtles()
                break



    def call_service(self,turtle_name,x,y,theta):
        while not self.spawn_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for spawn server")
        req=Spawn.Request()
        req.name=turtle_name
        req.x=x
        req.y=y
        req.theta=theta

        future=self.spawn_client.call_async(req)
        future.add_done_callback(partial(self.callback_spawn,request=req))

    def callback_spawn(self,future,request):
        response=future.result()
        if(response.name!=" "):
            new_turt=Turtle()
            new_turt.name=response.name
            new_turt.x=request.x
            new_turt.y=request.y
            new_turt.theta=request.theta
            self.alive_turtles.append(new_turt)
            self.get_logger().info("Spawned turtle "+response.name)
            self.publish_alive_turtles()



def main(args=None):
    rclpy.init(args=args)
    node=TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()