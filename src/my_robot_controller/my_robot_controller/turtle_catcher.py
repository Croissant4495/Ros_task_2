#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from functools import partial

class Turtle:
    def __init__(self, data=None) -> None:
        self.name = ""
        self.pose = Pose()
        self.speed = Twist()
    
    def get_attr(self):
        return [self.name, self.x, self.y, self.theta]


class turtleControl(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.speed_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10 )
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.my_turtle = Turtle()
        self.spawn_new_turtle()


        self.get_logger().info("Turtle controller started")

    def pose_callback(self, pose:Pose):
        self.my_turtle.pose = pose
        msg = Twist()
        
        if pose.x > 9.0 or pose.x < 2.0:
            msg.linear.x = 1.0
            msg.angular.z = 0.9
        else:
            msg.linear.x = 3.0
            msg.angular.z = 0.0
        self.speed_publisher.publish(msg)
        
        if self.has_collided(pose):
            self.spawn_new_turtle()

    def kill_sp_turtle(self):
        pass

    def spawn_new_turtle(self):
        self.get_logger().info("Calling spawn")
        # kill old turtle
        self.kill_sp_turtle()
        # spawn new turtle
        # Set up client
        client = self.create_client(Spawn, "turtlesim/srv/Spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for service.....")
        
        # set up request msg
        self.sp_turtle = Turtle("spawnee", random.random()*8 + 1, random.random()*8 + 1, random.random() * 2*3.14)
        request = Spawn.Request()
        request.name = self.sp_turtle.name
        request.x = self.sp_turtle.x
        request.y = self.sp_turtle.y
        request.theta = self.sp_turtle.theta

        # send request
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn))


    def callback_spawn(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r"%(e,))

    def has_collided(self, pose):
        pass



def main(args=None):
    rclpy.init(args=args)
    my_node = turtleControl()
    rclpy.spin(my_node)
    rclpy.shutdown()