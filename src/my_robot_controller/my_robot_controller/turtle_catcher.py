#!/usr/bin/env python3
import random
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from functools import partial

class Turtle:
    def __init__(self, name, data=None) -> None:
        self.name = ""
        if data:
            self.pose = data
        else:
            self.pose = Pose()
        self.speed = Twist()
    
    def get_attr(self):
        return [self.name, self.x, self.y, self.theta]


class turtleControl(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.speed_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10 )
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.my_turtle = Turtle("base")
        # self.spawn_new_turtle()
        # self.spawn_turtles = [random.random()*8 + 1, random.random()*8 + 1, random.random()*8 + 1, random.random() * 2 * 3.14]
        self.spawn_turtles = [11, -11, random.random() * 2 * 3.14]
        self.temp = False

        self.get_logger().info("Turtle controller started")

    def pose_callback(self, pose:Pose):
        self.my_turtle.pose = pose
        self.main()

    def pub_speed(self):
        # if turtle near border rotate
        msg = Twist()
        if self.my_turtle.pose.x > 9.0 or self.my_turtle.pose.x < 2.0:
            msg.linear.x = 1.0
            msg.angular.z = 0.9
        else:
            # msg.linear.x = 3.0
            msg.angular.z = 0.9
        self.get_logger().info('Current theta: "%f"' % self.my_turtle.pose.theta)
        self.adjust_angle()
        self.speed_publisher.publish(msg)
        
    def get_correction_angle(self):
        dy = self.spawn_turtles[1] - self.my_turtle.pose.y
        dx = self.spawn_turtles[0] - self.my_turtle.pose.x
        slope_angle = math.atan2(dy, dx)
        difference = self.my_turtle.pose.theta - slope_angle

        # Adjust differnce to be between pi and -pi
        if difference > math.pi:
            difference = difference - 2*math.pi
        elif difference < -math.pi:
            difference = difference + 2*math.pi

        self.get_logger().info('Slope angle: "%f"' % slope_angle)
        self.get_logger().info('Correction angle: "%f"' % (self.my_turtle.pose.theta - slope_angle))
        return difference
    
    def adjust_angle(self):
        difference = self.get_correction_angle()
        
        if abs(difference) < 0.3 or abs(difference) > (2*math.pi - 0.3):
            # Go forward
            self.get_logger().info('Forward')
        elif difference < 0:
            # Negative go Anti 
            self.get_logger().info('Anti-clockwise')
        else:
            # Positive go clock
            self.get_logger().info('clockwise')



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
        
        # create spawn turtle object
        self.get_logger().info("Craeted client")
        sp_pose = [random.random()*8 + 1, random.random()*8 + 1, random.random()*8 + 1, * 2 * 3.14]
        self.sp_turtle = Turtle("spawnee", sp_pose)

        # set up request msg
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
        return 0

    def main(self):
        self.pub_speed()



def main(args=None):
    rclpy.init(args=args)
    my_node = turtleControl()
    rclpy.spin(my_node)
    rclpy.shutdown()