#!/usr/bin/env python3
import random
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from std_srvs.srv import Empty

from functools import partial

class myPose:
    def __init__(self, x, y, theta) -> None:
        self.x = x
        self.y = y
        self.theta = theta
    
    def get_attr(self):
        return [self.x, self.y, self.theta]

class Turtle:
    def __init__(self, name, data=None) -> None:
        self.name = name
        if data:
            self.pose = myPose(*data)
        else:
            self.pose = Pose()
        self.speed = Twist()
    
    def get_attr(self):
        return [self.name, *self.pose.get_attr()]


class turtleControl(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.speed_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10 )
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.my_turtle = Turtle("base")
        self.spawn_turtles = [7, 7, random.random() * 2 * 3.14]
        self.spawn_turtle_arr = []
        self.spawn_counter = 0

        self.spawn_new_turtle()
        self.get_logger().info("Turtle controller started")

    def reset(self):
        # Set up client
        client = self.create_client(Empty, "/reset")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for service.....")
        request = Empty.Request()
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_reset))

    def callback_reset(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r"%(e,))

    def pose_callback(self, pose:Pose):
        self.my_turtle.pose = pose
        self.main()

    def pub_speed(self):
        msg = Twist()
        angle = self.get_adjust_angle()
        
        if angle == 0:
            msg.linear.x = 7.0
        else:
            # msg.linear.x = 1.0
            msg.angular.z = angle

        # msg.angular.z=0.9
        
        # self.get_logger().info('Current theta: "%f"' % self.my_turtle.pose.theta)
        self.speed_publisher.publish(msg)
    

    # ______Angle Correction_______
    def get_correction_angle(self):
        dy = self.spawn_turtle_arr[0].pose.y - self.my_turtle.pose.y
        dx = self.spawn_turtle_arr[0].pose.x - self.my_turtle.pose.x
        slope_angle = math.atan2(dy, dx)
        difference = self.my_turtle.pose.theta - slope_angle

        # Adjust differnce to be between pi and -pi
        if difference > math.pi:
            difference = difference - 2*math.pi
        elif difference < -math.pi:
            difference = difference + 2*math.pi

        # self.get_logger().info('Slope angle: "%f"' % slope_angle)
        # self.get_logger().info('Correction angle: "%f"' % (self.my_turtle.pose.theta - slope_angle))
        return difference
    
    def get_adjust_angle(self):
        difference = self.get_correction_angle()
        
        if abs(difference) < 0.2:
            # Go forward
            # self.get_logger().info('Forward')
            angle = 0
        elif difference < 0:
            # Negative go Anti 
            # self.get_logger().info('Anti-clockwise')
            angle = abs(difference)/math.pi + 1.5
        else:
            # Positive go clock
            # self.get_logger().info('clockwise')
            angle = -abs(difference)/math.pi - 1.5

        return angle
    
    # ______Turtle Spawner/Killer_______
    def kill_sp_turtle(self, name):
        # turtlesim/srv/Kill
        # Set up client
        client = self.create_client(Kill, "/kill")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for service.....")

        # Set up Request msg
        request = Kill.Request()
        request.name = name

        # Send Request and pop from array
        self.spawn_turtle_arr.pop(0)
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_kill))

    def callback_kill(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r"%(e,))


    def spawn_new_turtle(self):
        self.get_logger().info("Calling spawn")
        # Set up client
        client = self.create_client(Spawn, "/spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for service.....")
        
        # create spawn turtle object
        self.get_logger().info("Craeted client")
        sp_pose = [random.random()*9 + 1, random.random()*9 + 1, random.random()* 2 * 3.14]
        self.spawn_turtle_arr.append(Turtle("Turtle" + str(self.spawn_counter), sp_pose))
        self.spawn_counter +=1

        # set up request msg
        request = Spawn.Request()
        request.name, request.x, request.y, request.theta = self.spawn_turtle_arr[-1].get_attr()

        # send request
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn))

    def callback_spawn(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r"%(e,))

    # ______Collision Detection_______
    def get_dist(self, coord):
        dy = self.my_turtle.pose.y - coord[1]
        dx = self.my_turtle.pose.x - coord[0]
        return math.sqrt(dy*dy + dx*dx)

    def has_collided(self):
        if self.get_dist([self.spawn_turtle_arr[0].pose.x, self.spawn_turtle_arr[0].pose.y]) < 0.3:
            return True
        return False
    
    def collision_handler(self):
        if self.has_collided():
            self.kill_sp_turtle(self.spawn_turtle_arr[0].name)
            self.spawn_new_turtle()
    
    # ______Main_______
    def main(self):
        # Already update current pose
        self.collision_handler()
        self.pub_speed()



def main(args=None):
    rclpy.init(args=args)
    my_node = turtleControl()
    rclpy.spin(my_node)
    rclpy.shutdown()