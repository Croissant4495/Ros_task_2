#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.create_timer(1.0, self.timer_calllback)
        
    
    def timer_calllback(self):
        self.get_logger().info("Hello")

def main(args=None):
    rclpy.init(args=args)       # Start ROS2 Communication
    
    # Main prog
    node = MyNode()
    rclpy.spin(node)            # Node kept alive (cont to run until killed) and all callbacks able to run

    rclpy.shutdown()            # End ROS2 Communication

if __name__ == '__main__':
    main()

