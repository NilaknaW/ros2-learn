#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    # class minor wich inherits from rclpy's Node class. 
    def __init__(self):
        super().__init__('first_node') # because file name is my_first_node.py
        self.get_logger().info('Hello from ROS2') # log message to the console

def main(args=None):
    rclpy.init(args=args) # Initialize the ROS 2 context/ communication
    # in between init and shutdown, we can create nodes, publishers, subscribers, services, etc.

    # creating nodes is with OOP
    node = MyNode() # create an object of the MyNode class
    rclpy.spin(node) # keep the node running until it is stopped
    rclpy.shutdown() # Shutdown the ROS 2 context/ communication

if __name__ == '__main__':
    main()