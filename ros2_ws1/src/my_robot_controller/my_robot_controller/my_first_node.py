#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    # class minor wich inherits from rclpy's Node class. 
    def __init__(self):
        super().__init__('first_node') # because file name is my_first_node.py
        # self.get_logger().info('Hello world') # log message to the console

        self.counter = 0

        # create a timer that calls the timer_callback function every second
        self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Hello %d' % self.counter)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args) # Initialize the ROS 2 context/ communication
    # in between init and shutdown, we can create nodes, publishers, subscribers, services, etc.

    # creating nodes is with OOP
    node = MyNode() # create an object of the MyNode class
    rclpy.spin(node) # keep the node running until it is stopped, callback is enabled by this
    rclpy.shutdown() # Shutdown the ROS 2 context/ communication

if __name__ == '__main__':
    main()