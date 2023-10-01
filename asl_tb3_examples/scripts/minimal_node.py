#!/usr/bin/env python3

import rclpy                    # ROS2 client library
from rclpy.node import Node     # ROS2 node baseclass


class MinimalNode(Node):
    def __init__(self) -> None:
        # give it a default node name
        super().__init__("minimal_node")


if __name__ == "__main__":
    rclpy.init()            # initialize ROS client library
    node = MinimalNode()    # create the node instance
    rclpy.spin(node)        # call ROS2 default scheduler
    rclpy.shutdown()        # clean up after node exits
