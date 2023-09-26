#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from asl_tb3_msgs.msg import TurtleBotState
from asl_tb3_lib.tf_utils import transform_to_state

class StatePublisher(Node):
    """StatePublisher
    This node reads from TF tree and publish robot pose to /state channel as
    TurtleBotState messages (at 100Hz).
    """
    def __init__(self) -> None:
        super().__init__("state_publisher")

        # TF datastructure for getting relative poses from the TF tree
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # turtlebot state publisher
        self.pub = self.create_publisher(TurtleBotState, "/state", 10)

        # 100 Hz publish loop
        self.timer = self.create_timer(0.01, self.publish_state)

    def publish_state(self) -> None:
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", Time())
            self.pub.publish(transform_to_state(t.transform))
        except TransformException as e:
            self.get_logger().warn(f"cannot get latest pose of robot")


if __name__ == "__main__":
    rclpy.init()
    node = StatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

