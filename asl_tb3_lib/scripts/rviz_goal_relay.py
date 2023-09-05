#!/usr/bin/env python3

import rclpy

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from asl_tb3_msgs.msg import TurtleBotState
from asl_tb3_lib.tf_utils import pose_to_state


class RVIZGoalRelay(Node):
    """RVIZGoalRelay
    Relay RVIZ Goal Pose to any channel specified in the 'output_channel' parameter.
    This node also converts message from PoseStamped to TurtleBotState
    """

    def __init__(self) -> None:
        super().__init__("rviz_goal_relay")
        output_channel = self.declare_parameter("output_channel", "/cmd_pose").value

        self.rviz_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)
        self.relay_pub = self.create_publisher(TurtleBotState, output_channel, 10)

    def goal_callback(self, msg: PoseStamped) -> None:
        self.relay_pub.publish(pose_to_state(msg.pose))


if __name__ == "__main__":
    rclpy.init()
    node = RVIZGoalRelay()
    rclpy.spin(node)
    rclpy.shutdown()

