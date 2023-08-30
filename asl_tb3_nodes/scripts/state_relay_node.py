#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from asl_tb3_msgs.msg import TurtleBotStateStamped


class StateRelayNode(Node):

    def __init__(self) -> None:
        super().__init__("pose_relay_node")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.02, self.republish_latest_state)
        self.state_pub = self.create_publisher(
            TurtleBotStateStamped, "/state", rclpy.qos.qos_profile_system_default)

    def republish_latest_state(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())

            tb_state_stamped = TurtleBotStateStamped()
            tb_state_stamped.header = t.header
            tb_state_stamped.state.x = t.transform.translation.x
            tb_state_stamped.state.y = t.transform.translation.y
            tb_state_stamped.state.theta = math.atan2(
                2 * t.transform.rotation.w * t.transform.rotation.z,
                t.transform.rotation.w**2 - t.transform.rotation.z**2,
            )
            self.state_pub.publish(tb_state_stamped)
        except TransformException as e:
            pass


if __name__ == "__main__":
    rclpy.init()
    node = StateRelayNode()
    rclpy.spin(node)
    rclpy.shutdown()
