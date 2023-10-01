#!/usr/bin/env python3

import math
import rclpy

from asl_tb3_lib.tf_utils import yaw_to_quaternion
from asl_tb3_lib.control import BaseController
from asl_tb3_msgs.msg import TurtleBotControl


class ConstantController(BaseController):
    def __init__(self) -> None:
        super().__init__("my_controller")

    def compute_control(self) -> TurtleBotControl:
        return TurtleBotControl(v=0.5, omega=0.5)


if __name__ == "__main__":
    yaw = math.pi / 2
    print(f"convert yaw = {yaw} to quaternion = {yaw_to_quaternion(yaw)}")

    rclpy.init()
    node = ConstantController()
    rclpy.spin(node)
    rclpy.shutdown()
