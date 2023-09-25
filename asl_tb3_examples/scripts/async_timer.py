#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from asl_tb3_examples.dummy_sensors import get_imu_data, \
                                           get_lidar_data, \
                                           get_motor_encoder_data, \
                                           get_heart_beat, \
                                           get_joystick_data

class AsycnLoop(Node):
    def __init__(self) -> None:
        super().__init__("async_loop")

        self.hb_timer = self.create_timer(5.0, get_heart_beat)
        self.lidar_timer = self.create_timer(1.0, get_lidar_data)
        self.imu_timer = self.create_timer(0.5, get_imu_data)
        self.joy_timer = self.create_timer(0.2, get_joystick_data)
        self.motor_timer = self.create_timer(8.0, get_motor_encoder_data)


if __name__ == "__main__":
    rclpy.init()
    node = AsycnLoop()
    rclpy.spin(node)
    rclpy.shutdown()
