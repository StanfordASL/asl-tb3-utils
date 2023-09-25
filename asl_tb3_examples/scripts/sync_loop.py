#!/usr/bin/env python3

import time

from asl_tb3_examples.dummy_sensors import get_imu_data, \
                                           get_lidar_data, \
                                           get_motor_encoder_data, \
                                           get_heart_beat, \
                                           get_joystick_data

if __name__ == "__main__":
    while True:
        get_heart_beat()
        for i in range(5):
            get_lidar_data()
            for j in range(2):
                get_imu_data()
                time.sleep(0.5)
