#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare("asl_tb3_sim"),
        "worlds",
        "project_city.sdf",
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("asl_tb3_sim"),
                "launch",
                "root.launch.py",
            ]),
            launch_arguments={
                "world": world,
                "x_init": "3.15",
                "y_init": "1.6",
            }.items(),
        )
    ])
