#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    world = PathJoinSubstitution([
        FindPackageShare("asl_tb3_sim"),
        "worlds",
        "maze.world",
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
                "x_init": "1.0",
                "y_init": "-0.5",
            }.items(),
        )
    ])
