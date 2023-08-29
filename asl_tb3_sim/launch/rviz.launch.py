#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare("asl_tb3_sim"), "configs", "default.rviz"])
    rviz_config = LaunchConfiguration("config")

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_rviz_config),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
        ),
    ])
