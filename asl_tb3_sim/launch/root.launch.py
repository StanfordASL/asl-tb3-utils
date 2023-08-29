#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    x_init = LaunchConfiguration("x_init")
    y_init = LaunchConfiguration("y_init")
    world = LaunchConfiguration("world")

    default_world = PathJoinSubstitution([
        FindPackageShare("asl_tb3_sim"),
        "worlds",
        "turtlebot3_world.sdf",
    ])

    this_dir = FindPackageShare("asl_tb3_sim")

    sdf_file = os.path.join(this_dir.find("asl_tb3_sim"), "models", "asl_tb3.sdf")
    with open(sdf_file, "r") as f:
        robot_desc = f.read()
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
    )

    gz_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py",
        ]),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
    )

    spawn_turtlebot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "default",
            "-file", PathJoinSubstitution([FindPackageShare("asl_tb3_sim"), "models", "asl_tb3.sdf"]),
            "-name", "asl_tb3",
            "-x", x_init,
            "-y", y_init,
            "-z", "0.01",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": PathJoinSubstitution([
                FindPackageShare("asl_tb3_sim"),
                "configs",
                "gz_bridge.yaml",
            ]),
        }],
    )

    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_scan", "asl_tb3/base_scan/velodyne"],
    )
    imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "1", "imu_link", "asl_tb3/imu_link/imu"],
    )

    slam = Node(
       parameters=[
           PathJoinSubstitution([FindPackageShare("asl_tb3_driver"), "configs", "slam_params.yaml"]),
           {"use_sim_time": True},
       ],
       package='slam_toolbox',
       executable='async_slam_toolbox_node',
       name='slam_toolbox',
    )

    return LaunchDescription([
        DeclareLaunchArgument("x_init", default_value="-2.0"),
        DeclareLaunchArgument("y_init", default_value="-0.5"),
        DeclareLaunchArgument("world", default_value=default_world),
        robot_state_pub,
        gz_launch,
        spawn_turtlebot,
        bridge,
        lidar_tf,
        imu_tf,
        slam,
    ])
