#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_init = LaunchConfiguration("x_init", default="-2.0")
    y_init = LaunchConfiguration("y_init", default="-0.5")

    world = LaunchConfiguration("world", default=
        PathJoinSubstitution([
            FindPackageShare("turtlebot3_gazebo"),
            "worlds",
            "turtlebot3_world.world",
        ])
    )

    gazebo_ros_dir = FindPackageShare("gazebo_ros")
    tb3_gazebo_dir = FindPackageShare("turtlebot3_gazebo")
    this_dir = FindPackageShare("asl_tb3_sim")

    gzserver_cmd = IncludeLaunchDescription(
        PathJoinSubstitution([gazebo_ros_dir, "launch", "gzserver.launch.py"]),
        launch_arguments={"world": world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PathJoinSubstitution([gazebo_ros_dir, "launch", "gzclient.launch.py"])
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PathJoinSubstitution([tb3_gazebo_dir, "launch", "robot_state_publisher.launch.py"]),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )

    spawn_turtlebot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "burger",
            "-file", PathJoinSubstitution([this_dir, "models", "asl_tb3.sdf"]),
            "-x", x_init,
            "-y", y_init,
            "-z", "0.01",
        ],
        output="screen",
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot)

    return ld
