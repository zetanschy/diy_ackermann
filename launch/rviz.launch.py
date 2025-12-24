#!/usr/bin/env python3
"""
RViz Visualization Launch File for DIY Ackermann Robot

This launch file starts:
1. Robot State Publisher - publishes the robot's URDF to /robot_description
2. Joint State Publisher - provides joint states (only if not in simulation)
3. RViz2 - visualizes the robot model

Usage:
    # With Gazebo simulation (Terminal 2 after launching simulation):
    ros2 launch diy_ackermann rviz.launch.py
    
    # Standalone (without Gazebo):
    ros2 launch diy_ackermann rviz.launch.py use_sim_time:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package name
    package_name = "diy_ackermann"
    
    # Get package directory
    pkg_share = FindPackageShare(package=package_name)
    
    # Path to URDF file
    urdf_file_path = PathJoinSubstitution([pkg_share, "urdf", "diy_ackermann.urdf.xacro"])
    
    # Path to RViz config file
    rviz_config_path = PathJoinSubstitution([pkg_share, "rviz", "diy_ackermann.rviz"])
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true. Set to true when using with Gazebo."
    )
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_description": Command(["xacro ", urdf_file_path])
        }]
    )
    
    # Joint State Publisher Node (only when NOT using simulation)
    # When using simulation, Gazebo publishes joint_states via bridge
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
        condition=UnlessCondition(LaunchConfiguration("use_sim_time"))
    )
    
    # RViz2 Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }]
    )
    
    # Create and return launch description
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
