#!/usr/bin/env python3
"""
Gazebo Simulation Launch File for DIY Ackermann Robot

This launch file starts:
1. Gazebo Sim with a custom world
2. Robot State Publisher - publishes the robot's URDF
3. Spawns the robot into Gazebo
4. ROS-Gazebo Bridge for communication

Usage:
    ros2 launch diy_ackermann simulation.launch.py
    
Optional arguments:
    world:=<world_name>           - Specify world file (default: ackermann_world.sdf)
    robot_x:=<x_position>         - X spawn position (default: 0.0)
    robot_y:=<y_position>         - Y spawn position (default: 0.0)
    robot_yaw:=<yaw_angle>        - Yaw spawn angle (default: 0.0)
    use_rviz:=<true/false>        - Launch RViz (default: false)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package name
    package_name = "diy_ackermann"
    
    # Get package directories
    pkg_share = FindPackageShare(package=package_name)
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    
    # Paths
    urdf_file_path = PathJoinSubstitution([pkg_share, "urdf", "diy_ackermann.urdf.xacro"])
    world_file_path = PathJoinSubstitution([pkg_share, "worlds", LaunchConfiguration("world")])
    rviz_config_path = PathJoinSubstitution([pkg_share, "rviz", "diy_ackermann.rviz"])
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="ackermann_world.sdf",
        description="World file name"
    )
    
    robot_x_arg = DeclareLaunchArgument(
        "robot_x",
        default_value="0.0",
        description="Robot spawn X position"
    )
    
    robot_y_arg = DeclareLaunchArgument(
        "robot_y",
        default_value="0.0",
        description="Robot spawn Y position"
    )
    
    robot_yaw_arg = DeclareLaunchArgument(
        "robot_yaw",
        default_value="0.0",
        description="Robot spawn yaw angle"
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz for visualization (default: false, launch separately)"
    )
    
    # Gazebo Sim Launch
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r ", world_file_path],
            "on_exit_shutdown": "true"
        }.items()
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_description": Command(["xacro ", urdf_file_path])
        }]
    )
    
    # Spawn Robot in Gazebo
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=[
            "-name", "diy_ackermann_robot",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("robot_x"),
            "-y", LaunchConfiguration("robot_y"),
            "-z", "0.2",
            "-Y", LaunchConfiguration("robot_yaw")
        ]
    )
    
    # ROS-Gazebo Bridge for cmd_vel
    bridge_cmd_vel = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_cmd_vel",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
        ],
        output="screen"
    )
    
    # ROS-Gazebo Bridge for odometry
    bridge_odom = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_odom",
        arguments=[
            "/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
        ],
        output="screen"
    )
    
    # ROS-Gazebo Bridge for lidars
    bridge_lidar = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_lidar",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan"
        ],
        output="screen"
    )
    
    
    # ROS-Gazebo Bridge for 3D lidar PointCloud2
    # Gazebo publishes point cloud to /robot/lidar/points when topic is /robot/lidar
    bridge_lidar3d_points = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_lidar3d_points",
        arguments=[
            "/lidar_3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
        ],
        output="screen",
        parameters=[{
            "use_sim_time": True
        }]
    )

    # ROS-Gazebo Bridge for joint_states (bidirectional)
    bridge_joint_states = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_joint_states",
        arguments=[
            "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model"
        ],
        output="screen"
    )
    
    # ROS-Gazebo Bridge for clock
    bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_clock",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock"
        ],
        output="screen"
    )
    
    # RViz Node (conditional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{
            "use_sim_time": True
        }],
        condition=IfCondition(LaunchConfiguration("use_rviz"))
    )
    
    # Create and return launch description
    return LaunchDescription([
        world_arg,
        robot_x_arg,
        robot_y_arg,
        robot_yaw_arg,
        use_rviz_arg,
        gazebo_sim,
        robot_state_publisher_node,
        spawn_robot_node,
        bridge_lidar3d_points,
        bridge_cmd_vel,
        bridge_odom,
        bridge_lidar,
        bridge_joint_states,
        bridge_clock,
        rviz_node
    ])
