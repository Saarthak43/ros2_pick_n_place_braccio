#!/usr/bin/env python3
"""Launch file to display Braccio in RViz."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    
    # Paths
    urdf_file = PathJoinSubstitution([
        FindPackageShare('braccio_description'),
        'urdf',
        'braccio.urdf.xacro'
    ])
    
    rviz_config = PathJoinSubstitution([
        FindPackageShare('braccio_description'),
        'rviz',
        'braccio.rviz'
    ])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_gui = LaunchConfiguration('use_gui', default='true')
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_file.perform(None)).read(),
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(use_gui)
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_gui', default_value='true'),
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])