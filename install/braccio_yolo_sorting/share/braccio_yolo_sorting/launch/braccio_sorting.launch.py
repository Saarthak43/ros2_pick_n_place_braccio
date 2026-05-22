#!/usr/bin/env python3
"""Complete Braccio YOLO sorting system launch file."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Launch Gazebo with Braccio
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('braccio_gazebo'),
                'launch',
                'braccio_gazebo.launch.py'
            ])
        ])
    )
    
    # YOLO Detector (delayed start)
    yolo_detector = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='braccio_yolo_sorting',
                executable='yolo_detector_node.py',
                name='yolo_detector',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'confidence_threshold': 0.4
                }]
            )
        ]
    )
    
    # Sorting Controller (delayed start)
    sorting_controller = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='braccio_yolo_sorting',
                executable='braccio_sorting_controller.py',
                name='sorting_controller',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'min_confidence': 0.5,
                    'detection_stable_time': 3.0,
                    'auto_start': True
                }]
            )
        ]
    )
    
    return LaunchDescription([
        gazebo_launch,
        yolo_detector,
        sorting_controller,
    ])