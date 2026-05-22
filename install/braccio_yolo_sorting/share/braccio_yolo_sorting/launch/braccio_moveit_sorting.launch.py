#!/usr/bin/env python3
"""Complete Braccio MoveIt YOLO sorting launch file."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # 1. Launch Gazebo with Braccio
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('braccio_gazebo'),
                'launch',
                'braccio_gazebo.launch.py'
            ])
        ])
    )
    
    # 2. Launch MoveIt move_group
    moveit_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('braccio_moveit_config'),
                        'launch',
                        'move_group.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': 'true'
                }.items()
            )
        ]
    )
    
    # 3. YOLO Detector
    yolo_detector = TimerAction(
        period=8.0,
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
    
    # 4. MoveIt Sorting Controller
    moveit_sorting_controller = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='braccio_yolo_sorting',
                executable='braccio_moveit_sorting_controller.py',
                name='moveit_sorting_controller',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'min_confidence': 0.5,
                    'detection_stable_time': 3.0,
                    'auto_start': True,
                    'planning_group': 'arm'
                }]
            )
        ]
    )
    
    return LaunchDescription([
        gazebo_launch,
        moveit_launch,
        yolo_detector,
        moveit_sorting_controller,
    ])