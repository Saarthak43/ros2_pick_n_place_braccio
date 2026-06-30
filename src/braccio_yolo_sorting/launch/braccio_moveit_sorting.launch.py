#!/usr/bin/env python3
"""
Complete Braccio MoveIt YOLO sorting launch file.

Brings up, in order:

  1. Gazebo + bridges + controllers + robot_state_publisher
     (we pass `rviz:=false` here because we start our own RViz below
     with a sorting-specific layout that shows the YOLO annotated feed).
  2. MoveIt move_group (waits 5 s so controllers + TF are ready).
  3. The YOLO HSV detector (waits 10 s so the camera bridge has data).
  4. The sorting controller (waits 15 s so MoveIt's /compute_ik is up).
  5. RViz with the sorting config -- two image panels (raw + annotated)
     plus the robot model and TF.

The previous timing (3 / 8 / 12 s) was too tight: gz-sim takes ~6-8 s
just to publish the first camera frame, so the sorter used to come up
before either the camera bridge or /compute_ik existed and would log
"Waiting for /compute_ik..." for the rest of the session.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # 1. Gazebo + bridges + controllers (RViz disabled - we open our own).
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('braccio_gazebo'),
                'launch', 'braccio_gazebo.launch.py',
            ])
        ]),
        launch_arguments={'rviz': 'false'}.items(),
    )

    # 2. MoveIt move_group.
    moveit_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('braccio_moveit_config'),
                        'launch', 'move_group.launch.py',
                    ])
                ]),
                launch_arguments={'use_sim_time': 'true'}.items(),
            )
        ],
    )

    # 3. YOLO HSV detector.  Explicitly bound to /camera/image_raw -- the
    #    name the gazebo launch's bridge remaps the gz `camera` topic to.
    yolo_detector = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='braccio_yolo_sorting',
                executable='yolo_detector_node.py',
                name='yolo_detector',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'confidence_threshold': 0.4,
                    'image_topic': '/camera/image_raw',
                }],
            )
        ],
    )

    # 4. MoveIt sorting controller.
    moveit_sorting_controller = TimerAction(
        period=15.0,
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
                    'planning_group': 'arm',
                    'planning_frame': 'world',
                    # BUG FIX C: default 554.4 (HFOV formula) was wrong.
                    # Empirically derived from Gazebo ground truth: fx=fy=304.
                    # Wrong value caused ~55 mm XY error in pixel→world.
                    'camera_fx': 304.0,
                    'camera_fy': 304.0,
                }],
            )
        ],
    )

    # 5. RViz with the sorting layout (raw camera + annotated detections).
    rviz_config = PathJoinSubstitution([
        FindPackageShare('braccio_description'),
        'rviz', 'braccio_sorting.rviz',
    ])
    rviz = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
            )
        ],
    )

    return LaunchDescription([
        gazebo_launch,
        moveit_launch,
        yolo_detector,
        moveit_sorting_controller,
        rviz,
    ])