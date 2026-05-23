#!/usr/bin/env python3
"""
Launch Braccio in Gazebo (gz-sim) with ROS 2 Control.

This launch file:
  * Sets GZ_SIM_RESOURCE_PATH so Gazebo can resolve `package://` URIs to the
    braccio_description STL meshes.
  * Spawns Gazebo with the sorting world.
  * Spawns the robot from /robot_description.
  * Bridges essential topics:
      - /clock           (gz -> ros) so use_sim_time works for MoveIt etc.
      - /camera          (gz -> ros) remapped to /camera/image_raw so the
                         YOLO detector and RViz can subscribe to it.
      - /camera_info     (gz -> ros) remapped to /camera/camera_info.
  * Publishes a static transform world -> base_link.  MoveIt's SRDF declares
    a virtual joint with parent_frame="world", so this transform MUST exist or
    every MoveIt IK / planning call will fail with FRAME_TRANSFORM_FAILURE.
  * Loads the joint_state_broadcaster, arm_controller and gripper_controller
    sequentially AFTER the robot is spawned (so controller_manager exists).
  * Opens RViz with the braccio_with_camera.rviz config.

To disable RViz pass `rviz:=false`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ---------------------------------------------------------------- paths
    pkg_braccio_description = get_package_share_directory('braccio_description')
    pkg_braccio_gazebo = get_package_share_directory('braccio_gazebo')

    urdf_file = os.path.join(pkg_braccio_description, 'urdf', 'braccio.urdf.xacro')
    world_file = os.path.join(pkg_braccio_gazebo, 'worlds', 'braccio_sorting.world')
    rviz_config = os.path.join(
        pkg_braccio_description, 'rviz', 'braccio_with_camera.rviz'
    )

    # GZ_SIM_RESOURCE_PATH: walk up to the workspace `share` directory so
    # `package://braccio_description/stl/...` resolves correctly.
    install_share = os.path.dirname(pkg_braccio_description)
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path = os.environ['GZ_SIM_RESOURCE_PATH'] + ':' + install_share
    else:
        gz_resource_path = install_share

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path
    )
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', value=gz_resource_path
    )

    # ------------------------------------------------------------ arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock from Gazebo.'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz with the braccio_with_camera config.'
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value=world_file,
        description='Full path to the .world / .sdf file to load.'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ------------------------------------------------------ robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0,
        }],
    )

    # MoveIt's SRDF uses `world` as the planning frame (virtual_joint parent),
    # but our URDF root is `base_link`.  Publish the link so TF is complete.
    world_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
    )

    # --------------------------------------------------------------- gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_braccio',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'braccio',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
            '-allow-renaming', 'true',
        ],
    )

    # ------------------------------------------------------------- bridge
    # clock + camera (gz -> ros) with remap so /camera becomes /camera/image_raw
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/camera', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------ controllers
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen',
    )
    load_arm = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'arm_controller'],
        output='screen',
    )
    load_gripper = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'gripper_controller'],
        output='screen',
    )

    # Sequence:  spawn -> jsb -> arm -> gripper
    after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[load_jsb])
    )
    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=load_jsb, on_exit=[load_arm])
    )
    after_arm = RegisterEventHandler(
        OnProcessExit(target_action=load_arm, on_exit=[load_gripper])
    )

    # ---------------------------------------------------------------- rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        set_gz_resource_path,
        set_ign_resource_path,
        use_sim_time_arg,
        rviz_arg,
        world_arg,
        robot_state_publisher,
        world_to_base,
        gazebo,
        spawn_robot,
        gz_ros_bridge,
        after_spawn,
        after_jsb,
        after_arm,
        rviz,
    ])