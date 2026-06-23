#!/usr/bin/env python3
"""
Launch Braccio in Gazebo (gz-sim) with ROS 2 Control.

Why this changed (camera was showing "No Image" in RViz):
  * parameter_bridge from ros_gz_bridge publishes images with Reliable
    QoS by default.  RViz's Image display defaults to Reliable too, but
    this combination is widely known to mis-negotiate with gz_transport
    images -- RViz silently shows "No Image".
  * ros_gz_image's `image_bridge` uses image_transport, publishes with
    sensor_data QoS, and is the recommended way to expose Gazebo images
    to ROS.  rqt_image_view, RViz, and ros2 topic echo --qos-reliability
    best_effort all see it correctly.

So we now use TWO bridges:
  * parameter_bridge for /clock and /camera_info.
  * image_bridge     for the camera image stream.
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

    pkg_braccio_description = get_package_share_directory('braccio_description')
    pkg_braccio_gazebo = get_package_share_directory('braccio_gazebo')

    urdf_file = os.path.join(pkg_braccio_description, 'urdf', 'braccio.urdf.xacro')
    world_file = os.path.join(pkg_braccio_gazebo, 'worlds', 'braccio_sorting.world')
    rviz_config = os.path.join(
        pkg_braccio_description, 'rviz', 'braccio_with_camera.rviz'
    )

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

    world_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
    )

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

    # /clock + /camera_info via parameter_bridge.
    param_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='param_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/camera_info', '/camera/camera_info'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Image stream via ros_gz_image (uses image_transport, sensor_data QoS).
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge',
        output='screen',
        arguments=['/camera'],
        remappings=[
            ('/camera', '/camera/image_raw'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

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

    from launch.actions import TimerAction as _TimerAction
    after_spawn = _TimerAction(period=15.0, actions=[load_jsb])
    after_jsb = _TimerAction(period=18.0, actions=[load_arm])
    after_arm = _TimerAction(period=21.0, actions=[load_gripper])

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
        param_bridge,
        image_bridge,
        after_spawn,
        after_jsb,
        after_arm,
        rviz,
    ])