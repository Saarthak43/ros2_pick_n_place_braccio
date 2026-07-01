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

  ```python
    from launch.actions import TimerAction as _TimerAction

    # ================================================================
    # TODO (Part 5 — Gazebo/ROS Integration):
    # Connect the Gazebo simulation to the ROS 2 perception and
    # control stack.
    #
    # Step A — Parameter bridge
    # Create a ros_gz_bridge parameter_bridge node that bridges:
    #   - Gazebo /clock to rosgraph_msgs/msg/Clock
    #   - Gazebo /camera_info to sensor_msgs/msg/CameraInfo
    #
    # Remap:
    #   /camera_info  →  /camera/camera_info
    #
    # Step B — Image bridge
    # Create a ros_gz_image image_bridge node for the Gazebo camera
    # topic and remap:
    #   /camera  →  /camera/image_raw
    #
    # Step C — Controller activation
    # Create ExecuteProcess actions that activate:
    #   1. joint_state_broadcaster
    #   2. arm_controller
    #   3. gripper_controller
    #
    # Step D — Startup order
    # Wrap the three controller-loading actions in delayed launch
    # actions so controller_manager has time to become available.
    #
    # Required variable names:
    #   param_bridge
    #   image_bridge
    #   load_jsb
    #   load_arm
    #   load_gripper
    #   after_spawn
    #   after_jsb
    #   after_arm
    #
    # These names are already referenced by the LaunchDescription
    # returned at the bottom of this file.
    # ================================================================

    # ── YOUR CODE HERE ──────────────────────────────────────────────
    raise NotImplementedError(
        'Gazebo bridges and controller activation are not implemented yet'
    )
    # ─────────────────────────────────────────────────────────────────
```

## Keep unchanged

Do not remove or modify:

- `robot_state_publisher`
- `world_to_base`
- `gazebo`
- `spawn_robot`
- the RViz node
- the final `LaunchDescription` list

## Student completion condition

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once
ros2 control list_controllers
```

must confirm that the camera and controllers are available.

---
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
