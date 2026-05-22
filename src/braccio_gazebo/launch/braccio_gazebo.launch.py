#!/usr/bin/env python3
"""Launch Braccio in Gazebo with ROS 2 Control - CORRECT PATHS."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    
    # Package paths
    pkg_braccio_description = get_package_share_directory('braccio_description')
    pkg_braccio_gazebo = get_package_share_directory('braccio_gazebo')
    
    # Get the install prefix (where package is installed)
    install_dir = get_package_prefix('braccio_description')
    
    # CRITICAL: Point to the share directory where packages are installed
    # This is where Gazebo will look for package:// URIs
    gz_resource_path = os.path.join(install_dir, 'share')
    
    print("="*60)
    print(f"Setting GZ_SIM_RESOURCE_PATH to: {gz_resource_path}")
    print(f"STL files should be at: {pkg_braccio_description}/stl/")
    print("="*60)
    
    # Check if path exists
    stl_path = os.path.join(pkg_braccio_description, 'stl')
    if os.path.exists(stl_path):
        print(f"✅ STL directory found: {stl_path}")
        stl_files = os.listdir(stl_path)
        print(f"   STL files: {stl_files}")
    else:
        print(f"❌ WARNING: STL directory NOT found at: {stl_path}")
    print("="*60)
    
    # Add to existing path if it exists
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path = os.environ['GZ_SIM_RESOURCE_PATH'] + ':' + gz_resource_path
    
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )
    
    # Also set IGN_GAZEBO_RESOURCE_PATH (for compatibility with older versions)
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=gz_resource_path
    )
    
    # File paths
    urdf_file = os.path.join(pkg_braccio_description, 'urdf', 'braccio.urdf.xacro')
    world_file = os.path.join(pkg_braccio_gazebo, 'worlds', 'braccio_sorting.world')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_braccio',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'braccio',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-allow-renaming', 'true'
        ],
        output='screen'
    )
    
    # Bridge between Gazebo and ROS2
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )
    
    # Load joint state broadcaster controller
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    # Load arm controller
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )
    
    # Load gripper controller
    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_controller'],
        output='screen'
    )
    
    # Event handlers for sequential controller loading
    load_arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller]
        )
    )
    
    load_gripper_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_arm_controller,
            on_exit=[load_gripper_controller]
        )
    )
    
    # Delay controller loading until robot is spawned
    load_controllers_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_joint_state_broadcaster]
        )
    )
    
    return LaunchDescription([
        # Set resource paths FIRST
        set_gz_resource_path,
        set_ign_resource_path,
        
        # Then launch everything else
        use_sim_time_arg,
        robot_state_publisher,
        gazebo,
        gz_ros2_bridge,
        spawn_robot,
        load_controllers_event,
        load_arm_controller_event,
        load_gripper_controller_event,
    ])