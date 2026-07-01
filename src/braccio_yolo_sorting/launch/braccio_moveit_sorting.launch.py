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

 # ================================================================
    # TODO (Part 10 — Complete System Bringup):
    # Launch and connect the complete autonomous sorting stack.
    #
    # Create the following five launch actions:
    #
    # 1. gazebo_launch
    #    - Include braccio_gazebo/launch/braccio_gazebo.launch.py.
    #    - Pass rviz:=false because this launch file starts its own
    #      sorting-specific RViz instance.
    #
    # 2. moveit_launch
    #    - Include braccio_moveit_config/launch/move_group.launch.py.
    #    - Enable simulation time.
    #    - Start it after Gazebo, TF, and the controllers have had
    #      enough time to initialize.
    #
    # 3. yolo_detector
    #    - Start yolo_detector_node.py.
    #    - Enable simulation time.
    #    - Set image_topic to /camera/image_raw.
    #    - Pass the confidence-threshold parameter.
    #    - Start it after the camera bridge is publishing images.
    #
    # 4. moveit_sorting_controller
    #    - Start braccio_moveit_sorting_controller.py.
    #    - Enable simulation time.
    #    - Configure:
    #        min_confidence
    #        detection_stable_time
    #        auto_start
    #        planning_group = arm
    #        planning_frame = world
    #        camera_fx
    #        camera_fy
    #    - Use the calibrated focal values required by the current
    #      simulation camera.
    #    - Start it only after MoveIt's /compute_ik service exists.
    #
    # 5. rviz
    #    - Load braccio_description/rviz/braccio_sorting.rviz.
    #    - Enable simulation time.
    #
    # Recommended startup order:
    #
    #   Gazebo immediately
    #        ↓
    #   RViz after a short delay
    #        ↓
    #   MoveIt
    #        ↓
    #   HSV detector
    #        ↓
    #   sorting controller
    #
    # Return one LaunchDescription containing all five actions.
    #
    # Required variable names:
    #   gazebo_launch
    #   moveit_launch
    #   yolo_detector
    #   moveit_sorting_controller
    #   rviz
    # ================================================================

    # ── YOUR CODE HERE ──────────────────────────────────────────────
    raise NotImplementedError(
        'Complete Braccio sorting bringup is not implemented yet'
    )
    # ─────────────────────────────────────────────────────────────────
```

## Student completion condition

The following command must bring up the complete system:

```bash
ros2 launch braccio_yolo_sorting braccio_moveit_sorting.launch.py
```

A correct implementation should start:

- Gazebo and the sorting world
- the Braccio robot
- camera bridges
- arm and gripper controllers
- MoveIt `move_group`
- the HSV detector
- the autonomous sorting controller
- sorting RViz
