# BraccioSort — Autonomous Vision-Guided Pick-and-Place System



## Problem Statement 

Modern manufacturing, warehousing, recycling, and packaging facilities process thousands of objects every day. These objects often arrive mixed together, placed at unpredictable positions, and belonging to different categories that must be separated before the next stage of production.

Traditional robotic arms can perform repetitive motions with high precision, but they usually depend on fixed coordinates and carefully arranged workspaces. A small change in the position of an object can make a completely pre-programmed sequence inaccurate or unusable.

For a robotic arm to operate autonomously, it must do more than simply move between predefined joint configurations. It must be able to:

observe the workspace → identify an object → determine its position → plan a valid motion → grasp it → place it correctly

This creates a complete perception-and-manipulation problem. Information detected in a two-dimensional camera image must be transformed into a three-dimensional position that the robotic arm can physically reach. The system must then calculate a valid arm configuration, approach the object without losing control, close the gripper, lift the object, and transport it to the correct destination.

Before such systems are deployed on real production lines, engineers require a simulation environment where object detection, coordinate estimation, inverse kinematics, trajectory execution, and grasping behaviour can be developed and tested safely.

This project develops a complete ROS 2 simulation of a Braccio robotic arm capable of visually detecting coloured objects, estimating their positions, picking them from the workspace, and sorting them into their corresponding containers using MoveIt and Gazebo.


## The Story 

The evening shift begins at a small automated sorting facility.

Mixed components from red and blue assembly lines lie scattered on a table. No positions are recorded, and nothing is aligned.

A Braccio robotic arm stands ready beneath a camera, given one task:

Identify every object and place it in the correct container.

The camera detects colored objects and estimates their positions. These image coordinates are converted into real-world locations the arm can reach.

The system then plans each movement: locating the object, calculating a reachable pose, and determining how to grasp it.

Using inverse kinematics, the arm moves, picks up each item, and sorts it—red to red, blue to blue.

After each placement, it returns to scan again.

Without manual input or predefined paths, the system continuously observes, plans, and acts until the workspace is fully organized.

The challenge is not just movement, but turning visual perception into precise physical action.


## Objective

Develop a complete ROS 2 software stack capable of autonomously detecting, locating, picking, and sorting objects using a simulated Braccio robotic arm.

The system must:

> Observe the workspace through an overhead camera
> Detect and classify objects from the camera feed
> Calculate each object’s image centroid
> Convert image coordinates into world coordinates
> Compute reachable arm configurations using inverse kinematics
> Execute arm and gripper trajectories
> Pick objects from dynamically determined positions
> Place each object inside its corresponding container
> Repeat the process until all detected objects have been sorted


## The Pipeline 
src
├── braccio_description
│   ├── CMakeLists.txt
│   ├── config
│   │   └── braccio_controllers.yaml
│   ├── launch
│   │   └── display.launch.py
│   ├── package.xml
│   ├── rviz
│   │   ├── braccio.rviz
│   │   ├── braccio_sorting.rviz
│   │   ├── braccio_with_camera.rviz
│   │   ├── moveit.rviz
│   │   └── urdf.rviz
│   ├── stl
│   │   ├── OAK-D.stl
│   │   ├── braccio_base.stl
│   │   ├── braccio_elbow.stl
│   │   ├── braccio_left_gripper.stl
│   │   ├── braccio_right_gripper.stl
│   │   ├── braccio_shoulder.stl
│   │   ├── braccio_wrist_pitch.stl
│   │   └── braccio_wrist_roll.stl
│   └── urdf
│       └── braccio.urdf.xacro
├── braccio_gazebo
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── braccio.launch.py
│   │   └── braccio_gazebo.launch.py
│   ├── package.xml
│   ├── scripts
│   │   └── activate_controllers.sh
│   └── worlds
│       └── braccio_sorting.world
├── braccio_moveit_config
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── braccio.ros2_control.xacro
│   │   ├── braccio.srdf
│   │   ├── braccio.urdf.xacro
│   │   ├── initial_positions.yaml
│   │   ├── joint_limits.yaml
│   │   ├── kinematics.yaml
│   │   ├── moveit.rviz
│   │   ├── moveit_controllers.yaml
│   │   ├── pilz_cartesian_limits.yaml
│   │   └── ros2_controllers.yaml
│   ├── launch
│   │   ├── demo.launch.py
│   │   ├── move_group.launch.py
│   │   ├── moveit_rviz.launch.py
│   │   ├── rsp.launch.py
│   │   ├── setup_assistant.launch.py
│   │   ├── spawn_controllers.launch.py
│   │   ├── static_virtual_joint_tfs.launch.py
│   │   └── warehouse_db.launch.py
│   └── package.xml
└── braccio_yolo_sorting
    ├── LICENSE
    ├── braccio_yolo_sorting
    │   ├── __init__.py
    │   ├── __pycache__
    │   │   ├── __init__.cpython-312.pyc
    │   │   ├── braccio_moveit_sorting_controller.cpython-312.pyc
    │   │   └── yolo_detector_node.cpython-312.pyc
    │   ├── braccio_moveit_sorting_controller.py
    │   └── yolo_detector_node.py
    ├── launch
    │   └── braccio_moveit_sorting.launch.py
    ├── package.xml
    ├── resource
    │   └── braccio_yolo_sorting
    ├── setup.cfg
    ├── setup.py
    └── test
        ├── test_copyright.py
        ├── test_flake8.py
        └── test_pep257.py
