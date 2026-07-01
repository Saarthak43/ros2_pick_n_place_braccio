# BraccioSort — Autonomous Vision-Guided Pick-and-Place System

## Problem Statement 

Modern manufacturing, warehousing, recycling, and packaging facilities process thousands of objects every day. These objects often arrive mixed together, placed at unpredictable positions, and belonging to different categories that must be separated before the next stage of production.

Traditional robotic arms can perform repetitive motions with high precision, but they usually depend on fixed coordinates and carefully arranged workspaces. A small change in the position of an object can make a completely pre-programmed sequence inaccurate or unusable.

For a robotic arm to operate autonomously, it must do more than simply move between predefined joint configurations. It must be able to:

observe the workspace → identify an object → determine its position → plan a valid motion → grasp it → place it correctly

This creates a complete perception-and-manipulation problem. Information detected in a two-dimensional camera image must be transformed into a three-dimensional position that the robotic arm can physically reach. The system must then calculate a valid arm configuration, approach the object without losing control, close the gripper, lift the object, and transport it to the correct destination.

Before such systems are deployed on real production lines, engineers require a simulation environment where object detection, coordinate estimation, inverse kinematics, trajectory execution, and grasping behaviour can be developed and tested safely.

This project develops a complete ROS 2 simulation of a Braccio robotic arm capable of visually detecting coloured objects, estimating their positions, picking them from the workspace, and sorting them into their corresponding containers using MoveIt and Gazebo.
