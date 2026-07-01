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
```text
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
```

## System Overview 


The project consists of six major components.

### 1. Braccio Robot Model

The complete robot structure is defined in `braccio_description/urdf/braccio.urdf.xacro`, including the arm links, revolute joints, gripper fingers, collision geometry, visual meshes, joint limits, and Gazebo control interfaces.

The controller configuration in `braccio_description/config/braccio_controllers.yaml` connects the simulated arm and gripper joints to ROS 2 trajectory controllers, allowing both manipulators to receive executable joint commands.

### 2. Gazebo Sorting Environment

`braccio_gazebo/worlds/braccio_sorting.world` creates the simulated sorting cell containing the worktable, coloured cubes, destination containers, lighting, physics, and the overhead RGB camera used to observe the workspace.

`braccio_gazebo/launch/braccio_gazebo.launch.py` starts Gazebo, spawns the Braccio model, activates the arm and gripper controllers, and bridges the simulated camera feed into ROS 2.

### 3. Vision-Based Object Detection

`braccio_yolo_sorting/braccio_yolo_sorting/yolo_detector_node.py` processes images from the overhead camera and identifies red and blue objects using HSV colour segmentation, morphological filtering, contour detection, and bounding-box validation.

The centre of every accepted bounding box is published as a `Detection2DArray`, providing the object class and its position within the camera image.

### 4. MoveIt Kinematics Configuration

The `braccio_moveit_config` package describes how MoveIt understands and controls the arm.

`braccio.srdf` defines the arm and gripper planning groups, while `kinematics.yaml` configures the inverse-kinematics solver. `joint_limits.yaml`, `moveit_controllers.yaml`, and `ros2_controllers.yaml` define the motion limits and controller interfaces used during execution.

Together, these files allow the system to convert a requested end-effector position into a reachable Braccio joint configuration.

### 5. Autonomous Pick-and-Place Controller

`braccio_yolo_sorting/braccio_yolo_sorting/braccio_moveit_sorting_controller.py` coordinates the complete sorting operation.

It stabilizes incoming detections, converts image centroids into estimated world coordinates, requests inverse-kinematics solutions from MoveIt, and sends joint trajectories to the arm and gripper controllers.

The node then executes the full sequence:

```text
Detect Object
      ↓
Estimate World Position
      ↓
Move Above Object
      ↓
Open and Lower Gripper
      ↓
Grasp and Lift Object
      ↓
Move to Colour-Specific Container
      ↓
Release Object
      ↓
Return to Scanning Position
```

Red objects are transported to the red destination, while blue objects are transported to the blue destination.

### 6. Complete System Bringup

`braccio_yolo_sorting/launch/braccio_moveit_sorting.launch.py` acts as the main system launcher.

It brings together:

* The Braccio robot description
* Gazebo and the sorting world
* ROS 2 arm and gripper controllers
* The camera image bridge
* MoveIt and its inverse-kinematics service
* `yolo_detector_node.py`
* `braccio_moveit_sorting_controller.py`
* RViz visualization

This launch file connects perception, kinematics, simulation, and manipulation into one autonomous vision-guided sorting pipeline.

```bash 
ros2 launch braccio_yolo_sorting braccio_moveit_sorting.launch.py
```



## What You Need To Implement

This repository contains **11 TODOs distributed across seven files**. Together, they cover the complete perception-and-manipulation pipeline: Gazebo integration, visual detection, coordinate estimation, inverse kinematics, trajectory control, and autonomous object sorting.

Complete the TODOs in the order shown below. Each stage provides an interface required by the next one.

---

### 1. ROS 2 Arm and Gripper Controllers

**File:**
`src/braccio_description/config/braccio_controllers.yaml`

#### TODO 1 — Configure the Braccio Trajectory Controllers

Configure `arm_controller` for the five arm joints and `gripper_controller` for `gripper_joint`.

Your configuration must:

* Use `joint_trajectory_controller/JointTrajectoryController`
* Include all five Braccio arm joints in the correct order
* Configure the gripper controller for `gripper_joint`
* Use position command interfaces
* Expose position and velocity state interfaces
* Define suitable goal-time and stopped-velocity tolerances
* Preserve the controller names expected by the sorting controller

These controllers must provide:

```text
/arm_controller/follow_joint_trajectory
/gripper_controller/follow_joint_trajectory
```

Without these action servers, the sorting controller cannot move either the arm or the gripper.

---

### 2. Gazebo Sensor and Controller Bringup

**File:**
`src/braccio_gazebo/launch/braccio_gazebo.launch.py`

#### TODO 1 — Connect Gazebo to ROS 2 Perception and Control

Complete the Gazebo launch integration required by the rest of the system.

Your implementation must:

* Bridge simulation time through `/clock`
* Bridge the camera information topic
* Bridge the Gazebo RGB image topic into ROS 2
* Expose the image as `/camera/image_raw`
* Start `joint_state_broadcaster`
* Start `arm_controller`
* Start `gripper_controller`
* Activate controllers only after the robot and controller manager are available

This completes the connection:

```text
Gazebo Physics
      ↓
Braccio Robot Model
      ↓
ros2_control
      ↓
Arm and Gripper Trajectory Controllers
```

It also provides the camera images required by the object detector.

---

### 3. MoveIt Semantic Robot Configuration

**File:**
`src/braccio_moveit_config/config/braccio.srdf`

#### TODO 1 — Define the Arm, Gripper, and End Effector

Complete the semantic description used by MoveIt.

Your implementation must define:

* An `arm` planning group
* A kinematic chain from `base_link` to `tool_tip`
* A `gripper` planning group containing `gripper_joint`
* The gripper as an end effector attached to `tool_tip`
* `sub_gripper_joint` as a passive joint
* The existing `home` state for the arm group

The planning-group name must remain `arm` because this is the group requested by `braccio_moveit_sorting_controller.py`.

Without the correct semantic groups, MoveIt cannot determine which joints belong to the arm or calculate an inverse-kinematics solution for `tool_tip`.

---

### 4. MoveIt Inverse-Kinematics Configuration

**File:**
`src/braccio_moveit_config/config/kinematics.yaml`

#### TODO 1 — Configure TRAC-IK for the Arm

Configure the inverse-kinematics solver for the `arm` planning group.

Your configuration must specify:

* The TRAC-IK kinematics plugin
* A suitable search resolution
* A practical solver timeout
* The number of solver attempts
* Position-only inverse kinematics

Position-only IK is used because the sorting controller primarily requests reachable tool positions above objects and containers.

This completes the connection:

```text
Requested Tool Position
      ↓
MoveIt /compute_ik
      ↓
TRAC-IK
      ↓
Five Braccio Joint Positions
```

---

### 5. HSV-Based Object Detection

**File:**
`src/braccio_yolo_sorting/braccio_yolo_sorting/yolo_detector_node.py`

Despite the filename, this node uses HSV colour segmentation rather than neural-network YOLO inference.

#### TODO 1 — Generate Clean Red and Blue Masks

Implement the mask-generation stage inside `detect_objects_hsv()`.

Your implementation must:

* Convert the incoming BGR image into HSV
* Generate both red hue-range masks
* Combine the two red masks
* Generate the blue mask
* Apply morphological closing to fill small holes
* Apply morphological opening to remove isolated noise
* Produce one cleaned binary mask for each colour

Red requires two hue ranges because it lies at both ends of OpenCV’s HSV hue scale.

#### TODO 2 — Extract and Validate Cube Detections

Process the cleaned masks and convert suitable contours into detections.

Your implementation must:

* Find external contours
* Reject small noise contours
* Calculate contour area
* Generate bounding rectangles
* Reject regions that are too large to be cubes
* Reject destination-container-sized regions
* Reject detections outside the useful vertical image region
* Reject extreme aspect ratios
* Append the accepted bounding box, colour, and confidence

Return detections in the format expected by `image_callback()`:

```python
{
    'bbox': [x_min, y_min, x_max, y_max],
    'color': 'red' or 'blue',
    'confidence': confidence_value,
}
```

The existing callback will convert these results into `Detection2DArray` messages and publish the annotated image.

---

### 6. Autonomous MoveIt Sorting Controller

**File:**
`src/braccio_yolo_sorting/braccio_yolo_sorting/braccio_moveit_sorting_controller.py`

This file contains the four central manipulation TODOs.

#### TODO 1 — Calibrate the Container Drop Configurations

Find five-joint arm configurations that position the gripper above the red and blue destination containers.

Your implementation must:

* Test candidate arm configurations
* Observe the resulting pose in Gazebo and RViz
* Verify the `tool_tip` position using `/compute_fk`
* Store the final configurations as `drop_red` and `drop_blue`
* Ensure that each configuration is reachable without colliding with the table

The configurations must follow this joint order:

```text
base_joint
shoulder_joint
elbow_joint
wrist_pitch_joint
wrist_roll_joint
```

This is a calibration task. The final values should be obtained from the simulated robot rather than copied blindly.

#### TODO 2 — Convert Image Pixels into World Coordinates

Implement `_pixel_to_world()`.

The detector provides an object centroid as image coordinates `(u, v)`. Convert this point into an estimated object position in the world frame.

Use:

* Camera world position
* Camera principal point
* Camera focal lengths
* Known cube height
* Camera-to-world axis mapping
* Pinhole back-projection

For this camera orientation:

```text
Image columns → World Y
Image rows    → World X
```

Return:

```python
{
    'x': world_x,
    'y': world_y,
    'z': cube_world_z,
}
```

This completes the perception transformation:

```text
Bounding-Box Centre
      ↓
Pixel Coordinates
      ↓
Camera Projection
      ↓
Estimated World Position
```

#### TODO 3 — Compute a Valid Inverse-Kinematics Solution

Implement `_compute_ik()`.

Your implementation must:

* Construct a `GetPositionIK` request
* Use the `arm` planning group
* Express the target in the `world` frame
* Calculate a suitable seed for `base_joint`
* Create a complete seed state for all five arm joints
* Reuse the previous IK result when available
* Call `/compute_ik` asynchronously
* Wait for the response with a timeout
* Handle service failures and unsuccessful MoveIt error codes
* Match returned positions using joint names
* Return positions in controller joint order
* Store successful solutions for future warm starts

The function must return five joint positions when successful and `None` when no valid solution is found.

#### TODO 4 — Coordinate the Complete Sorting Mission

Implement `_sort_sequence()`.

The complete sequence must:

1. Move the arm to `home`
2. Move the arm to `scan`
3. Iterate through all detected objects
4. Determine whether each object is red or blue
5. Call `_pick()` for the selected object
6. Call `_place()` only after a successful pick
7. Place red objects at `drop_red`
8. Place blue objects at `drop_blue`
9. Continue safely after individual failures
10. Return to `scan` after every attempt
11. Return to `home` after all objects have been processed
12. Report successful and failed attempts for both colours

Use the provided helper functions:

```text
_move_named()
_pick()
_place()
```

Do not rewrite the action-client and trajectory infrastructure already provided by the repository.

---

### 7. Complete Sorting-System Bringup

**File:**
`src/braccio_yolo_sorting/launch/braccio_moveit_sorting.launch.py`

#### TODO 1 — Launch and Configure the Complete Autonomy Stack

Bring all subsystems together in the correct order.

Your implementation must:

* Start Gazebo and ros2_control first
* Start MoveIt after robot state and controllers are available
* Start the detector after camera images are available
* Start the sorting controller after `/compute_ik` is available
* Configure `/camera/image_raw` as the detector input
* Configure `arm` as the planning group
* Configure `world` as the planning frame
* Pass the confidence and stabilization parameters
* Pass the calibrated camera focal lengths
* Start RViz with `braccio_sorting.rviz`

The completed launch pipeline should be:

```text
Gazebo and Robot
      ↓
ROS 2 Controllers and Camera Bridge
      ↓
MoveIt move_group
      ↓
HSV Object Detector
      ↓
Autonomous Sorting Controller
      ↓
RViz Visualization
```

Fixed startup delays are acceptable for this assignment, but they must leave enough time for Gazebo, the camera bridge, controllers, and MoveIt to initialize.

---

## Recommended Implementation Order

```text
1. braccio_controllers.yaml
           ↓
2. braccio_gazebo.launch.py
           ↓
3. braccio.srdf
           ↓
4. kinematics.yaml
           ↓
5. yolo_detector_node.py
           ↓
6. Drop-pose calibration
           ↓
7. _pixel_to_world()
           ↓
8. _compute_ik()
           ↓
9. _sort_sequence()
           ↓
10. braccio_moveit_sorting.launch.py
```

Do not begin by running the complete sorting system. Test every layer independently before moving to the next one.

---

## Testing Your Implementation

Build the workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

### Verify the Controllers

```bash
ros2 control list_controllers
ros2 action list
```

The joint-state broadcaster, arm controller, and gripper controller should be active.

### Verify the Camera Pipeline

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once
```

### Verify Object Detection

```bash
ros2 topic echo /detections
```

The detector should report the red and blue cubes without treating the destination containers as cubes.

### Verify MoveIt

```bash
ros2 service list | grep compute_ik
ros2 service list | grep compute_fk
```

### Run the Complete System

```bash
ros2 launch braccio_yolo_sorting braccio_moveit_sorting.launch.py
```

A successful implementation should:

1. Spawn the Braccio arm and sorting environment
2. Publish camera images
3. Activate the arm and gripper controllers
4. Detect both coloured cubes
5. Estimate stable world coordinates
6. Obtain valid IK solutions
7. Approach and grasp each cube
8. Move it to the matching container
9. Return to the scanning position after every attempt
10. Finish at home and print a sorting summary





