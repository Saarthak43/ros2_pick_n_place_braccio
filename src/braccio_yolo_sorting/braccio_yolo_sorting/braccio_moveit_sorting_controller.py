#!/usr/bin/env python3
"""
Braccio MoveIt Sorting Controller
Uses MoveIt for inverse kinematics and motion planning
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import GetPositionIK
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import math


class BraccioMoveItSortingController(Node):
    """MoveIt-based sorting controller with real IK."""
    
    def __init__(self):
        super().__init__('braccio_moveit_sorting_controller')
        
        # Parameters
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('detection_stable_time', 3.0)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('planning_group', 'arm')
        
        self.min_confidence = self.get_parameter('min_confidence').value
        self.stable_time = self.get_parameter('detection_stable_time').value
        self.auto_start = self.get_parameter('auto_start').value
        self.planning_group = self.get_parameter('planning_group').value
        
        # Action clients
        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )
        
        # MoveIt IK service client
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )
        
        self.get_logger().info('Waiting for controllers and MoveIt services...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        
        # Wait for IK service
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')
        
        self.get_logger().info('MoveIt and controllers ready!')
        
        # Joint names
        self.arm_joint_names = [
            'base_joint',
            'shoulder_joint',
            'elbow_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint'
        ]
        
        self.gripper_joint_names = ['gripper_joint']
        
        # Current joint state
        self.current_joint_state = None
        
        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Named positions (as backup)
        self.named_positions = {
            'home': [2.5, 2.8, 2.8, 2.8, 2.6],
            'scan': [2.5, 2.3, 2.0, 3.2, 2.6],
        }
        
        # Gripper positions
        self.gripper_open = 3.85
        self.gripper_closed = 2.7
        
        # Object positions (from camera - will be populated by detection)
        self.object_positions = {}
        
        # Container positions (Cartesian - in meters)
        self.container_positions = {
            'red': {'x': 0.30, 'y': 0.20, 'z': 0.08},
            'blue': {'x': 0.30, 'y': -0.10, 'z': 0.08}
        }
        
        # Camera parameters for pixel to 3D conversion
        self.camera_height = 0.60  # meters
        self.camera_x_offset = 0.30  # meters
        
        # Detection tracking
        self.detected_objects = []
        self.first_detection_time = None
        self.is_sorting = False
        
        # Subscriber
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )
        
        # Auto-start timer
        if self.auto_start:
            self.timer = self.create_timer(2.0, self.check_and_start)
        
        self.get_logger().info('='*60)
        self.get_logger().info('🤖 Braccio MoveIt Sorting Controller Ready!')
        self.get_logger().info('Using MoveIt for IK and Motion Planning')
        self.get_logger().info('='*60)
    
    def joint_state_callback(self, msg):
        """Store current joint state."""
        self.current_joint_state = msg
    
    def detection_callback(self, msg):
        """Process detections and store object positions."""
        if self.is_sorting:
            return
        
        valid_detections = []
        
        for detection in msg.detections:
            if len(detection.results) == 0:
                continue
            
            hypothesis = detection.results[0]
            confidence = hypothesis.hypothesis.score
            class_id = hypothesis.hypothesis.class_id
            
            if confidence >= self.min_confidence:
                # Get pixel position
                cx = detection.bbox.center.position.x
                cy = detection.bbox.center.position.y
                
                # Convert to 3D position
                pos_3d = self.pixel_to_3d(cx, cy)
                
                if pos_3d:
                    obj_data = {
                        'class_id': class_id,
                        'position': pos_3d,
                        'confidence': confidence
                    }
                    valid_detections.append(obj_data)
        
        if len(valid_detections) > 0:
            self.detected_objects = valid_detections
            if self.first_detection_time is None:
                self.first_detection_time = time.time()
                self.get_logger().info(f'Detected {len(valid_detections)} objects')
    
    def pixel_to_3d(self, pixel_x, pixel_y):
        """
        Convert pixel coordinates to 3D position.
        Simple projection - adjust based on your camera calibration.
        """
        # Camera intrinsics (adjust for your camera)
        image_width = 640
        image_height = 480
        
        # Normalize pixel coordinates
        norm_x = (pixel_x - image_width/2) / image_width
        norm_y = (pixel_y - image_height/2) / image_height
        
        # Simple projection (assumes camera looking down)
        # Adjust these scaling factors based on your setup
        x = self.camera_x_offset + norm_x * 0.3
        y = norm_y * 0.3
        z = 0.025  # Cube height
        
        return {'x': x, 'y': y, 'z': z}
    
    def compute_ik(self, target_pose):
        """
        Compute inverse kinematics using MoveIt.
        
        Args:
            target_pose: dict with 'x', 'y', 'z' positions
        
        Returns:
            List of joint positions or None
        """
        # Create IK request
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = self.planning_group
        
        # Set target pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'world'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        pose_stamped.pose.position.x = target_pose['x']
        pose_stamped.pose.position.y = target_pose['y']
        pose_stamped.pose.position.z = target_pose['z']
        
        # Orientation (gripper pointing down)
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.707
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 0.707
        
        ik_request.ik_request.pose_stamped = pose_stamped
        ik_request.ik_request.timeout.sec = 1
        ik_request.ik_request.attempts = 10
        
        # Use current state as seed
        if self.current_joint_state:
            ik_request.ik_request.robot_state.joint_state = self.current_joint_state
        
        # Call IK service
        try:
            future = self.ik_client.call_async(ik_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            response = future.result()
            
            if response and response.error_code.val == 1:  # SUCCESS
                # Extract joint positions
                joint_positions = []
                joint_state = response.solution.joint_state
                
                for joint_name in self.arm_joint_names:
                    try:
                        idx = joint_state.name.index(joint_name)
                        joint_positions.append(joint_state.position[idx])
                    except ValueError:
                        self.get_logger().error(f'Joint {joint_name} not found in IK solution')
                        return None
                
                return joint_positions
            else:
                self.get_logger().warn(f'IK failed with error code: {response.error_code.val}')
                return None
        
        except Exception as e:
            self.get_logger().error(f'IK service call failed: {str(e)}')
            return None
    
    def check_and_start(self):
        """Check if ready to start sorting."""
        if self.is_sorting or len(self.detected_objects) == 0:
            return
        
        if self.first_detection_time is None:
            return
        
        elapsed = time.time() - self.first_detection_time
        
        if elapsed >= self.stable_time:
            self.get_logger().info('Starting MoveIt-based sorting!')
            self.is_sorting = True
            self.sort_sequence()
            self.is_sorting = False
            self.detected_objects = []
            self.first_detection_time = None
    
    def send_arm_command(self, positions, duration_sec=3.0):
        """Send arm command."""
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec))
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joint_names
        goal_msg.trajectory.points = [point]
        
        future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is None:
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, 
                                        timeout_sec=duration_sec+2.0)
        return True
    
    def control_gripper(self, position):
        """Control gripper."""
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=1)
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.gripper_joint_names
        goal_msg.trajectory.points = [point]
        
        future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result():
            goal_handle = future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=2.0)
                return True
        return False
    
    def move_to_pose(self, target_pose, description=""):
        """Move to Cartesian pose using MoveIt IK."""
        self.get_logger().info(f'  Computing IK for: {description}')
        self.get_logger().info(f'  Target: x={target_pose["x"]:.3f}, y={target_pose["y"]:.3f}, z={target_pose["z"]:.3f}')
        
        joint_positions = self.compute_ik(target_pose)
        
        if joint_positions is None:
            self.get_logger().error(f'  IK failed for {description}')
            return False
        
        self.get_logger().info(f'  IK solution found, executing motion...')
        success = self.send_arm_command(joint_positions, duration_sec=3.0)
        time.sleep(3.5)
        
        return success
    
    def pick_object(self, obj_data, obj_number):
        """Pick object using MoveIt IK."""
        color = 'red' if 'red' in obj_data['class_id'] else 'blue'
        pos = obj_data['position']
        
        self.get_logger().info(f'\n📦 Picking {color.upper()} cube #{obj_number}')
        self.get_logger().info(f'   Detected at: x={pos["x"]:.3f}, y={pos["y"]:.3f}, z={pos["z"]:.3f}')
        
        # Open gripper
        self.control_gripper(self.gripper_open)
        time.sleep(1.0)
        
        # Approach (above object)
        approach_pose = {
            'x': pos['x'],
            'y': pos['y'],
            'z': pos['z'] + 0.10  # 10cm above
        }
        
        if not self.move_to_pose(approach_pose, f"{color} cube approach"):
            self.get_logger().error('Failed to reach approach position')
            return False
        
        # Grasp (at object)
        grasp_pose = {
            'x': pos['x'],
            'y': pos['y'],
            'z': pos['z']
        }
        
        if not self.move_to_pose(grasp_pose, f"{color} cube grasp"):
            self.get_logger().error('Failed to reach grasp position')
            return False
        
        # Close gripper
        self.control_gripper(self.gripper_closed)
        time.sleep(1.0)
        
        # Lift
        if not self.move_to_pose(approach_pose, "lift"):
            self.get_logger().error('Failed to lift object')
            return False
        
        self.get_logger().info(f'  ✅ {color.upper()} cube #{obj_number} picked!')
        return True
    
    def place_in_container(self, color):
        """Place in container using MoveIt IK."""
        self.get_logger().info(f'  📥 Placing in {color.upper()} container')
        
        container_pos = self.container_positions[color]
        
        # Move to container
        place_pose = {
            'x': container_pos['x'],
            'y': container_pos['y'],
            'z': container_pos['z']
        }
        
        if not self.move_to_pose(place_pose, f"{color} container"):
            self.get_logger().error('Failed to reach container')
            return False
        
        # Open gripper
        self.control_gripper(self.gripper_open)
        time.sleep(1.0)
        
        self.get_logger().info(f'  ✅ Placed in {color.upper()} container!')
        return True
    
    def move_to_named_position(self, name):
        """Move to named position."""
        if name not in self.named_positions:
            return False
        
        self.get_logger().info(f'Moving to: {name}')
        success = self.send_arm_command(self.named_positions[name], duration_sec=2.0)
        time.sleep(2.5)
        return success
    
    def sort_sequence(self):
        """Main sorting sequence using MoveIt."""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('🤖 BRACCIO MOVEIT SORTING STARTED')
        self.get_logger().info('   Using MoveIt IK for all movements')
        self.get_logger().info('='*60)
        
        # Home
        self.move_to_named_position('home')
        
        # Scan
        self.move_to_named_position('scan')
        
        # Sort objects
        red_count = 0
        blue_count = 0
        
        for obj_data in self.detected_objects:
            color = 'red' if 'red' in obj_data['class_id'] else 'blue'
            
            if color == 'red':
                red_count += 1
                success = self.pick_object(obj_data, red_count)
            else:
                blue_count += 1
                success = self.pick_object(obj_data, blue_count)
            
            if success:
                self.place_in_container(color)
            
            # Return to scan
            self.move_to_named_position('scan')
        
        # Home
        self.move_to_named_position('home')
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info(f'✅ MOVEIT SORTING COMPLETE!')
        self.get_logger().info(f'   Red cubes: {red_count}')
        self.get_logger().info(f'   Blue cubes: {blue_count}')
        self.get_logger().info('='*60 + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = BraccioMoveItSortingController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()