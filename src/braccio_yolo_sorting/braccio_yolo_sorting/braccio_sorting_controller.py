#!/usr/bin/env python3
"""
Braccio Sorting Controller - WITH tool_tip End Effector
Pre-tuned positions - No offset calculations needed!
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from vision_msgs.msg import Detection2DArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class BraccioSortingController(Node):
    """Braccio sorting with tool_tip as end effector."""
    
    def __init__(self):
        super().__init__('braccio_sorting_controller')
        
        # Parameters
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('detection_stable_time', 3.0)
        self.declare_parameter('auto_start', True)
        
        self.min_confidence = self.get_parameter('min_confidence').value
        self.stable_time = self.get_parameter('detection_stable_time').value
        self.auto_start = self.get_parameter('auto_start').value
        
        # Action clients
        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Waiting for Braccio controllers...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('Braccio controllers ready!')
        
        # Joint names
        self.arm_joint_names = [
            'base_joint',
            'shoulder_joint',
            'elbow_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint'
        ]
        
        self.gripper_joint_names = ['gripper_joint']
        
        # PRE-TUNED POSITIONS (tool_tip at exact position - no offsets!)
        # These positions place the tool_tip exactly where you want it
        self.positions = {
            # Standard positions
            'home': [2.5, 2.8, 2.8, 2.8, 2.6],
            'scan': [2.5, 2.3, 2.0, 3.2, 2.6],
            
            # Red cube positions (TUNE THESE IN SIMULATION)
            # tool_tip will be at exact cube position
            'red_1_approach': [3.0, 2.0, 2.3, 3.3, 2.6],
            'red_1_grasp': [3.0, 2.2, 2.5, 3.2, 2.6],
            'red_1_lift': [3.0, 1.9, 2.2, 3.3, 2.6],
            
            'red_2_approach': [3.2, 2.1, 2.4, 3.2, 2.6],
            'red_2_grasp': [3.2, 2.3, 2.6, 3.1, 2.6],
            'red_2_lift': [3.2, 2.0, 2.3, 3.2, 2.6],
            
            # Blue cube positions (TUNE THESE IN SIMULATION)
            'blue_1_approach': [2.0, 2.0, 2.3, 3.3, 2.6],
            'blue_1_grasp': [2.0, 2.2, 2.5, 3.2, 2.6],
            'blue_1_lift': [2.0, 1.9, 2.2, 3.3, 2.6],
            
            'blue_2_approach': [1.8, 2.1, 2.4, 3.2, 2.6],
            'blue_2_grasp': [1.8, 2.3, 2.6, 3.1, 2.6],
            'blue_2_lift': [1.8, 2.0, 2.3, 3.2, 2.6],
            
            # Container positions
            'red_container': [3.5, 2.3, 2.6, 3.0, 2.6],
            'blue_container': [1.5, 2.3, 2.6, 3.0, 2.6],
        }
        
        # Gripper positions
        self.gripper_open = 3.85
        self.gripper_closed = 2.7
        
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
        self.get_logger().info('Braccio Sorting Controller Ready!')
        self.get_logger().info('Using tool_tip as End Effector')
        self.get_logger().info('No offset calculations needed!')
        self.get_logger().info('='*60)
    
    def detection_callback(self, msg):
        """Store detections."""
        if self.is_sorting:
            return
        
        valid_detections = []
        for detection in msg.detections:
            if len(detection.results) == 0:
                continue
            
            hypothesis = detection.results[0]
            confidence = hypothesis.hypothesis.score
            
            if confidence >= self.min_confidence:
                valid_detections.append(hypothesis.hypothesis.class_id)
        
        if len(valid_detections) > 0:
            self.detected_objects = valid_detections
            if self.first_detection_time is None:
                self.first_detection_time = time.time()
                self.get_logger().info(f'Detected {len(valid_detections)} objects')
    
    def check_and_start(self):
        """Check if ready to start sorting."""
        if self.is_sorting or len(self.detected_objects) == 0:
            return
        
        if self.first_detection_time is None:
            return
        
        elapsed = time.time() - self.first_detection_time
        
        if elapsed >= self.stable_time:
            self.get_logger().info('Starting sorting sequence!')
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
    
    def move_to_position(self, position_name, duration=3.0):
        """Move to named position."""
        if position_name not in self.positions:
            self.get_logger().error(f'Unknown position: {position_name}')
            return False
        
        self.get_logger().info(f'  → {position_name}')
        success = self.send_arm_command(self.positions[position_name], duration)
        time.sleep(duration + 0.5)
        return success
    
    def pick_object(self, color, number):
        """Pick object (tool_tip goes to exact position)."""
        self.get_logger().info(f'\n📦 Picking {color.upper()} cube #{number}')
        
        # Open gripper
        self.control_gripper(self.gripper_open)
        time.sleep(1.0)
        
        # Approach (tool_tip above cube)
        self.move_to_position(f'{color}_{number}_approach', duration=3.0)
        
        # Grasp (tool_tip at cube height)
        self.move_to_position(f'{color}_{number}_grasp', duration=2.0)
        
        # Close gripper
        self.control_gripper(self.gripper_closed)
        time.sleep(1.0)
        
        # Lift
        self.move_to_position(f'{color}_{number}_lift', duration=2.0)
        
        return True
    
    def place_in_container(self, color):
        """Place in container."""
        self.get_logger().info(f'  📥 Placing in {color.upper()} container')
        
        # Move to container
        self.move_to_position(f'{color}_container', duration=3.0)
        
        # Open gripper
        self.control_gripper(self.gripper_open)
        time.sleep(1.0)
        
        return True
    
    def sort_sequence(self):
        """Main sorting sequence."""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('🤖 BRACCIO COLOR SORTING STARTED')
        self.get_logger().info('='*60)
        
        # Home
        self.move_to_position('home', duration=2.0)
        
        # Scan
        self.move_to_position('scan', duration=2.0)
        
        # Sort objects
        red_count = 0
        blue_count = 0
        
        for obj in self.detected_objects:
            if 'red' in obj:
                red_count += 1
                self.pick_object('red', red_count)
                self.place_in_container('red')
            elif 'blue' in obj:
                blue_count += 1
                self.pick_object('blue', blue_count)
                self.place_in_container('blue')
            
            # Return to scan
            self.move_to_position('scan', duration=2.0)
        
        # Home
        self.move_to_position('home', duration=2.0)
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info(f'✅ SORTING COMPLETE!')
        self.get_logger().info(f'   Red cubes: {red_count}')
        self.get_logger().info(f'   Blue cubes: {blue_count}')
        self.get_logger().info('='*60 + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = BraccioSortingController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()