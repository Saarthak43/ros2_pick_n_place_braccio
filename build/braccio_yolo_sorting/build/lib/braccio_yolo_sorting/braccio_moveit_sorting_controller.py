#!/usr/bin/env python3
"""
Braccio MoveIt Sorting Controller.

Uses MoveIt's /compute_ik service for inverse kinematics and the
arm_controller / gripper_controller FollowJointTrajectory action servers
for execution.

Pixel → World projection
------------------------
The camera in braccio.urdf.xacro is mounted at world (0.30, 0.00, 0.59)
and looks straight down.  With the ROS optical-frame convention (Z forward,
X right, Y down) and the camera_optical_joint transform, the projection for
a point on a flat surface at known world Z is:

    depth   = camera_world_z - cube_world_z          # 0.59 - 0.27 = 0.32 m
    world_x = camera_world_x + (v - cy) * depth / fy
    world_y = camera_world_y + (u - cx) * depth / fx

where (u, v) is the pixel centroid of the detected bounding box and
(fx, fy, cx, cy) are derived from the URDF sensor tag:

    HFOV = 1.047 rad, image 640×480
    fx = fy = (320) / tan(1.047/2) ≈ 554.4
    cx = 320.0, cy = 240.0

All camera parameters are exposed as ROS parameters so they can be
overridden from the launch file without touching the source.

Key fixes vs earlier revisions
--------------------------------
* No ik_request.attempts — field does not exist in ROS 2 moveit_msgs.
* Sort sequence runs on a worker thread to avoid single-threaded executor
  deadlock.
* Event-based waits for service / action responses.
* Dead _pixel_to_3d helper (which used hardcoded cube positions) removed.
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from vision_msgs.msg import Detection2DArray


class BraccioMoveItSortingController(Node):
    """MoveIt-based sorting controller with real IK and live pixel→world."""

    def __init__(self):
        super().__init__('braccio_moveit_sorting_controller')

        # ------------------------------------------------------- parameters
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('detection_stable_time', 3.0)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('planning_frame', 'world')

        # Camera intrinsics — derived from URDF HFOV=1.047 rad, 640×480.
        # Override from launch file if the camera setup changes.
        self.declare_parameter('camera_fx', 554.4)
        self.declare_parameter('camera_fy', 554.4)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)

        # Camera world pose (from URDF camera_joint xyz).
        self.declare_parameter('camera_world_x', 0.30)
        self.declare_parameter('camera_world_y', 0.00)
        self.declare_parameter('camera_world_z', 0.59)

        # Known height of the surface the cubes rest on (world Z).
        self.declare_parameter('cube_world_z', 0.27)

        self.min_confidence  = self.get_parameter('min_confidence').value
        self.stable_time     = self.get_parameter('detection_stable_time').value
        self.auto_start      = self.get_parameter('auto_start').value
        self.planning_group  = self.get_parameter('planning_group').value
        self.planning_frame  = self.get_parameter('planning_frame').value

        self.cam_fx = self.get_parameter('camera_fx').value
        self.cam_fy = self.get_parameter('camera_fy').value
        self.cam_cx = self.get_parameter('camera_cx').value
        self.cam_cy = self.get_parameter('camera_cy').value
        self.cam_wx = self.get_parameter('camera_world_x').value
        self.cam_wy = self.get_parameter('camera_world_y').value
        self.cam_wz = self.get_parameter('camera_world_z').value
        self.cube_z = self.get_parameter('cube_world_z').value

        self.get_logger().info(
            f'Camera params: fx={self.cam_fx:.1f} fy={self.cam_fy:.1f} '
            f'cx={self.cam_cx:.1f} cy={self.cam_cy:.1f} | '
            f'cam_world=({self.cam_wx},{self.cam_wy},{self.cam_wz}) '
            f'cube_z={self.cube_z}'
        )

        # ---------------------------------------------------------- clients
        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
        )
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        self.get_logger().info('Waiting for controllers and MoveIt services...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik...')
        self.get_logger().info('MoveIt and controllers ready.')

        # ------------------------------------------------------- joint info
        self.arm_joint_names = [
            'base_joint',
            'shoulder_joint',
            'elbow_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
        ]
        self.gripper_joint_names = ['gripper_joint']

        # ------------------------------------------------------ joint state
        self.current_joint_state = None
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, 10,
        )

        # ---------------------------------------------------------- presets
        self.named_positions = {
            'home':       [2.5,   2.8,   2.8,   2.8,   2.6],
            'scan':       [2.5,   2.3,   2.0,   3.2,   2.6],
            'drop_red':   [3.28,  2.5,   3.8,   1.8,   2.6],
            'drop_blue':  [1.72,  2.5,   3.8,   1.8,   2.6],
        }
        self.gripper_open   = 3.85
        self.gripper_closed = 2.7

        # ------------------------------------------------- detection state
        self.detected_objects      = []
        self.first_detection_time  = None
        self.is_sorting            = False
        self._sort_lock            = threading.Lock()
        self.last_ik_solution      = None

        self.create_subscription(
            Detection2DArray, '/detections',
            self._detection_cb, 10,
        )

        if self.auto_start:
            self.create_timer(2.0, self._check_and_start)

        self.get_logger().info('Braccio MoveIt Sorting Controller ready.')

    # ------------------------------------------------------------ callbacks
    def _joint_state_cb(self, msg):
        self.current_joint_state = msg

    def _pixel_to_world(self, u, v):
        """
        Convert image pixel centroid (u=col, v=row) to world XY position.

        Camera transform analysis (from braccio.urdf.xacro):
          camera_joint      rpy = "0  pi/2  pi"
          camera_optical    rpy = "-pi/2  0  -pi/2"

        Combined, the optical frame axes in world frame are:
          Camera Z (forward) = world  -Z   (looks straight down)
          Camera X (right)   = world  +Y   (image columns → world +Y)
          Camera Y (down)    = world  +X   (image rows    → world +X)

        So for a point on the flat surface at known world Z:
            depth   = cam_wz - cube_z
            world_x = cam_wx + (v - cy) * depth / fy   ← rows    → +X
            world_y = cam_wy + (u - cx) * depth / fx   ← columns → +Y

        Verified against Gazebo ground truth:
          cube at (0.18,  0.08) appears at pixel (458, 34) ✓
          cube at (0.18, -0.08) appears at pixel (183, 34) ✓
        """
        depth   = self.cam_wz - self.cube_z
        world_x = self.cam_wx + (v - self.cam_cy) * depth / self.cam_fy
        world_y = self.cam_wy + (u - self.cam_cx) * depth / self.cam_fx
        return {'x': round(world_x, 4), 'y': round(world_y, 4), 'z': self.cube_z}

    def _detection_cb(self, msg):
        if self.is_sorting:
            return

        valid = []
        for det in msg.detections:
            if not det.results:
                continue
            h = det.results[0]
            if h.hypothesis.score < self.min_confidence:
                continue
            color_id = h.hypothesis.class_id   # 'red_cube' or 'blue_cube'
            if 'red' not in color_id and 'blue' not in color_id:
                continue

            # --- Pixel → World (live from what the camera actually sees) ---
            u = det.bbox.center.position.x   # column (horizontal)
            v = det.bbox.center.position.y   # row    (vertical)
            pos = self._pixel_to_world(u, v)

            self.get_logger().info(
                f'Detected {color_id} at pixel ({u:.0f},{v:.0f}) '
                f'→ world ({pos["x"]:.4f}, {pos["y"]:.4f}, {pos["z"]:.3f})'
            )
            valid.append({
                'class_id': color_id,
                'position': pos,
                'confidence': h.hypothesis.score,
            })

        if valid:
            self.detected_objects = valid
            if self.first_detection_time is None:
                self.first_detection_time = time.time()
                self.get_logger().info(f'Stable detection started — {len(valid)} objects')

    def _check_and_start(self):
        """Timer callback — fires the sort sequence on a worker thread."""
        if self.is_sorting or not self.detected_objects:
            return
        if self.first_detection_time is None:
            return
        if (time.time() - self.first_detection_time) < self.stable_time:
            return

        with self._sort_lock:
            if self.is_sorting:
                return
            self.is_sorting = True

        objects_to_sort = list(self.detected_objects)
        thread = threading.Thread(
            target=self._sort_worker, args=(objects_to_sort,), daemon=True,
        )
        thread.start()

    def _sort_worker(self, objects):
        try:
            self.get_logger().info('Starting sorting sequence (worker thread)')
            self._sort_sequence(objects)
        except Exception as exc:
            self.get_logger().error(f'Sort sequence crashed: {exc}')
        finally:
            self.detected_objects      = []
            self.first_detection_time  = None
            self.is_sorting            = False

    # ----------------------------------------------------------------- IK
    def _compute_ik(self, target):
        """Call /compute_ik service synchronously from worker thread."""
        req = GetPositionIK.Request()
        req.ik_request.group_name      = self.planning_group
        req.ik_request.avoid_collisions = False

        ps = PoseStamped()
        ps.header.frame_id  = self.planning_frame
        ps.header.stamp     = self.get_clock().now().to_msg()
        ps.pose.position.x  = float(target['x'])
        ps.pose.position.y  = float(target['y'])
        ps.pose.position.z  = float(target['z'])
        ps.pose.orientation.w = 1.0

        req.ik_request.pose_stamped   = ps
        req.ik_request.timeout.sec    = 5

        # Seed: bias base_joint toward the target direction
        from sensor_msgs.msg import JointState as JS
        seed = JS()
        seed.name = self.arm_joint_names
        base_target = 2.5 + math.atan2(target['y'], target['x'])
        base_target = max(2.0, min(3.0, base_target))
        if self.last_ik_solution is not None:
            seed.position = list(self.last_ik_solution)
            seed.position[0] = base_target
        else:
            seed.position = [base_target, 2.7, 2.5, 1.4, 2.6]
        req.ik_request.robot_state.joint_state = seed

        self.get_logger().info(
            f'IK target: x={target["x"]:.4f} y={target["y"]:.4f} z={target["z"]:.4f} | '
            f'seed base={base_target:.3f}'
        )

        future = self.ik_client.call_async(req)
        event  = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=8.0):
            self.get_logger().warn('IK service call timed out')
            return None

        resp = future.result()
        if resp is None or resp.error_code.val != 1:
            code = resp.error_code.val if resp else 'no response'
            self.get_logger().warn(f'IK failed (code {code})')
            return None

        js = resp.solution.joint_state
        positions = []
        for jn in self.arm_joint_names:
            try:
                positions.append(js.position[js.name.index(jn)])
            except ValueError:
                self.get_logger().error(f'Joint {jn} missing in IK solution')
                return None

        # Correct base_joint if IK returned the mirror solution
        correct_base = 2.5 + math.atan2(target['y'], target['x'])
        correct_base = max(2.0, min(3.0, correct_base))
        if abs(positions[0] - correct_base) > 0.5:
            positions[0] = correct_base

        # Keep wrist_roll in a sane range
        while positions[4] > 4.0:
            positions[4] -= math.pi
        while positions[4] < 1.0:
            positions[4] += math.pi

        self.get_logger().info(
            f'IK solution: base={positions[0]:.3f} shoulder={positions[1]:.3f} '
            f'elbow={positions[2]:.3f} wrist_pitch={positions[3]:.3f} '
            f'wrist_roll={positions[4]:.3f}'
        )
        self.last_ik_solution = list(positions)
        return positions

    def _send_trajectory(self, client, joint_names, positions, duration_sec):
        """Send a FollowJointTrajectory goal and wait for completion."""
        self.get_logger().info(
            f'Trajectory → {dict(zip(joint_names, [f"{p:.3f}" for p in positions]))} '
            f'over {duration_sec}s'
        )

        point = JointTrajectoryPoint()
        point.positions       = [float(p) for p in positions]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec - int(duration_sec)) * 1e9),
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points      = [point]
        goal.goal_time_tolerance    = Duration(sec=1, nanosec=0)

        send_future = client.send_goal_async(goal)
        send_evt    = threading.Event()
        send_future.add_done_callback(lambda _f: send_evt.set())
        if not send_evt.wait(timeout=5.0):
            self.get_logger().warn('Goal send timed out')
            return False

        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn(f'Goal REJECTED for joints {joint_names}')
            return False

        result_future = handle.get_result_async()
        result_evt    = threading.Event()
        result_future.add_done_callback(lambda _f: result_evt.set())
        if not result_evt.wait(timeout=duration_sec + 15.0):
            self.get_logger().warn('Trajectory execution timed out')
            return False

        result = result_future.result()
        if result and result.result and result.result.error_code != 0:
            self.get_logger().warn(
                f'Trajectory error {result.result.error_code}: '
                f'{result.result.error_string}'
            )
            return False

        self.get_logger().info(f'Trajectory complete for {joint_names}')
        return True

    def _send_arm(self, positions, duration_sec=3.0):
        return self._send_trajectory(
            self.arm_client, self.arm_joint_names, positions, duration_sec,
        )

    def _send_gripper(self, position, duration_sec=1.0):
        return self._send_trajectory(
            self.gripper_client, self.gripper_joint_names, [position],
            duration_sec,
        )

    def _move_to_pose(self, target, description=''):
        self.get_logger().info(
            f'Moving to {description}: '
            f'x={target["x"]:.4f} y={target["y"]:.4f} z={target["z"]:.4f}'
        )
        positions = self._compute_ik(target)
        if positions is None:
            return False
        return self._send_arm(positions, duration_sec=3.0)

    def _move_named(self, name):
        if name not in self.named_positions:
            self.get_logger().error(f'Unknown named position: {name}')
            return False
        return self._send_arm(self.named_positions[name], duration_sec=2.0)

    # --------------------------------------------------------- pick & place
    def _pick(self, obj, idx):
        color = 'red' if 'red' in obj['class_id'] else 'blue'
        pos   = obj['position']
        self.get_logger().info(
            f'Picking {color} cube #{idx} at '
            f'({pos["x"]:.4f}, {pos["y"]:.4f}, {pos["z"]:.3f})'
        )

        self._send_gripper(self.gripper_open)

        # Approach 40 mm above cube, then descend to grasp height
        approach = {'x': pos['x'], 'y': pos['y'], 'z': pos['z'] + 0.04}
        if not self._move_to_pose(approach, f'{color} approach'):
            return False

        grasp = {'x': pos['x'], 'y': pos['y'], 'z': pos['z']}
        if not self._move_to_pose(grasp, f'{color} grasp'):
            return False

        self._send_gripper(self.gripper_closed)
        time.sleep(0.5)   # let gripper close

        return self._move_to_pose(approach, 'lift')

    def _place(self, color):
        self.get_logger().info(f'Placing in {color} container')
        if not self._move_named(f'drop_{color}'):
            return False
        self._send_gripper(self.gripper_open)
        return True

    def _sort_sequence(self, objects):
        self._move_named('home')
        self._move_named('scan')

        red = blue = 0
        for obj in objects:
            color = 'red' if 'red' in obj['class_id'] else 'blue'
            if color == 'red':
                red  += 1
                idx   = red
            else:
                blue += 1
                idx   = blue

            if self._pick(obj, idx):
                self._place(color)
            self._move_named('scan')

        self._move_named('home')
        self.get_logger().info(
            f'Sorting complete — red: {red}, blue: {blue}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = BraccioMoveItSortingController()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()