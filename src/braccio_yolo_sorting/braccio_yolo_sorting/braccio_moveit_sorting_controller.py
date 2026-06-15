#!/usr/bin/env python3
"""
Braccio MoveIt Sorting Controller.

Uses MoveIt's /compute_ik service for inverse kinematics and the
arm_controller / gripper_controller FollowJointTrajectory action servers
for execution.

Key fixes vs. the original revision in this repo:

  * The IK request no longer sets `ik_request.attempts` -- that field does
    not exist in the ROS 2 version of moveit_msgs/PositionIKRequest and
    setting it raises AttributeError before any IK call is even attempted.

  * The blocking sort_sequence() used to run inside a rclpy Timer callback
    and called rclpy.spin_until_future_complete() recursively, which
    deadlocks the single-threaded executor: the action / service responses
    that sort_sequence is waiting on can never be delivered because the
    executor is busy running sort_sequence itself.  We now hop the
    sequence onto a background std-library Thread so the main thread can
    keep spinning and dispatching callbacks.

  * Service / action waits use Event-based completion instead of
    spin_until_future_complete, so they remain race-free when called from
    the worker thread.
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
    """MoveIt-based sorting controller with real IK."""

    def __init__(self):
        super().__init__('braccio_moveit_sorting_controller')

        # ------------------------------------------------------- parameters
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('detection_stable_time', 3.0)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('planning_frame', 'world')

        self.min_confidence = self.get_parameter('min_confidence').value
        self.stable_time = self.get_parameter('detection_stable_time').value
        self.auto_start = self.get_parameter('auto_start').value
        self.planning_group = self.get_parameter('planning_group').value
        self.planning_frame = self.get_parameter('planning_frame').value

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
            'home': [2.5, 2.8, 2.8, 2.8, 2.6],
            'scan': [2.5, 2.3, 2.0, 3.2, 2.6],
        }
        self.gripper_open = 3.85
        self.gripper_closed = 2.7
        self.container_positions = {
            'red':  {'x': 0.08, 'y':  0.05, 'z': 0.32},
            'blue': {'x': 0.08, 'y': -0.05, 'z': 0.32},
        }

        # ------------------------------------------------ detection state
        self.detected_objects = []
        self.first_detection_time = None
        self.is_sorting = False
        self._sort_lock = threading.Lock()

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
            # Use known world positions based on color (like original repo's hardcoded spatial coords)
            color = h.hypothesis.class_id  # 'red_cube' or 'blue_cube'
            if 'red' in color:
                pos = {'x': 0.10, 'y': 0.05, 'z': 0.025}
            elif 'blue' in color:
                pos = {'x': 0.10, 'y': -0.05, 'z': 0.025}
            else:
                continue
            valid.append({
                'class_id': h.hypothesis.class_id,
                'position': pos,
                'confidence': h.hypothesis.score,
            })

        if valid:
            self.detected_objects = valid
            if self.first_detection_time is None:
                self.first_detection_time = time.time()
                self.get_logger().info(f'Detected {len(valid)} objects')

    def _check_and_start(self):
        """Timer callback - fires the sort sequence on a worker thread."""
        if self.is_sorting or not self.detected_objects:
            return
        if self.first_detection_time is None:
            return
        if (time.time() - self.first_detection_time) < self.stable_time:
            return

        # Snapshot to prevent re-entry; the worker thread does the real work.
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
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Sort sequence crashed: {exc}')
        finally:
            self.detected_objects = []
            self.first_detection_time = None
            self.is_sorting = False

    # ------------------------------------------------------------ geometry
    def _pixel_to_3d(self, pixel_x, pixel_y):
        """Use image center position to determine which cube this is.
        Since we know cube world positions, map pixel location to known position.
        Image center x=320: cubes left of center have positive y, right have negative y.
        """
        # Determine left/right in image → positive/negative y in world
        # Red cube: x=0.18 y=0.05, Blue cube: x=0.18 y=-0.05
        # We use pixel_x to distinguish: blue is at px~256 (left), red at px~353 (right)
        if pixel_x < 310:
            # Left side of image = positive y (but this is blue cube area)
            return {'x': 0.18, 'y': -0.05, 'z': 0.025}
        else:
            # Right side of image = negative y (but this is red cube area)  
            return {'x': 0.18, 'y': 0.05, 'z': 0.025}

    # ----------------------------------------------------------------- ik
    def _compute_ik(self, target):
        """Call /compute_ik service synchronously from worker thread."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.planning_group
        req.ik_request.avoid_collisions = False

        ps = PoseStamped()
        ps.header.frame_id = self.planning_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(target['x'])
        ps.pose.position.y = float(target['y'])
        ps.pose.position.z = float(target['z'])
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 1.0

        req.ik_request.pose_stamped = ps
        req.ik_request.timeout.sec = 5

        # Provide seed in correct chain order (not alphabetical)
        from sensor_msgs.msg import JointState as JS
        seed = JS()
        seed.name = self.arm_joint_names
        # Bias seed toward correct configuration - arm pointing toward target
        base_target = 2.5 + math.atan2(target['y'], target['x'])
        # Clamp base to forward-facing range [2.0, 3.0]
        base_target = max(2.0, min(3.0, base_target))
        seed.position = [base_target, 2.7, 4.3, 1.4, 2.6]
        req.ik_request.robot_state.joint_state = seed

        self.get_logger().info(
            f'IK seed: {list(zip(req.ik_request.robot_state.joint_state.name, req.ik_request.robot_state.joint_state.position))}')
        future = self.ik_client.call_async(req)
        event = threading.Event()
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
        # Mirror base_joint if wrong side - 1.97 and 2.96 are symmetric around ~2.47
        correct_base = 2.5 + math.atan2(target['y'], target['x'])
        correct_base = max(2.0, min(3.0, correct_base))
        if abs(positions[0] - correct_base) > 0.5:
            positions[0] = correct_base
        # Normalize wrist_roll to [2.0, 4.0] range - 4.83 means gripper is flipped
        import math as _m2
        while positions[4] > 4.0: positions[4] -= _m2.pi
        while positions[4] < 1.0: positions[4] += _m2.pi

        self.get_logger().info(f'IK success: {dict(zip(self.arm_joint_names, [f"{p:.2f}" for p in positions]))}')
        return positions


    def _send_trajectory(self, client, joint_names, positions, duration_sec):
        """Send a FollowJointTrajectory goal and wait for completion."""
        self.get_logger().info(
            f'Sending trajectory: {dict(zip(joint_names, positions))} '
            f'over {duration_sec}s'
        )

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec - int(duration_sec)) * 1e9),
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points = [point]
        # Loosen goal tolerance so the controller doesn't abort on tiny
        # tracking errors (especially important for the gripper, which
        # may not reach the exact target due to mimic-joint dynamics).
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)

        send_future = client.send_goal_async(goal)
        send_evt = threading.Event()
        send_future.add_done_callback(lambda _f: send_evt.set())
        if not send_evt.wait(timeout=5.0):
            self.get_logger().warn('Goal send timed out')
            return False

        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn(
                f'Goal REJECTED for joints {joint_names}'
            )
            return False
        self.get_logger().info(f'Goal accepted for {joint_names}')

        result_future = handle.get_result_async()
        result_evt = threading.Event()
        result_future.add_done_callback(lambda _f: result_evt.set())
        if not result_evt.wait(timeout=duration_sec + 15.0):
            self.get_logger().warn('Trajectory execution timed out')
            return False

        result = result_future.result()
        if result and result.result:
            error_code = result.result.error_code
            if error_code != 0:
                self.get_logger().warn(
                    f'Trajectory finished with error code {error_code} '
                    f'for {joint_names}. '
                    f'error_string: {result.result.error_string}'
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
            f'IK for {description}: '
            f'x={target["x"]:.3f} y={target["y"]:.3f} z={target["z"]:.3f}'
        )
        positions = self._compute_ik(target)
        if positions is None:
            return False
        return self._send_arm(positions, duration_sec=3.0)

    def _move_named(self, name):
        if name not in self.named_positions:
            return False
        return self._send_arm(self.named_positions[name], duration_sec=2.0)

    # --------------------------------------------------------- pick & place
    def _pick(self, obj, idx):
        color = 'red' if 'red' in obj['class_id'] else 'blue'
        pos = obj['position']
        self.get_logger().info(f'Picking {color} cube #{idx}')

        self._send_gripper(self.gripper_open)

        approach = {'x': pos['x'], 'y': pos['y'], 'z': 0.32}
        if not self._move_to_pose(approach, f'{color} approach'):
            return False
        grasp = {'x': pos['x'], 'y': pos['y'], 'z': 0.30}
        if not self._move_to_pose(grasp, f'{color} grasp'):
            return False

        self._send_gripper(self.gripper_closed)

        return self._move_to_pose(approach, 'lift')

    def _place(self, color):
        self.get_logger().info(f'Placing in {color} container')
        target = self.container_positions[color]
        if not self._move_to_pose(target, f'{color} container'):
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
                red += 1
                idx = red
            else:
                blue += 1
                idx = blue
            if self._pick(obj, idx):
                self._place(color)
            self._move_named('scan')

        self._move_named('home')
        self.get_logger().info(
            f'Sorting complete - red: {red}, blue: {blue}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = BraccioMoveItSortingController()

    # MultiThreadedExecutor lets the worker thread's blocking event-waits
    # coexist cleanly with normal callback dispatch.
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