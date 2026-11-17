#!/usr/bin/env python3
"""Phase D: Room 2 (Beta) Pick and Place Task Node.

Drives the TurtleBot3 Manipulation robot to the Beta cube at map (-6.35, 7.42),
lowers the 4-DOF OpenManipulator-X arm, closes the gripper, attaches the cube
via Gazebo's DetachableJoint system (/beta_pick_item/attach), carries the cube
to the Beta drop spot at map (-0.23, 8.51), lowers and releases it
(/beta_pick_item/detach), publishes True to /room2/done, and exits with 0.

Supports both:
  1. Standalone / Mission Manager subprocess execution (default):
       ros2 run home_nav_sim beta_pick_place.py
       ros2 run robot_bringup beta_pick_place
     Runs immediately and exits with return code 0 (success) or 1 (failure).
  2. Topic handoff mode (waits for /room2/ready before starting):
       ros2 run robot_bringup beta_pick_place --ros-args -p wait_for_ready:=true

Also supports both Nav2 navigation (when explore.launch.py or nav.launch.py is
running) and direct odometry waypoint driving (when testing in the stripped-down
practice world via robot_in_home.launch.py world:=home_tasks.sdf).
"""
import math
import sys
import time
import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from builtin_interfaces.msg import Duration as MsgDuration
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Empty
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class BetaPickAndPlace(Node):
    ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']

    # Joint poses:
    # - PICK_POSE / PLACE_POSE keep link5 level (q2 + q3 + q4 = 0.0 rad) at the floor.
    # - CARRY_POSE lifts link5 to z=0.384 m and tilts the gripper up so the bottom of
    #   the 0.15 m cube is at z ~ 0.23 m — well above the 2D LiDAR scan plane (z=0.132 m)
    #   so Nav2's costmap and collision_monitor never see the carried cube as an obstacle.
    HOME_POSE = [0.0, -1.00, 0.30, 0.70]
    PRE_PICK_POSE = [0.0, 0.15, 0.10, -0.25]
    PICK_POSE = [0.0, 0.50, 0.10, -0.60]
    CARRY_POSE = [0.0, -0.85, -0.15, 0.65]
    PLACE_POSE = [0.0, 0.50, 0.10, -0.60]

    GRIPPER_OPEN = 0.018
    GRIPPER_CLOSED = -0.005

    # Distance from base_footprint to cube centre when grasping/placing (metres)
    GRASP_STANDOFF_M = 0.28

    def __init__(self):
        super().__init__('beta_pick_place')

        # Parameters
        self.declare_parameter('wait_for_ready', False)
        self.declare_parameter('box_x', -6.35)
        self.declare_parameter('box_y', 7.42)
        self.declare_parameter('drop_x', -0.23)
        self.declare_parameter('drop_y', 8.51)
        self.declare_parameter('use_visual_servo', True)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )
        self.attach_pub = self.create_publisher(Empty, '/beta_pick_item/attach', 10)
        self.detach_pub = self.create_publisher(Empty, '/beta_pick_item/detach', 10)
        self.done_pub = self.create_publisher(Bool, '/room2/done', latched_qos)

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )

        # TF & Subscriptions
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.room2_ready = False
        self.odom_pose = None
        self.latest_rgb = None
        self.latest_depth = None
        self.arm_joint_positions = {}

        self.create_subscription(Bool, '/room2/ready', self._ready_cb, latched_qos)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/image', self._rgb_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/depth_image', self._depth_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        self.get_logger().info('Beta Pick & Place node initialised.')

    # --- Callbacks -----------------------------------------------------------
    def _ready_cb(self, msg: Bool):
        if msg.data:
            self.room2_ready = True

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_pose = (p.x, p.y, yaw)

    def _rgb_cb(self, msg: Image):
        if msg.encoding in ('rgb8', 'bgr8'):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            self.latest_rgb = arr if msg.encoding == 'rgb8' else arr[:, :, ::-1]

    def _depth_cb(self, msg: Image):
        if msg.encoding == '32FC1':
            self.latest_depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width
            )

    def _joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.arm_joint_positions[name] = float(pos)

    # --- Helpers -------------------------------------------------------------
    def spin_for(self, duration_sec: float):
        end = time.time() + duration_sec
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def get_pose(self):
        """Return (x, y, yaw) in map frame if TF is available, else odom frame."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(), timeout=Duration(seconds=0.1)
            )
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            return t.transform.translation.x, t.transform.translation.y, yaw
        except Exception:
            return self.odom_pose

    def set_detachable_joint(self, attach: bool):
        pub = self.attach_pub if attach else self.detach_pub
        action_str = 'ATTACH' if attach else 'DETACH'
        self.get_logger().info(f'[DetachableJoint] Sending {action_str} command...')
        for _ in range(4):
            pub.publish(Empty())
            self.spin_for(0.1)

    def move_arm(self, positions, duration_sec: float = 2.5, description: str = ''):
        if description:
            self.get_logger().info(f'[Arm] {description}: {positions}')
        traj = JointTrajectory()
        traj.joint_names = list(self.ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        sec = int(duration_sec)
        pt.time_from_start = MsgDuration(sec=sec, nanosec=int((duration_sec - sec) * 1e9))
        traj.points.append(pt)
        self.arm_pub.publish(traj)

        # Wait until /joint_states actually reaches the target pose (accounting for slow
        # Gazebo Real-Time Factor) plus a short settle pause so the arm has zero velocity.
        start = time.time()
        max_wait_wall = max(12.0, duration_sec * 6.0)
        while rclpy.ok() and (time.time() - start) < max_wait_wall:
            rclpy.spin_once(self, timeout_sec=0.04)
            if all(j in self.arm_joint_positions for j in self.ARM_JOINTS):
                max_err = max(
                    abs(self.arm_joint_positions[j] - float(target))
                    for j, target in zip(self.ARM_JOINTS, positions)
                )
                if max_err < 0.035 and (time.time() - start) >= duration_sec * 0.8:
                    break
        # Let physics settle so the arm is completely stationary before attach/detach
        self.spin_for(0.6)

    def control_gripper(self, position: float, description: str = ''):
        if description:
            self.get_logger().info(f'[Gripper] {description} (pos={position:.3f} m)')
        if not self.gripper_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('Gripper action server not available; continuing.')
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 10.0
        self.gripper_client.send_goal_async(goal)
        self.spin_for(1.2)

    def stop_base(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    # --- Navigation & Closed-Loop Driving ------------------------------------
    def drive_to_pose_closed_loop(
        self,
        target_x: float,
        target_y: float,
        target_yaw: float,
        pos_tol: float = 0.03,
        yaw_tol: float = 0.04,
        max_lin: float = 0.22,
        max_ang: float = 0.75,
        timeout_sec: float = 180.0,
    ) -> bool:
        """Drive smoothly to (target_x, target_y, target_yaw) using /cmd_vel and get_pose()."""
        start = time.time()
        phase = 'ROTATE_TO_GOAL'

        while rclpy.ok() and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.02)
            pose = self.get_pose()
            if pose is None:
                continue

            x, y, yaw = pose
            dx = target_x - x
            dy = target_y - y
            dist = math.hypot(dx, dy)
            heading_to_goal = math.atan2(dy, dx)

            twist = Twist()

            if phase == 'ROTATE_TO_GOAL':
                if dist <= pos_tol:
                    phase = 'FINAL_YAW'
                    continue
                err_yaw = normalize_angle(heading_to_goal - yaw)
                if abs(err_yaw) < 0.12:
                    phase = 'TRANSLATE'
                else:
                    twist.angular.z = max(-max_ang, min(max_ang, 1.8 * err_yaw))
                    if abs(twist.angular.z) < 0.08:
                        twist.angular.z = math.copysign(0.08, err_yaw)

            elif phase == 'TRANSLATE':
                if dist <= pos_tol:
                    phase = 'FINAL_YAW'
                    continue
                err_yaw = normalize_angle(heading_to_goal - yaw)
                if abs(err_yaw) > 0.45 and dist > 0.10:
                    phase = 'ROTATE_TO_GOAL'
                    continue
                lin = min(max_lin, max(0.05, 0.8 * dist))
                twist.linear.x = lin * max(0.0, math.cos(err_yaw))
                twist.angular.z = max(-max_ang, min(max_ang, 1.5 * err_yaw))

            elif phase == 'FINAL_YAW':
                err_yaw = normalize_angle(target_yaw - yaw)
                if abs(err_yaw) <= yaw_tol:
                    self.stop_base()
                    return True
                twist.angular.z = max(-max_ang, min(max_ang, 1.6 * err_yaw))
                if abs(twist.angular.z) < 0.06:
                    twist.angular.z = math.copysign(0.06, err_yaw)

            self.cmd_pub.publish(twist)

        self.stop_base()
        self.get_logger().warn(
            f'Closed-loop drive to ({target_x:.2f}, {target_y:.2f}) timed out.'
        )
        return False

    def navigate_to_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        label: str = '',
        fine_align: bool = True,
        pos_tol: float = 0.03,
    ) -> bool:
        """Navigate using Nav2 if available, then refine with closed-loop control."""
        if label:
            self.get_logger().info(
                f'[Navigate] -> {label}: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})'
            )

        nav2_available = self.nav_client.wait_for_server(timeout_sec=1.5)
        if nav2_available:
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = Time().to_msg()
            goal.pose.pose.position.x = float(x)
            goal.pose.pose.position.y = float(y)
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

            send_future = self.nav_client.send_goal_async(goal)
            while rclpy.ok() and not send_future.done():
                rclpy.spin_once(self, timeout_sec=0.05)
            handle = send_future.result()

            if handle and handle.accepted:
                res_future = handle.get_result_async()
                start = time.time()
                while rclpy.ok() and not res_future.done() and (time.time() - start) < 120.0:
                    rclpy.spin_once(self, timeout_sec=0.1)
                if not res_future.done():
                    handle.cancel_goal_async()
                    self.spin_for(0.5)
                    self.get_logger().warn(f'Nav2 goal {label} timed out; using closed-loop.')
                    return self.drive_to_pose_closed_loop(x, y, yaw, pos_tol=pos_tol)
                status = res_future.result().status
                if status != 4:
                    self.get_logger().warn(
                        f'Nav2 goal {label} ended with status {status}; using closed-loop.'
                    )
                    return self.drive_to_pose_closed_loop(x, y, yaw, pos_tol=pos_tol)
            else:
                self.get_logger().warn(f'Nav2 rejected goal {label}; using closed-loop.')
                return self.drive_to_pose_closed_loop(x, y, yaw, pos_tol=pos_tol)

        if fine_align or not nav2_available:
            return self.drive_to_pose_closed_loop(x, y, yaw, pos_tol=pos_tol)
        return True

    def align_with_orange_cube_camera(self, timeout_sec: float = 8.0):
        """Optionally refine heading toward the orange cube using the RGB camera."""
        if not self.get_parameter('use_visual_servo').value:
            return

        start = time.time()
        while rclpy.ok() and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_rgb is None:
                continue

            rgb = self.latest_rgb.astype(np.float32)
            r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
            # Orange cube RGB ~ (240, 158, 38)
            cube_mask = (r > 165) & (g > 80) & (g < 195) & (b < 95) & ((r - b) > 90)
            ys, xs = np.nonzero(cube_mask)
            if len(xs) < 25:
                break

            cx = float(np.mean(xs))
            err_x = (cx - (rgb.shape[1] / 2.0)) / (rgb.shape[1] / 2.0)
            if abs(err_x) <= 0.06:
                self.stop_base()
                self.get_logger().info(
                    f'[Camera Align] Centered on orange cube (err_x={err_x:.3f}).'
                )
                return

            twist = Twist()
            twist.angular.z = max(-0.25, min(0.25, -0.45 * err_x))
            self.cmd_pub.publish(twist)

        self.stop_base()

    def backup_distance(self, distance_m: float = 0.25, speed: float = 0.14):
        pose0 = self.get_pose()
        if pose0 is None:
            return
        x0, y0, _ = pose0
        twist = Twist()
        twist.linear.x = -abs(speed)
        start = time.time()
        while rclpy.ok() and (time.time() - start) < 25.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            pose = self.get_pose()
            if pose is not None and math.hypot(pose[0] - x0, pose[1] - y0) >= distance_m:
                break
            self.cmd_pub.publish(twist)
        self.stop_base()

    # --- Main Pick and Place Mission -----------------------------------------
    def execute_mission(self) -> bool:
        # Wait for initial odometry / TF data
        self.get_logger().info('Waiting for robot pose (/odom or TF)...')
        start_wait = time.time()
        while rclpy.ok() and self.get_pose() is None and (time.time() - start_wait) < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.get_pose() is None:
            self.get_logger().error('No pose available on /odom or TF!')
            return False

        # If wait_for_ready is enabled, wait for /room2/ready trigger
        if self.get_parameter('wait_for_ready').value:
            self.get_logger().info('Waiting for /room2/ready trigger...')
            while rclpy.ok() and not self.room2_ready:
                rclpy.spin_once(self, timeout_sec=0.2)
            if not rclpy.ok():
                return False

        box_x = float(self.get_parameter('box_x').value)
        box_y = float(self.get_parameter('box_y').value)
        drop_x = float(self.get_parameter('drop_x').value)
        drop_y = float(self.get_parameter('drop_y').value)

        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'Starting Phase D: Beta Pick & Place | '
            f'Box=({box_x:.2f}, {box_y:.2f}) -> Drop=({drop_x:.2f}, {drop_y:.2f})'
        )
        self.get_logger().info('=' * 60)

        # 1. Ensure DetachableJoint starts detached and arm is folded safely
        self.set_detachable_joint(attach=False)
        self.control_gripper(self.GRIPPER_OPEN, 'Opening gripper')
        self.move_arm(self.HOME_POSE, duration_sec=2.0, description='Moving arm to Home Pose')

        # 2. Route into Room 2 (Beta) if currently outside Room 2
        cur_x, cur_y, _ = self.get_pose()
        # Room 2 Beta is north of wall y = 6.21 in map frame (world y = 7.0) and east of x = -9.21
        if cur_y < 6.40 or cur_x < -9.21:
            if cur_y > 6.40 and cur_x < -9.21:
                # Currently inside Room 1 (Alpha): exit Door 1 into Hall first
                self.navigate_to_pose(
                    -14.61, 5.20, -math.pi / 2.0, 'Exit Room 1 Doorway',
                    fine_align=False, pos_tol=0.15,
                )
            # Go to Hallway south of Door 2 (Beta entrance), then through Door 2
            self.navigate_to_pose(
                -3.81, 5.20, math.pi / 2.0, 'Beta Doorway Approach (Hall)',
                fine_align=False, pos_tol=0.15,
            )
            self.navigate_to_pose(
                -3.81, 7.42, math.pi, 'Inside Beta Room Entrance',
                fine_align=False, pos_tol=0.12,
            )

        # 3. Approach the Beta Cube from the East (facing West, yaw = pi)
        # Stop at box_x + 0.60 first so Nav2 stays outside the static map's
        # inflation radius around the cube, then close in with closed-loop control.
        pre_grasp_x = box_x + 0.60
        pre_grasp_y = box_y
        grasp_x = box_x + self.GRASP_STANDOFF_M
        grasp_y = box_y
        grasp_yaw = math.pi

        if not self.navigate_to_pose(
            pre_grasp_x, pre_grasp_y, grasp_yaw, 'Pre-Grasp Pose',
            fine_align=True, pos_tol=0.05,
        ):
            self.get_logger().warn('Pre-grasp pose had residual error; continuing to grasp pose.')

        self.move_arm(
            self.PRE_PICK_POSE, duration_sec=1.8, description='Preparing arm for pick'
        )

        # Fine approach to exact grasp standoff distance
        self.drive_to_pose_closed_loop(
            grasp_x, grasp_y, grasp_yaw, pos_tol=0.02, yaw_tol=0.03, max_lin=0.10
        )
        self.align_with_orange_cube_camera(timeout_sec=5.0)

        # 4. Grasp and Attach Cube
        self.stop_base()
        self.spin_for(0.4)
        pick_pose = self.get_pose()
        if pick_pose is not None:
            px, py, pyaw = pick_pose
            # Exact local offset of the cube centre in base_footprint frame at pick time
            dx_world = box_x - px
            dy_world = box_y - py
            held_dx_local = dx_world * math.cos(pyaw) + dy_world * math.sin(pyaw)
            held_dy_local = -dx_world * math.sin(pyaw) + dy_world * math.cos(pyaw)
            held_dx_local = max(0.20, min(0.38, held_dx_local))
            held_dy_local = max(-0.10, min(0.10, held_dy_local))
        else:
            held_dx_local = self.GRASP_STANDOFF_M
            held_dy_local = 0.0

        self.move_arm(self.PICK_POSE, duration_sec=2.5, description='Lowering arm to Pick Pose')
        self.control_gripper(self.GRIPPER_CLOSED, 'Closing gripper on Beta cube')
        self.set_detachable_joint(attach=True)
        self.move_arm(
            self.CARRY_POSE, duration_sec=2.5, description='Lifting Beta cube to high Carry Pose'
        )

        # Back up 0.38 m out of the static map's obstacle inflation zone around
        # the cube's original position before asking Nav2 to plan to the drop spot.
        self.get_logger().info('[Base] Backing out of static map cube inflation zone...')
        self.backup_distance(distance_m=0.38, speed=0.14)

        # 5. Carry Cube to the Beta Drop Spot (-0.23, 8.51)
        # Route via (-3.81, drop_y) to stay in the clear corridor north of meeting_table
        self.navigate_to_pose(
            -3.81, drop_y, 0.0, 'Room 2 Clear Corridor Waypoint',
            fine_align=False, pos_tol=0.15,
        )

        # Compute exact robot stop pose so the held cube centre lands dead-centre on (drop_x, drop_y)
        place_yaw = 0.0
        place_x = drop_x - (
            held_dx_local * math.cos(place_yaw) - held_dy_local * math.sin(place_yaw)
        )
        place_y = drop_y - (
            held_dx_local * math.sin(place_yaw) + held_dy_local * math.cos(place_yaw)
        )
        pre_drop_x = place_x - 0.28
        pre_drop_y = place_y

        self.navigate_to_pose(
            pre_drop_x, pre_drop_y, place_yaw, 'Pre-Drop Pose',
            fine_align=True, pos_tol=0.04,
        )
        self.drive_to_pose_closed_loop(
            place_x, place_y, place_yaw, pos_tol=0.015, yaw_tol=0.025, max_lin=0.08
        )
        self.stop_base()
        self.spin_for(0.5)

        # 6. Lower Arm, Detach Cube on Drop Spot, and Retract
        self.move_arm(
            self.PLACE_POSE, duration_sec=3.0, description='Lowering Beta cube gently onto Drop Spot'
        )
        self.spin_for(0.5)
        self.set_detachable_joint(attach=False)
        self.control_gripper(self.GRIPPER_OPEN, 'Opening gripper to release cube')
        self.spin_for(0.5)

        self.move_arm(
            self.HOME_POSE, duration_sec=2.0, description='Retracting arm to Home Pose'
        )
        self.backup_distance(distance_m=0.25, speed=0.12)

        # 7. Publish completion signal
        for _ in range(3):
            self.done_pub.publish(Bool(data=True))
            self.spin_for(0.1)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Phase D: Beta Pick & Place COMPLETE! (/room2/done published)')
        self.get_logger().info('=' * 60)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = BetaPickAndPlace()
    success = False
    try:
        success = node.execute_mission()
    except KeyboardInterrupt:
        node.get_logger().info('Beta Pick & Place interrupted by user.')
    finally:
        node.stop_base()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
