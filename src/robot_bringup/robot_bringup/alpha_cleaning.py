#!/usr/bin/env python3
"""Phase C: Room Alpha Vacuum Cleaning Task Node.

Supports two targeting modes selectable via parameter `discover_targets`:
1. Autonomous Discovery Mode (`discover_targets:=true`, used by mission.launch.py):
   Locates the Room Alpha slate-blue cleaning zone visually using the RGB-D camera,
   TF (map -> rgbd_camera_optical_frame), and live SLAM /map (seeded from
   /discovered/clean_zone and refined via active 360° scan / room viewpoints).
   Uses known nominal dimensions (2.29 x 1.57 m) clamped to /map free space,
   dynamically spawns dirt across the discovered zone, and sweeps it.
2. Standalone / Parameter Mode (`discover_targets:=false`, default):
   Uses the `clean_x_min/max`, `clean_y_min/max` parameters directly for
   isolated testing.

Supports both Nav2 Simple Commander (`use_nav2:=true`, default) and closed-loop
/cmd_vel fallback (`drive_to_pose_closed_loop`).
"""

import csv
from datetime import datetime, timezone
import json
import math
import os
import sys
import time
import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from builtin_interfaces.msg import Duration as MsgDuration
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity

from robot_bringup.perception import (
    classify_floor_color,
    detect_clean_zone_points,
    estimate_clean_zone_bounds,
    intrinsics_from_camera_info,
    sample_room_search_viewpoints,
    select_approach_pose,
    transform_stamped_to_matrix,
)


DIRT_SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="dirt_link">
      <visual name="dirt_visual">
        <geometry>
          <cylinder>
            <radius>0.025</radius>
            <length>0.004</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.42 0.24 0.08 1.0</ambient>
          <diffuse>0.48 0.28 0.10 1.0</diffuse>
          <specular>0.05 0.05 0.05 1.0</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

EVAL_CSV_COLUMNS = [
    'timestamp_utc',
    'trial_id',
    'nav_mode',
    'clean_x_min',
    'clean_x_max',
    'clean_y_min',
    'clean_y_max',
    'dirt_total',
    'dirt_cleaned',
    'dirt_ratio',
    'total_time_sec',
    'mission_success',
    'failure_reason',
]


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def append_eval_csv(csv_path: str, record: dict) -> int:
    parent = os.path.dirname(os.path.abspath(csv_path))
    if parent:
        os.makedirs(parent, exist_ok=True)

    file_exists = os.path.isfile(csv_path) and os.path.getsize(csv_path) > 0
    trial_id = 1
    if file_exists:
        with open(csv_path, 'r', newline='', encoding='utf-8') as f:
            trial_id = sum(1 for _ in csv.DictReader(f)) + 1

    record['trial_id'] = trial_id
    row = {col: str(record.get(col, '')) for col in EVAL_CSV_COLUMNS}

    with open(csv_path, 'a', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=EVAL_CSV_COLUMNS)
        if not file_exists:
            writer.writeheader()
        writer.writerow(row)

    return trial_id


class AlphaCleaning(Node):
    ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']
    HOME_POSE = [0.0, -1.00, 0.30, 0.70]

    VACUUM_OFFSET_X = -0.040   # m along base_link X axis
    VACUUM_HALF_SIZE = 0.080   # half-width of 0.16m x 0.16m pad
    DIRT_RADIUS = 0.025        # m radius of spawned dirt cylinder

    def __init__(self):
        super().__init__('alpha_cleaning')

        # Declare Parameters
        self.declare_parameter('use_nav2', True)
        self.declare_parameter('wait_for_ready', False)
        self.declare_parameter('discover_targets', False)
        self.declare_parameter('spawn_dirt', True)
        self.declare_parameter('clean_x_min', -13.6)
        self.declare_parameter('clean_x_max', -11.3)
        self.declare_parameter('clean_y_min', 7.6)
        self.declare_parameter('clean_y_max', 9.1)
        self.declare_parameter('nominal_zone_sx', 2.29)
        self.declare_parameter('nominal_zone_sy', 1.57)
        self.declare_parameter('lane_spacing', 0.22)
        self.declare_parameter('return_to_doorway', True)
        self.declare_parameter('spawn_x', 19.21)
        self.declare_parameter('spawn_y', 0.79)
        self.declare_parameter('eval_log_path', 'log/alpha_cleaning_eval.csv')

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Publishers & Subscriptions
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vacuum_pub = self.create_publisher(Bool, '/vacuum/active', 10)
        self.done_pub = self.create_publisher(Bool, '/room_alpha/done', latched_qos)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.detach_pub = self.create_publisher(Empty, '/beta_pick_item/detach', 10)
        self.eval_pub = self.create_publisher(String, '/alpha_cleaning/eval_metrics', latched_qos)

        self.create_subscription(Bool, '/room_alpha/ready', self._ready_cb, latched_qos)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/image', self._rgb_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/depth_image', self._depth_cb, 10)
        self.create_subscription(CameraInfo, '/rgbd_camera/camera_info', self._cam_info_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, latched_qos)
        self.create_subscription(String, '/discovered/clean_zone', self._discovered_zone_cb, latched_qos)

        # TF Buffer & Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Gazebo entity services
        self.spawn_client = self.create_client(SpawnEntity, '/world/empty/create')
        self.delete_client = self.create_client(DeleteEntity, '/world/empty/remove')

        self.room_alpha_ready = False
        self.vacuum_active = False
        self.odom_pose = None  # (x, y, yaw) from /odom
        self.latest_scan = None
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_info = None
        self.map_msg = None
        self.discovered_zone_msg = None
        self._rgb_seq = 0

        self.active_dirt = {}
        self.total_dirt_count = 0
        self.hoovered_count = 0

        # Optional BasicNavigator client if use_nav2 is True
        self.navigator = None
        if self.get_parameter('use_nav2').value:
            try:
                from nav2_simple_commander.robot_navigator import BasicNavigator
                self.navigator = BasicNavigator()
                self.get_logger().info('Initialized nav2_simple_commander BasicNavigator.')
            except Exception as e:
                self.get_logger().warn(f'Could not initialize BasicNavigator ({e}); using closed-loop.')

        mode_str = 'Nav2' if self.navigator is not None else 'Closed-Loop /cmd_vel'
        disc_str = 'autonomous-discovery' if self.get_parameter('discover_targets').value else 'parameter-coords'
        self.get_logger().info(f'Alpha Cleaning initialized in [{mode_str} | {disc_str}] mode.')

    # --- Callbacks -----------------------------------------------------------
    def _ready_cb(self, msg: Bool):
        if msg.data:
            self.room_alpha_ready = True

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_pose = (p.x, p.y, yaw)

        if self.vacuum_active and self.active_dirt:
            self._check_hoover_dirt()

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _rgb_cb(self, msg: Image):
        if msg.encoding in ('rgb8', 'bgr8'):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            self.latest_rgb = arr if msg.encoding == 'rgb8' else arr[:, :, ::-1]
            self._rgb_seq += 1

    def _depth_cb(self, msg: Image):
        if msg.encoding == '32FC1':
            self.latest_depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width
            )

    def _cam_info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def _map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg

    def _discovered_zone_cb(self, msg: String):
        try:
            self.discovered_zone_msg = json.loads(msg.data)
        except Exception:
            pass

    # --- Pose, LiDAR, TF & Map Helpers ---------------------------------------
    def _min_scan_in_arc(self, center_angle_rad: float, half_arc_rad: float = 0.50) -> float:
        """Return min valid LiDAR range (>= 0.25 m) within an angular cone, or inf."""
        s = self.latest_scan
        if s is None or not s.ranges:
            return math.inf
        arr = np.asarray(s.ranges, dtype=np.float64)
        angles = s.angle_min + np.arange(len(arr)) * s.angle_increment
        diffs = np.abs(np.arctan2(np.sin(angles - center_angle_rad), np.cos(angles - center_angle_rad)))
        mask = (diffs <= half_arc_rad) & np.isfinite(arr) & (arr >= 0.25)
        if not np.any(mask):
            return math.inf
        return float(np.min(arr[mask]))

    def nudge_clear_of_doorway(self, distance_m: float = 0.45, speed: float = 0.14):
        """If starting straddled inside a narrow doorway, drive forward cleanly into the room."""
        self.spin_for(0.3)
        left_min = self._min_scan_in_arc(math.pi / 2.0, half_arc_rad=0.55)
        right_min = self._min_scan_in_arc(-math.pi / 2.0, half_arc_rad=0.55)
        front_min = self._min_scan_in_arc(0.0, half_arc_rad=0.45)

        if not ((left_min < 0.78 and right_min < 0.78) or min(left_min, right_min) < 0.42):
            return
        if front_min < 0.70:
            return

        pose0 = self.get_pose()
        if pose0 is None:
            return
        self.get_logger().info(
            f'[Doorway Clear] Narrow sides (L={left_min:.2f} m, R={right_min:.2f} m, F={front_min:.2f} m); '
            f'nudging {distance_m:.2f} m cleanly into Room Alpha...'
        )
        x0, y0, _ = pose0
        twist = Twist()
        twist.linear.x = abs(speed)
        start = time.time()
        while rclpy.ok() and (time.time() - start) < 15.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self._min_scan_in_arc(0.0, half_arc_rad=0.45) < 0.40:
                break
            pose = self.get_pose()
            if pose is not None and math.hypot(pose[0] - x0, pose[1] - y0) >= distance_m:
                break
            self.cmd_pub.publish(twist)
        self.stop_base()

    def get_pose(self):
        """Return (x, y, yaw) in map frame if TF is available, else fallback to odom."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(), timeout=Duration(seconds=0.08)
            )
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            return t.transform.translation.x, t.transform.translation.y, yaw
        except Exception:
            return self.odom_pose

    def get_cam_transform(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'rgbd_camera_optical_frame', Time(), timeout=Duration(seconds=0.1)
            )
            return transform_stamped_to_matrix(t)
        except Exception:
            return None

    def get_map_array(self):
        if self.map_msg is None:
            return None, 0.0, 0.0, 0.05
        info = self.map_msg.info
        grid = np.array(self.map_msg.data, dtype=np.int16).reshape(info.height, info.width)
        return grid, info.origin.position.x, info.origin.position.y, info.resolution

    def stop_base(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_for(self, duration_sec: float):
        end = time.time() + duration_sec
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.04)

    # --- Autonomous Visual Discovery -----------------------------------------
    def _sample_clean_zone_frame(self, collected_pts: list):
        if self.latest_rgb is None:
            return 0
        T_map_cam = self.get_cam_transform()
        if T_map_cam is None:
            return 0
        intrinsics = intrinsics_from_camera_info(self.camera_info)
        pts = detect_clean_zone_points(
            self.latest_rgb, self.latest_depth, intrinsics, T_map_cam
        )
        if len(pts) > 0:
            collected_pts.extend(pts.tolist())
            return len(pts)
        return 0

    def _rotate_and_scan_clean_zone(self, collected_pts: list, target_angle: float = 2.0 * math.pi,
                                    ang_vel: float = 0.60, timeout_sec: float = 40.0):
        """Rotate in place while projecting any visible clean-zone mat pixels into map frame."""
        pose0 = self.get_pose()
        if pose0 is None:
            return
        prev_yaw = pose0[2]
        turned = 0.0
        last_seq = -1
        start = time.time()
        twist = Twist()
        twist.angular.z = ang_vel

        while rclpy.ok() and turned < target_angle and (time.time() - start) < timeout_sec:
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.03)
            self.cmd_pub.publish(twist)

            pose = self.get_pose()
            if pose is not None:
                turned += abs(normalize_angle(pose[2] - prev_yaw))
                prev_yaw = pose[2]

            if self._rgb_seq != last_seq:
                last_seq = self._rgb_seq
                self._sample_clean_zone_frame(collected_pts)

        self.stop_base()

    def discover_clean_zone(self):
        """Visually discover the Room Alpha cleaning zone bounds (x_min, x_max, y_min, y_max)."""
        self.get_logger().info('[Discovery] Searching for Room Alpha cleaning zone mat via RGB-D...')
        collected_pts = []

        # 0. Ensure robot is clear of the narrow Room Alpha doorway before scanning/planning
        self.nudge_clear_of_doorway()

        # 1. Seed from passive detections accumulated by doorway_navigator during exploration
        self.spin_for(0.5)
        if self.discovered_zone_msg and self.discovered_zone_msg.get('points_sample'):
            seed_pts = self.discovered_zone_msg['points_sample']
            collected_pts.extend(seed_pts)
            self.get_logger().info(
                f'[Discovery] Seeded {len(seed_pts)} clean-zone points from passive mapper.'
            )

        # 2. Perform an initial 360-degree scan from the current pose inside Room Alpha
        self._rotate_and_scan_clean_zone(collected_pts, target_angle=2.0 * math.pi + 0.15)
        self.get_logger().info(
            f'[Discovery] After initial 360° scan: {len(collected_pts)} mat points.'
        )

        # 3. If not found yet, visit reachable free-space viewpoints inside Room Alpha
        if len(collected_pts) < 40:
            grid, ox, oy, res = self.get_map_array()
            pose = self.get_pose()
            if grid is not None and pose is not None:
                viewpoints = sample_room_search_viewpoints(
                    (pose[0], pose[1]), grid, ox, oy, res,
                    max_radius_m=5.5, min_clearance_m=0.42, spacing_m=1.6,
                    max_points=5, forward_hint_yaw=pose[2],
                )
                self.get_logger().info(
                    f'[Discovery] Mat not yet seen; checking {len(viewpoints)} room viewpoints...'
                )
                for idx, (vx, vy) in enumerate(viewpoints, start=1):
                    self.navigate_to(vx, vy, 0.0, f'Alpha Search Viewpoint {idx}')
                    if self.latest_rgb is not None:
                        fl, _, _ = classify_floor_color(self.latest_rgb)
                        if fl == 'GREEN':
                            continue
                    self._rotate_and_scan_clean_zone(collected_pts, target_angle=2.0 * math.pi)
                    self.get_logger().info(
                        f'[Discovery] Viewpoint {idx}: accumulated {len(collected_pts)} mat points.'
                    )
                    if len(collected_pts) >= 60:
                        break

        if len(collected_pts) < 20:
            self.get_logger().error('[Discovery] Failed to visually detect Room Alpha cleaning zone!')
            return None

        # 4. If initial detections were far away (> 1.8 m), move closer for a crisp observation
        pts_arr = np.asarray(collected_pts, dtype=np.float64)
        init_c = np.median(pts_arr, axis=0)
        pose = self.get_pose()
        if pose is not None and math.hypot(pose[0] - init_c[0], pose[1] - init_c[1]) > 1.8:
            grid, ox, oy, res = self.get_map_array()
            obs_x, obs_y, obs_yaw = select_approach_pose(
                float(init_c[0]), float(init_c[1]), 1.5, grid, ox, oy, res,
                robot_xy=(pose[0], pose[1]), clearance_m=0.42, ignore_target_radius_m=0.0,
            )
            self.get_logger().info(
                f'[Discovery] Moving closer to ({obs_x:.2f}, {obs_y:.2f}) to refine clean-zone bounds...'
            )
            self.navigate_to(obs_x, obs_y, obs_yaw, 'Clean Zone Refinement Viewpoint')
            self._rotate_and_scan_clean_zone(collected_pts, target_angle=0.7, ang_vel=0.35, timeout_sec=12.0)
            self._rotate_and_scan_clean_zone(collected_pts, target_angle=1.4, ang_vel=-0.35, timeout_sec=18.0)

        grid, ox, oy, res = self.get_map_array()
        bounds = estimate_clean_zone_bounds(
            collected_pts,
            nominal_sx=float(self.get_parameter('nominal_zone_sx').value),
            nominal_sy=float(self.get_parameter('nominal_zone_sy').value),
            grid=grid, origin_x=ox, origin_y=oy, resolution=res,
        )
        if bounds is not None:
            self.get_logger().info(
                f'[Discovery] Localized Clean Zone: '
                f'x=[{bounds[0]:.2f}, {bounds[1]:.2f}], y=[{bounds[2]:.2f}, {bounds[3]:.2f}] '
                f'({bounds[1] - bounds[0]:.2f} x {bounds[3] - bounds[2]:.2f} m, '
                f'from {len(collected_pts)} floor points)'
            )
        return bounds

    # --- Movement: Closed-Loop /cmd_vel --------------------------------------
    def drive_to_pose_closed_loop(
        self,
        target_x: float,
        target_y: float,
        target_yaw: float,
        pos_tol: float = 0.05,
        yaw_tol: float = 0.05,
        max_lin: float = 0.22,
        max_ang: float = 0.70,
        timeout_sec: float = 60.0,
        obstacle_guard: bool = True,
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
                if obstacle_guard and dist > 0.20:
                    front_min = self._min_scan_in_arc(0.0, half_arc_rad=0.45)
                    if front_min < 0.30:
                        self.stop_base()
                        self.get_logger().warn(
                            f'[Closed-Loop Guard] Obstacle at {front_min:.2f} m ahead; stopping safely.'
                        )
                        return False
                err_yaw = normalize_angle(heading_to_goal - yaw)
                if abs(err_yaw) > 0.45 and dist > 0.12:
                    phase = 'ROTATE_TO_GOAL'
                    continue
                lin = min(max_lin, max(0.06, 0.8 * dist))
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
        return False

    def navigate_to(self, x: float, y: float, yaw: float, label: str = '',
                    nav2_timeout_sec: float = 90.0) -> bool:
        """Route to target using Nav2 Simple Commander with timeout & safe fallback."""
        if label:
            self.get_logger().info(f'[Navigate] -> {label}: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')

        if self.navigator is not None:
            from nav2_simple_commander.robot_navigator import TaskResult
            for attempt in range(1, 3):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = Time().to_msg()
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)

                self.navigator.goToPose(pose)
                start = time.time()
                timed_out = False
                while not self.navigator.isTaskComplete():
                    for _ in range(50):
                        rclpy.spin_once(self, timeout_sec=0.0)
                    time.sleep(0.05)
                    if (time.time() - start) > nav2_timeout_sec:
                        self.get_logger().warn(
                            f'Nav2 goal {label} timed out after {nav2_timeout_sec:.0f}s (attempt {attempt}); cancelling.'
                        )
                        self.navigator.cancelTask()
                        timed_out = True
                        break

                if not timed_out and self.navigator.getResult() == TaskResult.SUCCEEDED:
                    return True

                self.get_logger().warn(f'Nav2 goal {label} did not succeed (attempt {attempt}); clearing costmaps.')
                try:
                    self.navigator.clearAllCostmaps()
                except Exception:
                    pass
                self.spin_for(0.5)

            cur = self.get_pose()
            rem = math.hypot(x - cur[0], y - cur[1]) if cur is not None else math.inf
            if rem > 1.2:
                self.get_logger().warn(
                    f'Skipping blind closed-loop fallback for {label} ({rem:.2f} m away).'
                )
                return False

        return self.drive_to_pose_closed_loop(x, y, yaw, obstacle_guard=True)

    # --- Arm & Vacuum Controls -----------------------------------------------
    def detach_beta_cube(self):
        """Ensure Gazebo's DetachableJoint is detached so the Room Beta cube is not dragged."""
        self.get_logger().info('Ensuring Beta cube DetachableJoint is detached...')
        for _ in range(4):
            self.detach_pub.publish(Empty())
            self.spin_for(0.08)

    def move_arm_home(self):
        """Fold manipulator arm to compact home pose."""
        traj = JointTrajectory()
        traj.joint_names = list(self.ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in self.HOME_POSE]
        pt.time_from_start = MsgDuration(sec=2, nanosec=0)
        traj.points.append(pt)
        for _ in range(3):
            self.arm_pub.publish(traj)
            rclpy.spin_once(self, timeout_sec=0.04)
        self.spin_for(1.5)

    def set_vacuum(self, active: bool):
        self.vacuum_active = active
        msg = Bool(data=active)
        for _ in range(3):
            self.vacuum_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
        state_str = 'ACTIVE (ON)' if active else 'OFF'
        self.get_logger().info(f'[Vacuum] State -> {state_str}')

    # --- Gazebo Dirt Management ----------------------------------------------
    def _hoover_pose(self):
        """Robot pose in the map frame for the dirt check."""
        if self.odom_pose is None:
            return self.get_pose()
        try:
            t = self.tf_buffer.lookup_transform('map', 'odom', Time(),
                                                timeout=Duration(seconds=0.0))
        except Exception:
            return self.get_pose()
        q = t.transform.rotation
        oyaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        ox, oy = t.transform.translation.x, t.transform.translation.y
        x, y, yaw = self.odom_pose
        return (ox + x * math.cos(oyaw) - y * math.sin(oyaw),
                oy + x * math.sin(oyaw) + y * math.cos(oyaw),
                yaw + oyaw)

    def _check_hoover_dirt(self):
        pose = self._hoover_pose()
        if pose is None or not self.active_dirt:
            return

        rx, ry, ryaw = pose
        vx = rx + self.VACUUM_OFFSET_X * math.cos(ryaw)
        vy = ry + self.VACUUM_OFFSET_X * math.sin(ryaw)
        cos_yaw, sin_yaw = math.cos(ryaw), math.sin(ryaw)
        extent = self.VACUUM_HALF_SIZE + self.DIRT_RADIUS

        to_hoover = []
        for name, (px, py) in list(self.active_dirt.items()):
            dx = px - vx
            dy = py - vy
            loc_x = dx * cos_yaw + dy * sin_yaw
            loc_y = -dx * sin_yaw + dy * cos_yaw
            if abs(loc_x) <= extent and abs(loc_y) <= extent:
                to_hoover.append((name, px, py))

        for name, px, py in to_hoover:
            del self.active_dirt[name]
            self.hoovered_count += 1
            pct = int(100.0 * self.hoovered_count / max(1, self.total_dirt_count))
            self.get_logger().info(
                f'[Vacuum] Hoovered {name} at ({px:.2f}, {py:.2f}) '
                f'| Progress: {self.hoovered_count}/{self.total_dirt_count} ({pct}%)'
            )
            req = DeleteEntity.Request()
            req.entity.name = name
            req.entity.type = Entity.MODEL
            self.delete_client.call_async(req)

    def spawn_dirt_grid(self, x_min: float, x_max: float, y_min: float, y_max: float):
        if not self.spawn_client.wait_for_service(timeout_sec=2.5):
            self.get_logger().warn('Gazebo service /world/empty/create unavailable; skipping dirt spawn.')
            return

        spawn_x = float(self.get_parameter('spawn_x').value)
        spawn_y = float(self.get_parameter('spawn_y').value)

        mx_lo = x_min + min(0.30, 0.15 * (x_max - x_min))
        mx_hi = x_max - min(0.30, 0.15 * (x_max - x_min))
        my_lo = y_min + min(0.25, 0.18 * (y_max - y_min))
        my_hi = y_max - min(0.25, 0.18 * (y_max - y_min))
        xs = np.linspace(mx_lo, mx_hi, 3)
        ys = np.linspace(my_lo, my_hi, 3)
        dirt_coords = [(float(dx), float(dy)) for dy in ys for dx in xs]

        self.get_logger().info(
            f'Spawning {len(dirt_coords)} dirt particles across clean zone '
            f'x=[{x_min:.2f}, {x_max:.2f}], y=[{y_min:.2f}, {y_max:.2f}]...'
        )
        self.active_dirt.clear()
        for idx, (dx, dy) in enumerate(dirt_coords, start=1):
            name = f'alpha_dirt_{idx}'
            del_req = DeleteEntity.Request()
            del_req.entity.name = name
            del_req.entity.type = Entity.MODEL
            self.delete_client.call_async(del_req)

            req = SpawnEntity.Request()
            req.entity_factory.name = name
            req.entity_factory.allow_renaming = False
            req.entity_factory.sdf = DIRT_SDF_TEMPLATE.format(name=name)
            req.entity_factory.pose.position.x = float(dx + spawn_x)
            req.entity_factory.pose.position.y = float(dy + spawn_y)
            req.entity_factory.pose.position.z = 0.002
            req.entity_factory.pose.orientation.w = 1.0
            req.entity_factory.relative_to = 'world'
            self.spawn_client.call_async(req)
            self.active_dirt[name] = (dx, dy)

        self.total_dirt_count = len(self.active_dirt)
        self.hoovered_count = 0
        self.get_logger().info(f'Spawned {self.total_dirt_count} dirt particles.')

    # --- Lawnmower Coverage Generation ---------------------------------------
    def generate_lawnmower_waypoints(self, x_min: float, x_max: float,
                                     y_min: float, y_max: float) -> list:
        spacing = float(self.get_parameter('lane_spacing').value)

        x_west = x_min + 0.15
        x_east = x_max - 0.15
        y_start = y_min + self.VACUUM_HALF_SIZE
        y_end = y_max - self.VACUUM_HALF_SIZE

        num_lanes = max(2, int(round((y_end - y_start) / spacing)) + 1)
        actual_spacing = (y_end - y_start) / (num_lanes - 1)

        waypoints = []
        for i in range(num_lanes):
            lane_y = y_start + i * actual_spacing
            if i % 2 == 0:
                waypoints.append((x_west, lane_y, 0.0, f'Lane {i+1} West'))
                waypoints.append((x_east, lane_y, 0.0, f'Lane {i+1} East'))
            else:
                waypoints.append((x_east, lane_y, math.pi, f'Lane {i+1} East'))
                waypoints.append((x_west, lane_y, math.pi, f'Lane {i+1} West'))

        return waypoints

    # --- Main Mission --------------------------------------------------------
    def execute_mission(self) -> bool:
        self.get_logger().info('=' * 60)
        self.get_logger().info('Starting Room Alpha Cleaning Mission...')
        self.get_logger().info('=' * 60)

        # 1. Wait for initial robot pose (/odom or TF)
        self.get_logger().info('Waiting for robot pose (/odom or TF)...')
        start_wait = time.time()
        while rclpy.ok() and self.get_pose() is None and (time.time() - start_wait) < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.get_pose() is None:
            self.get_logger().error('No robot pose received!')
            return False

        # 2. Wait for /room_alpha/ready if handoff enabled
        if self.get_parameter('wait_for_ready').value:
            self.get_logger().info('Waiting for /room_alpha/ready trigger...')
            while rclpy.ok() and not self.room_alpha_ready:
                rclpy.spin_once(self, timeout_sec=0.2)

        discover = bool(self.get_parameter('discover_targets').value)
        clean_x_min = float(self.get_parameter('clean_x_min').value)
        clean_x_max = float(self.get_parameter('clean_x_max').value)
        clean_y_min = float(self.get_parameter('clean_y_min').value)
        clean_y_max = float(self.get_parameter('clean_y_max').value)

        mission_t0 = time.time()
        failure_reasons = []

        try:
            # 3. Ensure Beta cube starts detached and fold arm into safe home pose
            self.detach_beta_cube()
            self.move_arm_home()

            # 4. Determine target cleaning zone: either autonomous discovery or parameter coords
            if discover:
                bounds = self.discover_clean_zone()
                if bounds is None:
                    failure_reasons.append('clean_zone_not_found')
                    return False
                clean_x_min, clean_x_max, clean_y_min, clean_y_max = bounds
            else:
                # Standalone mode: route from Hallway into Room Alpha if outside
                cur_pose = self.get_pose()
                if cur_pose and (cur_pose[1] < 6.40 or cur_pose[0] > -9.21):
                    self.get_logger().info('Routing from Hallway into Room Alpha...')
                    self.navigate_to(-14.61, 5.20, math.pi / 2.0, 'Door Alpha Hallway Approach')
                    self.navigate_to(-14.61, 7.40, 0.0, 'Inside Room Alpha Entrance')

            # 5. Spawn dirt particles inside the active clean zone
            if self.get_parameter('spawn_dirt').value:
                self.spawn_dirt_grid(clean_x_min, clean_x_max, clean_y_min, clean_y_max)
                self.spin_for(0.5)

            # 6. Generate Lawnmower Pattern and Execute
            waypoints = self.generate_lawnmower_waypoints(
                clean_x_min, clean_x_max, clean_y_min, clean_y_max
            )
            first_wp = waypoints[0]
            self.navigate_to(first_wp[0], first_wp[1], first_wp[2], 'Clean Zone Entry')

            # Turn ON vacuum
            self.set_vacuum(True)
            self.get_logger().info(f'[Vacuum ON] Executing {len(waypoints) // 2} lawnmower cleaning lanes...')

            for wp in waypoints:
                self.navigate_to(wp[0], wp[1], wp[2], wp[3])

            # Turn OFF vacuum
            self.set_vacuum(False)
            self.get_logger().info('[Vacuum OFF] Cleaning coverage sweep completed!')

            # 7. Return to doorway only in standalone non-discovery mode
            if not discover and self.get_parameter('return_to_doorway').value:
                self.navigate_to(-14.61, 7.40, -math.pi / 2.0, 'Room Alpha Doorway Exit')

            # 8. Publish Completion Signal (/room_alpha/done)
            for _ in range(3):
                self.done_pub.publish(Bool(data=True))
                self.spin_for(0.1)

            total_time = time.time() - mission_t0
            dirt_ratio = (
                float(self.hoovered_count) / max(1, self.total_dirt_count)
                if self.total_dirt_count > 0 else 1.0
            )

            self.get_logger().info('=' * 60)
            self.get_logger().info(
                f'Alpha Cleaning COMPLETE in {total_time:.1f}s! '
                f'Hoovered: {self.hoovered_count}/{self.total_dirt_count} ({dirt_ratio * 100.0:.1f}%) | '
                f'/room_alpha/done published.'
            )
            self.get_logger().info('=' * 60)
            return True

        except KeyboardInterrupt:
            failure_reasons.append('interrupted')
            return False
        except Exception as exc:
            failure_reasons.append(f'exception:{type(exc).__name__}')
            self.get_logger().error(f'Mission failed with exception: {exc}')
            return False
        finally:
            self.set_vacuum(False)
            self.stop_base()
            eval_record = {
                'timestamp_utc': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
                'trial_id': 1,
                'nav_mode': 'nav2' if self.navigator is not None else 'closed_loop',
                'clean_x_min': f'{clean_x_min:.3f}',
                'clean_x_max': f'{clean_x_max:.3f}',
                'clean_y_min': f'{clean_y_min:.3f}',
                'clean_y_max': f'{clean_y_max:.3f}',
                'dirt_total': self.total_dirt_count,
                'dirt_cleaned': self.hoovered_count,
                'dirt_ratio': f'{(self.hoovered_count / max(1, self.total_dirt_count)):.2f}' if self.total_dirt_count > 0 else '1.00',
                'total_time_sec': f'{(time.time() - mission_t0):.1f}',
                'mission_success': len(failure_reasons) == 0,
                'failure_reason': '; '.join(failure_reasons),
            }
            csv_path = str(self.get_parameter('eval_log_path').value)
            try:
                trial_id = append_eval_csv(csv_path, eval_record)
                self.get_logger().info(f'[Eval] Logged Trial #{trial_id} to {csv_path}')
            except Exception as e:
                self.get_logger().error(f'[Eval] Failed writing CSV: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AlphaCleaning()
    success = False
    try:
        success = node.execute_mission()
    except KeyboardInterrupt:
        node.get_logger().info('Alpha Cleaning interrupted by user.')
    finally:
        node.set_vacuum(False)
        node.stop_base()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
