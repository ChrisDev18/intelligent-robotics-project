#!/usr/bin/env python3
"""Phase D: Room Beta Pick and Place Task Node.

Supports two targeting modes selectable via parameter `discover_targets`:
1. Autonomous Discovery Mode (`discover_targets:=true`, used by mission.launch.py):
   Visually locates both the orange 0.15 m cube (`beta_pick_item`) and the
   crimson/magenta drop pad (`beta_drop_spot`) using the RGB-D camera, TF, and
   live SLAM /map (seeded from /discovered/box_pose & /discovered/drop_pose plus
   active 360° scan and free-space room viewpoint search). Chooses collision-free
   approach angles dynamically on /map, refines the box pose at close range via
   RGB-D, grasps, carries via Nav2, and places on the discovered drop pad.
2. Standalone / Parameter Mode (`discover_targets:=false`, default):
   Uses `box_x/y` and `drop_x/y` parameters directly for isolated/practice testing.
"""
import csv
from datetime import datetime, timezone
import json
import math
import os
import statistics
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
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from std_msgs.msg import Bool, Empty, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot_bringup.perception import (
    classify_floor_color,
    detect_drop_spot_map,
    detect_orange_cube_map,
    estimate_cluster_centroid,
    intrinsics_from_camera_info,
    sample_room_search_viewpoints,
    segment_orange_cube,
    select_approach_pose,
    transform_stamped_to_matrix,
)


EVAL_CSV_COLUMNS = [
    'timestamp_utc',
    'trial_id',
    'scenario',
    'pose_source',
    'box_target_x',
    'box_target_y',
    'drop_target_x',
    'drop_target_y',
    'pickup_pose_x',
    'pickup_pose_y',
    'pickup_pose_yaw',
    'pickup_grasp_x',
    'pickup_grasp_y',
    'cube_initial_x',
    'cube_initial_y',
    'cube_initial_z',
    'cube_lifted_z',
    'cube_lift_delta_z',
    'pickup_dx_m',
    'pickup_dy_m',
    'pickup_distance_error_m',
    'pickup_estimated_error_m',
    'pick_threshold_m',
    'pick_success',
    'place_pose_x',
    'place_pose_y',
    'place_pose_yaw',
    'cube_final_x',
    'cube_final_y',
    'cube_final_z',
    'placement_dx_m',
    'placement_dy_m',
    'placement_distance_error_m',
    'placement_estimated_error_m',
    'place_threshold_m',
    'place_success',
    'mission_success',
    'pick_time_sec',
    'place_time_sec',
    'total_time_sec',
    'failure_reason',
]


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_grasp_point(rx: float, ry: float, ryaw: float, standoff_m: float):
    """Return nominal 2D grasp point (x, y) located `standoff_m` ahead of the robot."""
    return (
        rx + standoff_m * math.cos(ryaw),
        ry + standoff_m * math.sin(ryaw),
    )


def _fmt_float(val, digits: int = 4) -> str:
    if val is None:
        return ''
    return f'{float(val):.{digits}f}'


def append_eval_csv(csv_path: str, record: dict) -> int:
    """Append a single trial record to `csv_path` and return its assigned 1-based trial_id."""
    parent = os.path.dirname(os.path.abspath(csv_path))
    if parent:
        os.makedirs(parent, exist_ok=True)

    existing_rows = 0
    file_exists = os.path.isfile(csv_path) and os.path.getsize(csv_path) > 0
    if file_exists:
        with open(csv_path, 'r', newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            existing_rows = sum(1 for _ in reader)

    trial_id = existing_rows + 1
    record['trial_id'] = trial_id

    row = {}
    for col in EVAL_CSV_COLUMNS:
        val = record.get(col, '')
        if isinstance(val, bool):
            row[col] = 'True' if val else 'False'
        elif isinstance(val, float):
            row[col] = _fmt_float(val, 4)
        elif val is None:
            row[col] = ''
        else:
            row[col] = str(val)

    with open(csv_path, 'a', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=EVAL_CSV_COLUMNS)
        if not file_exists:
            writer.writeheader()
        writer.writerow(row)

    return trial_id


def summarize_eval_csv(csv_path: str) -> dict:
    """Compute separate pick, place, conditional place|pick, and overall stats from CSV."""
    if not os.path.isfile(csv_path):
        return {'total': 0}

    with open(csv_path, 'r', newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))

    total = len(rows)
    if total == 0:
        return {'total': 0}

    def _to_bool(v) -> bool:
        return str(v).strip().lower() in ('true', '1', 'yes')

    def _to_float(v):
        try:
            s = str(v).strip()
            return float(s) if s != '' else None
        except ValueError:
            return None

    pick_ok = [_to_bool(r.get('pick_success')) for r in rows]
    place_ok = [_to_bool(r.get('place_success')) for r in rows]
    mission_ok = [_to_bool(r.get('mission_success')) for r in rows]

    pick_errs = [e for e in (_to_float(r.get('pickup_distance_error_m')) for r in rows) if e is not None]
    place_errs = [e for e in (_to_float(r.get('placement_distance_error_m')) for r in rows) if e is not None]

    n_pick = sum(pick_ok)
    n_place = sum(place_ok)
    n_place_given_pick = sum(1 for pk, pl in zip(pick_ok, place_ok) if pk and pl)
    n_mission = sum(mission_ok)

    def _err_stats(vals):
        if not vals:
            return {'count': 0, 'mean': None, 'std': None, 'min': None, 'max': None}
        return {
            'count': len(vals),
            'mean': statistics.mean(vals),
            'std': statistics.stdev(vals) if len(vals) > 1 else 0.0,
            'min': min(vals),
            'max': max(vals),
        }

    return {
        'total': total,
        'pick_success_count': n_pick,
        'pick_success_rate': n_pick / total,
        'place_success_count': n_place,
        'place_success_rate': n_place / total,
        'place_given_pick_count': n_place_given_pick,
        'place_given_pick_rate': (n_place_given_pick / n_pick) if n_pick > 0 else None,
        'mission_success_count': n_mission,
        'mission_success_rate': n_mission / total,
        'pickup_error': _err_stats(pick_errs),
        'placement_error': _err_stats(place_errs),
    }


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
    # Nominal resting height of the 0.15 m cube centre on the floor (metres)
    CUBE_REST_Z_M = 0.075

    def __init__(self):
        super().__init__('beta_pick_place')

        # Task & Navigation Parameters
        self.declare_parameter('wait_for_ready', False)
        self.declare_parameter('discover_targets', False)
        self.declare_parameter('box_x', -6.35)
        self.declare_parameter('box_y', 7.42)
        self.declare_parameter('drop_x', -0.23)
        self.declare_parameter('drop_y', 8.51)
        self.declare_parameter('use_visual_servo', True)

        # Evaluation & Logging Parameters (read-only ground truth; never used for control)
        self.declare_parameter('spawn_x', 19.21)
        self.declare_parameter('spawn_y', 0.79)
        self.declare_parameter('eval_log_path', 'log/beta_pick_place_eval.csv')
        self.declare_parameter('scenario', '')
        self.declare_parameter('pick_error_threshold_m', 0.15)
        self.declare_parameter('pick_lift_threshold_m', 0.08)
        self.declare_parameter('place_error_threshold_m', 0.27)
        self.declare_parameter('place_max_z_m', 0.15)

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
        self.done_pub = self.create_publisher(Bool, '/room_beta/done', latched_qos)
        self.eval_pub = self.create_publisher(
            String, '/beta_pick_place/eval_metrics', latched_qos
        )

        # Action & Service clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )
        self.clear_local_costmap = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'
        )
        self.clear_global_costmap = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'
        )

        # TF & Subscriptions
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.room_beta_ready = False
        self.odom_pose = None
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_scan = None
        self.camera_info = None
        self.map_msg = None
        self.discovered_box_msg = None
        self.discovered_drop_msg = None
        self._rgb_seq = 0
        self.arm_joint_positions = {}

        # Gazebo ground-truth poses (evaluation only, populated via /eval/gz_dynamic_poses)
        self.gz_robot_pose = None  # (wx, wy, wz, yaw) in Gazebo world frame
        self.gz_cube_pose = None   # (wx, wy, wz, yaw) in Gazebo world frame

        self.create_subscription(Bool, '/room_beta/ready', self._ready_cb, latched_qos)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/image', self._rgb_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/depth_image', self._depth_cb, 10)
        self.create_subscription(CameraInfo, '/rgbd_camera/camera_info', self._cam_info_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, latched_qos)
        self.create_subscription(String, '/discovered/box_pose', self._discovered_box_cb, latched_qos)
        self.create_subscription(String, '/discovered/drop_pose', self._discovered_drop_cb, latched_qos)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_subscription(
            TFMessage, '/eval/gz_dynamic_poses', self._gz_poses_cb, 10
        )

        disc_str = 'autonomous-discovery' if self.get_parameter('discover_targets').value else 'parameter-coords'
        self.get_logger().info(f'Beta Pick & Place node initialised in [{disc_str}] mode.')

    # --- Callbacks -----------------------------------------------------------
    def _ready_cb(self, msg: Bool):
        if msg.data:
            self.room_beta_ready = True

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_pose = (p.x, p.y, yaw)

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _min_scan_in_arc(self, center_angle_rad: float, half_arc_rad: float = 0.55) -> float:
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

    def clear_costmaps(self):
        """Clear Nav2 local and global costmaps (e.g. after lifting the cube)."""
        for name, client in (
            ('local', self.clear_local_costmap),
            ('global', self.clear_global_costmap),
        ):
            if client.wait_for_service(timeout_sec=1.0):
                fut = client.call_async(ClearEntireCostmap.Request())
                t0 = time.time()
                while rclpy.ok() and not fut.done() and (time.time() - t0) < 2.0:
                    rclpy.spin_once(self, timeout_sec=0.05)
                self.get_logger().info(f'[Nav2] Cleared {name} costmap.')

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

    def _discovered_box_cb(self, msg: String):
        try:
            self.discovered_box_msg = json.loads(msg.data)
        except Exception:
            pass

    def _discovered_drop_cb(self, msg: String):
        try:
            self.discovered_drop_msg = json.loads(msg.data)
        except Exception:
            pass

    def _joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.arm_joint_positions[name] = float(pos)

    def _gz_poses_cb(self, msg: TFMessage):
        """Record Gazebo ground-truth model poses (strictly for evaluation logging)."""
        for tr in msg.transforms:
            child = tr.child_frame_id
            parent = tr.header.frame_id
            if parent not in ('empty', 'world', '') or child == parent:
                continue
            t = tr.transform.translation
            q = tr.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            pose = (float(t.x), float(t.y), float(t.z), yaw)
            if child == 'beta_pick_item':
                self.gz_cube_pose = pose
            elif child == 'turtlebot3_manipulation':
                self.gz_robot_pose = pose

    # --- Evaluation Helpers (Read-Only Ground Truth) -------------------------
    def _world_to_map(self, wx: float, wy: float):
        spawn_x = float(self.get_parameter('spawn_x').value)
        spawn_y = float(self.get_parameter('spawn_y').value)
        return wx - spawn_x, wy - spawn_y

    def has_gz_ground_truth(self) -> bool:
        return self.gz_robot_pose is not None or self.gz_cube_pose is not None

    def get_gz_robot_pose_map(self):
        if self.gz_robot_pose is None:
            return None
        wx, wy, wz, yaw = self.gz_robot_pose
        mx, my = self._world_to_map(wx, wy)
        return mx, my, wz, yaw

    def get_gz_cube_pose_map(self):
        if self.gz_cube_pose is None:
            return None
        wx, wy, wz, yaw = self.gz_cube_pose
        mx, my = self._world_to_map(wx, wy)
        return mx, my, wz, yaw

    def _record_and_report_eval(self, eval_record: dict):
        csv_path = str(self.get_parameter('eval_log_path').value)
        try:
            trial_id = append_eval_csv(csv_path, eval_record)
            summary = summarize_eval_csv(csv_path)
        except Exception as exc:
            self.get_logger().error(f'[Eval] Failed writing CSV log ({csv_path}): {exc}')
            trial_id = eval_record.get('trial_id', 0)
            summary = {'total': 0}

        try:
            if rclpy.ok():
                payload = json.dumps(eval_record, default=str)
                msg = String(data=payload)
                for _ in range(2):
                    self.eval_pub.publish(msg)
                    self.spin_for(0.05)
        except Exception:
            pass

        pk_err = eval_record.get('pickup_distance_error_m')
        pl_err = eval_record.get('placement_distance_error_m')
        pk_str = f'{pk_err:.4f} m' if pk_err is not None else 'N/A'
        pl_str = f'{pl_err:.4f} m' if pl_err is not None else 'N/A'

        self.get_logger().info('-' * 60)
        self.get_logger().info(
            f'[Eval] Trial #{trial_id} | source={eval_record.get("pose_source")} | '
            f'scenario="{eval_record.get("scenario", "")}"'
        )
        self.get_logger().info(
            f'[Eval]   PICK : {"SUCCESS" if eval_record["pick_success"] else "FAIL   "} | '
            f'dist_err={pk_str} (thresh={eval_record["pick_threshold_m"]:.3f} m) | '
            f'lift_dz={_fmt_float(eval_record.get("cube_lift_delta_z"), 3) or "N/A"} m'
        )
        self.get_logger().info(
            f'[Eval]   PLACE: {"SUCCESS" if eval_record["place_success"] else "FAIL   "} | '
            f'dist_err={pl_str} (thresh={eval_record["place_threshold_m"]:.3f} m) | '
            f'final_z={_fmt_float(eval_record.get("cube_final_z"), 3) or "N/A"} m'
        )
        self.get_logger().info(
            f'[Eval]   TOTAL: {"SUCCESS" if eval_record["mission_success"] else "FAIL   "} | '
            f'time={_fmt_float(eval_record.get("total_time_sec"), 1)} s | '
            f'log={os.path.abspath(csv_path)}'
        )

        if summary.get('total', 0) > 0:
            n = summary['total']
            pk_cnt = summary['pick_success_count']
            pl_cnt = summary['place_success_count']
            ms_cnt = summary['mission_success_count']
            pk_stats = summary['pickup_error']
            pl_stats = summary['placement_error']

            pk_mean = f'{pk_stats["mean"]:.4f}±{pk_stats["std"]:.4f} m' if pk_stats['mean'] is not None else 'N/A'
            pl_mean = f'{pl_stats["mean"]:.4f}±{pl_stats["std"]:.4f} m' if pl_stats['mean'] is not None else 'N/A'
            cond_rate = summary['place_given_pick_rate']
            cond_str = (
                f'{summary["place_given_pick_count"]}/{pk_cnt} ({100.0 * cond_rate:.1f}%)'
                if cond_rate is not None else 'N/A'
            )

            self.get_logger().info(f'[Eval] Cumulative Rates across {n} trial(s):')
            self.get_logger().info(
                f'[Eval]   Pick Success Rate       : {pk_cnt}/{n} ({100.0 * summary["pick_success_rate"]:.1f}%) | err={pk_mean}'
            )
            self.get_logger().info(
                f'[Eval]   Place Success Rate      : {pl_cnt}/{n} ({100.0 * summary["place_success_rate"]:.1f}%) | err={pl_mean}'
            )
            self.get_logger().info(
                f'[Eval]   Place | Pick Rate       : {cond_str}'
            )
            self.get_logger().info(
                f'[Eval]   Overall Mission Success : {ms_cnt}/{n} ({100.0 * summary["mission_success_rate"]:.1f}%)'
            )
        self.get_logger().info('-' * 60)

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

    # --- Autonomous Visual Discovery (Room Beta Box & Drop Spot) -------------
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
            f'nudging {distance_m:.2f} m cleanly into Room Beta...'
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

    def _sample_beta_targets_frame(self, box_obs: list, drop_obs: list):
        if self.latest_rgb is None:
            return
        T_map_cam = self.get_cam_transform()
        if T_map_cam is None:
            return
        intrinsics = intrinsics_from_camera_info(self.camera_info)

        box = detect_orange_cube_map(
            self.latest_rgb, self.latest_depth, intrinsics, T_map_cam
        )
        if box is not None:
            box_obs.append((box[0], box[1]))

        drop = detect_drop_spot_map(
            self.latest_rgb, self.latest_depth, intrinsics, T_map_cam
        )
        if drop is not None:
            drop_obs.append((drop[0], drop[1]))

    def _rotate_and_scan_beta(self, box_obs: list, drop_obs: list,
                              target_angle: float = 2.0 * math.pi,
                              ang_vel: float = 0.60, timeout_sec: float = 40.0,
                              stop_when_both_found: bool = True):
        """Rotate in place scanning for both the orange cube and crimson drop pad."""
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
                self._sample_beta_targets_frame(box_obs, drop_obs)
                if stop_when_both_found and len(box_obs) >= 5 and len(drop_obs) >= 5:
                    break

        self.stop_base()

    def discover_beta_targets(self):
        """Visually discover both (box_x, box_y) and (drop_x, drop_y) inside Room Beta."""
        self.get_logger().info(
            '[Discovery] Searching Room Beta for orange cube and crimson drop pad via RGB-D...'
        )
        box_obs = []
        drop_obs = []

        # 0. Ensure robot is clear of the narrow Room Beta doorway before scanning/planning
        self.nudge_clear_of_doorway()

        # 1. Seed from passive detections published by doorway_navigator
        self.spin_for(0.5)
        if self.discovered_box_msg and 'x' in self.discovered_box_msg:
            bx = float(self.discovered_box_msg['x'])
            by = float(self.discovered_box_msg['y'])
            box_obs.extend([(bx, by)] * min(3, int(self.discovered_box_msg.get('observations', 1))))
            self.get_logger().info(f'[Discovery] Seeded box pose ({bx:.2f}, {by:.2f}) from passive mapper.')
        if self.discovered_drop_msg and 'x' in self.discovered_drop_msg:
            dx = float(self.discovered_drop_msg['x'])
            dy = float(self.discovered_drop_msg['y'])
            drop_obs.extend([(dx, dy)] * min(3, int(self.discovered_drop_msg.get('observations', 1))))
            self.get_logger().info(f'[Discovery] Seeded drop pose ({dx:.2f}, {dy:.2f}) from passive mapper.')

        # 2. Initial 360-degree scan inside Room Beta
        if len(box_obs) < 2 or len(drop_obs) < 2:
            self._rotate_and_scan_beta(box_obs, drop_obs, target_angle=2.0 * math.pi + 0.15)
            self.get_logger().info(
                f'[Discovery] After initial 360° scan: box_obs={len(box_obs)}, drop_obs={len(drop_obs)}'
            )

        # 3. Active viewpoint search across free space in Room Beta until both targets are seen
        if len(box_obs) < 2 or len(drop_obs) < 2:
            grid, ox, oy, res = self.get_map_array()
            pose = self.get_pose()
            if grid is not None and pose is not None:
                viewpoints = sample_room_search_viewpoints(
                    (pose[0], pose[1]), grid, ox, oy, res,
                    max_radius_m=6.5, min_clearance_m=0.40, spacing_m=1.7,
                    max_points=8, forward_hint_yaw=pose[2],
                )
                self.get_logger().info(
                    f'[Discovery] Visiting up to {len(viewpoints)} Room Beta viewpoints to locate missing target(s)...'
                )
                for idx, (vx, vy) in enumerate(viewpoints, start=1):
                    if len(box_obs) >= 2 and len(drop_obs) >= 2:
                        break
                    self.navigate_to_pose(
                        vx, vy, 0.0, f'Beta Search Viewpoint {idx}',
                        fine_align=False, pos_tol=0.18,
                        max_nav2_retries=0, nav2_timeout_sec=45.0,
                    )
                    if self.latest_rgb is not None:
                        fl, _, _ = classify_floor_color(self.latest_rgb)
                        if fl == 'YELLOW':
                            continue
                    self._rotate_and_scan_beta(box_obs, drop_obs, target_angle=2.0 * math.pi)
                    self.get_logger().info(
                        f'[Discovery] Viewpoint {idx}: box_obs={len(box_obs)}, drop_obs={len(drop_obs)}'
                    )

        box_xy = estimate_cluster_centroid(box_obs) if box_obs else None
        drop_xy = estimate_cluster_centroid(drop_obs) if drop_obs else None

        if box_xy is None:
            self.get_logger().error('[Discovery] Failed to visually detect Beta orange cube!')
        else:
            self.get_logger().info(
                f'[Discovery] Localized Beta Cube at ({box_xy[0]:.3f}, {box_xy[1]:.3f}) '
                f'({len(box_obs)} obs)'
            )

        if drop_xy is None:
            self.get_logger().error('[Discovery] Failed to visually detect Beta drop pad!')
        else:
            self.get_logger().info(
                f'[Discovery] Localized Beta Drop Spot at ({drop_xy[0]:.3f}, {drop_xy[1]:.3f}) '
                f'({len(drop_obs)} obs)'
            )

        return box_xy, drop_xy

    def refine_box_pose_rgbd(self, box_x: float, box_y: float, duration_sec: float = 0.9):
        """At pre-grasp (~0.6 m), take fresh close-range RGB-D observations of the cube."""
        obs = []
        end = time.time() + duration_sec
        last_seq = -1
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.04)
            if self._rgb_seq == last_seq or self.latest_rgb is None:
                continue
            last_seq = self._rgb_seq
            T_map_cam = self.get_cam_transform()
            if T_map_cam is None:
                continue
            intrinsics = intrinsics_from_camera_info(self.camera_info)
            det = detect_orange_cube_map(
                self.latest_rgb, self.latest_depth, intrinsics, T_map_cam, max_range=1.8
            )
            if det is not None and math.hypot(det[0] - box_x, det[1] - box_y) <= 0.45:
                obs.append((det[0], det[1]))

        if not obs:
            return box_x, box_y
        refined = estimate_cluster_centroid(obs, max_cluster_radius=0.20)
        if refined is None:
            return box_x, box_y
        self.get_logger().info(
            f'[Discovery] Close-range cube refinement: ({box_x:.3f}, {box_y:.3f}) -> '
            f'({refined[0]:.3f}, {refined[1]:.3f}) ({len(obs)} frames)'
        )
        return refined[0], refined[1]

    def refine_drop_pose_rgbd(self, drop_x: float, drop_y: float, duration_sec: float = 0.9):
        """At pre-drop, refine drop-spot centre from close range if visible."""
        obs = []
        end = time.time() + duration_sec
        last_seq = -1
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.04)
            if self._rgb_seq == last_seq or self.latest_rgb is None:
                continue
            last_seq = self._rgb_seq
            T_map_cam = self.get_cam_transform()
            if T_map_cam is None:
                continue
            intrinsics = intrinsics_from_camera_info(self.camera_info)
            det = detect_drop_spot_map(
                self.latest_rgb, self.latest_depth, intrinsics, T_map_cam, max_range=2.2
            )
            if det is not None and math.hypot(det[0] - drop_x, det[1] - drop_y) <= 0.45:
                obs.append((det[0], det[1]))

        if not obs:
            return drop_x, drop_y
        refined = estimate_cluster_centroid(obs, max_cluster_radius=0.25)
        if refined is None:
            return drop_x, drop_y
        self.get_logger().info(
            f'[Discovery] Close-range drop refinement: ({drop_x:.3f}, {drop_y:.3f}) -> '
            f'({refined[0]:.3f}, {refined[1]:.3f}) ({len(obs)} frames)'
        )
        return refined[0], refined[1]

    # --- Arm & Gripper Control -----------------------------------------------
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

        start = time.time()
        max_wait_wall = max(35.0, duration_sec * 14.0)
        reached = False
        while rclpy.ok() and (time.time() - start) < max_wait_wall:
            rclpy.spin_once(self, timeout_sec=0.04)
            if all(j in self.arm_joint_positions for j in self.ARM_JOINTS):
                max_err = max(
                    abs(self.arm_joint_positions[j] - float(target))
                    for j, target in zip(self.ARM_JOINTS, positions)
                )
                if max_err < 0.035 and (time.time() - start) >= duration_sec * 0.8:
                    reached = True
                    break
        if not reached:
            self.get_logger().warn(
                f'[Arm] Timed out waiting for {description or positions} '
                f'(actual={ {j: round(self.arm_joint_positions.get(j, float("nan")), 3) for j in self.ARM_JOINTS} })'
            )
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
        max_nav2_retries: int = 2,
        nav2_timeout_sec: float = 120.0,
    ) -> bool:
        """Navigate using Nav2 if available, then refine short residual error with closed-loop.

        Never falls back to blind straight-line closed-loop driving across a room
        (> 0.95 m) when Nav2 is active, preventing wall collisions and odometry drift.
        """
        if label:
            self.get_logger().info(
                f'[Navigate] -> {label}: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})'
            )

        nav2_available = self.nav_client.wait_for_server(timeout_sec=1.5)
        if nav2_available:
            nav2_ok = False
            for attempt in range(1, max_nav2_retries + 2):
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
                    while rclpy.ok() and not res_future.done() and (time.time() - start) < nav2_timeout_sec:
                        rclpy.spin_once(self, timeout_sec=0.1)
                    if not res_future.done():
                        handle.cancel_goal_async()
                        self.spin_for(0.5)
                        self.get_logger().warn(
                            f'Nav2 goal {label} timed out (attempt {attempt}).'
                        )
                    else:
                        status = res_future.result().status
                        if status == 4:
                            nav2_ok = True
                            break
                        self.get_logger().warn(
                            f'Nav2 goal {label} ended with status {status} (attempt {attempt}).'
                        )
                else:
                    self.get_logger().warn(
                        f'Nav2 rejected goal {label} (attempt {attempt}).'
                    )

                if attempt <= max_nav2_retries:
                    self.clear_costmaps()
                    self.spin_for(0.5)

            pose = self.get_pose()
            rem_dist = math.hypot(x - pose[0], y - pose[1]) if pose is not None else math.inf
            if not nav2_ok and rem_dist > 0.95:
                self.get_logger().error(
                    f'Nav2 failed to reach {label} ({rem_dist:.2f} m away); '
                    'refusing blind long-distance closed-loop drive.'
                )
                return False

            if fine_align:
                return self.drive_to_pose_closed_loop(
                    x, y, yaw, pos_tol=pos_tol, obstacle_guard=True
                )
            return nav2_ok

        # Nav2 not running (e.g. stripped-down practice world home_tasks.sdf)
        return self.drive_to_pose_closed_loop(
            x, y, yaw, pos_tol=pos_tol, obstacle_guard=False
        )

    def align_with_orange_cube_camera(self, timeout_sec: float = 8.0):
        """Optionally refine heading toward the orange cube using the RGB camera."""
        if not self.get_parameter('use_visual_servo').value:
            return

        start = time.time()
        while rclpy.ok() and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_rgb is None:
                continue

            cube_mask = segment_orange_cube(self.latest_rgb)
            ys, xs = np.nonzero(cube_mask)
            if len(xs) < 25:
                break

            cx = float(np.mean(xs))
            err_x = (cx - (self.latest_rgb.shape[1] / 2.0)) / (self.latest_rgb.shape[1] / 2.0)
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
            rear_min = self._min_scan_in_arc(math.pi, half_arc_rad=0.45)
            if rear_min < 0.30:
                self.get_logger().warn(
                    f'[Backup Guard] Obstacle {rear_min:.2f} m behind robot; stopping backup early.'
                )
                break
            pose = self.get_pose()
            if pose is not None and math.hypot(pose[0] - x0, pose[1] - y0) >= distance_m:
                break
            self.cmd_pub.publish(twist)
        self.stop_base()

    # --- Main Pick and Place Mission -----------------------------------------
    def execute_mission(self) -> bool:
        self.get_logger().info('Waiting for robot pose (/odom or TF)...')
        start_wait = time.time()
        while rclpy.ok() and self.get_pose() is None and (time.time() - start_wait) < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.get_pose() is None:
            self.get_logger().error('No pose available on /odom or TF!')
            return False

        if self.get_parameter('wait_for_ready').value:
            self.get_logger().info('Waiting for /room_beta/ready trigger...')
            while rclpy.ok() and not self.room_beta_ready:
                rclpy.spin_once(self, timeout_sec=0.2)
            if not rclpy.ok():
                return False

        discover = bool(self.get_parameter('discover_targets').value)
        box_x = float(self.get_parameter('box_x').value)
        box_y = float(self.get_parameter('box_y').value)
        drop_x = float(self.get_parameter('drop_x').value)
        drop_y = float(self.get_parameter('drop_y').value)
        pick_thresh = float(self.get_parameter('pick_error_threshold_m').value)
        pick_lift_thresh = float(self.get_parameter('pick_lift_threshold_m').value)
        place_thresh = float(self.get_parameter('place_error_threshold_m').value)
        place_max_z = float(self.get_parameter('place_max_z_m').value)
        scenario = str(self.get_parameter('scenario').value)

        mission_t0 = time.time()
        pick_done_t = None
        failure_reasons = []

        eval_record = {
            'timestamp_utc': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
            'trial_id': 0,
            'scenario': scenario,
            'pose_source': 'ros_tf_odom_estimate',
            'box_target_x': box_x,
            'box_target_y': box_y,
            'drop_target_x': drop_x,
            'drop_target_y': drop_y,
            'pickup_pose_x': None,
            'pickup_pose_y': None,
            'pickup_pose_yaw': None,
            'pickup_grasp_x': None,
            'pickup_grasp_y': None,
            'cube_initial_x': box_x,
            'cube_initial_y': box_y,
            'cube_initial_z': self.CUBE_REST_Z_M,
            'cube_lifted_z': None,
            'cube_lift_delta_z': None,
            'pickup_dx_m': None,
            'pickup_dy_m': None,
            'pickup_distance_error_m': None,
            'pickup_estimated_error_m': None,
            'pick_threshold_m': pick_thresh,
            'pick_success': False,
            'place_pose_x': None,
            'place_pose_y': None,
            'place_pose_yaw': None,
            'cube_final_x': None,
            'cube_final_y': None,
            'cube_final_z': None,
            'placement_dx_m': None,
            'placement_dy_m': None,
            'placement_distance_error_m': None,
            'placement_estimated_error_m': None,
            'place_threshold_m': place_thresh,
            'place_success': False,
            'mission_success': False,
            'pick_time_sec': None,
            'place_time_sec': None,
            'total_time_sec': None,
            'failure_reason': '',
        }

        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'Starting Phase D: Beta Pick & Place | discover_targets={discover}'
        )
        self.get_logger().info('=' * 60)

        try:
            # 1. Ensure DetachableJoint starts detached and arm is folded safely
            self.set_detachable_joint(attach=False)
            self.control_gripper(self.GRIPPER_OPEN, 'Opening gripper')
            self.move_arm(self.HOME_POSE, duration_sec=2.0, description='Moving arm to Home Pose')

            # 2. Target localization (autonomous visual search vs standalone parameter coords)
            if discover:
                box_xy, drop_xy = self.discover_beta_targets()
                if box_xy is None:
                    failure_reasons.append('box_not_found')
                    return False
                if drop_xy is None:
                    failure_reasons.append('drop_spot_not_found')
                    return False
                box_x, box_y = box_xy
                drop_x, drop_y = drop_xy
                eval_record['box_target_x'] = box_x
                eval_record['box_target_y'] = box_y
                eval_record['drop_target_x'] = drop_x
                eval_record['drop_target_y'] = drop_y
            else:
                # Standalone mode: route into Room Beta if currently outside
                cur_x, cur_y, _ = self.get_pose()
                if cur_y < 6.40 or cur_x < -9.21:
                    if cur_y > 6.40 and cur_x < -9.21:
                        self.navigate_to_pose(
                            -14.61, 5.20, -math.pi / 2.0, 'Exit Room Alpha Doorway',
                            fine_align=False, pos_tol=0.15,
                        )
                    self.navigate_to_pose(
                        -3.81, 5.20, math.pi / 2.0, 'Door Beta Approach (Hall)',
                        fine_align=False, pos_tol=0.15,
                    )
                    self.navigate_to_pose(
                        -3.81, 7.42, math.pi, 'Inside Room Beta Entrance',
                        fine_align=False, pos_tol=0.12,
                    )

            self.get_logger().info(
                f'Active Targets: Box=({box_x:.2f}, {box_y:.2f}) -> Drop=({drop_x:.2f}, {drop_y:.2f})'
            )

            # 3. Approach the Beta Cube (dynamic collision-free approach angle when discovering)
            if discover:
                grid, ox, oy, res = self.get_map_array()
                cur_pose = self.get_pose()
                robot_xy = (cur_pose[0], cur_pose[1]) if cur_pose else None
                pre_grasp_x, pre_grasp_y, grasp_yaw = select_approach_pose(
                    box_x, box_y, 0.62, grid, ox, oy, res,
                    robot_xy=robot_xy, clearance_m=0.42, ignore_target_radius_m=0.18,
                )
            else:
                grasp_yaw = math.pi
                pre_grasp_x = box_x - 0.60 * math.cos(grasp_yaw)
                pre_grasp_y = box_y - 0.60 * math.sin(grasp_yaw)

            if not self.navigate_to_pose(
                pre_grasp_x, pre_grasp_y, grasp_yaw, 'Pre-Grasp Pose',
                fine_align=True, pos_tol=0.05,
            ):
                self.get_logger().warn('Pre-grasp pose had residual error; continuing to grasp pose.')

            # Close-range RGB-D refinement at ~0.6 m before closing in
            if discover:
                box_x, box_y = self.refine_box_pose_rgbd(box_x, box_y)
                eval_record['box_target_x'] = box_x
                eval_record['box_target_y'] = box_y
                cur_pose = self.get_pose()
                if cur_pose is not None:
                    grasp_yaw = math.atan2(box_y - cur_pose[1], box_x - cur_pose[0])

            grasp_x = box_x - self.GRASP_STANDOFF_M * math.cos(grasp_yaw)
            grasp_y = box_y - self.GRASP_STANDOFF_M * math.sin(grasp_yaw)

            self.move_arm(
                self.PRE_PICK_POSE, duration_sec=1.8, description='Preparing arm for pick'
            )

            # Fine approach to exact grasp standoff distance (disable front guard since cube is ahead)
            self.drive_to_pose_closed_loop(
                grasp_x, grasp_y, grasp_yaw, pos_tol=0.02, yaw_tol=0.03,
                max_lin=0.10, obstacle_guard=False,
            )
            self.align_with_orange_cube_camera(timeout_sec=5.0)

            # 4. Grasp and Attach Cube
            self.stop_base()
            self.spin_for(0.4)
            pick_pose = self.get_pose()
            if pick_pose is not None:
                px, py, pyaw = pick_pose
                dx_world = box_x - px
                dy_world = box_y - py
                held_dx_local = dx_world * math.cos(pyaw) + dy_world * math.sin(pyaw)
                held_dy_local = -dx_world * math.sin(pyaw) + dy_world * math.cos(pyaw)
                held_dx_local = max(0.20, min(0.38, held_dx_local))
                held_dy_local = max(-0.10, min(0.10, held_dy_local))
            else:
                held_dx_local = self.GRASP_STANDOFF_M
                held_dy_local = 0.0

            # --- Evaluation Measurement: Pickup Distance Error (Read-Only) ---
            use_gt = self.has_gz_ground_truth()
            eval_record['pose_source'] = (
                'gz_ground_truth' if use_gt else 'ros_tf_odom_estimate'
            )
            if not use_gt:
                self.get_logger().warn(
                    '[Eval] /eval/gz_dynamic_poses not received; '
                    'falling back to ROS TF/odom pose estimate for error logging.'
                )

            gt_cube_init = self.get_gz_cube_pose_map() if use_gt else None
            if gt_cube_init is not None:
                cube_init_x, cube_init_y, cube_init_z, _ = gt_cube_init
            else:
                cube_init_x, cube_init_y, cube_init_z = box_x, box_y, self.CUBE_REST_Z_M

            eval_record['cube_initial_x'] = cube_init_x
            eval_record['cube_initial_y'] = cube_init_y
            eval_record['cube_initial_z'] = cube_init_z

            if pick_pose is not None:
                est_gx, est_gy = compute_grasp_point(
                    pick_pose[0], pick_pose[1], pick_pose[2], self.GRASP_STANDOFF_M
                )
                eval_record['pickup_estimated_error_m'] = math.hypot(
                    est_gx - box_x, est_gy - box_y
                )

            gt_robot_pick = self.get_gz_robot_pose_map() if use_gt else None
            if gt_robot_pick is not None:
                rx, ry, _, ryaw = gt_robot_pick
            elif pick_pose is not None:
                rx, ry, ryaw = pick_pose
            else:
                rx = ry = ryaw = None

            if rx is not None:
                eval_record['pickup_pose_x'] = rx
                eval_record['pickup_pose_y'] = ry
                eval_record['pickup_pose_yaw'] = ryaw
                gx, gy = compute_grasp_point(rx, ry, ryaw, self.GRASP_STANDOFF_M)
                eval_record['pickup_grasp_x'] = gx
                eval_record['pickup_grasp_y'] = gy
                eval_record['pickup_dx_m'] = gx - cube_init_x
                eval_record['pickup_dy_m'] = gy - cube_init_y
                eval_record['pickup_distance_error_m'] = math.hypot(
                    eval_record['pickup_dx_m'], eval_record['pickup_dy_m']
                )

            self.move_arm(self.PICK_POSE, duration_sec=2.5, description='Lowering arm to Pick Pose')
            self.control_gripper(self.GRIPPER_CLOSED, 'Closing gripper on Beta cube')
            self.set_detachable_joint(attach=True)
            self.move_arm(
                self.CARRY_POSE, duration_sec=2.5, description='Lifting Beta cube to high Carry Pose'
            )
            self.spin_for(0.3)

            # --- Evaluation Check: Pick Success (Distance + Lift Verification) ---
            pick_done_t = time.time()
            eval_record['pick_time_sec'] = pick_done_t - mission_t0

            pick_dist_err = eval_record['pickup_distance_error_m']
            pick_dist_ok = pick_dist_err is not None and pick_dist_err <= pick_thresh

            if use_gt:
                gt_cube_lifted = self.get_gz_cube_pose_map()
                if gt_cube_lifted is not None:
                    cube_lifted_z = gt_cube_lifted[2]
                else:
                    cube_lifted_z = cube_init_z
                cube_lift_dz = cube_lifted_z - cube_init_z
                eval_record['cube_lifted_z'] = cube_lifted_z
                eval_record['cube_lift_delta_z'] = cube_lift_dz
                pick_lift_ok = cube_lift_dz >= pick_lift_thresh
            else:
                pick_lift_ok = True

            eval_record['pick_success'] = bool(pick_dist_ok and pick_lift_ok)
            if not pick_dist_ok:
                failure_reasons.append('pick_distance_exceeded')
            if not pick_lift_ok:
                failure_reasons.append('cube_not_lifted')

            self.get_logger().info(
                f'[Eval][PICK] {"SUCCESS" if eval_record["pick_success"] else "FAIL"} | '
                f'error={_fmt_float(pick_dist_err, 4) or "N/A"} m '
                f'(dx={_fmt_float(eval_record["pickup_dx_m"], 4) or "N/A"}, '
                f'dy={_fmt_float(eval_record["pickup_dy_m"], 4) or "N/A"}, '
                f'thresh={pick_thresh:.3f} m) | '
                f'lift_dz={_fmt_float(eval_record["cube_lift_delta_z"], 3) or "N/A"} m'
            )

            # Back up out of the static map's obstacle inflation zone around the cube,
            # then clear Nav2 costmaps of any transient hits recorded during the arm lift.
            self.get_logger().info('[Base] Backing out of static map cube inflation zone...')
            self.backup_distance(distance_m=0.38, speed=0.14)
            self.clear_costmaps()
            self.spin_for(0.4)

            # 5. Carry Cube to the Beta Drop Spot
            if discover:
                # Choose collision-free pre-drop pose dynamically from live SLAM /map
                # with clearance >= Nav2's 0.38 m inflation_radius so Nav2 always accepts it.
                grid, ox, oy, res = self.get_map_array()
                cur_pose = self.get_pose()
                robot_xy = (cur_pose[0], cur_pose[1]) if cur_pose else None
                pre_drop_x, pre_drop_y, place_yaw = select_approach_pose(
                    drop_x, drop_y, held_dx_local + 0.36, grid, ox, oy, res,
                    robot_xy=robot_xy, clearance_m=0.42, ignore_target_radius_m=0.0,
                )
            else:
                self.navigate_to_pose(
                    -3.81, drop_y, 0.0, 'Room Beta Clear Corridor Waypoint',
                    fine_align=False, pos_tol=0.15,
                )
                place_yaw = 0.0
                pre_drop_x = drop_x - (held_dx_local + 0.28)
                pre_drop_y = drop_y

            if not self.navigate_to_pose(
                pre_drop_x, pre_drop_y, place_yaw, 'Pre-Drop Pose',
                fine_align=True, pos_tol=0.05,
            ):
                failure_reasons.append('pre_drop_nav_failed')
                return False

            # Close-range drop pad refinement if visible from pre-drop pose
            if discover:
                drop_x, drop_y = self.refine_drop_pose_rgbd(drop_x, drop_y)
                eval_record['drop_target_x'] = drop_x
                eval_record['drop_target_y'] = drop_y
                cur_pose = self.get_pose()
                if cur_pose is not None:
                    place_yaw = math.atan2(drop_y - cur_pose[1], drop_x - cur_pose[0])

            # Compute exact robot stop pose so the held cube centre lands on (drop_x, drop_y)
            place_x = drop_x - (
                held_dx_local * math.cos(place_yaw) - held_dy_local * math.sin(place_yaw)
            )
            place_y = drop_y - (
                held_dx_local * math.sin(place_yaw) + held_dy_local * math.cos(place_yaw)
            )

            self.drive_to_pose_closed_loop(
                place_x, place_y, place_yaw, pos_tol=0.015, yaw_tol=0.025, max_lin=0.08
            )
            self.stop_base()
            self.spin_for(0.5)

            # Record robot pose at release for placement estimation comparison
            est_place_pose = self.get_pose()
            est_drop_x = est_drop_y = None
            if est_place_pose is not None:
                eplx, eply, eplyaw = est_place_pose
                est_drop_x = eplx + (
                    held_dx_local * math.cos(eplyaw) - held_dy_local * math.sin(eplyaw)
                )
                est_drop_y = eply + (
                    held_dx_local * math.sin(eplyaw) + held_dy_local * math.cos(eplyaw)
                )
                eval_record['placement_estimated_error_m'] = math.hypot(
                    est_drop_x - drop_x, est_drop_y - drop_y
                )

            gt_robot_place = self.get_gz_robot_pose_map() if use_gt else None
            if gt_robot_place is not None:
                eval_record['place_pose_x'] = gt_robot_place[0]
                eval_record['place_pose_y'] = gt_robot_place[1]
                eval_record['place_pose_yaw'] = gt_robot_place[3]
            elif est_place_pose is not None:
                eval_record['place_pose_x'] = est_place_pose[0]
                eval_record['place_pose_y'] = est_place_pose[1]
                eval_record['place_pose_yaw'] = est_place_pose[2]

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
            self.spin_for(0.4)

            # --- Evaluation Measurement: Placement Distance Error ---
            place_done_t = time.time()
            eval_record['place_time_sec'] = (
                (place_done_t - pick_done_t) if pick_done_t is not None else None
            )

            if use_gt:
                gt_cube_final = self.get_gz_cube_pose_map()
                if gt_cube_final is not None:
                    cube_final_x, cube_final_y, cube_final_z, _ = gt_cube_final
                else:
                    cube_final_x, cube_final_y, cube_final_z = cube_init_x, cube_init_y, cube_init_z
                eval_record['cube_final_x'] = cube_final_x
                eval_record['cube_final_y'] = cube_final_y
                eval_record['cube_final_z'] = cube_final_z
                eval_record['placement_dx_m'] = cube_final_x - drop_x
                eval_record['placement_dy_m'] = cube_final_y - drop_y
                eval_record['placement_distance_error_m'] = math.hypot(
                    eval_record['placement_dx_m'], eval_record['placement_dy_m']
                )
                place_floor_ok = cube_final_z <= place_max_z
            else:
                if est_drop_x is not None and est_drop_y is not None:
                    eval_record['cube_final_x'] = est_drop_x
                    eval_record['cube_final_y'] = est_drop_y
                    eval_record['placement_dx_m'] = est_drop_x - drop_x
                    eval_record['placement_dy_m'] = est_drop_y - drop_y
                    eval_record['placement_distance_error_m'] = math.hypot(
                        eval_record['placement_dx_m'], eval_record['placement_dy_m']
                    )
                place_floor_ok = True

            place_dist_err = eval_record['placement_distance_error_m']
            place_dist_ok = place_dist_err is not None and place_dist_err <= place_thresh
            eval_record['place_success'] = bool(place_dist_ok and place_floor_ok)

            if not place_dist_ok:
                failure_reasons.append('place_distance_exceeded')
            if not place_floor_ok:
                failure_reasons.append('cube_not_on_floor')

            self.get_logger().info(
                f'[Eval][PLACE] {"SUCCESS" if eval_record["place_success"] else "FAIL"} | '
                f'error={_fmt_float(place_dist_err, 4) or "N/A"} m '
                f'(dx={_fmt_float(eval_record["placement_dx_m"], 4) or "N/A"}, '
                f'dy={_fmt_float(eval_record["placement_dy_m"], 4) or "N/A"}, '
                f'thresh={place_thresh:.3f} m) | '
                f'final_z={_fmt_float(eval_record["cube_final_z"], 3) or "N/A"} m'
            )

            # 7. Publish completion signal so doorway_navigator / mission_supervisor can proceed
            for _ in range(3):
                self.done_pub.publish(Bool(data=True))
                self.spin_for(0.1)

            eval_record['mission_success'] = bool(
                eval_record['pick_success'] and eval_record['place_success']
            )
            self.get_logger().info('=' * 60)
            self.get_logger().info(
                f'Phase D: Beta Pick & Place COMPLETE '
                f'(pick={eval_record["pick_success"]}, place={eval_record["place_success"]}, '
                f'mission={eval_record["mission_success"]})! (/room_beta/done published)'
            )
            self.get_logger().info('=' * 60)
            return eval_record['mission_success']

        except KeyboardInterrupt:
            failure_reasons.append('interrupted')
            raise
        except Exception as exc:
            failure_reasons.append(f'exception:{type(exc).__name__}')
            self.get_logger().error(f'Mission failed with exception: {exc}')
            return False
        finally:
            eval_record['total_time_sec'] = time.time() - mission_t0
            eval_record['mission_success'] = bool(
                eval_record['pick_success'] and eval_record['place_success']
            )
            eval_record['failure_reason'] = '; '.join(failure_reasons)
            self._record_and_report_eval(eval_record)


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
