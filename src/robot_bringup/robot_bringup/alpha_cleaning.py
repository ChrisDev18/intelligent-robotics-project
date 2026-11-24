#!/usr/bin/env python3
"""Phase C: Room 1 (Alpha) Vacuum Cleaning Task Node.

Supports two navigation modes selectable via parameter `use_nav2`:
1. Closed-Loop / Direct /cmd_vel Mode (default: use_nav2:=false):
   Uses the same proven closed-loop odometry/TF velocity driving as `beta_pick_place.py`
   (`drive_to_pose_closed_loop`). Directly controls /cmd_vel with proportional heading
   and distance control referencing get_pose() (map -> base_footprint with /odom fallback).
   Works reliably without freezing, in both the full house and the practice world.
2. Nav2 Simple Commander Mode (use_nav2:=true):
   Uses BasicNavigator from `nav2_simple_commander` (goToPose) when full Nav2 stack
   is active and localized.

Target Cleaning Region:
  x in [-13.6, -11.3] m  (width = 2.3 m)
  y in [7.6, 9.1] m      (height = 1.5 m)
  Inside Room 1 (Alpha, yellow floor).

Features:
  - Dynamic dirt spawning and real-time hoovering via Gazebo entity services.
  - Parameter spawn_dirt:=true/false to toggle dirt particle generation.
  - Parameter use_nav2:=true/false (default false) to switch between closed-loop driving and Nav2.
  - Latched topic handoff via /room1/ready and /room1/done.
  - Evaluation logging to log/alpha_cleaning_eval.csv.
"""

import csv
from datetime import datetime, timezone
import math
import os
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from builtin_interfaces.msg import Duration as MsgDuration
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity


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
        self.declare_parameter('spawn_dirt', True)
        self.declare_parameter('clean_x_min', -13.6)
        self.declare_parameter('clean_x_max', -11.3)
        self.declare_parameter('clean_y_min', 7.6)
        self.declare_parameter('clean_y_max', 9.1)
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
        self.done_pub = self.create_publisher(Bool, '/room1/done', latched_qos)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.detach_pub = self.create_publisher(Empty, '/beta_pick_item/detach', 10)
        self.eval_pub = self.create_publisher(String, '/alpha_cleaning/eval_metrics', latched_qos)

        self.create_subscription(Bool, '/room1/ready', self._ready_cb, latched_qos)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # TF Buffer & Listener (identical to beta_pick_place.py)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Gazebo entity services
        self.spawn_client = self.create_client(SpawnEntity, '/world/empty/create')
        self.delete_client = self.create_client(DeleteEntity, '/world/empty/remove')

        self.room1_ready = False
        self.vacuum_active = False
        self.odom_pose = None  # (x, y, yaw) from /odom

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

        mode_str = 'Nav2' if self.navigator is not None else 'Closed-Loop /cmd_vel (beta_pick_place style)'
        self.get_logger().info(f'Alpha Cleaning initialized in [{mode_str}] mode.')

    # --- Callbacks -----------------------------------------------------------
    def _ready_cb(self, msg: Bool):
        if msg.data:
            self.room1_ready = True

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_pose = (p.x, p.y, yaw)

        if self.vacuum_active and self.active_dirt:
            self._check_hoover_dirt()

    # --- Pose & TF Lookup (identical to beta_pick_place.py) -------------------
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

    def stop_base(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_for(self, duration_sec: float):
        end = time.time() + duration_sec
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.04)

    # --- Movement: Closed-Loop /cmd_vel (same as beta_pick_place.py) ----------
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

    def navigate_to(self, x: float, y: float, yaw: float, label: str = '') -> bool:
        """Route to target using either Nav2 Simple Commander or beta_pick_place closed-loop."""
        if label:
            self.get_logger().info(f'[Navigate] -> {label}: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')

        # If Nav2 mode is enabled and navigator is active, try Nav2
        if self.navigator is not None:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            self.navigator.goToPose(pose)
            while not self.navigator.isTaskComplete():
                # Process every queued message, not just one: /odom (50 Hz) is first in
                # line and otherwise starves /tf, leaving the dirt check with a stale pose.
                for _ in range(50):
                    rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.05)

            from nav2_simple_commander.robot_navigator import TaskResult
            if self.navigator.getResult() == TaskResult.SUCCEEDED:
                return True
            self.get_logger().warn(f'Nav2 goal {label} did not succeed; using closed-loop fallback.')

        # Direct closed-loop driving (beta_pick_place style)
        return self.drive_to_pose_closed_loop(x, y, yaw)

    # --- Arm & Vacuum Controls -----------------------------------------------
    def detach_beta_cube(self):
        """Ensure Gazebo's DetachableJoint is detached so the Room 2 cube is not dragged."""
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
        """Robot pose in the map frame for the dirt check: the latest /odom pose (always
        fresh) moved into the map frame by SLAM/AMCL's map->odom correction, which changes
        slowly, so a slightly old value is fine. Falls back to get_pose()."""
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
        # Vacuum center is at x=-0.04m behind base_footprint
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

    def spawn_dirt_grid(self):
        if not self.spawn_client.wait_for_service(timeout_sec=2.5):
            self.get_logger().warn('Gazebo service /world/empty/create unavailable; skipping dirt spawn.')
            return

        x_min = float(self.get_parameter('clean_x_min').value)
        x_max = float(self.get_parameter('clean_x_max').value)
        y_min = float(self.get_parameter('clean_y_min').value)
        y_max = float(self.get_parameter('clean_y_max').value)
        spawn_x = float(self.get_parameter('spawn_x').value)
        spawn_y = float(self.get_parameter('spawn_y').value)

        # 3x3 grid of dirt particles across the clean zone
        dirt_coords = [
            (-13.3, 7.85), (-12.4, 7.85), (-11.6, 7.85),
            (-13.3, 8.35), (-12.4, 8.35), (-11.6, 8.35),
            (-13.3, 8.85), (-12.4, 8.85), (-11.6, 8.85),
        ]

        self.get_logger().info(f'Spawning {len(dirt_coords)} dirt particles across clean zone...')
        self.active_dirt.clear()
        for idx, (dx, dy) in enumerate(dirt_coords, start=1):
            name = f'alpha_dirt_{idx}'
            # Clean up old entity
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
    def generate_lawnmower_waypoints(self) -> list:
        x_min = float(self.get_parameter('clean_x_min').value)
        x_max = float(self.get_parameter('clean_x_max').value)
        y_min = float(self.get_parameter('clean_y_min').value)
        y_max = float(self.get_parameter('clean_y_max').value)
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
                # Eastbound: West -> East
                waypoints.append((x_west, lane_y, 0.0, f'Lane {i+1} West'))
                waypoints.append((x_east, lane_y, 0.0, f'Lane {i+1} East'))
            else:
                # Westbound: East -> West
                waypoints.append((x_east, lane_y, math.pi, f'Lane {i+1} East'))
                waypoints.append((x_west, lane_y, math.pi, f'Lane {i+1} West'))

        return waypoints

    # --- Main Mission --------------------------------------------------------
    def execute_mission(self) -> bool:
        self.get_logger().info('=' * 60)
        self.get_logger().info('Starting Room 1 (Alpha) Cleaning Mission...')
        self.get_logger().info('=' * 60)

        # 1. Wait for initial robot pose (/odom or TF)
        self.get_logger().info('Waiting for robot pose (/odom or TF)...')
        start_wait = time.time()
        while rclpy.ok() and self.get_pose() is None and (time.time() - start_wait) < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.get_pose() is None:
            self.get_logger().error('No robot pose received!')
            return False

        # 2. Wait for /room1/ready if handoff enabled
        if self.get_parameter('wait_for_ready').value:
            self.get_logger().info('Waiting for /room1/ready trigger...')
            while rclpy.ok() and not self.room1_ready:
                rclpy.spin_once(self, timeout_sec=0.2)

        mission_t0 = time.time()
        failure_reasons = []

        try:
            # 3. Ensure Beta cube starts detached and fold arm into safe home pose
            self.detach_beta_cube()
            self.move_arm_home()

            # 4. Route from Hallway into Room 1 (Door 1 is at map x=-14.61, y=6.21)
            cur_pose = self.get_pose()
            if cur_pose and (cur_pose[1] < 6.40 or cur_pose[0] > -9.21):
                self.get_logger().info('Routing from Hallway into Room 1 (Alpha)...')
                self.navigate_to(-14.61, 5.20, math.pi / 2.0, 'Door 1 Hallway Approach')
                self.navigate_to(-14.61, 7.40, 0.0, 'Inside Room 1 Entrance')

            # 5. Spawn dirt particles if requested
            if self.get_parameter('spawn_dirt').value:
                self.spawn_dirt_grid()
                self.spin_for(0.5)

            # 6. Generate Lawnmower Pattern and Execute
            waypoints = self.generate_lawnmower_waypoints()
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

            # 7. Return to Doorway if requested
            if self.get_parameter('return_to_doorway').value:
                self.navigate_to(-14.61, 7.40, -math.pi / 2.0, 'Room 1 Doorway Exit')

            # 8. Publish Completion Signal (/room1/done)
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
                f'/room1/done published.'
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
                'clean_x_min': float(self.get_parameter('clean_x_min').value),
                'clean_x_max': float(self.get_parameter('clean_x_max').value),
                'clean_y_min': float(self.get_parameter('clean_y_min').value),
                'clean_y_max': float(self.get_parameter('clean_y_max').value),
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
