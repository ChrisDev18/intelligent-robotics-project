#!/usr/bin/env python3
"""Autonomous Exploration + Interleaved Room-Task Coordinator & Passive Target Mapper.

Interleaved mode (default: interleaved_tasks:=true):
1. Watches /rgbd_camera/image, /rgbd_camera/depth_image, /rgbd_camera/camera_info,
   and TF while frontier_explorer maps the unknown house.
2. Passively maps visual sightings of the Room Alpha clean zone, Room Beta pick
   box, and Room Beta drop spot onto /discovered/clean_zone, /discovered/box_pose,
   and /discovered/drop_pose (latched JSON).
3. As soon as the robot is cleanly inside an undone room (confirmed over
   `confirm_frames` consecutive yellow- or green-floor near-bumper frames):
     - Pauses frontier_explorer (/exploration_pause = True)
     - Publishes /room_alpha/ready (yellow) or /room_beta/ready (green)
     - Waits for /room_alpha/done or /room_beta/done
     - Resumes frontier_explorer (/exploration_pause = False) so the robot keeps
       exploring until the other room is done AND 100% of the house is mapped.
4. Once /exploration_done fires (and both rooms are complete — with fallback
   navigation if a room was never entered during exploration), drives home via Nav2.

Legacy sequential mode (interleaved_tasks:=false) waits for /exploration_done
before visiting Room Alpha and Room Beta in order.
"""
import json
import math
import time
import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from robot_bringup.perception import (
    classify_floor_color,
    classify_near_floor_color,
    detect_clean_zone_points,
    detect_drop_spot_map,
    detect_floor_point_in_map,
    detect_orange_cube_map,
    estimate_clean_zone_bounds,
    estimate_cluster_centroid,
    intrinsics_from_camera_info,
    transform_stamped_to_matrix,
)


# World -> SLAM /map fallback offset (robot spawns at world x=19.21, y=0.79, yaw=0.0)
SPAWN_X, SPAWN_Y = 19.21, 0.79


def world_to_slam_map(wx: float, wy: float, wyaw: float):
    return wx - SPAWN_X, wy - SPAWN_Y, wyaw


# Keep exported name for backwards compatibility with any callers/tests
analyze_floor_color = classify_floor_color


class DoorwayWaypointHandoff(Node):
    def __init__(self):
        super().__init__('doorway_waypoint_handoff')
        self.declare_parameter('wait_for_teammates', False)
        self.declare_parameter('dwell_sec', 5.0)
        self.declare_parameter('interleaved_tasks', True)
        # Trigger once near-bumper floor sees >= 35% room floor for 3 consecutive frames
        # (~0.6 s at 5 Hz); task nodes also call nudge_clear_of_doorway() if still in doorway.
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('confirm_floor_ratio', 0.35)
        self.declare_parameter('nominal_zone_sx', 2.29)
        self.declare_parameter('nominal_zone_sy', 1.57)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.exploration_done = False
        self.room_done = {'room_alpha': False, 'room_beta': False}
        self.room_triggered = {'room_alpha': False, 'room_beta': False}
        self.active_room = None
        self.pending_room = None

        self.latest_rgb = None
        self.latest_depth = None
        self.camera_info = None
        self.map_msg = None

        # Consecutive floor-frame counters for robust room-entry confirmation
        self._floor_streak_label = 'HALL'
        self._floor_streak_count = 0

        # Discovered map poses for each room during exploration (for fallback)
        self.discovered_alpha_pose = None
        self.discovered_beta_pose = None
        self._last_log = 0.0

        # Passively accumulated target sightings in `map` frame
        self.clean_zone_pts = []
        self.box_obs = []
        self.drop_obs = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Bool, '/exploration_done', self._explore_done_cb, latched_qos)
        self.create_subscription(Bool, '/room_alpha/done', self._alpha_done_cb, latched_qos)
        self.create_subscription(Bool, '/room_beta/done', self._beta_done_cb, latched_qos)
        self.create_subscription(Image, '/rgbd_camera/image', self._rgb_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/depth_image', self._depth_cb, 10)
        self.create_subscription(CameraInfo, '/rgbd_camera/camera_info', self._cam_info_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, latched_qos)

        self.pause_pub = self.create_publisher(Bool, '/exploration_pause', 10)
        self.alpha_ready_pub = self.create_publisher(Bool, '/room_alpha/ready', latched_qos)
        self.beta_ready_pub = self.create_publisher(Bool, '/room_beta/ready', latched_qos)
        self.clean_zone_pub = self.create_publisher(String, '/discovered/clean_zone', latched_qos)
        self.box_pose_pub = self.create_publisher(String, '/discovered/box_pose', latched_qos)
        self.drop_pose_pub = self.create_publisher(String, '/discovered/drop_pose', latched_qos)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.clear_local_costmap = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'
        )
        self.clear_global_costmap = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'
        )

        mode = 'interleaved' if self.get_parameter('interleaved_tasks').value else 'sequential'
        self.get_logger().info(
            f'Doorway & Target Coordinator started in [{mode}] mode — '
            'watching RGB-D camera while exploring...'
        )

    # --- Callbacks -----------------------------------------------------------
    def _explore_done_cb(self, msg: Bool):
        if msg.data:
            self.exploration_done = True

    def _alpha_done_cb(self, msg: Bool):
        if msg.data:
            self.room_done['room_alpha'] = True

    def _beta_done_cb(self, msg: Bool):
        if msg.data:
            self.room_done['room_beta'] = True

    def _cam_info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def _map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg

    def _depth_cb(self, msg: Image):
        if msg.encoding == '32FC1':
            self.latest_depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width
            )

    def _get_current_map_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(), timeout=Duration(seconds=0.15)
            )
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            return t.transform.translation.x, t.transform.translation.y, yaw
        except Exception:
            return None

    def _get_cam_transform(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'rgbd_camera_optical_frame', Time(), timeout=Duration(seconds=0.1)
            )
            return transform_stamped_to_matrix(t)
        except Exception:
            return None

    def _publish_pause(self, paused: bool):
        msg = Bool(data=paused)
        for _ in range(3):
            self.pause_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info(f'Published /exploration_pause = {paused}')

    def _passive_target_scan(self, rgb: np.ndarray, floor_label: str):
        """Passively localize targets seen during exploration and publish latched updates."""
        T_map_cam = self._get_cam_transform()
        if T_map_cam is None:
            return
        intrinsics = intrinsics_from_camera_info(self.camera_info)

        # Room Alpha clean zone mat
        if floor_label == 'YELLOW' and not self.room_done['room_alpha']:
            pts = detect_clean_zone_points(rgb, self.latest_depth, intrinsics, T_map_cam)
            if len(pts) > 0:
                self.clean_zone_pts.extend(pts.tolist())
                if len(self.clean_zone_pts) > 3000:
                    self.clean_zone_pts = self.clean_zone_pts[-3000:]
                grid = None
                ox = oy = 0.0
                res = 0.05
                if self.map_msg is not None:
                    info = self.map_msg.info
                    grid = np.array(self.map_msg.data, dtype=np.int16).reshape(
                        info.height, info.width
                    )
                    ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution
                bounds = estimate_clean_zone_bounds(
                    self.clean_zone_pts,
                    nominal_sx=float(self.get_parameter('nominal_zone_sx').value),
                    nominal_sy=float(self.get_parameter('nominal_zone_sy').value),
                    grid=grid, origin_x=ox, origin_y=oy, resolution=res,
                )
                if bounds is not None:
                    step = max(1, len(self.clean_zone_pts) // 250)
                    payload = {
                        'x_min': bounds[0],
                        'x_max': bounds[1],
                        'y_min': bounds[2],
                        'y_max': bounds[3],
                        'num_points': len(self.clean_zone_pts),
                        'points_sample': self.clean_zone_pts[::step],
                    }
                    self.clean_zone_pub.publish(String(data=json.dumps(payload)))

        # Room Beta orange cube & drop spot
        if floor_label == 'GREEN' and not self.room_done['room_beta']:
            box = detect_orange_cube_map(rgb, self.latest_depth, intrinsics, T_map_cam)
            if box is not None:
                self.box_obs.append((box[0], box[1]))
                if len(self.box_obs) > 60:
                    self.box_obs = self.box_obs[-60:]
                c = estimate_cluster_centroid(self.box_obs)
                if c is not None:
                    payload = {'x': c[0], 'y': c[1], 'observations': len(self.box_obs)}
                    self.box_pose_pub.publish(String(data=json.dumps(payload)))

            drop = detect_drop_spot_map(rgb, self.latest_depth, intrinsics, T_map_cam)
            if drop is not None:
                self.drop_obs.append((drop[0], drop[1]))
                if len(self.drop_obs) > 60:
                    self.drop_obs = self.drop_obs[-60:]
                c = estimate_cluster_centroid(self.drop_obs)
                if c is not None:
                    payload = {'x': c[0], 'y': c[1], 'observations': len(self.drop_obs)}
                    self.drop_pose_pub.publish(String(data=json.dumps(payload)))

    def _rgb_cb(self, msg: Image):
        if msg.encoding not in ('rgb8', 'bgr8'):
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self.latest_rgb = arr if msg.encoding == 'rgb8' else arr[:, :, ::-1]

        label, yr, gr = classify_floor_color(self.latest_rgb)
        now = time.time()
        if now - self._last_log > 3.0:
            self._last_log = now
            self.get_logger().info(
                f'[RGB Floor Monitor] state={label} (yellow={yr:.2f}, green={gr:.2f}) '
                f'| active={self.active_room} | done={self.room_done}'
            )

        # Record a map pose *inside* each room whenever its floor is spotted, by
        # projecting the coloured floor pixels onto the ground plane (so even a
        # doorway glimpse gives a valid interior coordinate, not a hallway pose!).
        if label == 'YELLOW' and self.discovered_alpha_pose is None:
            pose = self._get_current_map_pose()
            T_map_cam = self._get_cam_transform()
            if pose is not None:
                floor_pt = None
                if T_map_cam is not None:
                    floor_pt = detect_floor_point_in_map(
                        self.latest_rgb, self.latest_depth,
                        intrinsics_from_camera_info(self.camera_info),
                        T_map_cam, 'YELLOW',
                    )
                if floor_pt is not None:
                    self.discovered_alpha_pose = (floor_pt[0], floor_pt[1], pose[2])
                else:
                    self.discovered_alpha_pose = pose
                self.get_logger().info(
                    f'>>> DISCOVERED YELLOW ROOM (Room Alpha) at slam_map='
                    f'({self.discovered_alpha_pose[0]:.2f}, {self.discovered_alpha_pose[1]:.2f})!'
                )
        elif label == 'GREEN' and self.discovered_beta_pose is None:
            pose = self._get_current_map_pose()
            T_map_cam = self._get_cam_transform()
            if pose is not None:
                floor_pt = None
                if T_map_cam is not None:
                    floor_pt = detect_floor_point_in_map(
                        self.latest_rgb, self.latest_depth,
                        intrinsics_from_camera_info(self.camera_info),
                        T_map_cam, 'GREEN',
                    )
                if floor_pt is not None:
                    self.discovered_beta_pose = (floor_pt[0], floor_pt[1], pose[2])
                else:
                    self.discovered_beta_pose = pose
                self.get_logger().info(
                    f'>>> DISCOVERED GREEN/MINT ROOM (Room Beta) at slam_map='
                    f'({self.discovered_beta_pose[0]:.2f}, {self.discovered_beta_pose[1]:.2f})!'
                )

        # Do not interfere while a room task is actively controlling the robot
        if self.active_room is not None:
            return

        # Passive semantic target mapping during exploration
        self._passive_target_scan(self.latest_rgb, label)

        # Interleaved room-entry confirmation using the near-bumper floor strip
        # (bottom 20% of image, ~0.29-0.43 m ahead) over `confirm_frames` so we only
        # pause once the robot has cleared the doorway and entered the room.
        if self.get_parameter('interleaved_tasks').value and self.pending_room is None:
            near_label, near_yr, near_gr = classify_near_floor_color(self.latest_rgb)
            min_ratio = float(self.get_parameter('confirm_floor_ratio').value)
            confirm_n = int(self.get_parameter('confirm_frames').value)
            ratio = near_yr if near_label == 'YELLOW' else (near_gr if near_label == 'GREEN' else 0.0)

            if near_label in ('YELLOW', 'GREEN') and ratio >= min_ratio:
                if near_label == self._floor_streak_label:
                    self._floor_streak_count += 1
                else:
                    self._floor_streak_label = near_label
                    self._floor_streak_count = 1
            else:
                self._floor_streak_label = 'HALL'
                self._floor_streak_count = 0

            if self._floor_streak_count >= confirm_n:
                room = 'room_alpha' if near_label == 'YELLOW' else 'room_beta'
                if not self.room_triggered[room] and not self.room_done[room]:
                    self.pending_room = room
                    self._floor_streak_count = 0

    # --- Navigation & Handoff ------------------------------------------------
    def _clear_costmaps(self):
        for client in (self.clear_local_costmap, self.clear_global_costmap):
            if client.wait_for_service(timeout_sec=1.0):
                fut = client.call_async(ClearEntireCostmap.Request())
                t0 = time.time()
                while rclpy.ok() and not fut.done() and (time.time() - t0) < 2.0:
                    rclpy.spin_once(self, timeout_sec=0.05)

    def go_to_map_pose(self, mx: float, my: float, myaw: float, label: str,
                       timeout_sec: float = 120.0, max_retries: int = 2) -> bool:
        self.get_logger().info(
            f'Navigating to {label}: slam_map=({mx:.2f}, {my:.2f}, yaw={myaw:.2f})'
        )
        self.nav_client.wait_for_server()

        for attempt in range(1, max_retries + 2):
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = Time().to_msg()
            goal.pose.pose.position.x = float(mx)
            goal.pose.pose.position.y = float(my)
            goal.pose.pose.orientation.z = math.sin(myaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(myaw / 2.0)

            send_future = self.nav_client.send_goal_async(goal)
            while rclpy.ok() and not send_future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
            handle = send_future.result()
            if not handle or not handle.accepted:
                self.get_logger().warn(f'Goal to {label} rejected by Nav2 (attempt {attempt}).')
            else:
                res_future = handle.get_result_async()
                t0 = time.time()
                while rclpy.ok() and not res_future.done() and (time.time() - t0) < timeout_sec:
                    rclpy.spin_once(self, timeout_sec=0.1)
                if not res_future.done():
                    handle.cancel_goal_async()
                    self.get_logger().warn(f'Goal to {label} timed out (attempt {attempt}).')
                else:
                    status = res_future.result().status
                    self.get_logger().info(f'Arrived at {label} (Nav2 status={status}).')
                    if status == 4:
                        return True
                    self.get_logger().warn(f'Goal to {label} ended with status {status} (attempt {attempt}).')

            if attempt <= max_retries:
                self._clear_costmaps()
                t_end = time.time() + 0.8
                while rclpy.ok() and time.time() < t_end:
                    rclpy.spin_once(self, timeout_sec=0.1)

        return False

    def _trigger_and_wait_room(self, room: str, wait_teammates: bool, dwell_sec: float):
        """Pause exploration, trigger /{room}/ready, wait for /{room}/done."""
        self.active_room = room
        self.room_triggered[room] = True
        label = 'Room Alpha (Yellow)' if room == 'room_alpha' else 'Room Beta (Green)'
        pub = self.alpha_ready_pub if room == 'room_alpha' else self.beta_ready_pub

        self.get_logger().info(f'=== Entering {label}: pausing exploration & starting task ===')
        self._publish_pause(True)

        # Let frontier_explorer's Nav2 goal cancellation settle before task takes Nav2
        t_end = time.time() + 1.0
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)

        pub.publish(Bool(data=True))
        self.get_logger().info(f'Published /{room}/ready = True')

        if wait_teammates:
            self.get_logger().info(f'Waiting for /{room}/done...')
            while rclpy.ok() and not self.room_done[room]:
                rclpy.spin_once(self, timeout_sec=0.2)
        else:
            t_end = time.time() + dwell_sec
            while rclpy.ok() and time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.2)
            self.room_done[room] = True

        self.active_room = None
        self._floor_streak_count = 0
        self.get_logger().info(f'=== {label} task finished! ===')

    def _run_interleaved(self):
        wait_teammates = bool(self.get_parameter('wait_for_teammates').value)
        dwell_sec = float(self.get_parameter('dwell_sec').value)

        # Loop until frontier_explorer has mapped the full house (Option 2)
        while rclpy.ok() and not self.exploration_done:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pending_room is not None:
                room = self.pending_room
                self.pending_room = None
                self._trigger_and_wait_room(room, wait_teammates, dwell_sec)
                if not rclpy.ok():
                    return
                if not self.exploration_done:
                    self.get_logger().info(
                        'Resuming frontier exploration to continue mapping the house...'
                    )
                    self._publish_pause(False)

        if not rclpy.ok():
            return

        # Fallback: if exploration finished while any room was never entered
        for room, label, fallback_world in (
            ('room_alpha', 'Room Alpha (Yellow)', (4.60, 8.00, math.pi / 2.0)),
            ('room_beta', 'Room Beta (Green)', (15.40, 8.00, math.pi / 2.0)),
        ):
            if not self.room_done[room]:
                self.get_logger().warn(
                    f'Exploration ended before {label} was completed — navigating there now.'
                )
                discovered = (
                    self.discovered_alpha_pose if room == 'room_alpha'
                    else self.discovered_beta_pose
                )
                fallback_wp = world_to_slam_map(*fallback_world)
                reached = False
                if discovered is not None:
                    reached = self.go_to_map_pose(*discovered, f'{label} (discovered interior)')
                if not reached:
                    self.get_logger().warn(
                        f'Retrying {label} via doorway interior waypoint {fallback_wp[:2]}...'
                    )
                    reached = self.go_to_map_pose(*fallback_wp, f'{label} (interior fallback)')
                if not reached:
                    self.get_logger().error(
                        f'Could not navigate into {label}; triggering task anyway as last resort.'
                    )
                self._trigger_and_wait_room(room, wait_teammates, dwell_sec)
                if not rclpy.ok():
                    return

        # Return home once BOTH rooms and full exploration are complete
        self.go_to_map_pose(0.50, 0.50, 0.0, 'Home Spawn Point')
        self.get_logger().info('=== All Room Tasks, Full Exploration & Return Home Complete! ===')

    def _run_sequential(self):
        while rclpy.ok() and not self.exploration_done:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not rclpy.ok():
            return

        self.get_logger().info('Exploration complete! Routing to Room Alpha & Room Beta doorway waypoints...')
        wait_teammates = bool(self.get_parameter('wait_for_teammates').value)
        dwell_sec = float(self.get_parameter('dwell_sec').value)

        wp_alpha = self.discovered_alpha_pose or world_to_slam_map(4.60, 7.70, math.pi / 2.0)
        self.go_to_map_pose(*wp_alpha, 'Room Alpha (Yellow)')
        self._trigger_and_wait_room('room_alpha', wait_teammates, dwell_sec)

        wp_beta = self.discovered_beta_pose or world_to_slam_map(15.40, 7.70, math.pi / 2.0)
        self.go_to_map_pose(*wp_beta, 'Room Beta (Green)')
        self._trigger_and_wait_room('room_beta', wait_teammates, dwell_sec)

        self.go_to_map_pose(0.50, 0.50, 0.0, 'Home Spawn Point')
        self.get_logger().info('=== All Doorway Waypoints, Handoffs & Return Home Complete! ===')

    def run(self):
        if self.get_parameter('interleaved_tasks').value:
            self._run_interleaved()
        else:
            self._run_sequential()


def main(args=None):
    rclpy.init(args=args)
    node = DoorwayWaypointHandoff()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
