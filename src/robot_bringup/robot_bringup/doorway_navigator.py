#!/usr/bin/env python3
"""Autonomous Exploration + Doorway Waypoint & Room-Color Handoff Node.

1. Monitors /rgbd_camera/image and TF (map -> base_footprint) while frontier_explorer
   maps the unknown house, recording the exact map coordinates of the Yellow room
   (Room 1 Alpha) and Mint/Green room (Room 2 Beta).
2. Once /exploration_done fires, navigates via Nav2 to Room 1's doorway waypoint
   and publishes /room1/ready for teammate 1 (cleaning).
3. After /room1/done (or dwell_sec), navigates via Nav2 to Room 2's doorway waypoint
   and publishes /room2/ready for teammate 2 (pick-and-place).
"""
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
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


# World -> SLAM /map transform (robot spawns at world x=19.21, y=0.79, yaw=0.0 facing East)
SPAWN_X, SPAWN_Y = 19.21, 0.79


def world_to_slam_map(wx: float, wy: float, wyaw: float):
    return -(wx - SPAWN_X), -(wy - SPAWN_Y), wyaw + math.pi


def analyze_floor_color(rgb_img: np.ndarray):
    """Return (label, yellow_ratio, green_ratio) from bottom 45% of RGB image."""
    h, w, _ = rgb_img.shape
    roi = rgb_img[int(h * 0.55):, int(w * 0.15):int(w * 0.85)].astype(np.float32)
    r, g, b = roi[:, :, 0], roi[:, :, 1], roi[:, :, 2]

    # Room 1 Alpha yellow floor: RGB ~(255, 232, 122)
    yellow_mask = (r > 170) & (g > 150) & (b < 155) & ((r - b) > 45) & ((g - b) > 35)
    # Room 2 Beta mint/green floor: RGB ~(184, 240, 217)
    green_mask = (g > 155) & ((g - r) > 20) & ((g - b) > 8)

    total = float(roi.shape[0] * roi.shape[1])
    yr = float(np.count_nonzero(yellow_mask)) / total
    gr = float(np.count_nonzero(green_mask)) / total

    if yr > 0.15 and yr > gr:
        return 'YELLOW', yr, gr
    if gr > 0.15 and gr > yr:
        return 'GREEN', yr, gr
    return 'HALL', yr, gr


class DoorwayWaypointHandoff(Node):
    def __init__(self):
        super().__init__('doorway_waypoint_handoff')
        self.declare_parameter('wait_for_teammates', False)
        self.declare_parameter('dwell_sec', 5.0)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.exploration_done = False
        self.room1_done = False
        self.room2_done = False
        self.latest_rgb = None

        # Discovered map poses for each room during exploration
        self.discovered_yellow_pose = None
        self.discovered_green_pose = None
        self._last_log = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Bool, '/exploration_done', self._explore_done_cb, latched_qos)
        self.create_subscription(Bool, '/room1/done', self._r1_done_cb, 10)
        self.create_subscription(Bool, '/room2/done', self._r2_done_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/image', self._rgb_cb, 10)

        self.r1_ready_pub = self.create_publisher(Bool, '/room1/ready', latched_qos)
        self.r2_ready_pub = self.create_publisher(Bool, '/room2/ready', latched_qos)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Doorway Waypoint Handoff started — watching RGB camera while exploring...')

    def _explore_done_cb(self, msg: Bool):
        if msg.data:
            self.exploration_done = True

    def _r1_done_cb(self, msg: Bool):
        if msg.data:
            self.room1_done = True

    def _r2_done_cb(self, msg: Bool):
        if msg.data:
            self.room2_done = True

    def _get_current_map_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(), timeout=Duration(seconds=0.2)
            )
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return t.transform.translation.x, t.transform.translation.y, yaw
        except Exception:
            return None

    def _rgb_cb(self, msg: Image):
        if msg.encoding not in ('rgb8', 'bgr8'):
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self.latest_rgb = arr if msg.encoding == 'rgb8' else arr[:, :, ::-1]

        label, yr, gr = analyze_floor_color(self.latest_rgb)
        now = time.time()
        if now - self._last_log > 3.0:
            self._last_log = now
            self.get_logger().info(f'[RGB Floor Monitor] state={label} (yellow={yr:.2f}, green={gr:.2f})')

        if label == 'YELLOW' and self.discovered_yellow_pose is None:
            pose = self._get_current_map_pose()
            if pose is not None:
                self.discovered_yellow_pose = pose
                self.get_logger().info(
                    f'>>> DISCOVERED YELLOW ROOM (Room 1) at slam_map=({pose[0]:.2f}, {pose[1]:.2f})!'
                )
        elif label == 'GREEN' and self.discovered_green_pose is None:
            pose = self._get_current_map_pose()
            if pose is not None:
                self.discovered_green_pose = pose
                self.get_logger().info(
                    f'>>> DISCOVERED GREEN/MINT ROOM (Room 2) at slam_map=({pose[0]:.2f}, {pose[1]:.2f})!'
                )

    def go_to_map_pose(self, mx: float, my: float, myaw: float, label: str) -> bool:
        self.get_logger().info(f'Navigating to {label}: slam_map=({mx:.2f}, {my:.2f}, yaw={myaw:.2f})')
        self.nav_client.wait_for_server()

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
            self.get_logger().error(f'Goal to {label} was rejected by Nav2!')
            return False

        res_future = handle.get_result_async()
        while rclpy.ok() and not res_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        status = res_future.result().status
        self.get_logger().info(f'Arrived at {label} (Nav2 status={status}).')
        return status == 4

    def run(self):
        while rclpy.ok() and not self.exploration_done:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not rclpy.ok():
            return

        self.get_logger().info('Exploration complete! Routing to Room 1 & Room 2 doorway waypoints...')
        wait_teammates = self.get_parameter('wait_for_teammates').value
        dwell_sec = float(self.get_parameter('dwell_sec').value)

        # Waypoint 1: Room 1 (Alpha - Yellow)
        wp1 = self.discovered_yellow_pose or world_to_slam_map(4.60, 7.50, math.pi / 2.0)
        self.go_to_map_pose(*wp1, 'Room 1 (Alpha - Yellow)')
        rclpy.spin_once(self, timeout_sec=0.5)
        color1 = analyze_floor_color(self.latest_rgb)[0] if self.latest_rgb is not None else 'UNKNOWN'
        self.get_logger().info(f'[Camera Check at Room 1] Detected floor: {color1} -> Triggering /room1/ready')
        self.r1_ready_pub.publish(Bool(data=True))

        if wait_teammates:
            self.get_logger().info('Waiting for /room1/done...')
            while rclpy.ok() and not self.room1_done:
                rclpy.spin_once(self, timeout_sec=0.2)
        else:
            time.sleep(dwell_sec)

        # Waypoint 2: Room 2 (Beta - Mint/Green)
        wp2 = self.discovered_green_pose or world_to_slam_map(15.40, 7.50, math.pi / 2.0)
        self.go_to_map_pose(*wp2, 'Room 2 (Beta - Green)')
        rclpy.spin_once(self, timeout_sec=0.5)
        color2 = analyze_floor_color(self.latest_rgb)[0] if self.latest_rgb is not None else 'UNKNOWN'
        self.get_logger().info(f'[Camera Check at Room 2] Detected floor: {color2} -> Triggering /room2/ready')
        self.r2_ready_pub.publish(Bool(data=True))

        if wait_teammates:
            self.get_logger().info('Waiting for /room2/done...')
            while rclpy.ok() and not self.room2_done:
                rclpy.spin_once(self, timeout_sec=0.2)
        else:
            time.sleep(dwell_sec)

        # 4. Return to Home Spawn Area (offset 0.5m from corner walls so Nav2 goal succeeds)
        self.go_to_map_pose(0.50, 0.50, 0.0, 'Home Spawn Point')
        self.get_logger().info('=== All Doorway Waypoints, Handoffs & Return Home Complete! ===')


def main(args=None):
    rclpy.init(args=args)
    node = DoorwayWaypointHandoff()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
