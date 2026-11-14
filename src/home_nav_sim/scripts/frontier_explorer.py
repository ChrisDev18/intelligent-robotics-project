#!/usr/bin/env python3
"""Autonomous frontier exploration for the home world.

The robot starts with no map. slam_toolbox builds /map as it drives; this node
repeatedly finds *frontiers* (free cells that touch unknown space), picks the
best one and sends it to Nav2 as a NavigateToPose goal, until no frontiers are
left. Then it publishes True on /exploration_done (latched) and, by default,
drives back to where it started.

Parameters
  min_frontier_size  smallest frontier worth visiting, in cells (default 10)
  goal_timeout       seconds before giving up on one goal (default 180)
  blacklist_radius   metres around a failed goal to skip later (default 0.6)
  return_home        drive back to the start pose when done (default true)
  start_delay        sim seconds to wait after the first map before exploring (default 20)
  finish_after       sim seconds with no frontiers before finishing (default 30)
"""
import math
from collections import deque

import numpy as np

FREE_MAX = 25        # OccupancyGrid values 0..25 count as free
UNKNOWN = -1


# --------------------------------------------------------------------------
# Frontier detection (pure numpy, no ROS, so it can be tested on its own)
# --------------------------------------------------------------------------
def dilate(mask, cells):
    """Grow a boolean mask by `cells` in every direction (8-connected)."""
    out = mask.copy()
    for _ in range(cells):
        grown = out.copy()
        grown[1:, :] |= out[:-1, :]
        grown[:-1, :] |= out[1:, :]
        grown[:, 1:] |= out[:, :-1]
        grown[:, :-1] |= out[:, 1:]
        grown[1:, 1:] |= out[:-1, :-1]
        grown[1:, :-1] |= out[:-1, 1:]
        grown[:-1, 1:] |= out[1:, :-1]
        grown[:-1, :-1] |= out[1:, 1:]
        out = grown
    return out


def find_frontiers(grid, min_size=10, clearance_cells=5):
    """Return frontier clusters as lists of (row, col) cells.

    grid: 2-D int array of OccupancyGrid values (-1 unknown, 0..100 occupancy),
          row 0 = bottom of the map, as in nav_msgs/OccupancyGrid.
    clearance_cells: frontier cells this close to an obstacle are ignored; the
          robot can't stand there, and they are usually unknown space inside
          furniture or behind a wall rather than unexplored rooms.
    """
    free = (grid >= 0) & (grid <= FREE_MAX)
    free &= ~dilate(grid > FREE_MAX, clearance_cells)
    unknown = grid == UNKNOWN

    # Free cells with at least one 4-connected unknown neighbour.
    nb_unknown = np.zeros_like(unknown)
    nb_unknown[1:, :] |= unknown[:-1, :]
    nb_unknown[:-1, :] |= unknown[1:, :]
    nb_unknown[:, 1:] |= unknown[:, :-1]
    nb_unknown[:, :-1] |= unknown[:, 1:]
    frontier = free & nb_unknown

    # Group frontier cells into 8-connected clusters.
    seen = np.zeros_like(frontier)
    clusters = []
    rows, cols = grid.shape
    for r0, c0 in zip(*np.nonzero(frontier)):
        if seen[r0, c0]:
            continue
        cluster, queue = [], deque([(r0, c0)])
        seen[r0, c0] = True
        while queue:
            r, c = queue.popleft()
            cluster.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < rows and 0 <= cc < cols and frontier[rr, cc] and not seen[rr, cc]:
                        seen[rr, cc] = True
                        queue.append((rr, cc))
        if len(cluster) >= min_size:
            clusters.append(cluster)
    return clusters


def goal_cell(cluster):
    """The cluster cell closest to the cluster's centroid (always a real frontier cell)."""
    pts = np.array(cluster, dtype=float)
    centroid = pts.mean(axis=0)
    return tuple(int(v) for v in pts[np.argmin(((pts - centroid) ** 2).sum(axis=1))])


def choose_frontier(clusters, robot_rc, resolution, blacklist_rc=(), blacklist_cells=12):
    """Pick the cluster with the best size/distance trade-off, skipping blacklisted goals.

    Returns (goal_rc, cluster) or None.
    """
    best, best_score = None, -math.inf
    for cluster in clusters:
        g = goal_cell(cluster)
        if any(math.hypot(g[0] - b[0], g[1] - b[1]) <= blacklist_cells for b in blacklist_rc):
            continue
        dist_m = math.hypot(g[0] - robot_rc[0], g[1] - robot_rc[1]) * resolution
        size_m = len(cluster) * resolution
        score = 2.0 * size_m - dist_m       # prefer big, nearby frontiers
        if score > best_score:
            best, best_score = (g, cluster), score
    return best


# --------------------------------------------------------------------------
# ROS 2 node
# --------------------------------------------------------------------------
def main():
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from std_msgs.msg import Bool
    from tf2_ros import Buffer, TransformListener

    class FrontierExplorer(Node):
        def __init__(self):
            super().__init__('frontier_explorer')
            self.min_size = self.declare_parameter('min_frontier_size', 10).value
            self.goal_timeout = self.declare_parameter('goal_timeout', 180.0).value
            self.blacklist_radius = self.declare_parameter('blacklist_radius', 0.6).value
            self.return_home = self.declare_parameter('return_home', True).value
            self.start_delay = self.declare_parameter('start_delay', 20.0).value
            self.finish_after = self.declare_parameter('finish_after', 30.0).value

            map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
            self.done_pub = self.create_publisher(Bool, '/exploration_done', map_qos)
            self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            self.map = None
            self.start_xy = None
            self.goal_handle = None
            self.goal_xy = None
            self.goal_sent_at = None
            self.blacklist_xy = []
            self.first_ready = None      # sim time when map, Nav2 and TF were first all ready
            self.empty_since = None      # sim time when frontiers first ran out
            self.retried_blacklist = False
            self.finished = False
            self.create_timer(2.0, self.tick)
            self.get_logger().info('Frontier explorer started, waiting for /map and Nav2...')

        # --- helpers -------------------------------------------------------
        def on_map(self, msg):
            self.map = msg

        def waiting(self, reason):
            self.get_logger().info(f'Waiting: {reason}', throttle_duration_sec=10.0)

        def robot_xy(self):
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time(),
                                                    timeout=Duration(seconds=0.2))
            except Exception:
                return None
            return t.transform.translation.x, t.transform.translation.y

        def to_rc(self, x, y):
            info = self.map.info
            return (int((y - info.origin.position.y) / info.resolution),
                    int((x - info.origin.position.x) / info.resolution))

        def to_xy(self, r, c):
            info = self.map.info
            return (info.origin.position.x + (c + 0.5) * info.resolution,
                    info.origin.position.y + (r + 0.5) * info.resolution)

        def send_goal(self, x, y, yaw=0.0):
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
            goal.pose.pose.orientation.z = math.sin(yaw / 2)
            goal.pose.pose.orientation.w = math.cos(yaw / 2)
            self.goal_xy = (x, y)
            self.goal_sent_at = self.get_clock().now()
            self.nav.send_goal_async(goal).add_done_callback(self.on_goal_accepted)

        def on_goal_accepted(self, future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warn('Nav2 rejected the goal, skipping it')
                self.fail_current_goal()
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(self.on_goal_result)

        def on_goal_result(self, future):
            status = future.result().status
            if status == 4:   # SUCCEEDED
                self.get_logger().info(f'Reached frontier goal ({self.goal_xy[0]:.2f}, {self.goal_xy[1]:.2f})')
                # Never pick this spot again: a real frontier vanishes once the robot is
                # there, so one that is still present can't be cleared from here.
                self.blacklist_xy.append(self.goal_xy)
                self.clear_goal()
            elif status == 5:  # CANCELED (by us, after a timeout or a vanished frontier)
                self.clear_goal()
            else:
                self.get_logger().warn(f'Frontier goal failed (status {status}), blacklisting it')
                self.fail_current_goal()

        def fail_current_goal(self):
            if self.goal_xy:
                self.blacklist_xy.append(self.goal_xy)
            self.clear_goal()

        def clear_goal(self):
            self.goal_handle = self.goal_xy = self.goal_sent_at = None

        # --- main loop -----------------------------------------------------
        def tick(self):
            if self.finished:
                return
            if self.map is None:
                self.waiting('no /map from slam_toolbox yet')
                return
            if not self.nav.server_is_ready():
                self.waiting('Nav2 navigate_to_pose action server not ready yet')
                return
            pos = self.robot_xy()
            if pos is None:
                self.waiting('no map -> base_footprint transform yet (is SLAM publishing TF?)')
                return
            now = self.get_clock().now()
            if self.first_ready is None:
                self.first_ready = now
                self.get_logger().info(
                    f'Map, Nav2 and TF ready; letting SLAM settle for {self.start_delay:.0f} s')
            if (now - self.first_ready).nanoseconds / 1e9 < self.start_delay:
                return
            if self.start_xy is None:
                self.start_xy = pos

            info = self.map.info
            grid = np.array(self.map.data, dtype=np.int16).reshape(info.height, info.width)
            clearance = max(1, int(0.25 / info.resolution))   # ~robot radius
            clusters = find_frontiers(grid, self.min_size, clearance)

            # A goal is in progress: give up on it if it timed out.
            if self.goal_xy is not None:
                if self.goal_handle is None:
                    return   # still waiting for Nav2 to accept it
                elapsed = (self.get_clock().now() - self.goal_sent_at).nanoseconds / 1e9
                if elapsed > self.goal_timeout:
                    self.get_logger().warn('Frontier goal timed out, blacklisting it')
                    self.blacklist_xy.append(self.goal_xy)
                    self.goal_handle.cancel_goal_async()
                return

            robot_rc = self.to_rc(*pos)
            bl_rc = [self.to_rc(x, y) for x, y in self.blacklist_xy]
            choice = choose_frontier(clusters, robot_rc, info.resolution, bl_rc,
                                     blacklist_cells=self.blacklist_radius / info.resolution)
            if choice is None:
                if self.empty_since is None:
                    self.empty_since = now
                    self.get_logger().info(f'No frontiers to visit ({len(clusters)} skipped); '
                                           f'waiting up to {self.finish_after:.0f} s')
                if (now - self.empty_since).nanoseconds / 1e9 < self.finish_after:
                    return
                if self.blacklist_xy and not self.retried_blacklist:
                    # Goals often fail early (SLAM settling, timing); try them all once more.
                    self.get_logger().info(f'Retrying {len(self.blacklist_xy)} skipped goals once')
                    self.blacklist_xy.clear()
                    self.retried_blacklist = True
                    self.empty_since = None
                    return
                self.finish()
                return
            self.empty_since = None
            (r, c), cluster = choice
            x, y = self.to_xy(r, c)
            yaw = math.atan2(y - pos[1], x - pos[0])
            self.get_logger().info(
                f'{len(clusters)} frontiers left; heading to ({x:.2f}, {y:.2f}), size {len(cluster)} cells')
            self.send_goal(x, y, yaw)

        def finish(self):
            self.finished = True
            self.get_logger().info('Exploration complete: no reachable frontiers left.')
            self.done_pub.publish(Bool(data=True))
            if self.return_home and self.start_xy is not None:
                self.get_logger().info('Returning to the start position.')
                self.send_goal(*self.start_xy)

    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
