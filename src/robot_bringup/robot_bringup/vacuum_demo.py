#!/usr/bin/env python3
"""
Task 1 Vacuum Cleaner & Dirt Hoovering Demonstration Node.

1. Waits for the robot's current pose from /odom.
2. Spawns a patch of 8 visible "dirty particles" (flat brown/orange cylinders)
   on the floor directly ahead of the robot in Gazebo Harmonic.
3. Activates the under-chassis square vacuum pad (vacuum_link: 0.16m x 0.16m,
   mounted at x=-0.04m under base_link).
4. Drives the robot over the dirty patch while tracking vacuum_link's square
   footprint in real time. Whenever the square vacuum pad passes over a dirt
   particle, the particle is "hoovered up" (deleted from Gazebo) and logged.
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ros_gz_interfaces.msg import Entity, EntityFactory
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity


DIRT_SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="dirt_link">
      <visual name="dirt_visual">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
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


def yaw_from_quaternion(q) -> float:
    """Extract yaw angle (rad) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class VacuumDemoNode(Node):
    """Demonstrates the under-chassis square vacuum hoovering up dirt particles."""

    # Square vacuum geometry (matches vacuum_link in turtlebot3_manipulation.urdf.xacro)
    VACUUM_OFFSET_X = -0.040   # m along base_link X axis
    VACUUM_OFFSET_Y = 0.000    # m along base_link Y axis
    VACUUM_HALF_SIZE = 0.080   # half of 0.16m x 0.16m square pad
    DIRT_RADIUS = 0.025        # m radius of each dirt particle disc

    def __init__(self):
        super().__init__('vacuum_demo')

        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vacuum_state_pub = self.create_publisher(Bool, '/vacuum/active', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10
        )

        # Gazebo entity spawn/delete service clients (bridged via ros_gz_bridge)
        self.spawn_client = self.create_client(SpawnEntity, '/world/empty/create')
        self.delete_client = self.create_client(DeleteEntity, '/world/empty/remove')

        # Robot pose state from /odom
        self.odom_received = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Active dirt particles: {name: (world_x, world_y)}
        self.active_dirt = {}
        self.total_dirt_count = 0
        self.hoovered_count = 0
        self.vacuum_active = False

        self.get_logger().info('Vacuum Demo Node initialized.')

    def _odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True

        if self.vacuum_active and self.active_dirt:
            self._check_and_hoover_dirt()

    def get_vacuum_world_pose(self):
        """Return (vac_x, vac_y, robot_yaw) of vacuum_link in the odom/world frame."""
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        vac_x = self.robot_x + self.VACUUM_OFFSET_X * cos_yaw - self.VACUUM_OFFSET_Y * sin_yaw
        vac_y = self.robot_y + self.VACUUM_OFFSET_X * sin_yaw + self.VACUUM_OFFSET_Y * cos_yaw
        return vac_x, vac_y, self.robot_yaw

    def _check_and_hoover_dirt(self):
        """Check if any remaining dirt particle is under the 0.16m x 0.16m square vacuum pad."""
        vac_x, vac_y, yaw = self.get_vacuum_world_pose()
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # Capture half-width includes the square vacuum half-size + dirt particle radius
        capture_half_extent = self.VACUUM_HALF_SIZE + self.DIRT_RADIUS

        to_hoover = []
        for name, (px, py) in self.active_dirt.items():
            # Transform particle position into vacuum_link's local square frame
            dx = px - vac_x
            dy = py - vac_y
            local_x = dx * cos_yaw + dy * sin_yaw
            local_y = -dx * sin_yaw + dy * cos_yaw

            if abs(local_x) <= capture_half_extent and abs(local_y) <= capture_half_extent:
                to_hoover.append((name, px, py))

        for name, px, py in to_hoover:
            del self.active_dirt[name]
            self.hoovered_count += 1
            pct = int(100.0 * self.hoovered_count / max(1, self.total_dirt_count))
            self.get_logger().info(
                f'[VACUUM] Hoovered up {name} at ({px:.2f}, {py:.2f}) '
                f'| Cleaned: {self.hoovered_count}/{self.total_dirt_count} ({pct}%)'
            )
            self._delete_entity_async(name)

    def _delete_entity_async(self, name: str):
        """Remove a dirt particle model from Gazebo Harmonic."""
        req = DeleteEntity.Request()
        req.entity.name = name
        req.entity.type = Entity.MODEL
        self.delete_client.call_async(req)

    def _spawn_dirt_particle(self, name: str, x: float, y: float) -> bool:
        """Spawn a single dirt particle disc on the floor at (x, y)."""
        # Clean up any prior entity with the same name first
        del_req = DeleteEntity.Request()
        del_req.entity.name = name
        del_req.entity.type = Entity.MODEL
        del_future = self.delete_client.call_async(del_req)
        self._spin_until_future(del_future, timeout_sec=1.0)

        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.allow_renaming = False
        req.entity_factory.sdf = DIRT_SDF_TEMPLATE.format(
            name=name, radius=self.DIRT_RADIUS
        )
        req.entity_factory.pose.position.x = float(x)
        req.entity_factory.pose.position.y = float(y)
        req.entity_factory.pose.position.z = 0.002
        req.entity_factory.pose.orientation.w = 1.0
        req.entity_factory.relative_to = 'world'

        future = self.spawn_client.call_async(req)
        if self._spin_until_future(future, timeout_sec=3.0):
            res = future.result()
            return bool(res and res.success)
        return False

    def _spin_until_future(self, future, timeout_sec: float) -> bool:
        start = time.time()
        while rclpy.ok() and not future.done() and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def _spin_sleep(self, duration_sec: float):
        start = time.time()
        while rclpy.ok() and (time.time() - start) < duration_sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_demo(self):
        self.get_logger().info('=' * 62)
        self.get_logger().info('Starting Task 1 Vacuum Cleaner & Dirt Hoovering Demo...')
        self.get_logger().info('=' * 62)

        # 1. Wait for /odom so we know where the robot is
        self.get_logger().info('Waiting for /odom from Gazebo...')
        start_wait = time.time()
        while rclpy.ok() and not self.odom_received and (time.time() - start_wait) < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.odom_received:
            self.get_logger().error(
                'No /odom received! Make sure simulation.launch.py is running in Terminal 1.'
            )
            return

        # 2. Wait for Gazebo Spawn/Delete services
        self.get_logger().info('Connecting to Gazebo world entity services...')
        if not self.spawn_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'Service /world/empty/create not available! Rebuild and relaunch simulation.launch.py.'
            )
            return
        self.delete_client.wait_for_service(timeout_sec=5.0)

        # 3. Spawn 8 dirty particles in a staggered patch ahead of the robot
        x0, y0, yaw0 = self.robot_x, self.robot_y, self.robot_yaw
        cos0, sin0 = math.cos(yaw0), math.sin(yaw0)

        # Offsets (forward_dist_m, lateral_offset_m) relative to initial robot pose
        dirt_offsets = [
            (0.14,  0.00),
            (0.20,  0.04),
            (0.26, -0.04),
            (0.32,  0.03),
            (0.38, -0.03),
            (0.44,  0.04),
            (0.50, -0.02),
            (0.56,  0.00),
        ]

        self.get_logger().info(
            f'Robot at ({x0:.2f}, {y0:.2f}, yaw={math.degrees(yaw0):.1f} deg). '
            f'Spawning {len(dirt_offsets)} dirty particles ahead...'
        )

        self.active_dirt.clear()
        for idx, (fwd, lat) in enumerate(dirt_offsets, start=1):
            name = f'dirt_particle_{idx}'
            px = x0 + fwd * cos0 - lat * sin0
            py = y0 + fwd * sin0 + lat * cos0
            ok = self._spawn_dirt_particle(name, px, py)
            self.active_dirt[name] = (px, py)
            status = 'Spawned' if ok else 'Queued'
            self.get_logger().info(f'  [{status}] {name} at ({px:.2f}, {py:.2f})')

        self.total_dirt_count = len(self.active_dirt)
        self.hoovered_count = 0

        # Short pause so the user can see all dirty particles on the floor in Gazebo
        self.get_logger().info('Dirty particles placed! Starting vacuum cleaner in 2 seconds...')
        self._spin_sleep(2.0)

        # 4. Activate vacuum and drive forward over the dirty patch
        self.vacuum_active = True
        vac_msg = Bool()
        vac_msg.data = True
        self.vacuum_state_pub.publish(vac_msg)
        self.get_logger().info(
            '[VACUUM ON] Driving over dirty patch (square vacuum pad = 0.16m x 0.16m)...'
        )

        # Target travel distance = 0.64m so the rear-offset vacuum (-0.04m) clears the last particle at 0.56m
        target_distance = 0.64
        linear_speed = 0.20  # m/s
        max_drive_time = 40.0

        twist = Twist()
        twist.linear.x = linear_speed
        drive_start = time.time()

        while rclpy.ok() and (time.time() - drive_start) < max_drive_time:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

            dist_travelled = math.hypot(self.robot_x - x0, self.robot_y - y0)
            if self.hoovered_count >= self.total_dirt_count or dist_travelled >= target_distance:
                break

        # Stop the robot and turn off the vacuum
        self.cmd_vel_pub.publish(Twist())
        self._spin_sleep(0.5)
        self.vacuum_active = False
        vac_msg.data = False
        self.vacuum_state_pub.publish(vac_msg)

        # 5. Summary Report
        pct = int(100.0 * self.hoovered_count / max(1, self.total_dirt_count))
        self.get_logger().info('=' * 62)
        self.get_logger().info(
            f'Vacuum Demo Finished! Hoovered {self.hoovered_count}/{self.total_dirt_count} '
            f'dirty particles ({pct}% cleaned).'
        )
        self.get_logger().info('=' * 62)


def main(args=None):
    rclpy.init(args=args)
    node = VacuumDemoNode()
    try:
        node.run_demo()
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
