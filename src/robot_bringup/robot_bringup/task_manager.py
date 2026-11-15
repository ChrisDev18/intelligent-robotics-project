#!/usr/bin/env python3
"""Autonomous Mission Executive: Unknown-World Floor Color Detection & Room Tasks."""
import math
import time
import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from builtin_interfaces.msg import Duration as MsgDuration
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def classify_floor_rgb(rgb_img: np.ndarray) -> str:
    """Classify the floor color in the bottom 35% ROI of an RGB image."""
    h, w, _ = rgb_img.shape
    roi = rgb_img[int(h * 0.55):, int(w * 0.15):int(w * 0.85)].astype(np.float32)
    r, g, b = roi[:, :, 0], roi[:, :, 1], roi[:, :, 2]

    yellow_mask = (r > 180) & (g > 160) & (b < 150) & ((r - b) > 55) & ((g - b) > 45)
    green_mask = (g > 165) & ((g - r) > 25) & ((g - b) > 10)

    total = float(roi.shape[0] * roi.shape[1])
    yellow_ratio = np.count_nonzero(yellow_mask) / total
    green_ratio = np.count_nonzero(green_mask) / total

    if yellow_ratio > 0.15 and yellow_ratio > green_ratio:
        return 'YELLOW'
    if green_ratio > 0.15 and green_ratio > yellow_ratio:
        return 'GREEN'
    return 'HALL'


class IntelligentTaskManager(Node):
    ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']

    def __init__(self):
        super().__init__('intelligent_task_manager')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vacuum_pub = self.create_publisher(Bool, '/vacuum/active', 10)
        self.pause_explore_pub = self.create_publisher(Bool, '/exploration_pause', 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_rgb = None
        self.latest_depth = None
        self.create_subscription(Image, '/rgbd_camera/image', self._rgb_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/depth_image', self._depth_cb, 10)

        self.state = 'EXPLORING'
        self.alpha_done = False
        self.beta_done = False
        self.yellow_streak = 0
        self.green_streak = 0

        self.get_logger().info('Intelligent Task Manager ready — monitoring floor color while exploring...')
        self.create_timer(0.25, self._control_loop)

    def _rgb_cb(self, msg: Image):
        if msg.encoding in ('rgb8', 'bgr8'):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            self.latest_rgb = arr if msg.encoding == 'rgb8' else arr[:, :, ::-1]

    def _depth_cb(self, msg: Image):
        if msg.encoding == '32FC1':
            self.latest_depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)

    def _get_map_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(), timeout=Duration(seconds=0.3)
            )
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return t.transform.translation.x, t.transform.translation.y, yaw
        except Exception:
            return None

    def _control_loop(self):
        if self.state != 'EXPLORING' or self.latest_rgb is None:
            return

        floor = classify_floor_rgb(self.latest_rgb)
        self.yellow_streak = (self.yellow_streak + 1) if floor == 'YELLOW' else 0
        self.green_streak = (self.green_streak + 1) if floor == 'GREEN' else 0

        if not self.alpha_done and self.yellow_streak >= 8:
            self.state = 'CLEANING_ALPHA'
            self.execute_alpha_cleaning()
        elif not self.beta_done and self.green_streak >= 8:
            self.state = 'PICK_PLACE_BETA'
            self.execute_beta_pick_and_place()

    def _set_exploration_paused(self, paused: bool):
        for _ in range(3):
            self.pause_explore_pub.publish(Bool(data=paused))
            rclpy.spin_once(self, timeout_sec=0.05)

    def _nav_relative(self, forward_m: float, lateral_m: float = 0.0, yaw_offset: float = 0.0) -> bool:
        pose = self._get_map_pose()
        if pose is None or not self.nav_client.wait_for_server(timeout_sec=3.0):
            return False
        x, y, yaw = pose
        gx = x + forward_m * math.cos(yaw) - lateral_m * math.sin(yaw)
        gy = y + forward_m * math.sin(yaw) + lateral_m * math.cos(yaw)
        gyaw = yaw + yaw_offset

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = Time().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(gyaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(gyaw / 2.0)

        send_future = self.nav_client.send_goal_async(goal)
        while rclpy.ok() and not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = send_future.result()
        if not handle or not handle.accepted:
            return False

        res_future = handle.get_result_async()
        start = time.time()
        while rclpy.ok() and not res_future.done() and (time.time() - start) < 45.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        return res_future.done()

    def _move_arm(self, positions, duration_sec: float = 2.0):
        traj = JointTrajectory()
        traj.joint_names = list(self.ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        sec = int(duration_sec)
        pt.time_from_start = MsgDuration(sec=sec, nanosec=int((duration_sec - sec) * 1e9))
        traj.points.append(pt)
        self.arm_pub.publish(traj)
        end = time.time() + duration_sec + 0.3
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _control_gripper(self, position: float):
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 10.0
        self.gripper_client.send_goal_async(goal)
        end = time.time() + 1.2
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _visual_servo_to_orange_cube(self) -> bool:
        self.get_logger().info('[Room 2] Searching & aligning with orange pick item via RGB-D...')
        start = time.time()
        while rclpy.ok() and (time.time() - start) < 35.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_rgb is None:
                continue
            rgb = self.latest_rgb.astype(np.float32)
            r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
            cube_mask = (r > 170) & (g > 85) & (g < 190) & (b < 95)
            ys, xs = np.nonzero(cube_mask)
            twist = Twist()
            if len(xs) < 40:
                twist.angular.z = 0.25
                self.cmd_pub.publish(twist)
                continue

            cx = float(np.mean(xs))
            cy = int(np.mean(ys))
            err_x = (cx - (rgb.shape[1] / 2.0)) / (rgb.shape[1] / 2.0)

            dist = 0.50
            if self.latest_depth is not None:
                d_val = float(self.latest_depth[cy, int(cx)])
                if math.isfinite(d_val) and d_val > 0.05:
                    dist = d_val

            if abs(err_x) > 0.08:
                twist.angular.z = -0.5 * err_x
            elif dist > 0.26:
                twist.linear.x = 0.12
            else:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f'[Room 2] Aligned with cube at depth={dist:.2f} m.')
                return True
            self.cmd_pub.publish(twist)

        self.cmd_pub.publish(Twist())
        return False

    def execute_alpha_cleaning(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('[TASK 1] YELLOW FLOOR DETECTED (Room 1 Alpha) — Pausing exploration & cleaning!')
        self.get_logger().info('=' * 60)
        self._set_exploration_paused(True)
        self.cmd_pub.publish(Twist())

        self.vacuum_pub.publish(Bool(data=True))
        self.get_logger().info('[Vacuum ON] Executing coverage pattern inside Room 1...')
        self._nav_relative(forward_m=1.4, lateral_m=0.0, yaw_offset=0.0)
        self._nav_relative(forward_m=0.0, lateral_m=-1.2, yaw_offset=0.0)
        self._nav_relative(forward_m=1.0, lateral_m=0.0, yaw_offset=0.0)
        self._nav_relative(forward_m=0.0, lateral_m=1.2, yaw_offset=0.0)
        self.vacuum_pub.publish(Bool(data=False))
        self.get_logger().info('[Vacuum OFF] Room 1 cleaning complete! Returning to Hall...')

        self._nav_relative(forward_m=-1.6, lateral_m=0.0, yaw_offset=3.14)
        self.alpha_done = True
        self.yellow_streak = 0
        self.state = 'EXPLORING'
        self._set_exploration_paused(False)

    def execute_beta_pick_and_place(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('[TASK 2] GREEN/MINT FLOOR DETECTED (Room 2 Beta) — Initiating Pick & Place!')
        self.get_logger().info('=' * 60)
        self._set_exploration_paused(True)
        self.cmd_pub.publish(Twist())

        self._nav_relative(forward_m=1.2, lateral_m=0.0, yaw_offset=0.0)
        self._visual_servo_to_orange_cube()

        self.get_logger().info('[Arm] Opening gripper and reaching for beta_pick_item...')
        self._control_gripper(0.018)
        self._move_arm([0.0, 0.45, 0.15, -0.60], duration_sec=2.5)
        self._control_gripper(-0.005)
        self._move_arm([0.0, -0.80, 0.20, 0.60], duration_sec=2.5)

        self.get_logger().info('[Nav2] Carrying item to designated drop spot in Room 2...')
        self._nav_relative(forward_m=0.0, lateral_m=-2.2, yaw_offset=-1.57)
        self._move_arm([0.0, 0.45, 0.15, -0.60], duration_sec=2.5)
        self._control_gripper(0.018)
        self._move_arm([0.0, -1.05, 0.35, 0.70], duration_sec=2.0)

        self.beta_done = True
        self.green_streak = 0
        self.get_logger().info('[TASK 2] Pick & Place complete!')

        if self.alpha_done:
            self.state = 'COMPLETE'
            self.get_logger().info('*** ALL ROOM TASKS COMPLETE! ***')
        else:
            self.state = 'EXPLORING'
            self._set_exploration_paused(False)


def main(args=None):
    rclpy.init(args=args)
    node = IntelligentTaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
