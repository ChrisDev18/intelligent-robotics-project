import math
import time
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, JointState, LaserScan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class RobotDemoAndSensorTest(Node):
    """Test node that verifies all sensors and demonstrates base + arm + gripper movement."""

    ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']

    def __init__(self):
        super().__init__('robot_demo_and_sensor_test')

        # Publishers & Action Clients
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )

        # Sensor state tracking
        self.sensor_status = {
            'scan': None,
            'odom': None,
            'imu': None,
            'rgb_image': None,
            'depth_image': None,
            'joint_states': None,
        }

        # Subscribers for all sensors
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/image', self.rgb_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/depth_image', self.depth_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 10)

        self.get_logger().info('=== TurtleBot3 Manipulation Demo & Sensor Test Started ===')

    # --- Sensor Callbacks ---
    def scan_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        min_dist = min(valid) if valid else float('inf')
        self.sensor_status['scan'] = f'{len(msg.ranges)} rays, min_range={min_dist:.2f}m'

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.sensor_status['odom'] = f'pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})'

    def imu_cb(self, msg: Imu):
        a = msg.linear_acceleration
        self.sensor_status['imu'] = f'accel=({a.x:.2f}, {a.y:.2f}, {a.z:.2f}) m/s^2'

    def rgb_cb(self, msg: Image):
        self.sensor_status['rgb_image'] = f'{msg.width}x{msg.height} ({msg.encoding})'

    def depth_cb(self, msg: Image):
        self.sensor_status['depth_image'] = f'{msg.width}x{msg.height} ({msg.encoding})'

    def joint_states_cb(self, msg: JointState):
        self.sensor_status['joint_states'] = f'joints={list(msg.name)}'

    def spin_for(self, duration_sec: float):
        end_time = time.time() + duration_sec
        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def print_sensor_report(self):
        self.get_logger().info('--- Sensor Status Report ---')
        for name, detail in self.sensor_status.items():
            status = 'OK' if detail is not None else 'NO DATA YET'
            self.get_logger().info(f'  [{status:11s}] {name:14s}: {detail}')
        self.get_logger().info('----------------------------')

    def move_arm(self, positions, duration_sec=2.0, description=''):
        if description:
            self.get_logger().info(f'[Arm] {description}: {positions}')
        traj = JointTrajectory()
        traj.joint_names = list(self.ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        traj.points.append(point)
        self.arm_pub.publish(traj)
        self.spin_for(duration_sec + 0.5)

    def move_gripper(self, position: float, description=''):
        if description:
            self.get_logger().info(f'[Gripper] {description} (pos={position:.3f}m)')
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Gripper action server not available; skipping gripper command.')
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 10.0
        self.gripper_client.send_goal_async(goal)
        self.spin_for(1.2)

    def drive(self, linear_x: float, angular_z: float, duration_sec: float, description=''):
        if description:
            self.get_logger().info(
                f'[Base] {description} (linear={linear_x:.2f} m/s, angular={angular_z:.2f} rad/s)'
            )
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        end_time = time.time() + duration_sec
        while time.time() < end_time and rclpy.ok():
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd_pub.publish(Twist())
        self.spin_for(0.5)

    def run_demo(self):
        # 1. Collect initial sensor messages
        self.get_logger().info('Waiting 2 seconds to collect sensor data from Gazebo...')
        self.spin_for(2.0)
        self.print_sensor_report()

        # 2. Test OpenManipulator-X Arm & Gripper
        self.move_arm([0.0, -1.05, 0.35, 0.70], duration_sec=2.0, description='Moving to Home Pose')
        self.move_gripper(0.018, description='Opening parallel-jaw gripper')
        self.move_arm([0.8, -0.4, 0.2, 0.2], duration_sec=2.0, description='Rotating arm left')
        self.move_arm([-0.8, -0.4, 0.2, 0.2], duration_sec=2.5, description='Rotating arm right')
        self.move_arm([0.0, 0.45, 0.15, -0.60], duration_sec=2.0, description='Reaching down (Pick Pose)')
        self.move_gripper(-0.005, description='Closing parallel-jaw gripper')
        self.move_arm([0.0, -1.05, 0.35, 0.70], duration_sec=2.0, description='Returning to Home Pose')

        # 3. Test Differential-Drive Base Movement
        self.drive(linear_x=0.25, angular_z=0.0, duration_sec=3.0, description='Driving forward')
        self.drive(linear_x=0.15, angular_z=0.8, duration_sec=4.0, description='Driving in an arc')
        self.drive(linear_x=-0.20, angular_z=0.0, duration_sec=2.0, description='Reversing')

        # 4. Final sensor report after movement
        self.print_sensor_report()
        self.get_logger().info('=== Demo & Sensor Test Complete! ===')


def main(args=None):
    rclpy.init(args=args)
    node = RobotDemoAndSensorTest()
    try:
        node.run_demo()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
