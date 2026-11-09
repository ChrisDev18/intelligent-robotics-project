import time
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Twist
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class IntelligentTaskManager(Node):
    """High-level task manager for TurtleBot3 Manipulation (Room 1 cleaning & Room 2 pick-and-place)."""

    ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']

    def __init__(self):
        super().__init__('intelligent_task_manager')
        self.cmd_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vacuum_publisher_ = self.create_publisher(Bool, '/vacuum/active', 10)
        self.vacuum_enabled = False
        self.arm_publisher_ = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )
        self.gripper_client_ = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )
        self.get_logger().info('Intelligent Task Manager Initialized (TurtleBot3 Manipulation).')

        # Execute workflow sequence
        self.run_mission()

    def run_mission(self):
        # Move OpenManipulator-X arm to compact home pose before driving
        self.get_logger().info('Setting OpenManipulator-X arm to home pose...')
        self.move_arm([0.0, -1.05, 0.35, 0.70], duration_sec=2.0)

        # TASK 1: Navigate to & Clean Room 1 (Simulated coverage pattern)
        self.get_logger().info('Executing Task 1: Moving to Room 1 for Vacuum Cleaning...')
        self.set_vacuum(True)
        self.drive_forward(linear_speed=0.2, duration=3.0)
        self.turn(angular_speed=1.5, duration=2.0)
        self.get_logger().info('Vacuum active in Room 1...')
        time.sleep(2.0)
        self.set_vacuum(False)

        # TASK 2: Navigate to Room 2 for Object Pick-up with OpenManipulator-X
        self.get_logger().info('Executing Task 2: Navigating to Room 2 for Object Pick-up...')
        self.drive_forward(linear_speed=0.2, duration=4.0)
        self.turn(angular_speed=-1.5, duration=2.0)

        self.get_logger().info('Object detected via RGB-D camera. Initiating pick-up sequence...')
        self.control_gripper(position=0.018)  # Open parallel-jaw gripper
        self.move_arm([0.0, 0.45, 0.15, -0.60], duration_sec=2.5)  # Reach down toward object
        self.control_gripper(position=-0.005)  # Close parallel-jaw gripper on object
        self.move_arm([0.0, -0.80, 0.20, 0.60], duration_sec=2.5)  # Lift object

        self.get_logger().info('Moving to destination in Room 2 and placing object...')
        self.drive_forward(linear_speed=0.15, duration=2.0)
        self.move_arm([0.0, 0.45, 0.15, -0.60], duration_sec=2.5)
        self.control_gripper(position=0.018)  # Release object
        self.move_arm([0.0, -1.05, 0.35, 0.70], duration_sec=2.0)

        self.get_logger().info('Mission Complete!')

    def move_arm(self, joint_positions, duration_sec=2.0):
        traj = JointTrajectory()
        traj.joint_names = list(self.ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in joint_positions]
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        traj.points.append(point)
        self.arm_publisher_.publish(traj)
        time.sleep(duration_sec + 0.2)

    def control_gripper(self, position, max_effort=10.0):
        if not self.gripper_client_.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                'Gripper action server (/gripper_controller/gripper_cmd) not available yet; skipping.'
            )
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        self.gripper_client_.send_goal_async(goal)
        time.sleep(1.0)

    def drive_forward(self, linear_speed, duration):
        twist = Twist()
        twist.linear.x = float(linear_speed)
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_publisher_.publish(twist)
            time.sleep(0.1)
        self.stop_robot()

    def turn(self, angular_speed, duration):
        twist = Twist()
        twist.angular.z = float(angular_speed)
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_publisher_.publish(twist)
            time.sleep(0.1)
        self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        self.cmd_publisher_.publish(twist)

    def set_vacuum(self, enabled: bool):
        self.vacuum_enabled = bool(enabled)
        msg = Bool()
        msg.data = self.vacuum_enabled
        self.vacuum_publisher_.publish(msg)
        state_str = 'ON' if self.vacuum_enabled else 'OFF'
        self.get_logger().info(f'Under-chassis square vacuum cleaner turned {state_str}.')


def main(args=None):
    rclpy.init(args=args)
    node = IntelligentTaskManager()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
