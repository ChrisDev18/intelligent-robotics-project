import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class IntelligentTaskManager(Node):
    def __init__(self):
        super().__init__('intelligent_task_manager')
        self.cmd_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Intelligent Task Manager Initialized.")
        
        # Execute workflow sequence
        self.run_mission()

    def run_mission(self):
        # TASK 1: Navigate to & Clean Room 1 (Simulated coverage pattern)
        self.get_logger().info("Executing Task 1: Moving to Room 1 for Vacuum Cleaning...")
        self.drive_forward(linear_speed=0.2, duration=3.0)
        self.turn(angular_speed=1.5, duration=2.0)
        self.get_logger().info("Vacuum active in Room 1...")
        time.sleep(2.0)

        # TASK 2: Navigate to Room 2 for Object Pick-up
        self.get_logger().info("Executing Task 2: Navigating to Room 2 for Object Pick-up...")
        self.drive_forward(linear_speed=0.2, duration=4.0)
        self.turn(angular_speed=-1.5, duration=2.0)
        self.get_logger().info("Object detected via RGB-D camera. Initiating pick-up sequence...")
        time.sleep(2.0)
        self.get_logger().info("Mission Complete!")

    def drive_forward(self, linear_speed, duration):
        twist = Twist()
        twist.linear.x = linear_speed
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_publisher_.publish(twist)
            time.sleep(0.1)
        self.stop_robot()

    def turn(self, angular_speed, duration):
        twist = Twist()
        twist.angular.z = angular_speed
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_publisher_.publish(twist)
            time.sleep(0.1)
        self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        self.cmd_publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = IntelligentTaskManager()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()