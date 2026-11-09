Intelligent Robotics Group 2

# Objective
Develop a simulated mobile robot that maps an unknown, dynamic indoor (structured) environment, which autonomously cleans Room 1 and moves an item in Room 2, avoiding moving obstacles.
# Scenario and Approach
We will simulate a differential-drive mobile robot in an unknown indoor environment with two rooms and a shared hall. Using ROS 2 and Gazebo, the robot will use LiDAR and wheel odometry with SLAM to map and explore the environment, then navigate between locations using Nav2. We will use a state machine to manage the robot’s goals; this state machine will be dependent on which room it has entered. RViz2 will help visualise the map and manages planned paths. We will model static obstacles in Room 1, and moving obstacles in the shared hall and Room 2, which the robot will detect and avoid by stopping and replanning. For task 1, we will use a vacuum attachment and implement an area coverage algorithm (such as Zigzag). For task 2, we plan to use a parallel jaw gripper.
# Minimum Working Version (MVP)
- There is a hallway (where the robot starts), Room 1 and 2 (modelled as rectangles with no obstacles).

- The robot will use SLAM to map out these rooms, and an RGB camera to the room it is currently in.

- The RGB camera will identify the blue (Room 1) and green rooms, helping the robot recognise which room it is currently in and navigate to the correct task location.

- When in Room 1, the robot will calculate a path to cover and vacuum the whole room.

- When in Room 2, the robot will locate the item and the destination point, pick up the item, move to the destination, and put it down.

- We will use SLAM and Nav2 as mentioned before and not model moving obstacles in the MVP.
# Plan, Responsibilities and Risks
The team will develop the simulation, mapping, and navigation first, followed by cleaning, delivery, and pedestrian avoidance. Responsibilities will cover simulation and SLAM, navigation and localisation (including Markov Localisation), and testing and evaluation, with all members supporting integration. The main risks are integration issues and localisation errors. If motion prediction proves too complex, the robot will use reactive obstacle avoidance.
# Team Allocation
- Tanush Karandikar and Shubham Worlikar: Build the Gazebo environment and configure the robot model using SDF.

- Chris Wilson and Ethan Katz: Select and configure the sensors, including LiDAR and wheel odometry.

- Ruiheng Guo and Baitong Hou: Define evaluation metrics and develop the test scenarios.

Evaluation

We will measure mapping accuracy by comparing the calculated map against the known Gazebo map, calculate localisation error via Absolute Trajectory Error, cleaning coverage, task completion time, and collision frequency. The robot will also be assessed across repeated scenarios with different pedestrian speeds and crossing patterns, and different shapes and sizes of the rooms. The baseline reactive avoidance system will be compared with any implemented improvements, using Gazebo ground truth for evaluation only.
