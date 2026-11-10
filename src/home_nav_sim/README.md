# home_nav_sim

Gazebo Harmonic world for ROS 2 Jazzy (Ubuntu 24.04): a 20 m x 12 m closed
rectangle with a hall, Room 1 **Alpha** (yellow, cleaning task) and Room 2
**Beta** (mint, pick-and-place task), all furniture, task markers and three
walking humans.

## 1. Install (once)

```bash
# ROS 2 Jazzy (skip if already installed): https://docs.ros.org/en/jazzy/Installation.html
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz python3-colcon-common-extensions
```

`ros-jazzy-ros-gz` pulls in Gazebo Harmonic, the matching version for Jazzy.

## 2. Build

```bash
mkdir -p ~/ros2_ws/src
cp -r home_nav_sim ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select home_nav_sim
source install/setup.bash
```

## 3. Run

```bash
ros2 launch home_nav_sim world.launch.py                 # with humans walking
ros2 launch home_nav_sim world.launch.py humans:=false   # no humans (use this for SLAM mapping)
```

The first launch with humans downloads the walking-person mesh from Gazebo
Fuel, so it needs internet once.

For a top view in Gazebo: open the plugin menu (the ⋮ at the top right),
add **View Angle**, and click its top-view button.

## Map coordinates

Origin (0, 0) = bottom-left outer corner, x east, y north, metres.

| Item | Position |
|---|---|
| Door 1 (Alpha) | x 4.0-5.2, y 7.0 |
| Door 2 (Beta) | x 14.8-16.0, y 7.0 |
| Robot spawn | (19.21, 0.79), yaw 1.5708 (facing north) |
| Alpha clean zone | centre (6.79, 9.14), 2.29 x 1.57 m |
| Beta pick item | (12.86, 8.21), 0.15 m cube, 0.2 kg |
| Beta drop spot | (18.98, 9.30) |

## Changing the layout

All furniture lives in `tools/generate_world.py` as simple lists. Edit a
position or size there, then regenerate and rebuild:

```bash
python3 tools/generate_world.py
colcon build --packages-select home_nav_sim
```

Furniture is solid boxes/cylinders from the floor up, so the robot's lidar
sees each piece as one obstacle and can never drive under a table.
