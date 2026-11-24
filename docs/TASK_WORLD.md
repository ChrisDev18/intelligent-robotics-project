# Task practice world: setup from scratch

A stripped-down copy of the home world for developing the **Alpha cleaning**
and **Beta pick-and-place** tasks. Same walls, doorways and coloured floors as
the full world, but **no furniture, no plants, no walking humans** — only the
pick-and-place cube in Beta. Nothing to collide with while you work on your task.

| | |
|---|---|
| World file | `src/robot_bringup/worlds/home_tasks.sdf` |
| Made by | `src/robot_bringup/tools/make_task_world.py` (from `home_no_humans.sdf`) |
| Opened with | `ros2 launch robot_bringup robot_in_home.launch.py world:=home_tasks.sdf` |

Everything below assumes **Ubuntu 24.04** (native or WSL2 on Windows). Run each
block in order; wait for one to finish before starting the next.

---

## 1. Install ROS 2 Jazzy and Gazebo Harmonic (once per machine)

Skip this section if `ls /opt/ros` already shows `jazzy`.

```bash
lsb_release -a          # must say 24.04 / noble
sudo apt update && sudo apt install -y software-properties-common curl unzip git
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz python3-colcon-common-extensions
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Install the robot's dependencies (once per machine)

```bash
sudo apt install -y ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-dynamixel-sdk ros-jazzy-turtlebot3-msgs \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-teleop-twist-keyboard
```

## 3. Get the code

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ChrisDev18/intelligent-robotics-project.git
cd intelligent-robotics-project
git checkout task-practice-world
# ROBOTIS robot description (meshes and macros; not available from apt on Jazzy)
cd src && git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
```

Already have the repo? Instead run:

```bash
cd ~/ros2_ws/src/intelligent-robotics-project
git fetch && git checkout task-practice-world && git pull
```

Then **start your own branch** for your task so you never work on `main`:

```bash
git checkout -b alpha-cleaning        # Chris
git checkout -b beta-pick-place       # Ethan
```

## 4. Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The first build takes a few minutes. The summary must say every package finished.

## 5. Open the world with the robot

Close anything left over from earlier runs first (two simulations at once make
ROS time jump backwards and nothing works):

```bash
pkill -9 -f "/opt/ros/jazzy"; pkill -9 -f "ros2_ws/install"; pkill -9 -f "gz sim"
ros2 daemon stop
```

Then, in **one** terminal:

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch robot_bringup robot_in_home.launch.py world:=home_tasks.sdf
```

The Gazebo server starts first; the Gazebo window opens about 12 seconds later.
The robot (black TurtleBot3 with the arm) appears in the bottom-right corner of
the hall, facing east, with the arm and gripper controllers loaded.

Add `start_rviz:=true` to open RViz as well.

## 6. Check it works: drive the robot

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.3
```

Click inside this terminal, then `i` forward, `,` back, `j`/`l` turn, `k` stop.

Useful checks:

```bash
ros2 topic info /clock --no-daemon     # Publisher count: 1  (2+ = an old simulation is still running)
ros2 topic hz /scan                     # lidar, ~15 Hz
ros2 topic echo /odom --once --field pose.pose.position
ros2 control list_controllers           # arm_controller, gripper_controller, joint_state_broadcaster active
```

---

## Key positions

The robot's `odom` frame (and any SLAM `map` frame) starts at the spawn corner,
facing east, so **map/odom = world − (19.21, 0.79)**.

| Thing | World (x, y) m | Map / odom (x, y) m |
|---|---|---|
| Robot spawn (faces east) | (19.21, 0.79) | (0.00, 0.00) |
| Door 1 → Alpha (gap x 4.0–5.2 at y = 7.0) | (4.60, 7.00) | (−14.61, 6.21) |
| Door 2 → Beta (gap x 14.8–16.0 at y = 7.0) | (15.40, 7.00) | (−3.81, 6.21) |
| **Alpha cleaning patch** (not drawn in this world) | x 5.65–7.94, y 8.36–9.93 | x −13.56 to −11.27, y 7.57 to 9.14 |
| **Beta cube** (0.15 m, 0.2 kg, model `beta_pick_item`) | (12.86, 8.21) | (−6.35, 7.42) |
| **Beta drop spot** (not drawn in this world) | (18.98, 9.30) | (−0.23, 8.51) |

Rooms: Alpha = yellow floor (x 0–10, y 7–12), Beta = mint floor (x 10–20, y 7–12),
hall = lavender floor (y 0–7).

## Topics you'll use

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | drive the base |
| `/odom` | `nav_msgs/Odometry` | wheel odometry (`odom` → `base_footprint`) |
| `/scan` | `sensor_msgs/LaserScan` | 720 points, 12 m |
| `/rgbd_camera/image`, `/depth_image`, `/points` | camera | 320 × 240 @ 5 Hz |
| `/arm_controller/follow_joint_trajectory` | action | joints `joint1`–`joint4` |
| `/gripper_controller/gripper_cmd` | action | `GripperCommand` |
| `/world/empty/create`, `/world/empty/remove` | services | spawn / delete entities (dirt for cleaning) |

## Troubleshooting

- **`Detected jump back in time` / Nav2 never ready** → two simulations are running. Ctrl+C everything, run the cleanup in step 5, check `/clock` has 1 publisher.
- **Gazebo window black or very slow** → `glxinfo -B | grep renderer` shows `llvmpipe` = software rendering. Update the Windows GPU driver, `wsl --update`, `wsl --shutdown`.
- **Added a new launch/world file but `file not found`** → `colcon build --symlink-install --packages-select robot_bringup`.
- **Changed the full worlds with `src/robot_bringup/tools/generate_world.py`** → re-run `python3 src/robot_bringup/tools/make_task_world.py` afterwards so this world matches.
