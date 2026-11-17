# Phase D: Room 2 (Beta) Pick and Place (`beta_pick_place.py`)

This document explains how to run, configure, and integrate the **Room 2 (Beta) Pick and Place** task node (`beta_pick_place.py`), as well as how its navigation, perception, kinematics, and Gazebo `DetachableJoint` grasping system work.

---

## 1. Overview & Files Modified

| File | Purpose |
| :--- | :--- |
| `src/home_nav_sim/scripts/beta_pick_place.py` | Main ROS 2 node implementing the Room 2 (Beta) pick-and-place mission (`ros2 run home_nav_sim beta_pick_place.py`). |
| `src/robot_bringup/robot_bringup/beta_pick_place.py` | Console-script entry-point wrapper (`ros2 run robot_bringup beta_pick_place`). |
| `src/robot_bringup/urdf/turtlebot3_manipulation_gz.xacro` | Added `gz-sim-detachable-joint-system` (`DetachableJoint`) plugin connecting `${prefix}link5` (gripper palm) to `beta_pick_item::link`. |
| `src/robot_bringup/config/bridge_params.yaml` | Bridged `/beta_pick_item/attach` and `/beta_pick_item/detach` (`std_msgs/msg/Empty` $\rightarrow$ `gz.msgs.Empty`). |
| `src/robot_bringup/launch/simulation.launch.py` | Publishes an initial `/beta_pick_item/detach` message immediately after `spawn_robot` exits so the cube starts detached during SLAM/exploration. |
| `src/home_nav_sim/CMakeLists.txt` & `src/robot_bringup/setup.py` | Registered `beta_pick_place` executables in both packages. |

---

## 2. How to Build and Run

### Build
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select home_nav_sim robot_bringup
source install/setup.bash
```

### Mode 1: Run on the Saved Map with Nav2 (`nav.launch.py`)
1. **Terminal 1 — Launch Gazebo + Nav2 + RViz on the saved map:**
   ```bash
   source /opt/ros/jazzy/setup.bash && source install/setup.bash
   ros2 launch home_nav_sim nav.launch.py
   ```
   *(Wait ~20–25 seconds until AMCL publishes the initial pose and Nav2 logs `Managed nodes are active`.)*

2. **Terminal 2 — Run `beta_pick_place`:**
   ```bash
   source /opt/ros/jazzy/setup.bash && source install/setup.bash
   ros2 run home_nav_sim beta_pick_place.py
   ```
   *(Or equivalently: `ros2 run robot_bringup beta_pick_place`)*

### Mode 2: Run in the Stripped-Down Practice World (`home_tasks.sdf`, no Nav2)
1. **Terminal 1 — Launch the practice world:**
   ```bash
   source /opt/ros/jazzy/setup.bash && source install/setup.bash
   ros2 launch home_nav_sim robot_in_home.launch.py world:=home_tasks.sdf
   ```
2. **Terminal 2 — Run `beta_pick_place`:**
   ```bash
   source /opt/ros/jazzy/setup.bash && source install/setup.bash
   ros2 run home_nav_sim beta_pick_place.py
   ```

### Mode 3: Integration with `mission_manager.py` (Phase E) or `doorway_navigator.py`
`beta_pick_place.py` supports two integration patterns out of the box:

1. **Subprocess Run-and-Exit (Default — `wait_for_ready:=false`):**
   * `mission_manager.py` can invoke `ros2 run home_nav_sim beta_pick_place.py` (or `ros2 run robot_bringup beta_pick_place`).
   * The node executes the full pick-and-place sequence immediately, publishes `Bool(data=True)` on `/room2/done` (`TRANSIENT_LOCAL` QoS), and exits with **return code `0` on success** or **`1` on failure** (making 1-retry logic in `mission_manager.py` straightforward).
2. **Topic Handoff Mode (`wait_for_ready:=true`):**
   * When launched upfront with:
     ```bash
     ros2 run robot_bringup beta_pick_place --ros-args -p wait_for_ready:=true
     ```
   * The node waits for `Bool(data=True)` on `/room2/ready` (`TRANSIENT_LOCAL` QoS) before starting, then publishes `Bool(data=True)` on `/room2/done` upon completion.

### Configurable ROS 2 Parameters

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `wait_for_ready` | `false` | Wait for `/room2/ready` (`std_msgs/msg/Bool`) before executing. |
| `box_x` | `-6.35` | Target cube $x$ coordinate in `map` / `odom` frame (world `12.86`). |
| `box_y` | `7.42` | Target cube $y$ coordinate in `map` / `odom` frame (world `8.21`). |
| `drop_x` | `-0.23` | Target drop spot $x$ coordinate in `map` / `odom` frame (world `18.98`). |
| `drop_y` | `8.51` | Target drop spot $y$ coordinate in `map` / `odom` frame (world `9.30`). |
| `use_visual_servo` | `true` | Refine final heading toward the orange cube using `/rgbd_camera/image`. |

---

## 3. How It Works Internally

### 3.1 Coordinate System & Localization
* The robot spawns at world `(19.21, 0.79)` facing East (`yaw = 0.0`), so both `/odom` and SLAM/AMCL `/map` share the transform:
  $$\text{map}(x, y) = \text{world}(x, y) - (19.21, 0.79)$$
* `get_pose()` queries TF (`map` $\rightarrow$ `base_footprint`) first and automatically falls back to `/odom` (`odom` $\rightarrow$ `base_footprint`) if SLAM/AMCL is not running.

### 3.2 Dual-Mode Navigation & Static Map Inflation Handling
1. **Entering Room 2 (Beta):**
   * If the robot starts outside Room 2 (e.g., at the hall spawn `(0, 0)` or inside Room 1 Alpha), it routes through the hallway (`(-3.81, 5.20)`) and through Door 2 into Room 2 (`(-3.81, 7.42)`).
   * If already delivered to the Room 2 entrance by `doorway_navigator.py` or `mission_manager.py`, it proceeds directly to the cube.
2. **Pre-Grasp & Static Map Obstacle Clearance:**
   * Because the saved map (`home_map.pgm`) was recorded while `beta_pick_item` sat at `(-6.35, 7.42)`, the cube's initial footprint is baked into the static map with a `0.45 m` inflation radius.
   * `beta_pick_place.py` uses Nav2 (`navigate_to_pose`) to travel to a safe pre-grasp pose just outside that inflation zone (`(-5.75, 7.42)`, `yaw = pi`), then uses closed-loop `/cmd_vel` control (`drive_to_pose_closed_loop`) and front RGB camera visual servoing (`align_with_orange_cube_camera`) to close in to the exact grasp standoff (`0.28 m` from cube center).
   * After picking up the cube, the robot backs out `0.38 m` (`backup_distance`) to clear the static map's inflated obstacle cells before requesting Nav2 to plan the path to the drop spot.

### 3.3 Arm Kinematics, LiDAR Clearance & Gentle Center Placement
1. **`DetachableJoint` Grasping:**
   * At simulation startup, `simulation.launch.py` and `beta_pick_place.py` publish `Empty` to `/beta_pick_item/detach` so the cube remains stationary on the floor during exploration.
   * When the robot reaches the cube, it lowers the arm to `PICK_POSE = [0.0, 0.50, 0.10, -0.60]` (where $q_2 + q_3 + q_4 = 0\text{ rad}$ keeps the wrist `link5` horizontal), waits for `/joint_states` to settle within `0.035 rad` so the arm has zero velocity, closes the gripper (`-0.005`), and publishes `Empty` to `/beta_pick_item/attach`.
2. **High Carry Pose Above the 2D LiDAR Plane (`z = 0.132 m`):**
   * On the TurtleBot3 Waffle Pi, `base_scan` sits at `z = 0.132 m` above the floor.
   * `CARRY_POSE = [0.0, -0.85, -0.15, 0.65]` raises `link5` to `z = 0.384 m` and tilts the gripper upward so the bottom of the `0.15 m` cube sits at `z ≈ 0.23 m` — ~`10 cm` above the horizontal LiDAR scan plane. This prevents `/scan`, Nav2's costmaps, and `collision_monitor` from detecting the carried cube as an obstacle in front of the robot.
3. **Held-Offset Compensation & Zero-Velocity Release at the Drop Spot:**
   * At the moment of attachment, the node records the cube's exact local offset $(\Delta x_{\text{local}}, \Delta y_{\text{local}})$ relative to `base_footprint`.
   * When positioning at the drop spot `(-0.23, 8.51)`, it stops at `(drop_x - dx_local, drop_y - dy_local)` so the held cube center aligns directly over `(-0.23, 8.51)`.
   * The arm then lowers smoothly to `PLACE_POSE = [0.0, 0.50, 0.10, -0.60]`, waits for `/joint_states` to come to a complete stop on the floor so there is no downward velocity or bounce, detaches via `/beta_pick_item/detach`, opens the gripper (`0.018`), folds the arm back to `HOME_POSE`, and reverses `0.25 m` clear of the placed cube.
