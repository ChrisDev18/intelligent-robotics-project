# Phase D: Room Beta Pick and Place (`beta_pick_place.py`)

This document explains how to run, configure, and integrate the **Room Beta Pick and Place** task node (`beta_pick_place.py`), as well as how its navigation, perception, kinematics, and Gazebo `DetachableJoint` grasping system work.

---

## 1. Overview & Files Modified

| File | Purpose |
| :--- | :--- |
| `src/robot_bringup/robot_bringup/beta_pick_place.py` | Main ROS 2 node implementing the Room Beta pick-and-place mission (`ros2 run robot_bringup beta_pick_place`). |
| `src/robot_bringup/urdf/turtlebot3_manipulation_gz.xacro` | Added `gz-sim-detachable-joint-system` (`DetachableJoint`) plugin connecting `${prefix}link5` (gripper palm) to `beta_pick_item::link`. |
| `src/robot_bringup/config/bridge_params.yaml` | Bridged `/beta_pick_item/attach`, `/beta_pick_item/detach`, and read-only ground truth `/eval/gz_dynamic_poses` (`/world/empty/dynamic_pose/info`). |
| `src/robot_bringup/tools/summarize_beta_eval.py` | Standalone CLI to aggregate pickup/placement distance error CSV logs into separate pick and place success rates. |
| `src/robot_bringup/launch/simulation.launch.py` | Publishes an initial `/beta_pick_item/detach` message immediately after `spawn_robot` exits so the cube starts detached during SLAM/exploration. |
| `src/robot_bringup/setup.py` | Registered the `beta_pick_place` console script entry point. |

---

## 2. How to Build and Run

### Build
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_bringup
source install/setup.bash
```

### Mode 1: Run on the Saved Map with Nav2 (`nav.launch.py`)
1. **Terminal 1 — Launch Gazebo + Nav2 + RViz on the saved map:**
   ```bash
   source /opt/ros/jazzy/setup.bash && source install/setup.bash
   ros2 launch robot_bringup nav.launch.py
   ```
   *(Wait ~20–25 seconds until AMCL publishes the initial pose and Nav2 logs `Managed nodes are active`.)*

2. **Terminal 2 — Run `beta_pick_place`:**
   ```bash
   source /opt/ros/jazzy/setup.bash && source install/setup.bash
   ros2 run robot_bringup beta_pick_place
   ```

### Mode 2: Run in the Stripped-Down Practice World (`home_tasks.sdf`, no Nav2)
1. **Terminal 1 — Launch the practice world:**
   ```bash
   source /opt/ros/jazzy/setup.bash && source install/setup.bash
   ros2 launch robot_bringup robot_in_home.launch.py world:=home_tasks.sdf
   ```
2. **Terminal 2 — Run `beta_pick_place`:**
   ```bash
   source /opt/ros/jazzy/setup.bash && source install/setup.bash
   ros2 run robot_bringup beta_pick_place
   ```

### Mode 3: Integration with `mission_manager.py` (Phase E) or `doorway_navigator.py`
`beta_pick_place.py` supports two integration patterns out of the box:

1. **Subprocess Run-and-Exit (Default — `wait_for_ready:=false`):**
   * `mission_manager.py` can invoke `ros2 run robot_bringup beta_pick_place`.
   * The node executes the full pick-and-place sequence immediately, publishes `Bool(data=True)` on `/room_beta/done` (`TRANSIENT_LOCAL` QoS), and exits with **return code `0` on success** or **`1` on failure** (making 1-retry logic in `mission_supervisor.py` straightforward).
2. **Topic Handoff Mode (`wait_for_ready:=true`):**
   * When launched upfront with:
     ```bash
     ros2 run robot_bringup beta_pick_place --ros-args -p wait_for_ready:=true
     ```
   * The node waits for `Bool(data=True)` on `/room_beta/ready` (`TRANSIENT_LOCAL` QoS) before starting, then publishes `Bool(data=True)` on `/room_beta/done` upon completion.

### Configurable ROS 2 Parameters

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `wait_for_ready` | `false` | Wait for `/room_beta/ready` (`std_msgs/msg/Bool`) before executing. |
| `box_x` | `-6.35` | Target cube $x$ coordinate in `map` / `odom` frame (world `12.86`). |
| `box_y` | `7.42` | Target cube $y$ coordinate in `map` / `odom` frame (world `8.21`). |
| `drop_x` | `-0.23` | Target drop spot $x$ coordinate in `map` / `odom` frame (world `18.98`). |
| `drop_y` | `8.51` | Target drop spot $y$ coordinate in `map` / `odom` frame (world `9.30`). |
| `use_visual_servo` | `true` | Refine final heading toward the orange cube using `/rgbd_camera/image`. |
| `eval_log_path` | `log/beta_pick_place_eval.csv` | CSV file where per-trial pickup & placement distance errors and success flags are appended. |
| `scenario` | `""` | Optional scenario tag saved in each CSV row (e.g. `nav_no_humans`, `nav_humans`, `practice_world`). |
| `spawn_x` / `spawn_y` | `19.21` / `0.79` | Robot world spawn coordinates used to map Gazebo ground-truth poses into `map` coordinates for evaluation. |
| `pick_error_threshold_m` | `0.15` | Max 2D pickup distance error (metres) for `pick_success = True`. |
| `pick_lift_threshold_m` | `0.08` | Min cube vertical lift $\Delta z$ (metres, ground truth) after `CARRY_POSE` for `pick_success = True`. |
| `place_error_threshold_m` | `0.27` | Max 2D placement distance error (metres) from `(drop_x, drop_y)` for `place_success = True` (half-width of the `0.54 m` drop zone). |
| `place_max_z_m` | `0.15` | Max final cube centre height $z$ (metres, ground truth) after retract to verify release onto the floor. |

---

## 3. How It Works Internally

### 3.1 Coordinate System & Localization
* The robot spawns at world `(19.21, 0.79)` facing East (`yaw = 0.0`), so both `/odom` and SLAM/AMCL `/map` share the transform:
  $$\text{map}(x, y) = \text{world}(x, y) - (19.21, 0.79)$$
* `get_pose()` queries TF (`map` $\rightarrow$ `base_footprint`) first and automatically falls back to `/odom` (`odom` $\rightarrow$ `base_footprint`) if SLAM/AMCL is not running.

### 3.2 Dual-Mode Navigation & Static Map Inflation Handling
1. **Entering Room Beta:**
   * If the robot starts outside Room Beta (e.g., at the hall spawn `(0, 0)` or inside Room Alpha), it routes through the hallway (`(-3.81, 5.20)`) and through Door Beta into Room Beta (`(-3.81, 7.42)`).
   * If already delivered to the Room Beta entrance by `doorway_navigator.py` or `mission_supervisor.py`, it proceeds directly to the cube.
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

---

## 4. Evaluation: Pickup & Placement Distance Error Logging

Every run of `beta_pick_place.py` logs **separate pickup and placement distance errors** and **separate pick and place success outcomes**, using Gazebo ground truth (`/eval/gz_dynamic_poses`, read-only and never used by navigation or control) with automatic fallback to ROS TF/odom estimates if the ground-truth bridge is unavailable.

### 4.1 Metrics Recorded Per Trial
* **Pickup Distance Error (`pickup_distance_error_m`)**: 2D Euclidean distance between the robot's nominal grasp point `GRASP_STANDOFF_M = 0.28 m` ahead of `base_footprint` and the cube's initial centre at grasp time.
  * `pick_success`: `pickup_distance_error_m <= pick_error_threshold_m` (default `0.15 m`) **and** (when ground truth is available) `cube_lift_delta_z >= pick_lift_threshold_m` (default `0.08 m`).
* **Placement Distance Error (`placement_distance_error_m`)**: 2D Euclidean distance between the cube's final resting centre (measured *after* retracting the arm and backing clear) and the target drop spot `(drop_x, drop_y)`.
  * `place_success`: `placement_distance_error_m <= place_error_threshold_m` (default `0.27 m`, matching the `0.54 m × 0.54 m` drop zone half-width) **and** (when ground truth is available) `cube_final_z <= place_max_z_m` (default `0.15 m`).
* **Overall Mission Success (`mission_success`)**: `pick_success and place_success` (also controls the process exit code `0` vs `1`).
* **ROS Estimate Comparison**: `pickup_estimated_error_m` and `placement_estimated_error_m` are logged alongside the ground-truth errors so localization drift can be separated from controller accuracy.
* **Live ROS 2 Topic**: Each trial's full JSON metrics dictionary is published on `/beta_pick_place/eval_metrics` (`std_msgs/msg/String`, `TRANSIENT_LOCAL` QoS).

### 4.2 Running Tagged Trials & Summarising Results
Run trials with an optional `scenario` tag (rows append automatically to `log/beta_pick_place_eval.csv`):
```bash
ros2 run robot_bringup beta_pick_place --ros-args -p scenario:=nav_no_humans
ros2 run robot_bringup beta_pick_place --ros-args -p scenario:=nav_humans
```

At the end of every run, the node prints the current trial's errors plus the cumulative **Pick Success Rate**, **Place Success Rate**, **Place | Pick Rate**, and **Overall Mission Success Rate**.

To inspect or re-analyse any CSV log offline (including grouping by scenario or testing stricter thresholds):
```bash
python3 src/robot_bringup/tools/summarize_beta_eval.py log/beta_pick_place_eval.csv
python3 src/robot_bringup/tools/summarize_beta_eval.py log/beta_pick_place_eval.csv --by-scenario
python3 src/robot_bringup/tools/summarize_beta_eval.py log/beta_pick_place_eval.csv --pick-thresh 0.10 --place-thresh 0.20
```
