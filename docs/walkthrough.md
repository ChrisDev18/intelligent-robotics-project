# Walkthrough: Unified `robot_bringup` Package (`ament_python`)

## Summary of Changes

All project code, launch files, configs, URDF/Xacro files, Gazebo worlds, saved SLAM maps, RViz configs, and tools have been consolidated from the split `home_nav_sim` (`ament_cmake`) + `robot_bringup` (`ament_python`) layout into a single **`robot_bringup`** `ament_python` package generated from the standard ROS 2 Jazzy `ros2 pkg create` template. `src/turtlebot3_manipulation/` (external gitignored ROBOTIS repo) remains untouched.

### Key Changes Made
1. **Regenerated `robot_bringup` from `ros2 pkg create --build-type ament_python`** with all **44 audited dependencies** actually used across both packages in [package.xml](file:///home/ethan/robotics_project/intelligent-robotics-project/src/robot_bringup/package.xml):
   - Core/launch/Python: `rclpy`, `ament_index_python`, `launch`, `launch_ros`, `python3-numpy`
   - Interfaces/TF: `builtin_interfaces`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `rosgraph_msgs`, `tf2_msgs`, `tf2_ros`, `trajectory_msgs`, `control_msgs`, `nav2_msgs`, `ros_gz_interfaces`
   - Robot/Gazebo/Control: `turtlebot3_manipulation_description`, `xacro`, `robot_state_publisher`, `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`, `controller_manager`, `ros2_controllers`, `joint_state_broadcaster`, `joint_trajectory_controller`, `gripper_controllers`
   - SLAM/Nav2/Viz: `slam_toolbox`, `nav2_bringup`, `nav2_common`, `nav2_simple_commander`, `nav2_controller`, `nav2_smoother`, `nav2_planner`, `nav2_behaviors`, `nav2_velocity_smoother`, `nav2_collision_monitor`, `nav2_bt_navigator`, `nav2_waypoint_follower`, `nav2_lifecycle_manager`, `nav2_map_server`, `rviz2`, `teleop_twist_keyboard`
2. **Eliminated circular dependencies and wrapper shims:**
   - Replaced the `importlib` shims in [alpha_cleaning.py](file:///home/ethan/robotics_project/intelligent-robotics-project/src/robot_bringup/robot_bringup/alpha_cleaning.py) and [beta_pick_place.py](file:///home/ethan/robotics_project/intelligent-robotics-project/src/robot_bringup/robot_bringup/beta_pick_place.py) with the full node implementations.
   - Moved [frontier_explorer.py](file:///home/ethan/robotics_project/intelligent-robotics-project/src/robot_bringup/robot_bringup/frontier_explorer.py) and [save_map.py](file:///home/ethan/robotics_project/intelligent-robotics-project/src/robot_bringup/robot_bringup/save_map.py) into `src/robot_bringup/robot_bringup/` and registered all 8 ROS 2 nodes in [setup.py](file:///home/ethan/robotics_project/intelligent-robotics-project/src/robot_bringup/setup.py) `console_scripts` (`alpha_cleaning`, `beta_pick_place`, `doorway_navigator`, `frontier_explorer`, `save_map`, `task_manager`, `test_robot`, `vacuum_demo`).
3. **Consolidated all launch files, configs, worlds, maps, RViz configs, and offline tools:**
   - Updated [setup.py](file:///home/ethan/robotics_project/intelligent-robotics-project/src/robot_bringup/setup.py) `data_files` to install `launch/*.py`, `config/*.yaml`, `urdf/*.xacro`, `worlds/*.sdf`, `maps/*.yaml` + `maps/*.pgm`, and `rviz/*.rviz`.
   - Migrated and updated [world.launch.py](file:///home/ethan/robotics_project/intelligent-robotics-project/src/robot_bringup/launch/world.launch.py), [robot_in_home.launch.py](file:///home/ethan/robotics_project/intelligent-robotics-project/src/robot_bringup/launch/robot_in_home.launch.py), [slam.launch.py](file:///home/ethan/robotics_project/intelligent-robotics-project/src/robot_bringup/launch/slam.launch.py), [nav.launch.py](file:///home/ethan/robotics_project/intelligent-robotics-project/src/robot_bringup/launch/nav.launch.py), and [explore.launch.py](file:///home/ethan/robotics_project/intelligent-robotics-project/src/robot_bringup/launch/explore.launch.py) (including updating `explore.launch.py` to launch `package='robot_bringup', executable='frontier_explorer'`).
   - Moved offline non-ROS CLI scripts (`generate_world.py`, `make_task_world.py`, `summarize_alpha_eval.py`, `summarize_beta_eval.py`) into `src/robot_bringup/tools/`.
   - Removed `src/home_nav_sim/`, `build/home_nav_sim/`, and `install/home_nav_sim/`.
4. **Updated all documentation:**
   - Updated [README.md](file:///home/ethan/robotics_project/intelligent-robotics-project/README.md), [BETA_PICK_PLACE.md](file:///home/ethan/robotics_project/intelligent-robotics-project/docs/BETA_PICK_PLACE.md), and [TASK_WORLD.md](file:///home/ethan/robotics_project/intelligent-robotics-project/docs/TASK_WORLD.md) to reflect the single `robot_bringup` package and updated commands.

---

## Validation Results

| Verification Check | Result |
|---|---|
| Clean rebuild (`colcon build --symlink-install --packages-select robot_bringup`) | **PASS** — finished cleanly in `2.58s` |
| Package index (`ros2 pkg list` / `ros2 pkg executables robot_bringup`) | **PASS** — `robot_bringup` indexed with all 8 executables; `home_nav_sim` absent |
| Stale reference scan (`grep -rn "home_nav_sim" src/robot_bringup/ docs/ README.md`) | **PASS** — zero stale references |
| World generation (`generate_world.py` & `make_task_world.py`) & SDF XML check | **PASS** — `home.sdf` (41 models + 3 walking humans: `human_alpha`, `human_beta`, `human_hall`), `home_no_humans.sdf` (41 models, 0 actors), `home_tasks.sdf` (13 models, 0 actors); installed symlinks verified |
| URDF/Xacro compilation (`turtlebot3_manipulation.urdf.xacro`) | **PASS** — valid URDF (`23` links, `22` joints) |
| Launch file instantiation (`world`, `simulation`, `robot_in_home`, `slam`, `nav`, `explore`) | **PASS** — all 6 return valid `LaunchDescription`s with resolved paths |
| Python module imports & unit checks (`frontier_explorer`, `doorway_navigator`, `save_map`) | **PASS** — all 8 modules expose `main()`; frontier detection, RGB floor classification, and PGM map serialization checks passed |
| Live headless smoke test (`explore.launch.py humans:=false gui:=false rviz:=false return_home:=false` + `doorway_navigator`) | **PASS** — Gazebo Harmonic, `ros2_control` (`joint_state_broadcaster`, `arm_controller`, `gripper_controller`), `slam_toolbox`, all 8 Nav2 servers (`Managed nodes are active`), `frontier_explorer` (`Map, Nav2 and TF ready`), and `doorway_navigator` (live RGB floor monitoring + mission flag topics `/exploration_done`, `/room1/ready`, `/room2/ready`) booted cleanly |

---

## Quick Reference Commands

```bash
# 1. Regenerate worlds (furniture/obstacles, humans, and task practice world)
python3 src/robot_bringup/tools/generate_world.py
python3 src/robot_bringup/tools/make_task_world.py

# 2. Build & source
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select turtlebot3_manipulation_description robot_bringup
source install/setup.bash

# 3. Launch autonomous SLAM exploration + Nav2 + mission handoff flags
ros2 launch robot_bringup explore.launch.py humans:=false return_home:=false
ros2 run robot_bringup doorway_navigator --ros-args -p wait_for_teammates:=true
ros2 run robot_bringup alpha_cleaning --ros-args -p wait_for_ready:=true
ros2 run robot_bringup beta_pick_place --ros-args -p wait_for_ready:=true

# Save live SLAM map / run saved-map Nav2 navigation
ros2 run robot_bringup save_map src/robot_bringup/maps/explored_map
ros2 launch robot_bringup nav.launch.py humans:=true
```
