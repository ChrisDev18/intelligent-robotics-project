# Troubleshooting & Performance Tips

### 1. Simulation Feels Slow on Integrated Graphics / WSL2:

* Because the robot runs a 3D GPU LiDAR and an RGB-D depth camera via Gazebo's Ogre2 rendering engine, laptops with integrated graphics in WSL2 typically run Gazebo at a **Real-Time Factor (RTF) of ~25%–40%** (1 simulated second takes ~2.5–4 real seconds).
  
* This is completely normal and does **not** break ROS 2 or controller accuracy—both `test_robot` and `vacuum_demo` have been tuned to work reliably at lower RTFs.


### 2. `[NO DATA YET]` on Sensors or `/clock` Warning on Launch:

* If a previous Gazebo terminal was closed without shutting down cleanly, an orphaned `gz sim server` process may still be running in the background and blocking `/clock`.

* Run this command to clean up any stuck background simulation processes before relaunching:
```bash
pkill -9 -f "gz sim" && pkill -9 -f "parameter_bridge"
```


### 3. Simulation won't run on Virtual Machine

If you are running this project on a Mac using UTM to run Ubuntu, Ogre2 won't be supported. To fall back to Ogre which is supported:

1. Open [turtlebot3_manipulation_gz.xacro](../src/robot_bringup/urdf/turtlebot3_manipulation_gz.xacro), 
   and under _Gazebo Harmonic System Plugins_, change
   ```xml
   <render_engine>ogre2</render_engine>`
   ```
   to
   ```xml
   <render_engine>ogre</render_engine>`
   ```