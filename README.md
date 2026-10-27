# Sesame Quadruped: Autonomous Path Planning & Interactive Locomotion (RPP-2026)

An end-to-end autonomous robotics project featuring dynamic A* path planning, real-time obstacle avoidance, top-down computer vision perception, and physics-grounded quadruped locomotion for the **Sesame Mini Quadruped Robot** in ROS 2 Humble and Gazebo Classic.

---

## Key Features

1. **Autonomous Dynamic Path Planning (Auto-Waypoint Generation)**
   * **Dynamic A\* Grid Planner:** Solves optimal collision-free trajectories across the $1.5\text{ m} \times 1.0\text{ m}$ chart paper arena.
   * **String-Pulling Shortcut Algorithm:** Automatically extracts clean geometric waypoints without hardcoded paths.
   * **Euclidean Minkowski-Sum Inflation:** Exact rounded-box obstacle dilation eliminates false corner collision deadlocks.

2. **Interactive Live Gazebo Environment**
   * **5 Draggable Obstacle Boxes:** Click and drag any of the 5 boxes in Gazebo at runtime.
   * **Real-Time Obstacle Avoidance:** If an obstacle is placed directly in the robot's forward path, the robot halts, holds its stance, computes a new collision-free detour, and resumes navigation.
   * **Green Start Marker:** Dynamically tracks directly below the robot wherever it walks or is placed.
   * **Red Goal Marker:** Interactive target disk. Drag it anywhere on the chart paper, and the robot immediately detects the new target and navigates to it.

3. **Physics-Grounded Locomotion & Actuation**
   * **Central Pattern Generator (CPG) Gait:** Ported from [`sesame-ml`](https://github.com/lukehollis/sesame-ml) with smooth cycloidal foot swing ($\max(\sin\phi, 0)^2$) for zero-impact touchdown.
   * **Closed-Loop Attitude Stabilization:** Reads live 50 Hz IMU telemetry (`/sesame/imu`) and injects pitch/roll feedback into hip servos to actively prevent tipping.
   * **TowerPro MG90S Actuator Dynamics:** Enforces a physical slew-rate ceiling ($10.47\text{ rad/s}$ / $600^\circ/\text{s}$), torque saturation ($0.18\text{ N}\cdot\text{m}$), and first-order motor lag ($\tau = 35\text{ ms}$).
   * **Realistic Locomotion Pacing:** Nominal forward walking speed of $\approx 3.8\text{ cm/s}$ and turning speed of $\approx 16^\circ/\text{s}$.

4. **Multi-Source Overhead Perception**
   * Top-down overhead camera mounted at $z = 2.0\text{ m}$ streaming at 30 Hz on `/overhead_camera/image_raw`.
   * Real-time OpenCV contour detection annotating bounding boxes and the active navigation trajectory on `/overhead_camera/annotated_image`.
   * Standard ROS 2 topic broadcasts: `/map` (`OccupancyGrid`), `/plan` (`Path`), `/waypoints_markers` (`MarkerArray`), and `/odom`.

---

## Repository Structure

```text
RPP-Project-2026/
├── src/
│   └── sesame_description/            # ROS 2 Simulation & Description Package
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── launch/
│       │   ├── gazebo.launch.py       # Gazebo world + robot + overhead camera launch
│       │   └── view_robot.launch.py   # RViz URDF visualizer
│       ├── worlds/
│       │   └── chartpaper_arena.world # Interactive arena with 5 draggable boxes & markers
│       ├── urdf/
│       │   ├── sesame_gazebo.urdf     # Calibrated production URDF with IMU & trajectory plugins
│       │   ├── sesame_ros2.urdf
│       │   └── sesame.urdf
│       └── meshes/                    # Production 3D STL meshes for 8 links & covers
├── scripts/
│   ├── navigate_arena.py              # Interactive real-time path planner & CPG controller
│   ├── sesame_path_planner.py         # Standalone A* path search module
│   ├── sesame_kinematics.py           # 2-DOF leg inverse & forward kinematics
│   ├── cpg.py                         # Central Pattern Generator oscillator baseline
│   ├── constants.py                   # Canonical Sesame servo limits and conventions
│   ├── test_gait.py                   # Locomotion & CPG validation test suite
│   └── validate_urdf.py               # URDF joint limit and kinematic validator
├── firmware/                          # Physical Robot ESP32 Firmware
│   ├── sesame-firmware-main.ino       # Core Arduino/ESP32 locomotion & Web server
│   ├── movement-sequences.h           # Keyframe gait definitions
│   ├── captive-portal.h               # Wi-Fi Access Point configuration
│   ├── face-bitmaps.h                 # OLED facial animations
│   └── quadruped_calibration.csv      # Servo angle zero-offset calibration
└── docs/
    ├── path_planning_result.png       # Generated A* trajectory plot
    ├── urdf_docs.md                   # Joint order & coordinate frame conventions
    └── sesame-angle-guide.png         # Physical servo horn alignment guide
```

---

## Getting Started

### Prerequisites
* **Operating System:** Ubuntu 22.04 LTS (Native or WSL2)
* **ROS 2:** Humble Hawksbill
* **Gazebo:** Gazebo Classic 11
* **Python Libraries:**
  ```bash
  pip install numpy opencv-python
  ```

### 1. Build the ROS 2 Simulation Package
Symlink or copy `src/sesame_description` to your ROS 2 workspace:
```bash
mkdir -p ~/ros2_ws/src
ln -s "$(pwd)/src/sesame_description" ~/ros2_ws/src/sesame_description
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sesame_description
```

### 2. Launch Gazebo Simulation
In Terminal 1:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch sesame_description gazebo.launch.py
```
This loads the white chart paper arena with Sesame, the 5 interactive obstacle boxes, the green start marker, the red goal marker, and the overhead camera.

### 3. Run the Interactive Autonomous Navigator
In Terminal 2:
```bash
source ~/ros2_ws/install/setup.bash
python3 scripts/navigate_arena.py
```

---

## Interactive Controls

* **Move the Goal:** Click on the **Red Circle** (`goal_marker`) in Gazebo, press `T` (Translate tool), and drag it anywhere on the chart paper. Sesame immediately senses the new target and re-plans its trajectory.
* **Block the Robot:** Click on any of the **5 black boxes** (`obstacle_box_1` to `5`), press `T`, and drag it directly in front of Sesame. The robot halts, holds its stand stance, re-plans a new collision-free detour, and resumes moving.
* **Green Circle:** Always tracks dynamically directly underneath the robot.
* **Overhead View:** View the live perception feed with annotated bounding boxes and planned trajectory:
  ```bash
  ros2 run rqt_image_view rqt_image_view /overhead_camera/annotated_image
  ```

---

## Author
* **Prithvi Bhende** ([@PrithviBhende](https://github.com/PrithviBhende))
