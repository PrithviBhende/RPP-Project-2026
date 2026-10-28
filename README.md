# Sesame Quadruped: Autonomous Vision-Guided Path Planning & Digital Twin (RPP-2026)

An end-to-end autonomous robotics project featuring real-time overhead computer vision tracking (ArUco planar homography), dynamic A* path planning with strict $\ge 5\text{ cm}$ obstacle clearance, Stanley steering path-following control, human hand safety gating, ESP32 Wi-Fi hardware-in-the-loop actuation, and a live ROS 2 Humble RViz 3D Digital Twin.

---

## Architecture Overview

```
          Overhead Camera (USB DroidCam / IP Webcam / Phone)
                                 |
                                 v
               +-----------------------------------+
               |       vision_tracker.py           |
               | - Multi-version ArUco Detector    |
               | - 4-Corner Planar Homography      |
               | - Real-time Hand Detection (HSV)  |
               | - Metric Pose Extraction (cm, deg)|
               +-----------------+-----------------+
                                 |
                                 v
               +-----------------------------------+
               |       vision_planner.py           |
               | - Dynamic A* Search on Grid       |
               | - Strict >= 5 cm Box Clearance    |
               | - String-Pulling Waypoint Smoother|
               | - Hand-Gated Auto-Replanning      |
               +-----------------+-----------------+
                                 |
                                 v
               +-----------------------------------+
               |     stanley_controller.py         |
               | - Cross-Track Error (CTE) Law     |
               | - Heading Error Correction        |
               | - Deadbands (< 12 deg, < 2 cm)    |
               +--------+-----------------+--------+
                        |                 |
       (Wi-Fi /cmd?go=) |                 | (ROS 2 /joint_states & TF)
                        v                 v
            +-----------------------+ +-----------------------+
            |  Physical ESP32 Robot | |  ROS 2 RViz Twin      |
            | - 8x MG90S Servos     | | - Fixed Frame: map    |
            | - WROOM-32 Pinout     | | - 3D Robot Model      |
            | - Stand on Arrival    | | - Green /plan_path    |
            | - Auto-Halt on Hand   | | - 3D Box Markers      |
            +-----------------------+ +-----------------------+
```

---

## ArUco Marker ID Layout & Coordinates

Using standard high-contrast **`cv2.aruco.DICT_4X4_50`** markers:

| Marker ID | Function | Position | Real Size Scale |
| :---: | :--- | :--- | :---: |
| **0** | **Arena Origin $(0, 0)$** | Bottom-Left Corner on floor | $6.0 \times 6.0\text{ cm}$ |
| **1** | **Arena Corner BR** | Bottom-Right Corner on floor | $6.0 \times 6.0\text{ cm}$ |
| **2** | **Arena Corner TR** | Top-Right Corner on floor | $6.0 \times 6.0\text{ cm}$ |
| **3** | **Arena Corner TL** | Top-Left Corner on floor | $6.0 \times 6.0\text{ cm}$ |
| **4** | **Robot Base Tracker** | Mounted flat on top of Sesame body *(arrow points to head)* | **$4.0 \times 4.0\text{ cm}$** |
| **5** | **Goal Target** | Placed anywhere as destination | $6.0 \times 6.0\text{ cm}$ |
| **10 – 14** | **Obstacle Blocks** | Centered on top of each box/block | $5.0 \times 5.0\text{ cm}$ |

> **Printable Sheets:** Ready-to-print A4 sheets are located in the [`markers/`](markers/) directory:
> - [`markers/print_sheet_1_arena_corners.png`](markers/print_sheet_1_arena_corners.png) (Corners 0 to 3)
> - [`markers/print_sheet_2_robot_goal_obstacles.png`](markers/print_sheet_2_robot_goal_obstacles.png) (Robot ID 4, Goal ID 5, Obstacles 10 & 11)
> - [`markers/print_sheet_3_more_obstacles.png`](markers/print_sheet_3_more_obstacles.png) (Obstacles 12, 13, 14)

---

## Repository Structure

```text
RPP-Project-2026/
├── markers/                           # 1-Click Printable ArUco Markers
│   ├── print_sheet_1_arena_corners.png
│   ├── print_sheet_2_robot_goal_obstacles.png
│   ├── print_sheet_3_more_obstacles.png
│   └── *.png                          # Individual marker cards with crosshairs
├── scripts/
│   ├── main_vision_navigator.py       # Master autonomous navigation node + HUD
│   ├── vision_tracker.py              # Overhead ArUco tracking + homography + hand mask
│   ├── vision_planner.py              # Dynamic A* planner with >= 5 cm clearance
│   ├── stanley_controller.py          # Stanley steering law + ESP32 Wi-Fi bridge
│   ├── test_camera_feed.py            # Live camera & marker diagnostic tool
│   ├── generate_markers.py            # ArUco marker generator script
│   └── test_esp32_rviz_walk.py        # Hardware-in-the-loop walk test bridge
├── firmware/
│   └── quadruped_mini/                # Exact Calibrated ESP32 WROOM-32 Firmware
│       ├── quadruped_mini.ino         # Arduino sketch (SoftAP: Quadruped-Controller)
│       ├── movement-sequences.h       # Calibrated rest, stand, and trot gaits
│       ├── captive-portal.h           # Web dashboard
│       └── face-bitmaps.h             # OLED screen expressions
├── src/sesame_description/            # ROS 2 Humble Digital Twin Package
│   ├── config/
│   │   ├── vision_nav.rviz            # RViz config (Fixed Frame: map, path, 3D boxes)
│   │   └── walk_test.rviz
│   ├── launch/
│   │   ├── vision_nav_rviz.launch.py  # Launches RViz + robot_state_publisher
│   │   └── walk_rviz.launch.py
│   ├── urdf/                          # Calibrated URDF models
│   └── meshes/                        # 3D STL mesh files
└── tests/
    └── test_vision_system.py          # Automated verification test suite (5 tests)
```

---

## Quickstart Guide

### 1. Test Your Camera & Markers (Diagnostic Mode)
Run the diagnostic tool to verify that all 4 corners, robot, and obstacles are recognized:

* **Using USB Webcam / DroidCam Wired (Windows PowerShell):**
  ```powershell
  cd "C:\Users\prith\Documents\MYSPACE\RPP-Project-2026\scripts"
  python test_camera_feed.py --source 0
  ```
* **Using Phone Wi-Fi Stream (IP Webcam / DroidCam):**
  ```powershell
  python test_camera_feed.py --source "http://192.168.0.131:8080/video"
  ```
* *Press `w` in the window to see the top-down flattened arena.*
* *Press `h` to toggle the skin-detection mask.*

---

### 2. Run Autonomous Navigation (Windows PowerShell)

* **Mode A: Synthetic Mock Demo (No camera or hardware needed):**
  ```powershell
  python main_vision_navigator.py --source mock --no-esp32
  ```

* **Mode B: Real Camera Tracking (Dry-run without robot moving):**
  ```powershell
  python main_vision_navigator.py --source 0 --no-esp32
  ```

* **Mode C: Live Hardware Navigation (Phone Camera + Physical Robot):**
  1. Connect PC Wi-Fi to the robot: **SSID:** `Quadruped-Controller` (Password: `12345678`).
  2. Connect phone via USB cable running DroidCam.
  3. Start navigation:
     ```powershell
     python main_vision_navigator.py --source 0 --esp32-ip 192.168.4.1
     ```

---

### 3. Run with ROS 2 RViz Digital Twin (WSL / Ubuntu)

* **Terminal 1 (Launch RViz 3D View):**
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch sesame_description vision_nav_rviz.launch.py
  ```

* **Terminal 2 (Start Navigator with ROS 2 Bridge):**
  ```bash
  source ~/ros2_ws/install/setup.bash
  cd "/mnt/c/Users/prith/Documents/MYSPACE/RPP-Project-2026/scripts"
  python3 main_vision_navigator.py --source "http://192.168.0.131:8080/video" --ros2 --esp32-ip 192.168.4.1
  ```

---

## Running the Automated Test Suite

To verify all system subsystems (Marker generation, Homography accuracy, Hand detection, $\ge 5\text{ cm}$ clearance, and Stanley steering):

```powershell
python "C:\Users\prith\Documents\MYSPACE\RPP-Project-2026\tests\test_vision_system.py"
```
Or in WSL:
```bash
python3 /mnt/c/Users/prith/Documents/MYSPACE/RPP-Project-2026/tests/test_vision_system.py
```
*(All 5 tests run in $< 2\text{ s}$ and report detailed pass/fail status).*

---

## Safety Features Active

1. **Planar Homography Auto-Rectification:** Even if your phone camera is held at an angle or vibrates slightly, OpenCV recalculates the homography matrix $H$ every frame, keeping coordinates stable down to pixel precision.
2. **Hand-Presence Safety Interlock:** When a human hand enters the arena to reposition boxes or the goal marker, the robot immediately halts (`stop=1` / stand) and path replanning is held until the hand exits for $> 0.4\text{ s}$, preventing erratic trajectories.
3. **Emergency Stop:** Press <kbd>q</kbd> or <kbd>ESC</kbd> in the OpenCV window or <kbd>Ctrl</kbd> + <kbd>C</kbd> in the terminal at any time to immediately cut power to the servos and level the robot upright.
