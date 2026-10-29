#!/usr/bin/env python3
"""
Master Vision-Guided Autonomous Navigator & RViz Digital Twin.

Integrates:
1. Overhead Vision Tracking (VisionTracker):
   - Multi-version ArUco detection (DICT_4X4_50).
   - 4-Corner Planar Homography: auto-rectifies tilted/handheld video into calibrated cm coordinates.
   - Real-time Human Hand Detection: pauses locomotion and replanning during object placement.
2. Dynamic Path Planner (VisionPathPlanner):
   - A* search with Euclidean line-of-sight shortcutting.
   - Strictly enforces >= 5 cm safety clearance from all obstacles.
3. Path Follower (StanleyController):
   - Computes cross-track error and heading error relative to path tangent.
4. Hardware Actuation (ESP32CommandBridge):
   - Commands physical quadruped over Wi-Fi (192.168.4.1).
5. Digital Twin (ROS 2 RViz):
   - Broadcasts TF (map -> odom -> base_footprint).
   - Publishes /path (nav_msgs/Path) and /obstacle_markers (MarkerArray).
   - Articulates leg joints via /joint_states so simulation mirrors physical reality.
"""

from __future__ import annotations
import os
import math
import time
import json
import socket
import subprocess
import argparse
import numpy as np
import cv2


def get_wsl_ip() -> str:
    """Detects WSL2 virtual IP address so Windows can stream UDP telemetry directly to RViz in WSL."""
    try:
        res = subprocess.run(["wsl", "hostname", "-I"], capture_output=True, text=True, timeout=1.0)
        if res.returncode == 0 and res.stdout.strip():
            return res.stdout.strip().split()[0]
    except Exception:
        pass
    return "127.0.0.1"

# Import local subsystems
from vision_tracker import (
    VisionTracker, SyntheticMockCamera,
    DEFAULT_ARENA_WIDTH_CM, DEFAULT_ARENA_HEIGHT_CM, WARP_PX_PER_CM
)
from vision_planner import VisionPathPlanner, ROBOT_RADIUS_CM
from stanley_controller import StanleyController, ESP32CommandBridge, normalize_angle_rad

# Optional ROS 2 integration
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped, PoseStamped
    from sensor_msgs.msg import JointState
    from nav_msgs.msg import Path
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_ros import TransformBroadcaster
    ROS2_AVAILABLE = True
except ImportError:
    Node = object
    ROS2_AVAILABLE = False


JOINT_NAMES = ('R1', 'R2', 'L1', 'L2', 'R4', 'R3', 'L3', 'L4')


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy
    )


class CPGGaitGenerator:
    """Trot gait oscillator for RViz digital twin articulation."""
    _LOWER_INDICES = np.asarray([5, 4, 6, 7])
    _LOWER_DIRECTION = np.asarray([-1.0, 1.0, 1.0, -1.0])
    _HIP_DIRECTION = np.asarray([-1.0, 1.0, 1.0, -1.0])
    _OFFSETS = np.asarray([0.0, math.pi, math.pi, 0.0])

    def __init__(self, freq_hz: float = 1.65):
        self.freq = freq_hz
        self.phase = 0.0

    def step(self, dt: float, is_moving: bool) -> dict[str, float]:
        if not is_moving:
            return {j: 0.0 for j in JOINT_NAMES}
        self.phase = (self.phase + 2.0 * math.pi * self.freq * dt) % (2.0 * math.pi)
        phases = self.phase + self._OFFSETS
        stride = np.sin(phases)
        swing = np.maximum(np.sin(phases), 0.0) ** 2
        hip = self._HIP_DIRECTION * 0.38 * stride
        lower = self._LOWER_DIRECTION * 0.62 * (0.12 + 0.88 * swing)
        action = np.zeros(8, dtype=np.float64)
        action[:4] = hip
        action[self._LOWER_INDICES] = lower
        target_rad = np.clip(action, -1.0, 1.0) * 0.58
        return {name: float(target_rad[i]) for i, name in enumerate(JOINT_NAMES)}


class VisionNavigatorNode:
    """Central coordinator orchestrating Vision, Planning, Control, ESP32, and RViz."""

    def __init__(self, source: str = "mock", width_cm: float = DEFAULT_ARENA_WIDTH_CM,
                 height_cm: float = DEFAULT_ARENA_HEIGHT_CM, esp32_ip: str = "192.168.4.1",
                 enable_esp32: bool = True, enable_ros2: bool = False, headless: bool = False,
                 max_frames: int = 0, enable_hand_detect: bool = False, latency: float = 0.5):
        self.source = source
        self.width_cm = width_cm
        self.height_cm = height_cm
        self.headless = headless
        self.max_frames = max_frames
        self.base_latency = max(0.5, float(latency))
        self.last_step_time = 0.0
        self.last_action = "stand"
        self.last_steer = 0.0
        self.last_aligned = False
        self.current_step_interval = self.base_latency

        # Subsystems
        self.tracker = VisionTracker(width_cm=width_cm, height_cm=height_cm, enable_hand_detect=enable_hand_detect)
        self.planner = VisionPathPlanner(arena_width_cm=width_cm, arena_height_cm=height_cm)
        self.stanley = StanleyController()
        self.esp32 = ESP32CommandBridge(esp32_ip=esp32_ip, enabled=enable_esp32)
        self.cpg = CPGGaitGenerator()

        # State
        self.current_path: list[tuple[float, float]] = []
        self.hand_cooldown_t0: float = 0.0
        self.running = True

        # High-Speed Local UDP + Shared File Telemetry Broadcaster (Streams to RViz Digital Twin on localhost and WSL)
        self.telemetry_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "telemetry.json")
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        wsl_ip = get_wsl_ip()
        self.udp_targets = [("127.0.0.1", 9090)]
        if wsl_ip != "127.0.0.1":
            self.udp_targets.append((wsl_ip, 9090))
        print(f"[Navigator] RViz Telemetry Streaming to: {self.udp_targets} & File IPC: {self.telemetry_path}")

        # Camera Capture Setup
        self.static_image = None
        if source.lower() == "mock":
            print("[Navigator] Using Synthetic Mock Camera feed.")
            self.cap = SyntheticMockCamera()
        elif isinstance(source, str) and os.path.isfile(source) and source.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp', '.webp')):
            print(f"[Navigator] Loading Static Test Image: {source}")
            self.static_image = cv2.imread(source)
            self.cap = None
        elif source.isdigit():
            print(f"[Navigator] Opening USB Webcam index {source}...")
            self.cap = cv2.VideoCapture(int(source))
        else:
            print(f"[Navigator] Opening IP Camera Stream {source}...")
            self.cap = cv2.VideoCapture(source)

        # ROS 2 Setup
        self.ros_node = None
        self.tf_broadcaster = None
        self.joint_pub = None
        self.path_pub = None
        self.marker_pub = None
        if enable_ros2 and ROS2_AVAILABLE:
            if not rclpy.ok():
                rclpy.init()
            self.ros_node = Node('vision_navigator')
            self.tf_broadcaster = TransformBroadcaster(self.ros_node)
            self.joint_pub = self.ros_node.create_publisher(JointState, '/joint_states', 10)
            self.path_pub = self.ros_node.create_publisher(Path, '/plan_path', 10)
            self.marker_pub = self.ros_node.create_publisher(MarkerArray, '/obstacle_markers', 10)
            print("[Navigator] ROS 2 RViz publishers initialized.")

    def check_frontal_obstacle(self, robot_pose: tuple[float, float, float],
                               obstacles: dict[int, tuple[float, float]],
                               lookahead_dist_cm: float = 24.0,
                               half_cone_deg: float = 45.0) -> tuple[bool, str | None, float, int | None]:
        """
        Detects if an obstacle block is directly in front of the robot within collision range.
        Returns: (is_blocked, turn_recommendation, distance_to_obstacle, obstacle_id)
        """
        rx, ry, r_th = robot_pose
        min_dist = float('inf')
        blocked = False
        turn_cmd = None
        blocking_id = None

        for oid, (ox, oy) in obstacles.items():
            dx = ox - rx
            dy = oy - ry
            dist = math.hypot(dx, dy)
            # If within forward reaction envelope (<= 24 cm covers 8 cm robot + 5 cm block + 11 cm buffer)
            if dist <= lookahead_dist_cm:
                angle_to_obs = math.atan2(dy, dx)
                rel_bearing = normalize_angle_rad(angle_to_obs - r_th)
                rel_bearing_deg = math.degrees(rel_bearing)

                # Check if inside forward cone (+/- 45 deg)
                if abs(rel_bearing_deg) <= half_cone_deg:
                    blocked = True
                    if dist < min_dist:
                        min_dist = dist
                        blocking_id = oid
                        # Obstacle to the left (+bearing) -> steer right away from it
                        # Obstacle to the right (-bearing) -> steer left away from it
                        turn_cmd = "right" if rel_bearing_deg >= 0 else "left"

        return blocked, turn_cmd, min_dist, blocking_id

    def run(self):
        print("\n" + "=" * 75)
        print("AUTONOMOUS VISION-GUIDED NAVIGATION ACTIVE")
        print("Arena Size:   {:.0f} cm x {:.0f} cm | Clearance: >= 5.0 cm".format(self.width_cm, self.height_cm))
        print("Hardware:     ESP32 ({}) | RViz Digital Twin: {}".format(
            self.esp32.status_msg if self.esp32.enabled else "Disabled",
            "Active" if self.ros_node else "Off"
        ))
        print("Controls:     Press 'q' or 'ESC' in HUD window to quit.")
        print("=" * 75 + "\n")

        last_time = time.time()
        frame_idx = 0

        try:
            while self.running:
                if self.static_image is not None:
                    ret, frame = True, self.static_image.copy()
                    time.sleep(0.03)  # ~30 fps simulated loop
                else:
                    ret, frame = self.cap.read()
                    if not ret or frame is None:
                        time.sleep(0.01)
                        continue

                now = time.time()
                dt = now - last_time
                last_time = now
                frame_idx += 1

                # 1. Vision Pipeline: detect ArUco, warp perspective, extract metric poses
                annotated_frame, telemetry = self.tracker.process_frame(frame)

                robot_pose = telemetry["robot_pose"]   # (rx_cm, ry_cm, r_heading_rad)
                goal_pos = telemetry["goal_pos"]       # (gx_cm, gy_cm)
                obstacles = telemetry["obstacles"]     # {id: (ox_cm, oy_cm)}
                # 2. Check arena locking & poses
                if telemetry["arena_locked"] and robot_pose is not None and goal_pos is not None:
                    # Update dynamic obstacle positions
                    obs_moved = self.planner.update_obstacles(obstacles)
                    start_pt = (robot_pose[0], robot_pose[1])

                    # 3. Frontal Obstacle Detection: Check if an obstacle is directly in front
                    front_blocked, turn_evade, obs_dist, block_oid = self.check_frontal_obstacle(robot_pose, obstacles)

                    # Dynamic Path Planning: replan if obstacles moved, path empty, or front path obstructed
                    force_replan = obs_moved or front_blocked
                    if force_replan or not self.current_path:
                        self.current_path = self.planner.plan(start_pt, goal_pos, force_replan=force_replan)

                    # 4. Stanley Path Following Controller
                    ctrl_res = self.stanley.compute_control(robot_pose, self.current_path)
                    steer_deg = ctrl_res["steering_deg"]
                    nav_status = ctrl_res["status"]

                    # 5. Check Alignment with Waypoint & Frontal Obstacle Clearance
                    is_aligned = (abs(steer_deg) <= 12.0) and not front_blocked

                    if front_blocked and nav_status != "ARRIVED":
                        desired_action = turn_evade if turn_evade else ("left" if steer_deg > 0 else "right")
                        nav_status = f"FRONT_OBS_AVOID (Obs {block_oid} at {obs_dist:.1f}cm -> Turn {desired_action.upper()})"
                    elif nav_status == "ARRIVED":
                        desired_action = "stand"
                    elif is_aligned:
                        desired_action = "forward"
                        nav_status = "ALIGNED_FORWARD"
                    elif steer_deg > 12.0:
                        desired_action = "left"
                        nav_status = f"ALIGN_LEFT ({steer_deg:+.0f} deg)"
                    else:
                        desired_action = "right"
                        nav_status = f"ALIGN_RIGHT ({steer_deg:+.0f} deg)"

                    ctrl_res["status"] = nav_status
                    ctrl_res["action"] = desired_action
                else:
                    desired_action = "stand"
                    nav_status = "SEARCHING"
                    ctrl_res = {
                        "action": "stand", "cross_track_err": 0.0,
                        "heading_err_deg": 0.0, "steering_deg": 0.0,
                        "dist_to_goal": 0.0, "status": nav_status
                    }
                    front_blocked = False
                    block_oid = None
                    obs_dist = 0.0
                    is_aligned = False
                    steer_deg = 0.0

                # 6. Discrete Step-and-Assess Architecture:
                # Interval = max(0.5s, camera_stream_latency, ESP32_RTT + 0.15s)
                step_interval = max(0.5, self.base_latency, (self.esp32.last_rtt + 0.15) if self.esp32.connected else 0.5)
                self.current_step_interval = step_interval
                dt_since_step = now - self.last_step_time

                if telemetry["arena_locked"] and robot_pose is not None and goal_pos is not None:
                    if desired_action == "stand":
                        action = "stand"
                        if self.last_action != "stand":
                            self.esp32.trigger_step("stand")
                            self.last_action = "stand"
                    else:
                        # Only take physical step when step_interval has elapsed and bridge is not busy
                        if dt_since_step >= step_interval and not self.esp32.is_busy:
                            action = desired_action
                            self.esp32.trigger_step(action, pulse_sec=0.35)
                            self.last_step_time = now
                            self.last_action = action
                            self.last_steer = steer_deg
                            self.last_aligned = is_aligned
                            print(f"[Step-Assess] dt: {dt_since_step:.2f}s >= {step_interval:.2f}s | "
                                  f"Aligned: {'YES' if is_aligned else 'NO'} (Steer: {steer_deg:+.1f} deg) -> "
                                  f"Step {action.upper()} | CTE: {ctrl_res['cross_track_err']:+.1f} cm | "
                                  f"ESP32: {self.esp32.status_msg}")
                        else:
                            # Step is currently running or waiting for camera stream to settle
                            action = self.last_action if self.esp32.is_busy else "stand"
                else:
                    action = "stand"
                    if self.last_action != "stand":
                        self.esp32.trigger_step("stand")
                        self.last_action = "stand"

                # 5b. Stream Telemetry to RViz Digital Twin Bridge (High-speed File IPC + UDP)
                try:
                    packet = {
                        "robot_pose": [float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2])] if robot_pose else None,
                        "action": action,
                        "path": [[float(p[0]), float(p[1])] for p in self.current_path] if self.current_path else [],
                        "obstacles": {str(k): [float(v[0]), float(v[1])] for k, v in obstacles.items()},
                        "goal": [float(goal_pos[0]), float(goal_pos[1])] if goal_pos else None
                    }
                    data_bytes = json.dumps(packet).encode('utf-8')

                    # 1. Atomic file write for instant cross-platform Windows <-> WSL2 sync
                    tmp_file = self.telemetry_path + ".tmp"
                    with open(tmp_file, 'w') as f:
                        f.write(json.dumps(packet))
                    os.replace(tmp_file, self.telemetry_path)

                    # 2. UDP socket broadcast
                    for target in self.udp_targets:
                        try:
                            self.udp_sock.sendto(data_bytes, target)
                        except Exception:
                            pass
                except Exception:
                    pass

                # 6. Publish to ROS 2 RViz Digital Twin
                if self.ros_node:
                    self._publish_ros2(robot_pose, goal_pos, obstacles, self.current_path, action, dt)

                # 7. Render Path & Navigation Overlays on OpenCV HUD
                annotated_frame = self._render_navigation_hud(annotated_frame, telemetry, ctrl_res)

                # Periodic Terminal Log
                if frame_idx % 30 == 0:
                    r_str = f"({robot_pose[0]:.1f}, {robot_pose[1]:.1f})" if robot_pose else "Lost"
                    g_str = f"({goal_pos[0]:.1f}, {goal_pos[1]:.1f})" if goal_pos else "None"
                    print(f"[Nav] Status: {nav_status} | Action: {action.upper()} | Robot: {r_str} | Goal: {g_str} | CTE: {ctrl_res['cross_track_err']:.1f} cm | ESP32: {self.esp32.status_msg}")

                # Display GUI
                if not self.headless:
                    cv2.imshow("Overhead Arena Autonomous Navigator (Rectified)", annotated_frame)
                    key = cv2.waitKey(15) & 0xFF
                    if key == ord('q') or key == 27:
                        break

                if self.max_frames > 0 and frame_idx >= self.max_frames:
                    print(f"[Navigator] Reached max test frames ({self.max_frames}). Clean exit.")
                    break

        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def _render_navigation_hud(self, hud: np.ndarray, telemetry: dict, ctrl_res: dict) -> np.ndarray:
        """Overlays planned path, active waypoints, and navigation telemetry."""
        # Draw planned path polyline
        if self.current_path and len(self.current_path) >= 2:
            pts_px = [self.tracker.transformer.cm_to_warp_px(p[0], p[1]) for p in self.current_path]
            for i in range(len(pts_px) - 1):
                cv2.line(hud, pts_px[i], pts_px[i + 1], (0, 255, 120), 3, cv2.LINE_AA)
            for i, pt in enumerate(pts_px):
                cv2.circle(hud, pt, 5, (0, 220, 255), -1, cv2.LINE_AA)
                cv2.putText(hud, f"WP{i}", (pt[0] + 6, pt[1] - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 255), 1, cv2.LINE_AA)

        # Bottom Telemetry Dashboard Bar
        dh = 45
        dy = hud.shape[0] - dh
        cv2.rectangle(hud, (0, dy), (hud.shape[1], hud.shape[0]), (25, 25, 25), -1)
        cv2.line(hud, (0, dy), (hud.shape[1], dy), (70, 70, 70), 1)

        action = ctrl_res["action"].upper()
        if ctrl_res.get("status", "").startswith("FRONT_OBS_AVOID"):
            act_color = (0, 140, 255)
            cv2.putText(hud, f"[!] OBSTACLE IN FRONT -> TURNING {action} TO AVOID", (260, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 140, 255), 2, cv2.LINE_AA)
        else:
            act_color = (0, 255, 0) if action == "FORWARD" else (0, 200, 255) if action in ("LEFT", "RIGHT") else (0, 0, 255)

        align_label = "YES" if abs(ctrl_res['steering_deg']) <= 12.0 else "NO"
        hud_text = f"CMD: {action} | Align: {align_label} | CTE: {ctrl_res['cross_track_err']:+.1f} cm | Steer: {ctrl_res['steering_deg']:+.0f} deg | Step: {self.current_step_interval:.2f}s | Lat: {self.base_latency:.2f}s"
        cv2.putText(hud, hud_text, (15, dy + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.52, act_color, 2, cv2.LINE_AA)

        return hud

    def _publish_ros2(self, robot_pose, goal_pos, obstacles, path, action, dt: float):
        """Streams real-world state into RViz as ground-truth digital twin."""
        now_msg = self.ros_node.get_clock().now().to_msg()

        # 1. Articulate joints via /joint_states
        is_moving = action in ("forward", "left", "right")
        joint_angles = self.cpg.step(dt, is_moving)
        js = JointState()
        js.header.stamp = now_msg
        js.name = list(JOINT_NAMES)
        js.position = [joint_angles[j] for j in JOINT_NAMES]
        self.joint_pub.publish(js)

        # 2. Broadcast TF: map -> odom -> base_footprint (coordinates in meters)
        if robot_pose is not None:
            rx_m = robot_pose[0] / 100.0
            ry_m = robot_pose[1] / 100.0
            r_th = robot_pose[2]
            qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, r_th)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = now_msg
            tf_msg.header.frame_id = "map"
            tf_msg.child_frame_id = "base_footprint"
            tf_msg.transform.translation.x = rx_m
            tf_msg.transform.translation.y = ry_m
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)

        # 3. Publish /plan_path in meters
        if path and len(path) >= 2:
            path_msg = Path()
            path_msg.header.stamp = now_msg
            path_msg.header.frame_id = "map"
            for wp in path:
                p = PoseStamped()
                p.header.stamp = now_msg
                p.header.frame_id = "map"
                p.pose.position.x = wp[0] / 100.0
                p.pose.position.y = wp[1] / 100.0
                p.pose.orientation.w = 1.0
                path_msg.poses.append(p)
            self.path_pub.publish(path_msg)

        # 4. Publish /obstacle_markers (3D Cubes with 5 cm safety buffer in RViz)
        m_array = MarkerArray()
        for oid, (ox, oy) in obstacles.items():
            m = Marker()
            m.header.stamp = now_msg
            m.header.frame_id = "map"
            m.id = oid
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = ox / 100.0
            m.pose.position.y = oy / 100.0
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = 0.10
            m.scale.y = 0.10
            m.scale.z = 0.10
            m.color.r = 0.85
            m.color.g = 0.20
            m.color.b = 0.20
            m.color.a = 0.90
            m_array.markers.append(m)

            # Safety clearance cylinder visualization
            m_cyl = Marker()
            m_cyl.header.stamp = now_msg
            m_cyl.header.frame_id = "map"
            m_cyl.id = oid + 1000
            m_cyl.type = Marker.CYLINDER
            m_cyl.action = Marker.ADD
            m_cyl.pose.position.x = ox / 100.0
            m_cyl.pose.position.y = oy / 100.0
            m_cyl.pose.position.z = 0.005
            m_cyl.pose.orientation.w = 1.0
            # Radius = 5 cm (box half) + 8 cm (bot) + 5 cm (buffer) = 18 cm radius = 36 cm diameter
            m_cyl.scale.x = 0.36
            m_cyl.scale.y = 0.36
            m_cyl.scale.z = 0.01
            m_cyl.color.r = 1.0
            m_cyl.color.g = 0.65
            m_cyl.color.b = 0.0
            m_cyl.color.a = 0.25
            m_array.markers.append(m_cyl)

        if obstacles:
            self.marker_pub.publish(m_array)

        # Spin ROS 2 executor once non-blocking
        try:
            rclpy.spin_once(self.ros_node, timeout_sec=0.001)
        except Exception:
            pass

    def shutdown(self):
        print("\n[Navigator] Stopping physical motors and cleaning up...")
        self.running = False
        self.esp32.stop_robot()
        if hasattr(self.cap, 'release'):
            self.cap.release()
        cv2.destroyAllWindows()
        if self.ros_node:
            try:
                # Level robot stand pose in RViz
                js = JointState()
                js.header.stamp = self.ros_node.get_clock().now().to_msg()
                js.name = list(JOINT_NAMES)
                js.position = [0.0] * 8
                self.joint_pub.publish(js)
                self.ros_node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
        print("[Navigator] Shutdown complete.")


def main():
    parser = argparse.ArgumentParser(description="Master Overhead Vision-Guided Navigator & Digital Twin")
    parser.add_argument("--source", type=str, default="mock",
                        help="Camera source: 'mock', webcam index ('0', '1'), or IP Stream URL (http://... / rtsp://...)")
    parser.add_argument("--esp32-ip", type=str, default="192.168.4.1", help="ESP32 IP (Default: 192.168.4.1)")
    parser.add_argument("--no-esp32", action="store_true", help="Run without sending Wi-Fi commands to physical robot")
    parser.add_argument("--ros2", action="store_true", help="Enable ROS 2 TF, path, and marker streaming to RViz")
    parser.add_argument("--width-cm", type=float, default=DEFAULT_ARENA_WIDTH_CM, help="Arena width in cm")
    parser.add_argument("--height-cm", type=float, default=DEFAULT_ARENA_HEIGHT_CM, help="Arena height in cm")
    parser.add_argument("--headless", action="store_true", help="Run without GUI windows (for automated testing)")
    parser.add_argument("--max-frames", type=int, default=0, help="Maximum frames to process before exiting (0 = infinite)")
    parser.add_argument("--enable-hand-detect", action="store_true", help="Enable pausing motion when human hand is in arena (Default: False)")
    parser.add_argument("--latency", type=float, default=0.5,
                        help="Camera stream latency in seconds (minimum 0.5s, default 0.5)")
    args = parser.parse_args()

    node = VisionNavigatorNode(
        source=args.source,
        width_cm=args.width_cm,
        height_cm=args.height_cm,
        esp32_ip=args.esp32_ip,
        enable_esp32=not args.no_esp32,
        enable_ros2=args.ros2,
        headless=args.headless,
        max_frames=args.max_frames,
        enable_hand_detect=args.enable_hand_detect,
        latency=args.latency
    )

    node.run()


if __name__ == "__main__":
    main()
