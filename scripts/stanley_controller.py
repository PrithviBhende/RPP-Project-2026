#!/usr/bin/env python3
"""
Stanley Path-Following Controller & Hardware Actuation Bridge.

Translates Cartesian metric poses (from VisionTracker) and planned paths (from VisionPlanner)
into closed-loop steering commands for the physical ESP32 quadruped.

Stanley Control Law:
    delta(t) = theta_e(t) + arctan( (k * e(t)) / (v + epsilon) )
where:
    theta_e: heading error relative to path tangent
    e: signed cross-track distance error
    k: Stanley gain parameter
"""

from __future__ import annotations
import math
import time
import urllib.request
import threading

STANLEY_GAIN_K = 0.85          # Cross-track error scaling gain
NOMINAL_VEL_CM_S = 3.8         # Quadruped nominal trot speed in cm/s
GOAL_REACHED_RADIUS_CM = 5.0   # Stop distance threshold
STEER_DEADBAND_DEG = 12.0      # Angular deadband before triggering left/right turn
CROSS_TRACK_DEADBAND_CM = 2.0  # Cross-track deadband in cm


def normalize_angle_rad(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class StanleyController:
    """Computes cross-track error, heading error, and discrete steering commands."""

    def __init__(self, k_gain: float = STANLEY_GAIN_K, speed_cm_s: float = NOMINAL_VEL_CM_S):
        self.k_gain = k_gain
        self.speed = speed_cm_s
        self.current_wp_idx = 0

    def compute_control(self, robot_pose: tuple[float, float, float],
                        path: list[tuple[float, float]]) -> dict:
        """
        Calculates Stanley steering angle and discrete action.
        robot_pose: (rx_cm, ry_cm, r_heading_rad)
        path: [(x0, y0), (x1, y1), ...]
        """
        rx, ry, r_th = robot_pose

        if not path or len(path) == 0:
            return {
                "action": "stop",
                "cross_track_err": 0.0,
                "heading_err_deg": 0.0,
                "steering_deg": 0.0,
                "dist_to_goal": 0.0,
                "status": "NO_PATH"
            }

        # Check final goal arrival
        gx, gy = path[-1]
        dist_to_goal = math.hypot(gx - rx, gy - ry)
        if dist_to_goal <= GOAL_REACHED_RADIUS_CM:
            return {
                "action": "stand",
                "cross_track_err": 0.0,
                "heading_err_deg": 0.0,
                "steering_deg": 0.0,
                "dist_to_goal": dist_to_goal,
                "status": "ARRIVED"
            }

        # Advance waypoint index if close to current target waypoint
        if self.current_wp_idx < len(path) - 1:
            curr_target = path[self.current_wp_idx]
            if math.hypot(curr_target[0] - rx, curr_target[1] - ry) < 8.0:
                self.current_wp_idx += 1

        # Use current path segment: P_a -> P_b
        idx_a = min(self.current_wp_idx, len(path) - 2)
        idx_b = idx_a + 1
        pa = path[idx_a]
        pb = path[idx_b]

        seg_dx = pb[0] - pa[0]
        seg_dy = pb[1] - pa[1]
        seg_len = math.hypot(seg_dx, seg_dy)

        if seg_len < 1e-4:
            path_heading = r_th
            cross_track_err = 0.0
        else:
            path_heading = math.atan2(seg_dy, seg_dx)
            # Normal vector to path (+90 deg left)
            nx = -seg_dy / seg_len
            ny = seg_dx / seg_len
            # Signed distance from segment line to robot (positive = robot is to the left of path)
            cross_track_err = (rx - pa[0]) * nx + (ry - pa[1]) * ny

        # 1. Heading Error: theta_p - theta_r
        heading_err = normalize_angle_rad(path_heading - r_th)

        # 2. Cross-Track Correction: arctan( -k * e / v )
        # If robot is to the LEFT (e > 0), steer RIGHT (negative angle) to correct
        crosstrack_steer = math.atan2(-self.k_gain * cross_track_err, self.speed + 0.1)

        # Total Stanley Steering Angle
        total_steer = normalize_angle_rad(heading_err + crosstrack_steer)
        steer_deg = math.degrees(total_steer)

        # Map to Quadruped Locomotion Commands
        if abs(steer_deg) <= STEER_DEADBAND_DEG:
            action = "forward"
        elif steer_deg > STEER_DEADBAND_DEG:
            action = "left"    # Steer left to realign
        else:
            action = "right"   # Steer right to realign

        return {
            "action": action,
            "cross_track_err": cross_track_err,
            "heading_err_deg": math.degrees(heading_err),
            "steering_deg": steer_deg,
            "dist_to_goal": dist_to_goal,
            "status": "TRACKING",
            "active_wp": idx_b
        }


class ESP32CommandBridge:
    """Non-blocking Wi-Fi command transmitter to the physical ESP32."""

    def __init__(self, esp32_ip: str = "192.168.4.1", enabled: bool = True,
                 invert_fwd_bwd: bool = False, invert_left_right: bool = True):
        self.ip = esp32_ip
        self.enabled = enabled
        self.invert_fwd_bwd = invert_fwd_bwd
        self.invert_left_right = invert_left_right
        self.running = True
        self.connected = False
        self.current_cmd = "stand"
        self.lock = threading.Lock()
        self.status_msg = "Bridge Connecting..." if self.enabled else "Hardware Disabled (--no-esp32)"

        if self.enabled:
            self.thread = threading.Thread(target=self._worker_loop, daemon=True)
            self.thread.start()

    def set_action(self, action: str):
        with self.lock:
            self.current_cmd = action

    def _send_http_cmd(self, param: str, timeout: float = 0.25) -> bool:
        if not self.enabled:
            return False
        url = f"http://{self.ip}/cmd?{param}"
        try:
            req = urllib.request.Request(url, headers={'User-Agent': 'Sesame-Stanley-Bridge'})
            with urllib.request.urlopen(req, timeout=timeout) as response:
                if response.status == 200:
                    with self.lock:
                        self.connected = True
                        self.status_msg = f"Connected ({param})"
                    return True
        except Exception:
            with self.lock:
                self.connected = False
                self.status_msg = f"Searching http://{self.ip}..."
            return False

    def _worker_loop(self):
        last_sent = ""
        while self.running:
            with self.lock:
                cmd = self.current_cmd

            # Apply mechanical inversion to match physical robot motor assembly
            hw_action = cmd
            if cmd == "forward" and self.invert_fwd_bwd:
                hw_action = "backward"
            elif cmd == "backward" and self.invert_fwd_bwd:
                hw_action = "forward"
            elif cmd == "left" and self.invert_left_right:
                hw_action = "right"
            elif cmd == "right" and self.invert_left_right:
                hw_action = "left"

            if hw_action in ("forward", "backward", "left", "right"):
                self._send_http_cmd(f"go={hw_action}", timeout=0.30)
            elif hw_action == "stand":
                if last_sent != "stand":
                    self._send_http_cmd("stop=1", timeout=0.20)
                    self._send_http_cmd("pose=stand", timeout=0.20)
            elif hw_action == "stop":
                if last_sent != "stop":
                    self._send_http_cmd("stop=1", timeout=0.20)

            last_sent = hw_action
            time.sleep(0.35)

    def stop_robot(self):
        """Emergency stop & level servos into stand pose."""
        self.running = False
        if self.enabled:
            try:
                self._send_http_cmd("stop=1", timeout=0.20)
                self._send_http_cmd("pose=stand", timeout=0.20)
            except Exception:
                pass


def test_stanley():
    controller = StanleyController()

    path = [(10.0, 10.0), (40.0, 10.0), (80.0, 50.0)]

    # Case 1: Robot is on path, aligned
    pose1 = (20.0, 10.0, 0.0)
    res1 = controller.compute_control(pose1, path)
    print(f"[Test 1 - Aligned] Action: {res1['action']} | Steer: {res1['steering_deg']:.1f}° | CTE: {res1['cross_track_err']:.1f} cm")

    # Case 2: Robot drifted left (y = 14 cm instead of 10 cm)
    pose2 = (20.0, 14.0, 0.0)
    res2 = controller.compute_control(pose2, path)
    print(f"[Test 2 - Drift Left] Action: {res2['action']} | Steer: {res2['steering_deg']:.1f}° | CTE: {res2['cross_track_err']:.1f} cm")
    assert res2['action'] == 'right', f"Expected right turn, got {res2['action']}"

    # Case 3: Robot drifted right (y = 6 cm instead of 10 cm)
    pose3 = (20.0, 6.0, 0.0)
    res3 = controller.compute_control(pose3, path)
    print(f"[Test 3 - Drift Right] Action: {res3['action']} | Steer: {res3['steering_deg']:.1f}° | CTE: {res3['cross_track_err']:.1f} cm")
    assert res3['action'] == 'left', f"Expected left turn, got {res3['action']}"

    # Case 4: Arrived at goal
    pose4 = (80.0, 49.0, math.radians(45))
    res4 = controller.compute_control(pose4, path)
    print(f"[Test 4 - Arrived] Action: {res4['action']} | Status: {res4['status']}")
    assert res4['status'] == 'ARRIVED'

    print("[Stanley Test] SUCCESS: All control cases passed!")


if __name__ == "__main__":
    test_stanley()
