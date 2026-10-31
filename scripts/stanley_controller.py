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
import queue

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
        self.target_wp_idx = 1

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

        # Check if path changed: reset target waypoint index
        if path != getattr(self, '_last_path', None):
            self._last_path = list(path)
            self.target_wp_idx = 1

        # Advance waypoint index if close to current target waypoint
        if len(path) > 2 and self.target_wp_idx < len(path) - 1:
            curr_target = path[self.target_wp_idx]
            if math.hypot(curr_target[0] - rx, curr_target[1] - ry) < 8.0:
                self.target_wp_idx += 1

        # Use active path segment: P_a -> P_b
        idx_b = min(max(1, self.target_wp_idx), len(path) - 1)
        idx_a = max(0, idx_b - 1)
        pa = path[idx_a]
        pb = path[idx_b]

        seg_dx = pb[0] - pa[0]
        seg_dy = pb[1] - pa[1]
        seg_len = math.hypot(seg_dx, seg_dy)

        # 1. Direct Target Waypoint Heading (Pure Pursuit Line-of-Sight)
        target_dx = pb[0] - rx
        target_dy = pb[1] - ry
        target_bearing = math.atan2(target_dy, target_dx)
        target_heading_err = normalize_angle_rad(target_bearing - r_th)

        # 2. Signed Cross-Track Error relative to segment (positive = left of path)
        if seg_len < 1e-4:
            path_heading = r_th
            cross_track_err = 0.0
        else:
            path_heading = math.atan2(seg_dy, seg_dx)
            nx = -seg_dy / seg_len
            ny = seg_dx / seg_len
            cross_track_err = (rx - pa[0]) * nx + (ry - pa[1]) * ny

        # 3. Soft Cross-Track Correction with Clamping (+/- 20 deg max)
        # Prevents violent over-steering and endless S-curve oscillations
        raw_crosstrack = math.atan2(-self.k_gain * cross_track_err, self.speed + 0.1)
        max_cte_rad = math.radians(20.0)
        clamped_crosstrack = max(-max_cte_rad, min(max_cte_rad, raw_crosstrack))

        # Total Steering Angle: Pure Pursuit Target Bearing + Gentle Bounded CTE Pull
        total_steer = normalize_angle_rad(target_heading_err + 0.35 * clamped_crosstrack)
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
            "heading_err_deg": math.degrees(target_heading_err),
            "steering_deg": steer_deg,
            "dist_to_goal": dist_to_goal,
            "status": "TRACKING",
            "active_wp": idx_b
        }


class ESP32CommandBridge:
    """
    Non-blocking Wi-Fi command transmitter to the physical ESP32.
    Implements Discrete Step-and-Assess Architecture:
    - Sends individual calibrated pulses (0.35s) instead of continuous spamming.
    - Configures ESP32 settings (/setSettings?walkCycles=1&frameDelay=80) to eliminate command accumulation.
    - Robust HTTP socket management with 'Connection: close' and generous 1.2s timeout to prevent socket leaks and spurious disconnects.
    """

    def __init__(self, esp32_ip: str = "192.168.4.1", enabled: bool = True,
                 invert_fwd_bwd: bool = False, invert_turn: bool = False):
        self.ip = esp32_ip
        self.enabled = enabled
        self.invert_fwd_bwd = invert_fwd_bwd
        self.invert_turn = invert_turn
        self.running = True
        self.connected = False
        self.settings_configured = False
        self.is_busy = False
        self.last_rtt = 0.08  # Estimated round trip time in seconds
        self.last_step_time = 0.0
        self.current_cmd = "stand"
        self.cmd_queue: queue.Queue = queue.Queue(maxsize=3)
        self.lock = threading.Lock()
        self.status_msg = "Bridge Connecting..." if self.enabled else "Hardware Disabled (--no-esp32)"

        if self.enabled:
            self.thread = threading.Thread(target=self._worker_loop, daemon=True)
            self.thread.start()

    def trigger_step(self, action: str, pulse_sec: float = 0.65) -> bool:
        """
        Requests execution of a single discrete movement step.
        Returns True if accepted, False if bridge is busy executing another step.
        """
        if not self.enabled:
            return True
        with self.lock:
            if self.is_busy:
                return False
            self.is_busy = True
            self.current_cmd = action

        try:
            self.cmd_queue.put_nowait((action, pulse_sec))
            return True
        except queue.Full:
            with self.lock:
                self.is_busy = False
            return False

    def set_action(self, action: str):
        """Legacy helper for backward compatibility."""
        with self.lock:
            self.current_cmd = action

    def _send_raw_http(self, url: str, timeout: float = 1.2) -> bool:
        if not self.enabled:
            return False
        t0 = time.time()
        try:
            req = urllib.request.Request(
                url,
                headers={
                    'User-Agent': 'Sesame-Stanley-Bridge',
                    'Connection': 'close'  # Free lwIP socket immediately on ESP32
                }
            )
            with urllib.request.urlopen(req, timeout=timeout) as response:
                if response.status == 200:
                    rtt = time.time() - t0
                    with self.lock:
                        self.connected = True
                        self.last_rtt = 0.8 * self.last_rtt + 0.2 * rtt
                    return True
        except Exception:
            with self.lock:
                self.connected = False
                self.status_msg = f"Searching http://{self.ip}..."
            return False
        return False

    def _send_http_cmd(self, param: str, timeout: float = 1.2) -> bool:
        url = f"http://{self.ip}/cmd?{param}"
        success = self._send_raw_http(url, timeout=timeout)
        if success:
            with self.lock:
                self.status_msg = f"Connected ({param})"
        return success

    def _init_esp32_settings(self):
        """Configures ESP32 firmware for single-cycle execution (walkCycles=1) to eliminate queue lag."""
        url = f"http://{self.ip}/setSettings?walkCycles=1&frameDelay=80"
        if self._send_raw_http(url, timeout=1.5):
            self.settings_configured = True

    def _worker_loop(self):
        while self.running:
            # 1. Ensure initial settings (walkCycles=1) are synced upon connection
            if not self.settings_configured:
                self._init_esp32_settings()
                if not self.settings_configured:
                    time.sleep(0.5)
                    continue

            # 2. Wait for next discrete step command
            try:
                cmd_item = self.cmd_queue.get(timeout=0.20)
            except queue.Empty:
                continue

            action, pulse_sec = cmd_item

            if action in ("stand", "stop"):
                self._send_http_cmd("stop=1", timeout=1.0)
                self._send_http_cmd("pose=stand", timeout=1.0)
            elif action in ("forward", "backward", "left", "right"):
                # Handle forward/backward inversion if hardware moves in reverse
                send_act = action
                if self.invert_fwd_bwd:
                    if action == "forward":
                        send_act = "backward"
                    elif action == "backward":
                        send_act = "forward"
                # Handle steering inversion if physical robot turns opposite to intended
                if self.invert_turn:
                    if action == "left":
                        send_act = "right"
                    elif action == "right":
                        send_act = "left"
                # Trigger movement pulse
                self._send_http_cmd(f"go={send_act}", timeout=1.2)
                # Allow physical execution for pulse duration
                time.sleep(pulse_sec)
                # Crisp stop & return to stand pose
                self._send_http_cmd("stop=1", timeout=1.0)
                self._send_http_cmd("pose=stand", timeout=1.0)

            with self.lock:
                self.is_busy = False
                self.last_step_time = time.time()

            self.cmd_queue.task_done()

    def stop_robot(self):
        """Emergency stop & level servos into stand pose."""
        self.running = False
        if self.enabled:
            try:
                self._send_http_cmd("stop=1", timeout=0.8)
                self._send_http_cmd("pose=stand", timeout=0.8)
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
