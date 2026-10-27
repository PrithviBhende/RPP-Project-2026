#!/usr/bin/env python3
"""
Simultaneous RViz Simulation & Physical ESP32 Hardware Walk Test.

Demonstrates real-time hardware/software synchronization:
1. RViz Simulation:
   - Articulates all 8 leg joints via /joint_states using the authentic Sesame CPG trot gait.
   - Advances the robot forward along /odom and /tf (odom -> base_footprint) across the RViz grid.
   - Simulates authentic 3D dynamic body attitude (roll wobble, pitch oscillation, compliance bobbing).

2. Physical Hardware Execution:
   - Sends real-time locomotion commands over Wi-Fi to the ESP32 ("Sesame-Controller" @ 192.168.4.1).
   - Triggers the physical MG90S servos to physically step forward in real life.
   - On shutdown (Ctrl+C), commands the ESP32 to stop motors and return to stand pose.
"""

from __future__ import annotations
import math
import time
import urllib.request
import urllib.error
import threading
import argparse
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, TransformStamped
    from sensor_msgs.msg import JointState
    from nav_msgs.msg import Odometry
    from tf2_ros import TransformBroadcaster
except ImportError:
    Node = object

# Canonical Joint Names & Order
JOINT_ORDER = ('R1', 'R2', 'L1', 'L2', 'R4', 'R3', 'L3', 'L4')

# Kinematics & Locomotion Parameters
NOMINAL_SPEED_M_S = 0.038       # 3.8 cm/s nominal trot speed
STEP_FREQ_HZ = 1.65             # 1.65 Hz nominal stepping cadence
ACTION_SCALE_RAD = 0.58         # 0.58 rad (~33.2 deg) servo residual stroke
ROLL_WOBBLE_RAD = math.radians(2.0)
PITCH_WOBBLE_RAD = math.radians(1.0)
BOB_AMPLITUDE_M = 0.0015


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


class SesameCPGEngine:
    """Central Pattern Generator trot oscillator."""
    _LOWER_INDICES = np.asarray([5, 4, 6, 7])
    _LOWER_DIRECTION = np.asarray([-1.0, 1.0, 1.0, -1.0])
    _HIP_DIRECTION = np.asarray([-1.0, 1.0, 1.0, -1.0])
    _OFFSETS = np.asarray([0.0, math.pi, math.pi, 0.0])

    def __init__(self, freq: float = STEP_FREQ_HZ):
        self.freq = freq
        self.phase = 0.0

    def step(self, dt: float) -> dict[str, float]:
        self.phase = (self.phase + 2.0 * math.pi * self.freq * dt) % (2.0 * math.pi)
        phases = self.phase + self._OFFSETS
        stride = np.sin(phases)
        swing = np.maximum(np.sin(phases), 0.0) ** 2

        # Hip forward/backward swing
        hip = self._HIP_DIRECTION * 0.38 * stride
        # Distal knee lift
        lower = self._LOWER_DIRECTION * 0.62 * (0.12 + 0.88 * swing)

        action = np.zeros(8, dtype=np.float64)
        action[:4] = hip
        action[self._LOWER_INDICES] = lower
        target_rad = np.clip(action, -1.0, 1.0) * ACTION_SCALE_RAD

        return {name: float(target_rad[i]) for i, name in enumerate(JOINT_ORDER)}


class ESP32WiFiBridge:
    """Non-blocking background thread communicating with the physical ESP32 over Wi-Fi."""
    def __init__(self, ip: str, enabled: bool = True):
        self.ip = ip
        self.enabled = enabled
        self.running = True
        self.connected = False
        self.lock = threading.Lock()
        self.last_ping_ms = 0.0
        self.status_msg = "Initializing..."

        if self.enabled:
            self.thread = threading.Thread(target=self._worker_loop, daemon=True)
            self.thread.start()

    def _send_http_cmd(self, param: str, timeout: float = 0.25) -> bool:
        if not self.enabled:
            return False
        url = f"http://{self.ip}/cmd?{param}"
        t0 = time.time()
        try:
            req = urllib.request.Request(url, headers={'User-Agent': 'Sesame-ROS2-Bridge'})
            with urllib.request.urlopen(req, timeout=timeout) as response:
                dt_ms = (time.time() - t0) * 1000.0
                if response.status == 200:
                    with self.lock:
                        self.connected = True
                        self.last_ping_ms = dt_ms
                        self.status_msg = f"Connected ({dt_ms:.0f} ms)"
                    return True
        except Exception:
            with self.lock:
                self.connected = False
                self.status_msg = f"Searching for robot at http://{self.ip}..."
            return False

    def _worker_loop(self):
        while self.running:
            # Send continuous walk keepalive command
            self._send_http_cmd("go=forward", timeout=0.30)
            time.sleep(0.35)

    def stop_robot(self):
        """Immediately halts the physical robot servos and returns to stand pose."""
        self.running = False
        if self.enabled:
            try:
                self._send_http_cmd("stop=1", timeout=0.20)
                self._send_http_cmd("pose=stand", timeout=0.20)
            except Exception:
                pass


class SesameHardwareRVizWalkNode(Node):
    def __init__(self, esp32_ip: str = "192.168.4.1", enable_esp32: bool = True, walk_speed: float = NOMINAL_SPEED_M_S):
        super().__init__('sesame_esp32_rviz_walk')

        self.speed = walk_speed
        self.cpg = SesameCPGEngine(freq=STEP_FREQ_HZ)
        self.esp32 = ESP32WiFiBridge(ip=esp32_ip, enabled=enable_esp32)

        # ROS 2 Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Simulation state
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.dt = 0.02  # 50 Hz update rate
        self.last_time = time.time()
        self.last_log_time = 0.0

        self.timer = self.create_timer(self.dt, self._update_loop)

        self.get_logger().info("=" * 70)
        self.get_logger().info("SESAME HARDWARE & RVIZ SYNCHRONOUS WALK TEST STARTED")
        self.get_logger().info(f"Target Speed: {self.speed * 100:.1f} cm/s (Trot cadence: {STEP_FREQ_HZ} Hz)")
        self.get_logger().info(f"ESP32 Wi-Fi:  http://{esp32_ip} (Enabled: {enable_esp32})")
        self.get_logger().info("RViz View:    Publishing /joint_states, /odom, and TF (odom -> base_footprint)")
        self.get_logger().info("=" * 70)

    def _update_loop(self):
        now = time.time()
        actual_dt = now - self.last_time
        self.last_time = now

        # 1. Advance position forward in +X
        self.pos_x += self.speed * actual_dt

        # 2. Compute 8 joint angles from CPG trot engine
        joint_angles = self.cpg.step(actual_dt)

        # 3. Publish /joint_states for RViz robot model articulation
        now_msg = self.get_clock().now().to_msg()
        js_msg = JointState()
        js_msg.header.stamp = now_msg
        js_msg.name = list(JOINT_ORDER)
        js_msg.position = [joint_angles[j] for j in JOINT_ORDER]
        self.joint_pub.publish(js_msg)

        # 4. Authentic 3D attitude dynamics (Roll, Pitch, Compliance Bobbing)
        current_roll = ROLL_WOBBLE_RAD * math.sin(self.cpg.phase)
        current_pitch = PITCH_WOBBLE_RAD * math.cos(self.cpg.phase)
        current_z = BOB_AMPLITUDE_M * (1.0 - abs(math.sin(self.cpg.phase)))

        qx, qy, qz, qw = euler_to_quaternion(current_roll, current_pitch, 0.0)

        # 5. Broadcast TF: odom -> base_footprint (robot moves forward across RViz grid)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now_msg
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_footprint"
        tf_msg.transform.translation.x = self.pos_x
        tf_msg.transform.translation.y = self.pos_y
        tf_msg.transform.translation.z = current_z
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

        # 6. Publish /odom
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.pos_x
        odom.pose.pose.position.y = self.pos_y
        odom.pose.pose.position.z = current_z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.speed
        self.odom_pub.publish(odom)

        # 7. Publish /cmd_vel
        cmd = Twist()
        cmd.linear.x = self.speed
        self.cmd_pub.publish(cmd)

        # Status logging (every 1 second)
        if now - self.last_log_time > 1.0:
            self.last_log_time = now
            esp_status = self.esp32.status_msg if self.esp32.enabled else "Disabled (--no-esp32)"
            color = "\033[92m" if self.esp32.connected else "\033[93m"
            reset = "\033[0m"
            print(f"[RViz Walk] Pos: {self.pos_x:.2f} m | Gait Phase: {math.degrees(self.cpg.phase):.0f}° | ESP32: {color}{esp_status}{reset}")

    def shutdown(self):
        """Clean shutdown restoring upright stand pose on both RViz and hardware."""
        print("\n[Sesame] Stopping robot motors and returning to stand pose...")
        self.esp32.stop_robot()

        # Send stand pose to RViz (all joints 0.0 rad)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JOINT_ORDER)
        js.position = [0.0] * 8
        self.joint_pub.publish(js)

        # Send 0 velocity
        self.cmd_pub.publish(Twist())


def main():
    parser = argparse.ArgumentParser(description="Synchronous RViz & Physical ESP32 Walking Controller")
    parser.add_argument("--esp32-ip", type=str, default="192.168.4.1", help="ESP32 IP (Default: 192.168.4.1)")
    parser.add_argument("--speed", type=float, default=NOMINAL_SPEED_M_S, help="Forward speed in m/s (Default: 0.038)")
    parser.add_argument("--no-esp32", action="store_true", help="Run RViz visualization only without sending Wi-Fi commands")
    args = parser.parse_args()

    rclpy.init()
    node = SesameHardwareRVizWalkNode(
        esp32_ip=args.esp32_ip,
        enable_esp32=not args.no_esp32,
        walk_speed=args.speed
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
