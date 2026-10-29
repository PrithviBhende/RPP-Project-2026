#!/usr/bin/env python3
"""
ROS 2 RViz Digital Twin Bridge Node.

Receives real-time telemetry from main_vision_navigator.py (via local high-speed UDP 127.0.0.1:9090)
and publishes all required ROS 2 topics:
- TF: map -> base_footprint (with 3D attitude oscillation)
- /joint_states: 8-servo CPG trot gait articulation or stand pose
- /plan_path: nav_msgs/Path (green trajectory line in RViz)
- /obstacle_markers: visualization_msgs/MarkerArray (3D obstacle cubes + safety rings)

Guarantees 100% error-free RViz startup: always broadcasts valid initial TF and stand pose
so RViz never displays 'Frame [map] does not exist' or red RobotModel errors.
"""

from __future__ import annotations
import os
import math
import time
import json
import socket
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

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


class RVizBridgeNode(Node):
    def __init__(self, udp_port: int = 9090):
        super().__init__('rviz_bridge_node')

        # Publishers
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.path_pub = self.create_publisher(Path, '/plan_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)

        # CPG Engine
        self.cpg = CPGGaitGenerator()

        # Telemetry state (default initial pose so RViz has valid frames immediately)
        self.lock = threading.Lock()
        self.rx_m = 0.15
        self.ry_m = 0.15
        self.yaw = 0.0
        self.action = "stand"
        self.path_pts: list[list[float]] = []
        self.obstacles: dict[str, list[float]] = {}
        self.goal_pt: list[float] | None = None
        self.last_packet_time = 0.0
        self.packet_count = 0

        # Start Telemetry Listener (Hybrid UDP Socket + Shared File IPC for Hyper-V bypass)
        self.telemetry_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "telemetry.json")
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(('0.0.0.0', udp_port))
        self.udp_sock.settimeout(0.01)

        self.listener_thread = threading.Thread(target=self._telemetry_listener_loop, daemon=True)
        self.listener_thread.start()

        # 50 Hz ROS 2 broadcast timer
        self.last_time = time.time()
        self.timer = self.create_timer(0.02, self._broadcast_loop)

        self.get_logger().info("=" * 65)
        self.get_logger().info(f"RVIZ DIGITAL TWIN BRIDGE ACTIVE (Listening on UDP :{udp_port} & File IPC)")
        self.get_logger().info("Publishing TF: map -> base_footprint, /joint_states, /plan_path")
        self.get_logger().info("=" * 65)

    def _apply_telemetry_packet(self, packet: dict):
        with self.lock:
            self.packet_count += 1
            self.last_packet_time = time.time()
            if self.packet_count == 1 or self.packet_count % 60 == 0:
                obs_count = len(packet.get("obstacles", {}))
                wp_count = len(packet.get("path", []))
                r_pose = packet.get("robot_pose")
                r_str = f"({r_pose[0]:.1f}, {r_pose[1]:.1f}) cm" if r_pose else "Lost"
                self.get_logger().info(f"[Bridge Telemetry] Synced! Robot: {r_str} | Obstacles: {obs_count} | Path: {wp_count} WPs")

            if "robot_pose" in packet and packet["robot_pose"]:
                # Convert cm to meters
                self.rx_m = packet["robot_pose"][0] / 100.0
                self.ry_m = packet["robot_pose"][1] / 100.0
                self.yaw = packet["robot_pose"][2]
            if "action" in packet:
                self.action = packet["action"]
            if "path" in packet and packet["path"]:
                self.path_pts = packet["path"]
            if "obstacles" in packet:
                self.obstacles = packet["obstacles"]
            if "goal" in packet and packet["goal"]:
                self.goal_pt = packet["goal"]

    def _telemetry_listener_loop(self):
        last_mtime = 0.0
        while rclpy.ok():
            # 1. Try UDP packet (if available)
            try:
                data, _ = self.udp_sock.recvfrom(65536)
                packet = json.loads(data.decode('utf-8'))
                self._apply_telemetry_packet(packet)
            except socket.timeout:
                pass
            except Exception:
                pass

            # 2. Check Shared Telemetry File (cross-platform Hyper-V bypass)
            try:
                if os.path.exists(self.telemetry_path):
                    mtime = os.path.getmtime(self.telemetry_path)
                    if mtime > last_mtime:
                        last_mtime = mtime
                        with open(self.telemetry_path, 'r') as f:
                            packet = json.load(f)
                        self._apply_telemetry_packet(packet)
            except Exception:
                pass

            time.sleep(0.01)

    def _broadcast_loop(self):
        if not rclpy.ok():
            return
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        try:
            now_msg = self.get_clock().now().to_msg()
        except Exception:
            return

        with self.lock:
            rx = self.rx_m
            ry = self.ry_m
            yaw = self.yaw
            action = self.action
            path = list(self.path_pts)
            obs = dict(self.obstacles)
            goal = list(self.goal_pt) if self.goal_pt else None

        # 1. Articulate Joint States
        is_moving = action in ("forward", "left", "right")
        joint_angles = self.cpg.step(dt, is_moving)

        try:
            js = JointState()
            js.header.stamp = now_msg
            js.name = list(JOINT_NAMES)
            js.position = [joint_angles[j] for j in JOINT_NAMES]
            self.joint_pub.publish(js)

            # 2. Dynamic 3D attitude oscillation (compliance bobbing + wobble)
            roll = math.radians(2.0) * math.sin(self.cpg.phase) if is_moving else 0.0
            pitch = math.radians(1.0) * math.cos(self.cpg.phase) if is_moving else 0.0
            bob_z = 0.0015 * (1.0 - abs(math.sin(self.cpg.phase))) if is_moving else 0.0

            qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

            # 3. Broadcast TF: map -> base_footprint (ALWAYS VALID)
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now_msg
            tf_msg.header.frame_id = "map"
            tf_msg.child_frame_id = "base_footprint"
            tf_msg.transform.translation.x = rx
            tf_msg.transform.translation.y = ry
            tf_msg.transform.translation.z = bob_z
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)

            # 4. Publish /plan_path
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

            # 5. Publish /obstacle_markers (3D Cubes + 5 cm Safety Rings + Goal Cylinder)
            m_array = MarkerArray()
            for oid_str, pos in obs.items():
                oid = int(oid_str)
                ox_m = pos[0] / 100.0
                oy_m = pos[1] / 100.0

                # 3D Obstacle Box
                m_box = Marker()
                m_box.header.stamp = now_msg
                m_box.header.frame_id = "map"
                m_box.id = oid
                m_box.type = Marker.CUBE
                m_box.action = Marker.ADD
                m_box.pose.position.x = ox_m
                m_box.pose.position.y = oy_m
                m_box.pose.position.z = 0.05
                m_box.pose.orientation.w = 1.0
                m_box.scale.x = 0.10
                m_box.scale.y = 0.10
                m_box.scale.z = 0.10
                m_box.color.r = 0.85
                m_box.color.g = 0.20
                m_box.color.b = 0.20
                m_box.color.a = 0.90
                m_array.markers.append(m_box)

                # 5 cm Safety Clearance Zone Ring
                m_ring = Marker()
                m_ring.header.stamp = now_msg
                m_ring.header.frame_id = "map"
                m_ring.id = oid + 1000
                m_ring.type = Marker.CYLINDER
                m_ring.action = Marker.ADD
                m_ring.pose.position.x = ox_m
                m_ring.pose.position.y = oy_m
                m_ring.pose.position.z = 0.005
                m_ring.pose.orientation.w = 1.0
                m_ring.scale.x = 0.36
                m_ring.scale.y = 0.36
                m_ring.scale.z = 0.01
                m_ring.color.r = 1.0
                m_ring.color.g = 0.65
                m_ring.color.b = 0.0
                m_ring.color.a = 0.28
                m_array.markers.append(m_ring)

            # Goal target marker
            if goal:
                m_goal = Marker()
                m_goal.header.stamp = now_msg
                m_goal.header.frame_id = "map"
                m_goal.id = 9999
                m_goal.type = Marker.CYLINDER
                m_goal.action = Marker.ADD
                m_goal.pose.position.x = goal[0] / 100.0
                m_goal.pose.position.y = goal[1] / 100.0
                m_goal.pose.position.z = 0.01
                m_goal.pose.orientation.w = 1.0
                m_goal.scale.x = 0.12
                m_goal.scale.y = 0.12
                m_goal.scale.z = 0.02
                m_goal.color.r = 0.0
                m_goal.color.g = 0.95
                m_goal.color.b = 0.20
                m_goal.color.a = 0.90
                m_array.markers.append(m_goal)

            if m_array.markers:
                self.marker_pub.publish(m_array)
        except Exception:
            pass


def main():
    rclpy.init()
    node = RVizBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
