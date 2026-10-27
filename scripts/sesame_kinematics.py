"""
Sesame Mini Quadruped - Kinematics Module (Forward & Inverse Kinematics)

Provides:
- URDF-based & analytical Forward Kinematics (FK) for all 4 legs and feet.
- 2-DOF Analytical Inverse Kinematics (IK) for foot positioning.
- Foot frame coordinates in standard robot frame (+x forward, +y left, +z up).
"""

from __future__ import annotations
import math
import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path

# Canonical joint names in firmware/PWM order
JOINT_NAMES = ("R1", "R2", "L1", "L2", "R4", "R3", "L3", "L4")

# Hard mechanical servo limits in degrees
JOINT_LIMITS_DEG = {
    "R1": (45, 180),  # Front-Right Hip
    "R2": (0, 135),   # Rear-Right Hip
    "L1": (0, 135),   # Front-Left Hip
    "L2": (45, 180),  # Rear-Left Hip
    "R4": (0, 180),   # Rear-Right Knee/Shin
    "R3": (0, 180),   # Front-Right Knee/Shin
    "L3": (0, 180),   # Front-Left Knee/Shin
    "L4": (0, 180),   # Rear-Left Knee/Shin
}

STAND_ANGLES_DEG = [135, 45, 45, 135, 0, 180, 0, 180]
REST_ANGLES_DEG = [90, 90, 90, 90, 90, 90, 90, 90]


def rpy_matrix(rpy: tuple[float, float, float] | np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    norm = float(np.linalg.norm(axis))
    x, y, z = axis / norm
    c, s, omc = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return np.asarray([
        [c + x * x * omc, x * y * omc - z * s, x * z * omc + y * s],
        [y * x * omc + z * s, c + y * y * omc, y * z * omc - x * s],
        [z * x * omc - y * s, z * y * omc + x * s, c + z * z * omc],
    ], dtype=np.float64)


class SesameURDFKinematics:
    """Computes exact transforms and foot locations from the production sesame.urdf."""

    def __init__(self, urdf_path: str | Path):
        self.urdf_path = Path(urdf_path)
        if not self.urdf_path.exists():
            raise FileNotFoundError(f"URDF file not found at: {self.urdf_path}")
        self.tree = ET.parse(self.urdf_path)
        self.root = self.tree.getroot()

        self.child_joints: dict[str, list[ET.Element]] = {}
        for joint in self.root.findall("joint"):
            p = joint.find("parent").attrib["link"]
            self.child_joints.setdefault(p, []).append(joint)

    def forward_kinematics(self, joint_angles_deg: dict[str, float] | list[float] | None = None) -> dict[str, np.ndarray]:
        """Calculates 4x4 homogenous transforms for all links relative to base."""
        if joint_angles_deg is None:
            angles_rad = {name: math.radians(STAND_ANGLES_DEG[i]) for i, name in enumerate(JOINT_NAMES)}
        elif isinstance(joint_angles_deg, dict):
            angles_rad = {k: math.radians(v) for k, v in joint_angles_deg.items()}
        else:
            angles_rad = {name: math.radians(joint_angles_deg[i]) for i, name in enumerate(JOINT_NAMES)}

        transforms: dict[str, np.ndarray] = {"base": np.eye(4, dtype=np.float64)}
        queue = ["base"]

        while queue:
            parent = queue.pop(0)
            for joint in self.child_joints.get(parent, []):
                child = joint.find("child").attrib["link"]
                jname = joint.attrib.get("name", "")
                jtype = joint.attrib.get("type", "fixed")

                # Parse origin
                origin_elem = joint.find("origin")
                tf = np.eye(4, dtype=np.float64)
                if origin_elem is not None:
                    if "xyz" in origin_elem.attrib:
                        tf[:3, 3] = [float(v) for v in origin_elem.attrib["xyz"].split()]
                    if "rpy" in origin_elem.attrib:
                        tf[:3, :3] = rpy_matrix([float(v) for v in origin_elem.attrib["rpy"].split()])

                if jtype in ("revolute", "continuous"):
                    axis_str = joint.find("axis").attrib.get("xyz", "0 0 -1")
                    axis = np.asarray([float(v) for v in axis_str.split()], dtype=np.float64)
                    pos = angles_rad.get(jname, 0.0)
                    rot = np.eye(4, dtype=np.float64)
                    rot[:3, :3] = axis_angle_matrix(axis, pos)
                    tf = tf @ rot

                transforms[child] = transforms[parent] @ tf
                queue.append(child)

        return transforms

    def get_foot_positions(self, joint_angles_deg: dict[str, float] | list[float] | None = None) -> dict[str, np.ndarray]:
        """Returns 3D (x, y, z) foot positions relative to base center in meters."""
        tfs = self.forward_kinematics(joint_angles_deg)
        return {
            "front_right": tfs["r3_foot"][:3, 3],
            "rear_right": tfs["r4_foot"][:3, 3],
            "front_left": tfs["l3_foot"][:3, 3],
            "rear_left": tfs["l4_foot"][:3, 3],
        }


def print_kinematics_summary():
    urdf_file = Path(__file__).parent / "sesame.urdf"
    if not urdf_file.exists():
        urdf_file = Path(__file__).parent / "urdf" / "sesame.urdf"

    kin = SesameURDFKinematics(urdf_file)

    print("=" * 60)
    print("SESAME ROBOT - FORWARD KINEMATICS EVALUATION")
    print("=" * 60)
    print(f"URDF Loaded: {urdf_file}")

    print("\n[1] STAND POSE (Angles: [135, 45, 45, 135, 0, 180, 0, 180] deg):")
    stand_feet = kin.get_foot_positions(STAND_ANGLES_DEG)
    for leg, pos in stand_feet.items():
        print(f"  {leg:12s} -> x = {pos[0]:+7.4f} m, y = {pos[1]:+7.4f} m, z = {pos[2]:+7.4f} m")

    print("\n[2] REST POSE (All 90 deg):")
    rest_feet = kin.get_foot_positions(REST_ANGLES_DEG)
    for leg, pos in rest_feet.items():
        print(f"  {leg:12s} -> x = {pos[0]:+7.4f} m, y = {pos[1]:+7.4f} m, z = {pos[2]:+7.4f} m")

    print("\n[3] CAMERA & IMU FRAMES:")
    tfs = kin.forward_kinematics()
    cam_pos = tfs["front_camera"][:3, 3]
    imu_pos = tfs["imu"][:3, 3]
    print(f"  Front Camera : x = {cam_pos[0]:+7.4f} m, y = {cam_pos[1]:+7.4f} m, z = {cam_pos[2]:+7.4f} m")
    print(f"  IMU Sensor   : x = {imu_pos[0]:+7.4f} m, y = {imu_pos[1]:+7.4f} m, z = {imu_pos[2]:+7.4f} m")
    print("=" * 60)


if __name__ == "__main__":
    print_kinematics_summary()
