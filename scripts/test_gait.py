import math
import numpy as np

# Leg geometric parameters (meters)
# Thigh link length (hip axis to knee axis)
L1 = 0.04159
# Shin link length (knee axis to foot center)
L2 = 0.04317
# Foot radius
R_FOOT = 0.007

# Canonical stand servo angles in degrees
STAND_ANGLES_DEG = {
    'R1': 135.0, 'R2': 45.0,  'L1': 45.0,  'L2': 135.0,
    'R4': 0.0,   'R3': 180.0, 'L3': 0.0,   'L4': 180.0
}

class SesameLegIK:
    """
    Analytical Inverse Kinematics for a 2-DOF Sesame leg.
    Computes hip servo angle and knee servo angle to position foot at (x, y, z)
    relative to hip joint origin.
    """
    def __init__(self, leg_name: str):
        self.name = leg_name
        self.is_front = '1' in leg_name or '3' in leg_name
        self.is_left = 'L' in leg_name

    def solve_ik(self, dx_m: float, dz_m: float) -> tuple[float, float]:
        """
        Given desired foot displacement (dx forward, dz down) relative to stand foot position:
        Returns (hip_deg, knee_deg).
        """
        # Nominal stand angles
        if self.name == 'FR': # R1 (hip), R3 (knee)
            nominal_hip = 135.0
            nominal_knee = 180.0
            # Hip sweep: positive dx decreases R1 angle
            hip_deg = nominal_hip - (dx_m / 0.04) * 35.0
            # Knee lift: positive dz (upward) decreases R3 angle
            knee_deg = nominal_knee - (dz_m / 0.02) * 45.0
        elif self.name == 'RR': # R2 (hip), R4 (knee)
            nominal_hip = 45.0
            nominal_knee = 0.0
            hip_deg = nominal_hip + (dx_m / 0.04) * 35.0
            knee_deg = nominal_knee + (dz_m / 0.02) * 45.0
        elif self.name == 'FL': # L1 (hip), L3 (knee)
            nominal_hip = 45.0
            nominal_knee = 0.0
            hip_deg = nominal_hip + (dx_m / 0.04) * 35.0
            knee_deg = nominal_knee + (dz_m / 0.02) * 45.0
        else: # RL: L2 (hip), L4 (knee)
            nominal_hip = 135.0
            nominal_knee = 180.0
            hip_deg = nominal_hip - (dx_m / 0.04) * 35.0
            knee_deg = nominal_knee - (dz_m / 0.02) * 45.0

        return hip_deg, knee_deg


class SesameGaitGenerator:
    """
    Diagonal Trot Gait Generator.
    Coordinates all 4 legs to generate walking and turning joint trajectories.
    """
    def __init__(self, step_frequency_hz: float = 2.0, step_length_m: float = 0.035, step_height_m: float = 0.015):
        self.freq = step_frequency_hz
        self.step_len = step_length_m
        self.step_h = step_height_m
        self.ik_solvers = {
            'FR': SesameLegIK('FR'),
            'RR': SesameLegIK('RR'),
            'FL': SesameLegIK('FL'),
            'RL': SesameLegIK('RL')
        }

    def compute_joint_angles(self, v_linear: float, w_angular: float, phase: float) -> dict[str, float]:
        """
        Computes 8 servo angles in degrees for the given velocities and cycle phase (0 to 2*pi).
        """
        # If robot is stationary, return nominal standing angles
        speed = math.hypot(v_linear, w_angular * 0.05)
        if speed < 0.005:
            return dict(STAND_ANGLES_DEG)

        scale = min(1.0, speed / 0.15)
        step_l = self.step_len * scale
        step_h = self.step_h * scale

        # Differential stride for turning
        track_w = 0.09 # meters
        v_r = v_linear - w_angular * (track_w / 2.0)
        v_l = v_linear + w_angular * (track_w / 2.0)

        # Diagonal leg pairs:
        # Pair A: FR (Right Front) & RL (Left Rear) -> phase offset = 0
        # Pair B: FL (Left Front) & RR (Right Rear) -> phase offset = pi
        leg_phases = {
            'FR': phase,
            'RL': phase,
            'FL': (phase + math.pi) % (2.0 * math.pi),
            'RR': (phase + math.pi) % (2.0 * math.pi)
        }

        leg_strides = {
            'FR': step_l * (v_r / (abs(v_linear) + 1e-5)),
            'RR': step_l * (v_r / (abs(v_linear) + 1e-5)),
            'FL': step_l * (v_l / (abs(v_linear) + 1e-5)),
            'RL': step_l * (v_l / (abs(v_linear) + 1e-5))
        }

        angles = {}
        for leg, p in leg_phases.items():
            stride = leg_strides[leg]
            # Swing Phase: p in [0, pi]
            if p < math.pi:
                norm_p = p / math.pi
                # Foot lifts up and swings forward
                dx = -stride / 2.0 + stride * norm_p
                # Cycloid or sine lift
                dz = step_h * math.sin(norm_p * math.pi)
            else: # Stance Phase: p in [pi, 2*pi]
                norm_p = (p - math.pi) / math.pi
                # Foot on ground, pushing backward
                dx = stride / 2.0 - stride * norm_p
                dz = 0.0 # Planted on ground

            hip_deg, knee_deg = self.ik_solvers[leg].solve_ik(dx, dz)

            if leg == 'FR':
                angles['R1'] = hip_deg
                angles['R3'] = knee_deg
            elif leg == 'RR':
                angles['R2'] = hip_deg
                angles['R4'] = knee_deg
            elif leg == 'FL':
                angles['L1'] = hip_deg
                angles['L3'] = knee_deg
            elif leg == 'RL':
                angles['L2'] = hip_deg
                angles['L4'] = knee_deg

        return angles

# Quick test
gait = SesameGaitGenerator()
print("Stand angles:", gait.compute_joint_angles(0.0, 0.0, 0.0))
print("Walk t=0 (Swing Pair A):", gait.compute_joint_angles(0.15, 0.0, 0.0))
print("Walk t=pi/2 (Mid-Swing):", gait.compute_joint_angles(0.15, 0.0, math.pi/2))
print("Walk t=pi (Touchdown):", gait.compute_joint_angles(0.15, 0.0, math.pi))
