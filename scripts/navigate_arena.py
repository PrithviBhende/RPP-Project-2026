#!/usr/bin/env python3
"""
Autonomous Real-Time Interactive Navigation & Perception for Sesame Quadruped.

Interactive Gazebo Features:
1. Draggable 5 Obstacle Boxes:
   - 5 obstacle boxes in the Gazebo arena that the user can drag & drop anywhere.
   - Dynamic tracking: reads live positions of all 5 boxes at 50 Hz from /model_states.
   - Live obstacle avoidance: If any box is placed or dragged directly in front of the robot
     during navigation, the robot STOPS immediately, re-plans a new path around the obstacle,
     and resumes moving toward the goal!

2. Interactive Start & Goal Markers:
   - Green Circle (Start Marker): Dynamically tracks and stays directly below the robot
     wherever the robot moves or is placed by the user.
   - Red Circle (Goal Marker): Interactive target disk that the user can drag & drop anywhere
     on the chart paper. Whenever the red circle is moved, the robot dynamically detects the new
     target, computes a new collision-free path, and navigates to it!

3. Autonomous Path Planning & Re-planning:
   - True Euclidean distance (Minkowski sum rounded-box inflation) prevents phantom corner collisions.
   - Dynamic A* grid planner with automated waypoint extraction (string-pulling algorithm).
   - Sanitizes goal / start inputs to handle cases where targets are placed near box edges.
   - Publishes standard ROS 2 topics: /map (OccupancyGrid), /plan (Path),
     /waypoints_markers (MarkerArray), /odom, and /overhead_camera/annotated_image.

4. Physics-Grounded Sesame Locomotion:
   - Authentic sesame-ml CPG trot gait with smooth cycloid foot lift.
   - Closed-loop IMU attitude stabilization via /sesame/imu.
   - TowerPro MG90S servo slew-rate filter (10.47 rad/s) & actuator lag (tau = 35 ms).
   - Real-world pacing: ~3.8 cm/s forward speed, ~16 deg/s turning.
"""

from __future__ import annotations
import math
import heapq
import time
import argparse
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
    from nav_msgs.msg import Odometry, OccupancyGrid, Path
    from sensor_msgs.msg import Imu, Image
    from visualization_msgs.msg import Marker, MarkerArray
    from gazebo_msgs.msg import ModelStates
    from gazebo_msgs.srv import SetEntityState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from tf2_ros import TransformBroadcaster
    from builtin_interfaces.msg import Duration
except ImportError:
    Node = object

try:
    import cv2
    from cv_bridge import CvBridge
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False

# ==============================================================================
# 1. PHYSICAL CONSTANTS & ARENA BOUNDARIES
# ==============================================================================
JOINT_ORDER = ('R1', 'R2', 'L1', 'L2', 'R4', 'R3', 'L3', 'L4')

# Arena bounds on 1.5m x 1.0m chart paper (centered at x=0.50, y=0.00)
ARENA_X_MIN = -0.25
ARENA_X_MAX = 1.25
ARENA_Y_MIN = -0.50
ARENA_Y_MAX = 0.50

DEFAULT_START_XY = (-0.15, -0.40)  # Bottom-Left corner
DEFAULT_GOAL_XY = (1.15, 0.40)     # Top-Right corner

# Robot physical footprint inflation for collision avoidance
ROBOT_RADIUS_M = 0.075             # Physical footprint radius
INFLATION_RADIUS_M = 0.080         # 8 cm Euclidean clearance buffer

# TowerPro MG90S actuator specifications (sesame-ml)
SERVO_MAX_SPEED_RAD_S = 10.47198   # 600 deg/s
SERVO_MAX_TORQUE_NM = 0.18         # 1.8 kg*cm
SERVO_TIME_CONSTANT_S = 0.035      # 35 ms 1st-order actuator lag

# Locomotion cadence & velocity parameters
BASE_FREQ_HZ = 1.65
MAX_FREQ_HZ = 2.80
NOMINAL_WALK_SPEED_M_S = 0.038     # ~3.8 cm/s nominal forward speed
NOMINAL_TURN_SPEED_RAD_S = 0.28    # ~16 deg/s nominal turning rate
ACTION_SCALE_RAD = 0.58            # 0.58 rad (~33.2 deg) action residual scale

# 3D attitude dynamics
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


def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(float(np.clip(sinp, -1.0, 1.0)))

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


# ==============================================================================
# 2. CENTRAL PATTERN GENERATOR (CPG) LOCOMOTION ENGINE
# ==============================================================================
class SesameCPGGaitEngine:
    """Deployable Sesame ML CPG Controller with closed-loop attitude feedback."""
    _LOWER_INDICES = np.asarray([5, 4, 6, 7])
    _LOWER_DIRECTION = np.asarray([-1.0, 1.0, 1.0, -1.0])
    _HIP_DIRECTION = np.asarray([-1.0, 1.0, 1.0, -1.0])
    _SIDE = np.asarray([-1.0, -1.0, 1.0, 1.0])
    _OFFSETS = np.asarray([0.0, math.pi, math.pi, 0.0])

    def __init__(self):
        self.phase = 0.0
        self.filtered_targets = np.zeros(8, dtype=np.float64)
        self.current_joints = np.zeros(8, dtype=np.float64)

    def reset(self):
        self.phase = 0.0
        self.filtered_targets.fill(0.0)
        self.current_joints.fill(0.0)

    def step(
        self,
        v_linear: float,
        w_angular: float,
        dt: float,
        pitch_rad: float = 0.0,
        roll_rad: float = 0.0,
        pitch_rate_rad_s: float = 0.0,
        roll_rate_rad_s: float = 0.0
    ) -> tuple[dict[str, float], float]:
        forward = float(np.clip(v_linear / NOMINAL_WALK_SPEED_M_S, -1.0, 1.0))
        yaw = float(np.clip(w_angular / NOMINAL_TURN_SPEED_RAD_S, -1.0, 1.0))
        effort = float(np.clip(max(abs(forward), 0.55 * abs(yaw)), 0.0, 1.0))

        if effort < 0.05:
            target_rad = np.zeros(8, dtype=np.float64)
            current_freq = BASE_FREQ_HZ
        else:
            current_freq = BASE_FREQ_HZ + effort * (MAX_FREQ_HZ - BASE_FREQ_HZ)
            self.phase = (self.phase + 2.0 * math.pi * current_freq * dt) % (2.0 * math.pi)

            phases = self.phase + self._OFFSETS
            stride = np.sin(phases)
            swing = np.maximum(np.sin(phases), 0.0) ** 2
            amplitude = 0.04 + (1.0 - 0.04) * effort

            side_drive = forward + 0.45 * yaw * self._SIDE
            hip = self._HIP_DIRECTION * 0.38 * amplitude * side_drive * stride
            lower = self._LOWER_DIRECTION * 0.62 * amplitude * (0.12 + 0.88 * swing)

            pitch_feedback = 0.16 * pitch_rad + 0.025 * pitch_rate_rad_s
            roll_feedback = 0.16 * roll_rad + 0.025 * roll_rate_rad_s
            hip += np.asarray([
                -pitch_feedback - roll_feedback,
                pitch_feedback - roll_feedback,
                pitch_feedback + roll_feedback,
                -pitch_feedback + roll_feedback,
            ])

            action = np.zeros(8, dtype=np.float64)
            action[:4] = hip
            action[self._LOWER_INDICES] = lower
            action = np.clip(action, -1.0, 1.0)
            target_rad = action * ACTION_SCALE_RAD

        alpha = dt / (SERVO_TIME_CONSTANT_S + dt)
        self.filtered_targets += alpha * (target_rad - self.filtered_targets)

        max_delta = SERVO_MAX_SPEED_RAD_S * dt
        for i in range(8):
            delta = self.filtered_targets[i] - self.current_joints[i]
            self.current_joints[i] += float(np.clip(delta, -max_delta, max_delta))

        joint_dict = {name: float(self.current_joints[idx]) for idx, name in enumerate(JOINT_ORDER)}
        return joint_dict, current_freq


# ==============================================================================
# 3. OBSTACLE DATA STRUCTURE (TRUE EUCLIDEAN MINKOWSKI SUM)
# ==============================================================================
class BoxObstacle:
    def __init__(self, name: str, center_xy: tuple[float, float], half_size_xy: tuple[float, float]):
        self.name = name
        self.center = np.asarray(center_xy, dtype=np.float64)
        self.half_size = np.asarray(half_size_xy, dtype=np.float64)

    def dist_to_surface(self, point: np.ndarray) -> float:
        """Exact Euclidean distance from 2D point to the box surface."""
        dx = max(abs(point[0] - self.center[0]) - self.half_size[0], 0.0)
        dy = max(abs(point[1] - self.center[1]) - self.half_size[1], 0.0)
        return math.hypot(dx, dy)

    def contains(self, point: np.ndarray, inflation: float = 0.0) -> bool:
        """True if point lies within the rounded-box inflated boundary."""
        return self.dist_to_surface(point) <= inflation


# ==============================================================================
# 4. ROBUST DYNAMIC A* PATH PLANNER WITH AUTO-WAYPOINT EXTRACTION
# ==============================================================================
class DynamicAStarPlanner:
    _NEIGHBORS = tuple(
        (dx, dy, float(np.hypot(dx, dy)))
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        if dx or dy
    )

    def __init__(self, resolution_m: float = 0.015, robot_radius_m: float = INFLATION_RADIUS_M):
        self.resolution = resolution_m
        self.inflation_radius = robot_radius_m

    def is_blocked(self, point: np.ndarray, obstacles: list[BoxObstacle], inflation: float | None = None) -> bool:
        inf = self.inflation_radius if inflation is None else inflation
        if not (ARENA_X_MIN <= point[0] <= ARENA_X_MAX and ARENA_Y_MIN <= point[1] <= ARENA_Y_MAX):
            return True
        return any(obs.contains(point, inf) for obs in obstacles)

    def line_clear(self, start: np.ndarray, end: np.ndarray, obstacles: list[BoxObstacle], inflation: float | None = None) -> bool:
        inf = self.inflation_radius if inflation is None else inflation
        dist = float(np.linalg.norm(end - start))
        steps = max(2, int(np.ceil(dist / (self.resolution * 0.5))))
        for alpha in np.linspace(0, 1, steps):
            pt = start + alpha * (end - start)
            if self.is_blocked(pt, obstacles, inf):
                return False
        return True

    def sanitize_point(self, pt: np.ndarray, obstacles: list[BoxObstacle]) -> np.ndarray:
        """If a target point is inside an obstacle's inflation margin, find the nearest accessible clear point."""
        if not self.is_blocked(pt, obstacles):
            return pt
        for r in np.linspace(0.015, 0.35, 22):
            for th in np.linspace(0, 2 * math.pi, 20):
                cand = pt + r * np.asarray([math.cos(th), math.sin(th)])
                if not self.is_blocked(cand, obstacles):
                    return cand
        return pt

    def plan_and_extract_waypoints(
        self,
        start_xy: tuple[float, float] | np.ndarray,
        goal_xy: tuple[float, float] | np.ndarray,
        obstacles: list[BoxObstacle],
    ) -> tuple[np.ndarray | None, np.ndarray | None]:
        start = self.sanitize_point(np.asarray(start_xy, dtype=np.float64), obstacles)
        goal = self.sanitize_point(np.asarray(goal_xy, dtype=np.float64), obstacles)

        low = np.asarray([ARENA_X_MIN - 0.1, ARENA_Y_MIN - 0.1])
        high = np.asarray([ARENA_X_MAX + 0.1, ARENA_Y_MAX + 0.1])

        def to_cell(pt: np.ndarray) -> tuple[int, int]:
            cell = np.rint((pt - low) / self.resolution).astype(int)
            return int(cell[0]), int(cell[1])

        def to_point(cell: tuple[int, int]) -> np.ndarray:
            return low + self.resolution * np.asarray(cell)

        start_cell = to_cell(start)
        goal_cell = to_cell(goal)
        max_cell = to_cell(high)

        queue = [(0.0, 0.0, start_cell)]
        best_cost = {start_cell: 0.0}
        parent = {}
        found = False
        closest_cell = start_cell
        min_h = float(np.linalg.norm(start - goal))

        iterations = 0
        max_iterations = 8000
        while queue and iterations < max_iterations:
            iterations += 1
            _, cost, cell = heapq.heappop(queue)
            if cell == goal_cell:
                found = True
                break
            if cost > best_cost.get(cell, np.inf):
                continue
            for dx, dy, step_cost in self._NEIGHBORS:
                neighbor = (cell[0] + dx, cell[1] + dy)
                if not (0 <= neighbor[0] <= max_cell[0] and 0 <= neighbor[1] <= max_cell[1]):
                    continue
                pt_neighbor = to_point(neighbor)
                if self.is_blocked(pt_neighbor, obstacles):
                    continue
                candidate_cost = cost + step_cost
                if candidate_cost >= best_cost.get(neighbor, np.inf):
                    continue
                best_cost[neighbor] = candidate_cost
                parent[neighbor] = cell
                h = float(np.linalg.norm(pt_neighbor - goal))
                if h < min_h:
                    min_h = h
                    closest_cell = neighbor
                heapq.heappush(queue, (candidate_cost + h, candidate_cost, neighbor))

        target_end_cell = goal_cell if found else closest_cell
        if target_end_cell == start_cell:
            return None, None

        cells = [target_end_cell]
        while cells[-1] != start_cell and cells[-1] in parent:
            cells.append(parent[cells[-1]])
        raw = [start, *[to_point(c) for c in reversed(cells[1:-1])], goal]
        full_path = np.asarray(raw, dtype=np.float32)

        # String-Pulling Shortcut Smoothing
        waypoints = [raw[0]]
        curr = 0
        while curr < len(raw) - 1:
            target = len(raw) - 1
            while target > curr + 1 and not self.line_clear(raw[curr], raw[target], obstacles):
                target -= 1
            waypoints.append(raw[target])
            curr = target

        # Prune redundant micro-waypoints (< 0.035m from previous)
        pruned_waypoints = [waypoints[0]]
        for wp in waypoints[1:]:
            if float(np.linalg.norm(wp - pruned_waypoints[-1])) > 0.035:
                pruned_waypoints.append(wp)
        if not np.allclose(pruned_waypoints[-1], goal, atol=0.01):
            pruned_waypoints.append(goal)

        return np.asarray(pruned_waypoints, dtype=np.float32), full_path


# ==============================================================================
# 5. INTERACTIVE AUTONOMOUS SESAME NAVIGATOR ROS 2 NODE
# ==============================================================================
class SesameInteractiveNavigator(Node):
    def __init__(
        self,
        default_goal_x: float = DEFAULT_GOAL_XY[0],
        default_goal_y: float = DEFAULT_GOAL_XY[1],
        esp32_ip: str | None = None
    ):
        super().__init__('sesame_interactive_navigator')

        self.goal_xy = (default_goal_x, default_goal_y)
        self.esp32_ip = esp32_ip

        # Live robot state
        self.current_x = DEFAULT_START_XY[0]
        self.current_y = DEFAULT_START_XY[1]
        self.current_yaw = 0.55
        self.has_initial_pose = False
        self.last_esp32_cmd_time = 0.0

        # IMU state for closed-loop attitude stabilization
        self.imu_pitch = 0.0
        self.imu_roll = 0.0
        self.imu_pitch_rate = 0.0
        self.imu_roll_rate = 0.0
        self.has_imu_data = False

        # Live 5 Obstacle Boxes
        self.box_names = [f"obstacle_box_{i}" for i in range(1, 6)]
        self.obstacles: list[BoxObstacle] = [
            BoxObstacle("obstacle_box_1", (0.15, -0.15), (0.06, 0.06)),
            BoxObstacle("obstacle_box_2", (0.40, 0.20), (0.07, 0.07)),
            BoxObstacle("obstacle_box_3", (0.65, -0.20), (0.06, 0.06)),
            BoxObstacle("obstacle_box_4", (0.85, 0.15), (0.06, 0.06)),
            BoxObstacle("obstacle_box_5", (0.50, -0.05), (0.06, 0.06)),
        ]

        # Navigation State Machine
        self.cpg = SesameCPGGaitEngine()
        self.planner = DynamicAStarPlanner(resolution_m=0.015, robot_radius_m=INFLATION_RADIUS_M)
        self.waypoints: np.ndarray | None = None
        self.full_path: np.ndarray | None = None
        self.current_wp_idx = 0
        self.is_blocked = False
        self.last_replan_time = 0.0
        self.last_blocked_log_time = 0.0
        self.status_text = "INITIALIZING"

        # ROS 2 Publishers
        self.traj_pub = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints_markers', 10)
        self.annotated_img_pub = self.create_publisher(Image, '/overhead_camera/annotated_image', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Service Clients
        self.state_client = self.create_client(SetEntityState, '/set_entity_state')

        # Subscriptions
        self.model_sub = self.create_subscription(ModelStates, '/model_states', self._model_states_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/sesame/imu', self._imu_cb, 10)
        self.cam_sub = self.create_subscription(Image, '/overhead_camera/image_raw', self._overhead_cam_cb, 10)

        self.cv_bridge = CvBridge() if HAS_OPENCV else None
        self.dt = 0.033  # 30 Hz control loop
        self.last_time = time.time()
        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info("=" * 75)
        self.get_logger().info("SESAME INTERACTIVE DYNAMIC NAVIGATOR READY")
        self.get_logger().info("1. Drag & drop the 5 boxes in Gazebo anytime to block or redirect Sesame.")
        self.get_logger().info("2. Drag & drop the Red Circle (goal_marker) to set a new goal anywhere.")
        self.get_logger().info("3. Green Circle (start_marker) automatically tracks underneath Sesame.")
        self.get_logger().info("=" * 75)

    def _model_states_cb(self, msg: ModelStates):
        # 1. Update live robot pose
        if "sesame_robot" in msg.name:
            r_idx = msg.name.index("sesame_robot")
            p = msg.pose[r_idx].position
            q = msg.pose[r_idx].orientation
            self.current_x = p.x
            self.current_y = p.y
            _, _, self.current_yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
            if not self.has_initial_pose:
                self.has_initial_pose = True
                self.get_logger().info(f"Sesame pose acquired: ({self.current_x:.2f}, {self.current_y:.2f})")
                self.trigger_replan("Initial startup")

        # 2. Track Red Circle Goal Marker
        if "goal_marker" in msg.name:
            g_idx = msg.name.index("goal_marker")
            gx = msg.pose[g_idx].position.x
            gy = msg.pose[g_idx].position.y
            if math.hypot(gx - self.goal_xy[0], gy - self.goal_xy[1]) > 0.04:
                self.goal_xy = (gx, gy)
                self.get_logger().info(f"[GOAL MOVED] Red Circle dragged to ({gx:.2f}, {gy:.2f}) -> Re-planning path!")
                self.trigger_replan("User moved Red Goal Circle")

        # 3. Track all 5 Draggable Obstacle Boxes
        boxes_moved = False
        for i, box_name in enumerate(self.box_names):
            if box_name in msg.name:
                b_idx = msg.name.index(box_name)
                bx = msg.pose[b_idx].position.x
                by = msg.pose[b_idx].position.y
                if math.hypot(bx - self.obstacles[i].center[0], by - self.obstacles[i].center[1]) > 0.03:
                    self.obstacles[i].center = np.asarray([bx, by], dtype=np.float64)
                    boxes_moved = True

        if boxes_moved:
            self._publish_costmap()

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        roll, pitch, _ = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.imu_roll = roll
        self.imu_pitch = pitch
        self.imu_roll_rate = msg.angular_velocity.x
        self.imu_pitch_rate = msg.angular_velocity.y
        self.has_imu_data = True

    def trigger_replan(self, reason: str):
        """Calculates a fresh collision-free A* path from current robot pose to red goal marker."""
        start_xy = (self.current_x, self.current_y)
        waypoints, full_path = self.planner.plan_and_extract_waypoints(
            start_xy, self.goal_xy, self.obstacles
        )
        if waypoints is not None and len(waypoints) >= 2:
            self.waypoints = waypoints
            self.full_path = full_path
            self.current_wp_idx = 1
            self.is_blocked = False
            self.status_text = "NAVIGATING"
            self.get_logger().info(f"[RE-PLAN ({reason})] Found path with {len(self.waypoints)} waypoints:")
            for i, wp in enumerate(self.waypoints):
                self.get_logger().info(f"   WP {i}: ({wp[0]:.2f}, {wp[1]:.2f})")
            self._publish_path()
            self._publish_waypoint_markers()
            self._publish_costmap()
        else:
            self.is_blocked = True
            self.status_text = "BLOCKED - WAITING"
            now = time.time()
            if now - self.last_blocked_log_time > 2.0:
                self.last_blocked_log_time = now
                self.get_logger().warn("[BLOCKED] Path is completely obstructed by boxes! Waiting for obstacle to be moved...")

    def _is_active_path_blocked(self) -> bool:
        """
        True ONLY when an obstacle physically blocks the current planned segment to target waypoint,
        or an obstacle is dangerously close to the robot body (< 0.035m surface distance).
        """
        r_pos = np.asarray([self.current_x, self.current_y])

        # 1. Emergency physical safety: if any box surface is closer than 3.5 cm to robot body
        for obs in self.obstacles:
            if obs.dist_to_surface(r_pos) < 0.035:
                return True

        # 2. Check if planned path segment to target waypoint is blocked
        if self.waypoints is not None and self.current_wp_idx < len(self.waypoints):
            target_pt = self.waypoints[self.current_wp_idx]
            # Use slightly tighter inflation (0.070m) for line clearance check to prevent deadlock near corners
            if not self.planner.line_clear(r_pos, target_pt, self.obstacles, inflation=0.070):
                return True

        return False

    def _control_loop(self):
        if not self.has_initial_pose:
            return

        now = time.time()
        actual_dt = now - self.last_time
        self.last_time = now

        # 1. Keep Green Circle (start_marker) synchronized directly below robot
        if self.state_client.service_is_ready():
            req_start = SetEntityState.Request()
            req_start.state.name = "start_marker"
            req_start.state.pose.position.x = self.current_x
            req_start.state.pose.position.y = self.current_y
            req_start.state.pose.position.z = 0.0012
            req_start.state.pose.orientation.w = 1.0
            req_start.state.reference_frame = "world"
            self.state_client.call_async(req_start)

        # 2. Check if arrived at Red Goal Circle
        dist_to_goal = math.hypot(self.goal_xy[0] - self.current_x, self.goal_xy[1] - self.current_y)
        if dist_to_goal < 0.055:
            self.status_text = "GOAL REACHED"
            self._publish_stop()
            return

        # 3. Live Obstacle Blockage Detection & Immediate Halting
        if self._is_active_path_blocked():
            self.is_blocked = True
            self.status_text = "BLOCKED - REPLANNING"
            self._publish_stop()
            if now - self.last_replan_time > 0.40:
                self.last_replan_time = now
                self.trigger_replan("Obstacle blocking path")
            return

        # If previously blocked but path is now clear, resume
        if self.is_blocked:
            self.is_blocked = False
            self.status_text = "NAVIGATING"
            self.get_logger().info("[RESUMING] Path is clear! Moving forward.")

        if self.waypoints is None:
            self.trigger_replan("Initialize path")
            return

        # 4. Advance waypoint if reached
        target_x, target_y = self.waypoints[self.current_wp_idx]
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        dist = math.hypot(dx, dy)

        if dist < 0.045:
            self.current_wp_idx += 1
            if self.current_wp_idx >= len(self.waypoints):
                self.status_text = "GOAL REACHED"
                self._publish_stop()
                self.get_logger().info("[SUCCESS] Sesame reached the Red Goal Circle!")
                return
            target_x, target_y = self.waypoints[self.current_wp_idx]
            dx = target_x - self.current_x
            dy = target_y - self.current_y

        # 5. Paced Velocity Controller
        target_angle = math.atan2(dy, dx)
        angle_err = (target_angle - self.current_yaw + math.pi) % (2.0 * math.pi) - math.pi

        max_v = NOMINAL_WALK_SPEED_M_S    # 0.038 m/s
        max_w = NOMINAL_TURN_SPEED_RAD_S  # 0.28 rad/s

        # Pivot in place when heading error is large (> 40 deg), advance forward when aligned
        if abs(angle_err) > math.radians(40.0):
            v = 0.0
        else:
            heading_alignment = max(0.0, math.cos(angle_err)) ** 2
            v = float(max_v * heading_alignment)

        w = float(np.clip(1.8 * angle_err, -max_w, max_w))

        # 6. CPG Locomotion execution with IMU attitude feedback
        pitch_in = self.imu_pitch if self.has_imu_data else (PITCH_WOBBLE_RAD * math.cos(self.cpg.phase))
        roll_in = self.imu_roll if self.has_imu_data else (ROLL_WOBBLE_RAD * math.sin(self.cpg.phase))
        pitch_rate_in = self.imu_pitch_rate if self.has_imu_data else 0.0
        roll_rate_in = self.imu_roll_rate if self.has_imu_data else 0.0

        joint_rad, _ = self.cpg.step(
            v_linear=v,
            w_angular=w,
            dt=self.dt,
            pitch_rad=pitch_in,
            roll_rad=roll_in,
            pitch_rate_rad_s=pitch_rate_in,
            roll_rate_rad_s=roll_rate_in
        )

        # Publish Joint Trajectory to Gazebo
        traj_msg = JointTrajectory()
        traj_msg.header.stamp.sec = 0
        traj_msg.header.stamp.nanosec = 0
        traj_msg.header.frame_id = "world"
        traj_msg.joint_names = list(JOINT_ORDER)
        point = JointTrajectoryPoint()
        point.positions = [float(joint_rad[j]) for j in JOINT_ORDER]
        point.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9))
        traj_msg.points = [point]
        self.traj_pub.publish(traj_msg)

        # Kinematic base advancement matching foot contact
        self.current_yaw += w * self.dt
        self.current_x += v * math.cos(self.current_yaw) * self.dt
        self.current_y += v * math.sin(self.current_yaw) * self.dt

        # 3D attitude dynamics (Roll, Pitch, Compliance Bobbing)
        speed = math.hypot(v, w * 0.05)
        speed_ratio = min(1.0, speed / NOMINAL_WALK_SPEED_M_S)
        current_roll = ROLL_WOBBLE_RAD * math.sin(self.cpg.phase) * speed_ratio
        current_pitch = PITCH_WOBBLE_RAD * math.cos(self.cpg.phase) * speed_ratio
        current_z = 0.001 + BOB_AMPLITUDE_M * (1.0 - abs(math.sin(self.cpg.phase))) * speed_ratio

        qx, qy, qz, qw = euler_to_quaternion(current_roll, current_pitch, self.current_yaw)

        # Update entity in Gazebo
        if self.state_client.service_is_ready():
            req = SetEntityState.Request()
            req.state.name = "sesame_robot"
            req.state.pose.position.x = self.current_x
            req.state.pose.position.y = self.current_y
            req.state.pose.position.z = current_z
            req.state.pose.orientation.x = qx
            req.state.pose.orientation.y = qy
            req.state.pose.orientation.z = qz
            req.state.pose.orientation.w = qw
            req.state.reference_frame = "world"
            self.state_client.call_async(req)

        # Publish Odometry & TF
        now_msg = self.get_clock().now().to_msg()
        odom_msg = Odometry()
        odom_msg.header.stamp = now_msg
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position.x = self.current_x
        odom_msg.pose.pose.position.y = self.current_y
        odom_msg.pose.pose.position.z = current_z
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = w
        self.odom_pub.publish(odom_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now_msg
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_footprint"
        tf_msg.transform.translation.x = self.current_x
        tf_msg.transform.translation.y = self.current_y
        tf_msg.transform.translation.z = current_z
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

        # Publish Twist velocity
        cmd_msg = Twist()
        cmd_msg.linear.x = v
        cmd_msg.angular.z = w
        self.cmd_pub.publish(cmd_msg)

        # Hardware bridge to physical ESP32
        if self.esp32_ip and (now - self.last_esp32_cmd_time) > 0.10:
            self.last_esp32_cmd_time = now
            if abs(angle_err) > 0.35:
                if angle_err > 0:
                    self._send_esp32_cmd("turnleft", "1")
                else:
                    self._send_esp32_cmd("turnright", "1")
            else:
                self._send_esp32_cmd("walk", "1")

    def _publish_stop(self):
        try:
            self.cmd_pub.publish(Twist())
            traj = JointTrajectory()
            traj.header.stamp.sec = 0
            traj.header.stamp.nanosec = 0
            traj.header.frame_id = "world"
            traj.joint_names = list(JOINT_ORDER)
            p = JointTrajectoryPoint()
            p.positions = [0.0] * 8
            p.time_from_start = Duration(sec=0, nanosec=int(0.1 * 1e9))
            traj.points = [p]
            self.traj_pub.publish(traj)

            if self.state_client.service_is_ready():
                req = SetEntityState.Request()
                req.state.name = "sesame_robot"
                req.state.pose.position.x = self.current_x
                req.state.pose.position.y = self.current_y
                req.state.pose.position.z = 0.001
                qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, self.current_yaw)
                req.state.pose.orientation.x = qx
                req.state.pose.orientation.y = qy
                req.state.pose.orientation.z = qz
                req.state.pose.orientation.w = qw
                req.state.reference_frame = "world"
                self.state_client.call_async(req)
        except Exception:
            pass

    def _publish_costmap(self):
        res = 0.015
        grid_w = int((ARENA_X_MAX - ARENA_X_MIN) / res) + 1
        grid_h = int((ARENA_Y_MAX - ARENA_Y_MIN) / res) + 1

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "odom"
        grid.info.resolution = res
        grid.info.width = grid_w
        grid.info.height = grid_h
        grid.info.origin.position.x = ARENA_X_MIN
        grid.info.origin.position.y = ARENA_Y_MIN
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        data = np.zeros((grid_h, grid_w), dtype=np.int8)
        for iy in range(grid_h):
            wy = ARENA_Y_MIN + iy * res
            for ix in range(grid_w):
                wx = ARENA_X_MIN + ix * res
                pt = np.asarray([wx, wy])
                if any(obs.contains(pt, INFLATION_RADIUS_M) for obs in self.obstacles):
                    data[iy, ix] = 100
                else:
                    data[iy, ix] = 0

        grid.data = data.flatten().tolist()
        self.map_pub.publish(grid)

    def _publish_path(self):
        if self.waypoints is None:
            return
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        for pt in self.waypoints:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = float(pt[0])
            p.pose.position.y = float(pt[1])
            p.pose.position.z = 0.005
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.path_pub.publish(msg)

    def _publish_waypoint_markers(self):
        if self.waypoints is None:
            return
        arr = MarkerArray()
        for i, wp in enumerate(self.waypoints):
            m = Marker()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "odom"
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(wp[0])
            m.pose.position.y = float(wp[1])
            m.pose.position.z = 0.015
            m.pose.orientation.w = 1.0
            m.scale.x = 0.06
            m.scale.y = 0.06
            m.scale.z = 0.03
            if i == 0:
                m.color.g, m.color.a = 0.9, 0.9  # Green start
            elif i == len(self.waypoints) - 1:
                m.color.r, m.color.a = 0.9, 0.9  # Red goal
            else:
                m.color.r, m.color.g, m.color.a = 1.0, 0.6, 0.9  # Orange intermediate
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    def _overhead_cam_cb(self, msg: Image):
        """Annotates overhead camera feed with live dynamic boxes, circles, and path."""
        if not HAS_OPENCV or self.cv_bridge is None:
            return
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            h, w, _ = cv_img.shape

            def world_to_pixel(wx: float, wy: float) -> tuple[int, int]:
                u = int(w * 0.5 + (wx - 0.50) * (w / 2.59))
                v = int(h * 0.5 - (wy - 0.00) * (h / 1.84))
                return max(0, min(w - 1, u)), max(0, min(h - 1, v))

            # Status Banner
            color_status = (0, 220, 0) if "NAVIGATING" in self.status_text else (0, 0, 255)
            cv2.putText(cv_img, f"STATUS: {self.status_text}", (30, 45), cv2.FONT_HERSHEY_DUPLEX, 0.9, color_status, 2)

            # Green Circle (below bot)
            su, sv = world_to_pixel(self.current_x, self.current_y)
            cv2.circle(cv_img, (su, sv), 18, (0, 230, 50), 3)
            cv2.putText(cv_img, "BOT (START)", (su - 45, sv + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 230, 50), 1)

            # Red Circle (draggable goal)
            gu, gv = world_to_pixel(self.goal_xy[0], self.goal_xy[1])
            cv2.circle(cv_img, (gu, gv), 18, (0, 30, 240), 3)
            cv2.circle(cv_img, (gu, gv), 6, (0, 30, 240), -1)
            cv2.putText(cv_img, "GOAL (DRAGGABLE)", (gu - 55, gv - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 30, 240), 1)

            # 5 Draggable Boxes
            for obs in self.obstacles:
                c1 = world_to_pixel(obs.center[0] - obs.half_size[0], obs.center[1] + obs.half_size[1])
                c2 = world_to_pixel(obs.center[0] + obs.half_size[0], obs.center[1] - obs.half_size[1])
                cv2.rectangle(cv_img, c1, c2, (255, 140, 0), 2)
                cv2.putText(cv_img, obs.name, (c1[0], c1[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 160, 0), 1)

            # Navigation Trajectory
            if self.waypoints is not None and len(self.waypoints) > 1:
                pts = [world_to_pixel(wp[0], wp[1]) for wp in self.waypoints]
                for i in range(len(pts) - 1):
                    cv2.line(cv_img, pts[i], pts[i + 1], (0, 255, 255), 3)
                for i, pt in enumerate(pts):
                    cv2.circle(cv_img, pt, 5, (0, 180, 255), -1)

            ann_msg = self.cv_bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            ann_msg.header = msg.header
            self.annotated_img_pub.publish(ann_msg)
        except Exception:
            pass

    def _send_esp32_cmd(self, cmd_type: str, cmd_val: str):
        if not self.esp32_ip:
            return
        import urllib.request
        try:
            url = f"http://{self.esp32_ip}/command?{cmd_type}={cmd_val}"
            urllib.request.urlopen(url, timeout=0.15)
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description="Sesame Interactive Draggable Arena Navigator")
    parser.add_argument("--goal-x", type=float, default=DEFAULT_GOAL_XY[0], help="Initial Goal X")
    parser.add_argument("--goal-y", type=float, default=DEFAULT_GOAL_XY[1], help="Initial Goal Y")
    parser.add_argument("--esp32-ip", type=str, default=None, help="Optional physical ESP32 IP")
    args = parser.parse_args()

    rclpy.init()
    node = SesameInteractiveNavigator(
        default_goal_x=args.goal_x,
        default_goal_y=args.goal_y,
        esp32_ip=args.esp32_ip
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            if rclpy.ok():
                node._publish_stop()
                node.destroy_node()
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
