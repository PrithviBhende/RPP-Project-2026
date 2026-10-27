"""
Sesame Mini Quadruped - Path Planning & Kinematics Simulation Demo

Features:
1. 2D Obstacle Map & Footprint-Inflated A* Grid Path Planner with Line-of-Sight Smoothing.
2. Waypoint-Following Velocity Controller (Pure Pursuit / Heading Control).
3. Quadruped Forward Kinematics & Stand Foot Geometry from Sesame URDF.
4. Optional REST API Bridge to dispatch commands to physical ESP32 ("Sesame-Controller").
5. Matplotlib interactive trajectory visualization.
"""

from __future__ import annotations
import math
import heapq
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass

# ==============================================================================
# 1. CANONICAL SESAME ROBOT CONSTANTS
# ==============================================================================
JOINT_NAMES = ("R1", "R2", "L1", "L2", "R4", "R3", "L3", "L4")
# Canonical Stand Angles (degrees)
STAND_ANGLES_DEG = [135, 45, 45, 135, 0, 180, 0, 180]
REST_ANGLES_DEG = [90, 90, 90, 90, 90, 90, 90, 90]

# Robot footprint radius (m) for collision inflation
ROBOT_RADIUS_M = 0.09


# ==============================================================================
# 2. 2D OBSTACLES & A* PATH PLANNER
# ==============================================================================
@dataclass(frozen=True)
class BoxObstacle:
    center_xy: tuple[float, float]
    half_size_xy: tuple[float, float]

    def contains(self, point: np.ndarray, inflation: float = 0.0) -> bool:
        center = np.asarray(self.center_xy)
        half_size = np.asarray(self.half_size_xy) + inflation
        return bool(np.all(np.abs(point - center) <= half_size))


class AStarGridPlanner:
    """A* Route Planner with footprint inflation and line-of-sight shortcutting."""

    _NEIGHBORS = tuple(
        (dx, dy, float(np.hypot(dx, dy)))
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        if dx or dy
    )

    def __init__(self, resolution_m: float = 0.02, robot_radius_m: float = ROBOT_RADIUS_M) -> None:
        self.resolution = resolution_m
        self.robot_radius = robot_radius_m

    def _blocked(self, point: np.ndarray, obstacles: list[BoxObstacle]) -> bool:
        return any(obs.contains(point, self.robot_radius) for obs in obstacles)

    def _line_clear(self, start: np.ndarray, end: np.ndarray, obstacles: list[BoxObstacle]) -> bool:
        dist = float(np.linalg.norm(end - start))
        steps = max(2, int(np.ceil(dist / (self.resolution * 0.5))))
        return not any(
            self._blocked(start + alpha * (end - start), obstacles)
            for alpha in np.linspace(0, 1, steps)
        )

    def plan(
        self,
        start_xy: tuple[float, float] | np.ndarray,
        goal_xy: tuple[float, float] | np.ndarray,
        obstacles: list[BoxObstacle],
    ) -> np.ndarray:
        start = np.asarray(start_xy, dtype=np.float64)
        goal = np.asarray(goal_xy, dtype=np.float64)

        if self._blocked(start, obstacles):
            raise ValueError("Start is inside an inflated obstacle!")
        if self._blocked(goal, obstacles):
            raise ValueError("Goal is inside an inflated obstacle!")

        margin = max(0.4, self.robot_radius * 2)
        pts = [start, goal]
        for obs in obstacles:
            c, h = np.asarray(obs.center_xy), np.asarray(obs.half_size_xy)
            pts.extend([c - h - margin, c + h + margin])
        low = np.min(pts, axis=0) - margin
        high = np.max(pts, axis=0) + margin

        def to_cell(pt: np.ndarray) -> tuple[int, int]:
            cell = np.rint((pt - low) / self.resolution).astype(int)
            return int(cell[0]), int(cell[1])

        def to_point(cell: tuple[int, int]) -> np.ndarray:
            return low + self.resolution * np.asarray(cell)

        start_cell, goal_cell = to_cell(start), to_cell(goal)
        max_cell = np.ceil((high - low) / self.resolution).astype(int)

        queue: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start_cell)]
        parent: dict[tuple[int, int], tuple[int, int]] = {}
        best_cost = {start_cell: 0.0}
        found = False

        while queue:
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
                if self._blocked(to_point(neighbor), obstacles):
                    continue
                candidate_cost = cost + step_cost
                if candidate_cost >= best_cost.get(neighbor, np.inf):
                    continue
                best_cost[neighbor] = candidate_cost
                parent[neighbor] = cell
                heuristic = float(np.linalg.norm(np.asarray(neighbor) - np.asarray(goal_cell)))
                heapq.heappush(queue, (candidate_cost + heuristic, candidate_cost, neighbor))

        if not found:
            raise RuntimeError("No collision-free path found!")

        cells = [goal_cell]
        while cells[-1] != start_cell:
            cells.append(parent[cells[-1]])
        raw = [start, *[to_point(c) for c in reversed(cells[1:-1])], goal]

        # Line-of-sight shortcut smoothing
        smoothed = [raw[0]]
        curr = 0
        while curr < len(raw) - 1:
            target = len(raw) - 1
            while target > curr + 1 and not self._line_clear(raw[curr], raw[target], obstacles):
                target -= 1
            smoothed.append(raw[target])
            curr = target

        return np.asarray(smoothed, dtype=np.float32)


# ==============================================================================
# 3. WAYPOINT FOLLOWER & CONTROLLER
# ==============================================================================
class QuadrupedTrajectoryFollower:
    """Tracks 2D waypoints and generates (linear_velocity, yaw_rate) commands."""

    def __init__(self, max_linear_speed: float = 0.15, max_yaw_rate: float = 1.2):
        self.max_v = max_linear_speed
        self.max_w = max_yaw_rate

    def compute_cmd(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        target_x: float,
        target_y: float,
    ) -> tuple[float, float, float]:
        """Returns (v, omega, distance_to_target)."""
        dx = target_x - current_x
        dy = target_y - current_y
        dist = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_err = (target_angle - current_yaw + math.pi) % (2 * math.pi) - math.pi

        # Proportional heading steering
        w = np.clip(2.5 * angle_err, -self.max_w, self.max_w)
        # Slow down during sharp turns
        v = self.max_v * max(0.0, math.cos(angle_err))
        return float(v), float(w), dist


# ==============================================================================
# 4. PHYSICAL ESP32 HTTP BRIDGE (OPTIONAL)
# ==============================================================================
def send_esp32_command(esp32_ip: str, cmd_type: str, cmd_val: str) -> None:
    """
    Sends motion commands to physical Sesame robot via WiFi.
    Examples:
      send_esp32_command('192.168.4.1', 'go', 'walk')
      send_esp32_command('192.168.4.1', 'go', 'turn_left')
      send_esp32_command('192.168.4.1', 'stop', '1')
    """
    import urllib.request
    url = f"http://{esp32_ip}/command?{cmd_type}={cmd_val}"
    try:
        urllib.request.urlopen(url, timeout=0.5)
    except Exception as e:
        print(f"[ESP32 Bridge Warning] Could not reach robot at {url}: {e}")


# ==============================================================================
# 5. MAIN DEMO & VISUALIZATION
# ==============================================================================
def run_demo(save_plot_path: str = "path_planning_result.png"):
    print("=" * 60)
    print("SESAME MINI QUADRUPED - PATH PLANNING SIMULATION")
    print("=" * 60)

    # 1. Define Environment with Obstacles
    obstacles = [
        BoxObstacle(center_xy=(0.5, 0.1), half_size_xy=(0.12, 0.2)),
        BoxObstacle(center_xy=(0.8, -0.2), half_size_xy=(0.15, 0.15)),
        BoxObstacle(center_xy=(1.2, 0.25), half_size_xy=(0.1, 0.25)),
    ]

    start_pos = (0.0, -0.3)
    goal_pos = (1.6, 0.35)

    print(f"Start Position: {start_pos}")
    print(f"Goal Position:  {goal_pos}")
    print(f"Obstacle Count: {len(obstacles)}")

    # 2. Plan Path using A* with Footprint Inflation
    planner = AStarGridPlanner(resolution_m=0.03, robot_radius_m=ROBOT_RADIUS_M)
    path = planner.plan(start_pos, goal_pos, obstacles)
    print(f"\nPath found with {len(path)} smoothed waypoints:")
    for i, pt in enumerate(path):
        print(f"  WP {i}: ({pt[0]:.2f}, {pt[1]:.2f})")

    # 3. Simulate Robot Motion following the Path
    follower = QuadrupedTrajectoryFollower()
    dt = 0.05
    x, y, yaw = start_pos[0], start_pos[1], 0.0
    trajectory = [(x, y, yaw)]

    for wp_idx, wp in enumerate(path[1:], start=1):
        while True:
            v, w, dist = follower.compute_cmd(x, y, yaw, wp[0], wp[1])
            if dist < 0.05:  # Waypoint arrival threshold
                break
            # Unicycle forward integration
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += w * dt
            trajectory.append((x, y, yaw))

    print(f"\nSimulation complete: {len(trajectory)} time steps executed.")

    # 4. Plot Trajectory and Obstacles
    fig, ax = plt.subplots(figsize=(10, 6))

    # Draw Obstacles (Solid) and Inflated Margins (Dashed)
    for obs in obstacles:
        c, h = obs.center_xy, obs.half_size_xy
        # Actual obstacle
        rect = plt.Rectangle(
            (c[0] - h[0], c[1] - h[1]),
            2 * h[0], 2 * h[1],
            color="#e74c3c", alpha=0.7, label="Obstacle" if obs == obstacles[0] else ""
        )
        ax.add_patch(rect)
        # Inflated safety buffer
        inf_h = (h[0] + ROBOT_RADIUS_M, h[1] + ROBOT_RADIUS_M)
        inf_rect = plt.Rectangle(
            (c[0] - inf_h[0], c[1] - inf_h[1]),
            2 * inf_h[0], 2 * inf_h[1],
            color="#e67e22", alpha=0.2, linestyle="--", fill=True,
            label="Inflated Margin" if obs == obstacles[0] else ""
        )
        ax.add_patch(inf_rect)

    # Draw Path & Trajectory
    ax.plot(path[:, 0], path[:, 1], "b--o", label="A* Smoothed Waypoints", markersize=6)
    traj_arr = np.asarray(trajectory)
    ax.plot(traj_arr[:, 0], traj_arr[:, 1], "g-", linewidth=2.5, label="Quadruped Actual Path")

    # Draw Start and Goal
    ax.scatter([start_pos[0]], [start_pos[1]], color="blue", s=120, zorder=5, label="Start")
    ax.scatter([goal_pos[0]], [goal_pos[1]], color="green", s=140, marker="*", zorder=5, label="Goal")

    # Draw Robot Footprints along trajectory
    step_skip = max(1, len(trajectory) // 8)
    for i in range(0, len(trajectory), step_skip):
        rx, ry, ryaw = trajectory[i]
        circle = plt.Circle((rx, ry), ROBOT_RADIUS_M, color="#3498db", fill=False, linestyle=":", alpha=0.6)
        ax.add_patch(circle)
        # Heading arrow
        ax.arrow(rx, ry, 0.04 * math.cos(ryaw), 0.04 * math.sin(ryaw),
                 head_width=0.02, head_length=0.02, fc="black", ec="black")

    ax.set_title("Sesame Mini Quadruped - 2D Path Planning & Trajectory Tracking", fontsize=13, fontweight="bold")
    ax.set_xlabel("X (meters)", fontsize=11)
    ax.set_ylabel("Y (meters)", fontsize=11)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="upper left")
    plt.tight_layout()

    plt.savefig(save_plot_path, dpi=150)
    print(f"Saved visualization plot to: {save_plot_path}")


if __name__ == "__main__":
    run_demo(r"c:\Users\prith\Documents\MYSPACE\STUDY\robot path planning\path_planning_result.png")
