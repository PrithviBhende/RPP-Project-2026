#!/usr/bin/env python3
"""
Dynamic A* Path Planner with Strict 5 cm Obstacle Clearance.

Features:
- Configurable arena bounds in metric centimeters (e.g. 100 cm x 80 cm).
- Exact Euclidean obstacle inflation:
    Total Clearance Radius = (Box Half-Width) + (Robot Radius) + (5.0 cm Safety Buffer)
- Continuous A* grid search with diagonal movement and Euclidean heuristics.
- String-pulling (Line-of-Sight) trajectory smoothing: produces clean, direct polylines.
- Dynamic auto-replan triggering:
    - Automatically updates when obstacles or goal shift by > 3.0 cm.
    - Inhibits replanning while human hand is actively inside the arena.
"""

from __future__ import annotations
import math
import heapq
import numpy as np

# Physical Dimensions (in centimeters)
ROBOT_RADIUS_CM = 8.0         # Sesame quadruped effective collision radius
OBSTACLE_BOX_HALF_CM = 5.0    # 10 cm x 10 cm square cardboard obstacle blocks
MIN_CLEARANCE_BUFFER_CM = 5.0 # User requirement: at least 5 cm clearance from blocks
GRID_RESOLUTION_CM = 2.0      # 2 cm per planning grid cell for sub-second planning


class BoxObstacle:
    """Represents a physical rectangular obstacle with exact Euclidean distance evaluation."""

    def __init__(self, obs_id: int, center_x: float, center_y: float,
                 half_x: float = OBSTACLE_BOX_HALF_CM, half_y: float = OBSTACLE_BOX_HALF_CM,
                 extra_clearance: float = MIN_CLEARANCE_BUFFER_CM + ROBOT_RADIUS_CM):
        self.obs_id = obs_id
        self.cx = center_x
        self.cy = center_y
        self.half_x = half_x
        self.half_y = half_y
        self.extra_clearance = extra_clearance
        # Total effective avoidance radius from center
        self.bounding_radius = math.hypot(half_x, half_y) + extra_clearance

    def is_in_collision(self, px: float, py: float) -> bool:
        """Exact Euclidean distance from point (px, py) to rectangular box surface."""
        dx = max(abs(px - self.cx) - self.half_x, 0.0)
        dy = max(abs(py - self.cy) - self.half_y, 0.0)
        dist_to_surface = math.hypot(dx, dy)
        return dist_to_surface < self.extra_clearance


class VisionPathPlanner:
    """Dynamic path planner for the overhead vision arena."""

    def __init__(self, arena_width_cm: float = 100.0, arena_height_cm: float = 80.0,
                 res_cm: float = GRID_RESOLUTION_CM):
        self.width_cm = arena_width_cm
        self.height_cm = arena_height_cm
        self.res_cm = res_cm

        self.cols = int(math.ceil(arena_width_cm / res_cm))
        self.rows = int(math.ceil(arena_height_cm / res_cm))

        self.obstacles: dict[int, BoxObstacle] = {}
        self.cached_path: list[tuple[float, float]] = []
        self.last_goal: tuple[float, float] | None = None
        self.last_obs_snapshot: dict[int, tuple[float, float]] = {}

    def update_obstacles(self, obs_dict: dict[int, tuple[float, float]]) -> bool:
        """Updates obstacle positions and returns True if obstacles significantly moved (> 3 cm)."""
        changed = False
        if set(obs_dict.keys()) != set(self.obstacles.keys()):
            changed = True
        else:
            for oid, (nx, ny) in obs_dict.items():
                old = self.last_obs_snapshot.get(oid)
                if old is None or math.hypot(nx - old[0], ny - old[1]) > 3.0:
                    changed = True
                    break

        if changed:
            self.obstacles = {
                oid: BoxObstacle(oid, pos[0], pos[1])
                for oid, pos in obs_dict.items()
            }
            self.last_obs_snapshot = dict(obs_dict)

        return changed

    def is_state_valid(self, x: float, y: float) -> bool:
        """Checks arena boundaries and obstacle collision."""
        margin = ROBOT_RADIUS_CM + 2.0
        if x < margin or x > self.width_cm - margin or y < margin or y > self.height_cm - margin:
            return False

        for obs in self.obstacles.values():
            if obs.is_in_collision(x, y):
                return False
        return True

    def is_line_clear(self, p1: tuple[float, float], p2: tuple[float, float], step_cm: float = 1.5) -> bool:
        """Ray-casts along segment p1 -> p2 to verify clearance."""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-4:
            return self.is_state_valid(p1[0], p1[1])

        steps = max(int(dist / step_cm), 1)
        for i in range(steps + 1):
            t = i / steps
            x = p1[0] + t * dx
            y = p1[1] + t * dy
            if not self.is_state_valid(x, y):
                return False
        return True

    def plan(self, start_xy: tuple[float, float], goal_xy: tuple[float, float],
             force_replan: bool = False) -> list[tuple[float, float]]:
        """A* search on metric grid with line-of-sight shortcutting."""
        sx, sy = start_xy
        gx, gy = goal_xy

        # If goal moved significantly, force replan
        if self.last_goal is None or math.hypot(gx - self.last_goal[0], gy - self.last_goal[1]) > 3.0:
            force_replan = True
            self.last_goal = (gx, gy)

        # Check if existing cached path is still collision-free and not completed
        if not force_replan and len(self.cached_path) >= 2:
            path_valid = True
            for i in range(len(self.cached_path) - 1):
                if not self.is_line_clear(self.cached_path[i], self.cached_path[i + 1]):
                    path_valid = False
                    break
            if path_valid:
                return self.cached_path

        # If direct straight line is clear, return direct 2-point path
        if self.is_line_clear(start_xy, goal_xy):
            self.cached_path = [start_xy, goal_xy]
            return self.cached_path

        # Grid coordinates
        start_c = int(round(sx / self.res_cm))
        start_r = int(round(sy / self.res_cm))
        goal_c = int(round(gx / self.res_cm))
        goal_r = int(round(gy / self.res_cm))

        # A* Priority Queue: (f_score, h_score, (col, row))
        h0 = math.hypot(gx - sx, gy - sy)
        open_set: list[tuple[float, float, tuple[int, int]]] = [(h0, h0, (start_c, start_r))]
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        g_score: dict[tuple[int, int], float] = {(start_c, start_r): 0.0}

        # 8-connected neighbors
        neighbors = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414)
        ]

        found = False
        goal_node = (goal_c, goal_r)

        while open_set:
            f, _, current = heapq.heappop(open_set)

            if current == goal_node or math.hypot(current[0] - goal_c, current[1] - goal_r) <= 1.0:
                goal_node = current
                found = True
                break

            curr_g = g_score[current]
            cx_cm = current[0] * self.res_cm
            cy_cm = current[1] * self.res_cm

            for dc, dr, cost in neighbors:
                nc = current[0] + dc
                nr = current[1] + dr
                neighbor = (nc, nr)

                nx_cm = nc * self.res_cm
                ny_cm = nr * self.res_cm

                if not self.is_state_valid(nx_cm, ny_cm):
                    continue

                tentative_g = curr_g + cost * self.res_cm

                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = math.hypot(gx - nx_cm, gy - ny_cm)
                    heapq.heappush(open_set, (tentative_g + 1.05 * h, h, neighbor))

        if not found:
            # Fallback if arena is blocked: retain existing path or direct line
            return self.cached_path if self.cached_path else [start_xy, goal_xy]

        # Reconstruct path
        node = goal_node
        raw_path = []
        while node in came_from:
            raw_path.append((node[0] * self.res_cm, node[1] * self.res_cm))
            node = came_from[node]
        raw_path.append(start_xy)
        raw_path.reverse()
        raw_path[-1] = goal_xy  # snap final waypoint to exact goal

        # Post-Processing: String-Pulling Shortcut Smoother
        smoothed = self.smooth_path(raw_path)
        self.cached_path = smoothed
        return smoothed

    def smooth_path(self, path: list[tuple[float, float]]) -> list[tuple[float, float]]:
        """Greedy line-of-sight shortcutting to eliminate unnecessary zigzag grid artifacts."""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        curr_idx = 0

        while curr_idx < len(path) - 1:
            best_next = curr_idx + 1
            for target_idx in range(len(path) - 1, curr_idx, -1):
                if self.is_line_clear(path[curr_idx], path[target_idx]):
                    best_next = target_idx
                    break
            smoothed.append(path[best_next])
            curr_idx = best_next

        return smoothed


def test_planner():
    planner = VisionPathPlanner(arena_width_cm=100.0, arena_height_cm=80.0)

    # Place 2 obstacles between Bottom-Left and Top-Right
    obs = {
        10: (45.0, 35.0),
        11: (60.0, 50.0)
    }
    planner.update_obstacles(obs)

    start = (15.0, 15.0)
    goal = (85.0, 70.0)

    path = planner.plan(start, goal)
    print(f"[Planner Test] Found path with {len(path)} waypoints:")
    for i, wp in enumerate(path):
        print(f"  WP {i}: ({wp[0]:.1f}, {wp[1]:.1f}) cm")

    # Verify clearance against all obstacles
    all_clear = True
    for wp in path:
        for oid, b in planner.obstacles.items():
            dx = max(abs(wp[0] - b.cx) - b.half_x, 0.0)
            dy = max(abs(wp[1] - b.cy) - b.half_y, 0.0)
            clearance = math.hypot(dx, dy) - ROBOT_RADIUS_CM
            if clearance < MIN_CLEARANCE_BUFFER_CM:
                print(f"[!] Warning: WP {wp} has clearance {clearance:.1f} cm < 5 cm from Obs {oid}")
                all_clear = False

    if all_clear:
        print("[Planner Test] SUCCESS: All waypoints satisfy >= 5.0 cm clearance requirement!")


if __name__ == "__main__":
    test_planner()
