#!/usr/bin/env python3
"""
Real-Time Overhead Vision Tracking & Coordinate Engine.

Features:
- Ingests USB Webcam, Phone IP Stream (DroidCam/IP Webcam HTTP/RTSP), or Synthetic Mock Feed.
- Multi-version ArUco detection (OpenCV 4.x & 5.x compatible).
- 4-Corner Planar Homography: auto-rectifies and warps handheld or tilted video into a calibrated metric top-down plane (in cm).
- Real-time Human Hand Detection (HSV skin mask) for safety gating and pause-during-rearrangement.
- EMA temporal smoothing on robot pose (x, y, theta) to eliminate sub-pixel jitter.
- Interactive OpenCV HUD overlay with metric coordinates and safety alerts.
"""

from __future__ import annotations
import math
import time
import argparse
import numpy as np
import cv2

# Target ArUco dictionary
ARUCO_DICT_ID = cv2.aruco.DICT_4X4_50
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)

# Marker ID Definitions
ID_CORNER_BL = 0   # Origin (0, 0)
ID_CORNER_BR = 1   # (Width, 0)
ID_CORNER_TR = 2   # (Width, Height)
ID_CORNER_TL = 3   # (0, Height)
ID_ROBOT = 4       # Quadruped Body Tracker
ID_GOAL = 5        # Target Destination
OBSTACLE_IDS = (10, 11, 12, 13, 14, 15, 16, 17, 18, 19)

# Default Metric Arena Size in Centimeters
DEFAULT_ARENA_WIDTH_CM = 100.0   # 1.0 m width
DEFAULT_ARENA_HEIGHT_CM = 80.0   # 0.8 m height
WARP_PX_PER_CM = 10.0            # 10 pixels per cm (e.g. 1000 x 800 px warped arena)


def detect_markers(gray_img: np.ndarray) -> tuple[list[np.ndarray], np.ndarray | None]:
    """Cross-version ArUco marker detection (works across OpenCV 4.x & 5.x)."""
    if hasattr(cv2.aruco, 'ArucoDetector'):
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(ARUCO_DICT, params)
        corners, ids, _ = detector.detectMarkers(gray_img)
    elif hasattr(cv2.aruco, 'detectMarkers'):
        params = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, 'DetectorParameters_create') else cv2.aruco.DetectorParameters()
        corners, ids, _ = cv2.aruco.detectMarkers(gray_img, ARUCO_DICT, parameters=params)
    else:
        corners, ids = [], None

    return corners, ids


def normalize_angle_rad(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class MetricArenaTransformer:
    """Maintains perspective homography from raw camera pixels to metric centimeters."""

    def __init__(self, width_cm: float = DEFAULT_ARENA_WIDTH_CM, height_cm: float = DEFAULT_ARENA_HEIGHT_CM):
        self.width_cm = width_cm
        self.height_cm = height_cm
        self.width_px = int(width_cm * WARP_PX_PER_CM)
        self.height_px = int(height_cm * WARP_PX_PER_CM)

        self.H_cam_to_warp: np.ndarray | None = None
        self.H_warp_to_cam: np.ndarray | None = None
        self.is_calibrated = False

        # Canonical destination points in warped image (Top-Left is (0,0) in OpenCV image coords, Bottom-Left is (0, H))
        # Point order: Corner 0 (BL), Corner 1 (BR), Corner 2 (TR), Corner 3 (TL)
        self.dst_pts = np.float32([
            [0, self.height_px],               # BL: (0, Height) in img coords
            [self.width_px, self.height_px],   # BR: (Width, Height)
            [self.width_px, 0],                # TR: (Width, 0)
            [0, 0]                             # TL: (0, 0)
        ])

    def update_corners(self, corner_centers: dict[int, np.ndarray]) -> bool:
        """Computes homography if all 4 corners (0, 1, 2, 3) are visible."""
        if all(k in corner_centers for k in (ID_CORNER_BL, ID_CORNER_BR, ID_CORNER_TR, ID_CORNER_TL)):
            src_pts = np.float32([
                corner_centers[ID_CORNER_BL],
                corner_centers[ID_CORNER_BR],
                corner_centers[ID_CORNER_TR],
                corner_centers[ID_CORNER_TL],
            ])
            H = cv2.getPerspectiveTransform(src_pts, self.dst_pts)
            self.H_cam_to_warp = H
            self.H_warp_to_cam = np.linalg.inv(H)
            self.is_calibrated = True
            return True
        return False

    def warp_frame(self, frame: np.ndarray) -> np.ndarray:
        if self.H_cam_to_warp is None:
            return frame
        return cv2.warpPerspective(frame, self.H_cam_to_warp, (self.width_px, self.height_px))

    def warp_px_to_cm(self, px_x: float, px_y: float) -> tuple[float, float]:
        """Converts warped image pixel coordinates to metric arena coordinates where (0,0) is Bottom-Left."""
        cm_x = px_x / WARP_PX_PER_CM
        cm_y = (self.height_px - px_y) / WARP_PX_PER_CM
        return cm_x, cm_y

    def cm_to_warp_px(self, cm_x: float, cm_y: float) -> tuple[int, int]:
        """Converts metric arena cm coordinates to warped image pixel coordinates."""
        px_x = int(round(cm_x * WARP_PX_PER_CM))
        px_y = int(round(self.height_px - cm_y * WARP_PX_PER_CM))
        return px_x, px_y


class HandDetector:
    """Detects human hands entering the arena to pause navigation during object relocation."""

    def __init__(self, min_area_px: int = 3500, enabled: bool = True):
        self.min_area_px = min_area_px
        self.enabled = enabled

        # Tuned HSV skin range: minimum Saturation 52 rejects cream/beige ceramic floor tiles
        self.lower_skin = np.array([0, 52, 40], dtype=np.uint8)
        self.upper_skin = np.array([25, 255, 255], dtype=np.uint8)

    def detect_in_arena(self, warped_frame: np.ndarray) -> tuple[bool, list[tuple[int, int, int, int]]]:
        if not self.enabled:
            return False, []

        hsv = cv2.cvtColor(warped_frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_skin, self.upper_skin)

        # Morphological filtering to clean noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        hand_boxes = []
        hand_detected = False

        for c in contours:
            area = cv2.contourArea(c)
            if area > self.min_area_px:
                x, y, w, h = cv2.boundingRect(c)
                hand_boxes.append((x, y, w, h))
                hand_detected = True

        return hand_detected, hand_boxes


class VisionTracker:
    """Primary vision tracking coordinator."""

    def __init__(self, width_cm: float = DEFAULT_ARENA_WIDTH_CM, height_cm: float = DEFAULT_ARENA_HEIGHT_CM,
                 enable_hand_detect: bool = True):
        self.transformer = MetricArenaTransformer(width_cm, height_cm)
        self.hand_detector = HandDetector(enabled=enable_hand_detect)

        # Filtered state
        self.robot_x_cm: float | None = None
        self.robot_y_cm: float | None = None
        self.robot_heading_rad: float = 0.0

        self.goal_x_cm: float | None = None
        self.goal_y_cm: float | None = None

        self.obstacles: dict[int, tuple[float, float]] = {}  # id -> (cm_x, cm_y)
        self.hand_detected: bool = False
        self.hand_boxes: list[tuple[int, int, int, int]] = []

        # EMA filter alpha (0.0: sluggish/smooth, 1.0: raw unfiltered)
        self.ema_alpha = 0.65

    def process_frame(self, frame: np.ndarray) -> tuple[np.ndarray, dict]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = detect_markers(gray)

        marker_centers: dict[int, np.ndarray] = {}
        marker_corners: dict[int, np.ndarray] = {}

        if ids is not None and len(ids) > 0:
            flat_ids = ids.flatten()
            for i, mid in enumerate(flat_ids):
                pts = corners[i].reshape((4, 2))
                center = pts.mean(axis=0)
                marker_centers[int(mid)] = center
                marker_corners[int(mid)] = pts

        # 1. Update perspective homography using corner markers
        arena_locked = self.transformer.update_corners(marker_centers)

        # 2. Warp frame to top-down metric perspective
        warped = self.transformer.warp_frame(frame) if self.transformer.is_calibrated else frame.copy()

        # 3. Transform detected marker points to metric coordinates via homography
        if self.transformer.is_calibrated and self.transformer.H_cam_to_warp is not None:
            # Map raw marker corners to warped image coordinates
            w_pts_dict: dict[int, np.ndarray] = {}
            w_centers: dict[int, np.ndarray] = {}

            for mid, pts in marker_corners.items():
                pts_transformed = cv2.perspectiveTransform(pts.reshape(-1, 1, 2).astype(np.float32),
                                                            self.transformer.H_cam_to_warp).reshape((4, 2))
                w_pts_dict[mid] = pts_transformed
                w_centers[mid] = pts_transformed.mean(axis=0)

            # A. Robot Pose (ID 4)
            if ID_ROBOT in w_centers:
                cx, cy = w_centers[ID_ROBOT]
                pts = w_pts_dict[ID_ROBOT]
                # Heading vector from marker bottom edge center to top edge center
                top_mid = (pts[0] + pts[1]) * 0.5
                bot_mid = (pts[3] + pts[2]) * 0.5
                # In OpenCV image coords, +Y is DOWN, so invert dy for standard Cartesian coordinates (+Y UP)
                fwd_dx = float(top_mid[0] - bot_mid[0])
                fwd_dy = float(bot_mid[1] - top_mid[1])
                raw_heading = math.atan2(fwd_dy, fwd_dx)

                raw_x_cm, raw_y_cm = self.transformer.warp_px_to_cm(cx, cy)

                # EMA Filter
                if self.robot_x_cm is None:
                    self.robot_x_cm = raw_x_cm
                    self.robot_y_cm = raw_y_cm
                    self.robot_heading_rad = raw_heading
                else:
                    self.robot_x_cm = self.ema_alpha * raw_x_cm + (1.0 - self.ema_alpha) * self.robot_x_cm
                    self.robot_y_cm = self.ema_alpha * raw_y_cm + (1.0 - self.ema_alpha) * self.robot_y_cm
                    d_heading = normalize_angle_rad(raw_heading - self.robot_heading_rad)
                    self.robot_heading_rad = normalize_angle_rad(self.robot_heading_rad + self.ema_alpha * d_heading)

            # B. Goal Pose (ID 5)
            if ID_GOAL in w_centers:
                gx, gy = w_centers[ID_GOAL]
                self.goal_x_cm, self.goal_y_cm = self.transformer.warp_px_to_cm(gx, gy)

            # C. Dynamic Obstacle Blocks (IDs 10+)
            current_obs: dict[int, tuple[float, float]] = {}
            for mid in OBSTACLE_IDS:
                if mid in w_centers:
                    ox, oy = w_centers[mid]
                    current_obs[mid] = self.transformer.warp_px_to_cm(ox, oy)
            self.obstacles = current_obs

            # D. Hand Presence Detection
            self.hand_detected, self.hand_boxes = self.hand_detector.detect_in_arena(warped)

        # 4. Construct telemetry dictionary
        telemetry = {
            "arena_locked": self.transformer.is_calibrated,
            "robot_pose": (self.robot_x_cm, self.robot_y_cm, self.robot_heading_rad) if self.robot_x_cm is not None else None,
            "goal_pos": (self.goal_x_cm, self.goal_y_cm) if self.goal_x_cm is not None else None,
            "obstacles": dict(self.obstacles),
            "hand_detected": self.hand_detected,
        }

        # 5. Render HUD overlays on the warped frame
        annotated_frame = self.render_hud(warped, telemetry)

        return annotated_frame, telemetry

    def render_hud(self, warped_frame: np.ndarray, telemetry: dict) -> np.ndarray:
        hud = warped_frame.copy()

        # Draw Arena Boundary
        if self.transformer.is_calibrated:
            cv2.rectangle(hud, (0, 0), (self.transformer.width_px - 1, self.transformer.height_px - 1), (0, 255, 0), 2)

        # Draw Obstacles & 5 cm Safety Margin
        inflation_px = int(round(5.0 * WARP_PX_PER_CM))
        for obs_id, (ox_cm, oy_cm) in telemetry["obstacles"].items():
            px, py = self.transformer.cm_to_warp_px(ox_cm, oy_cm)
            # Physical box outline (~10 cm square)
            box_half = int(round(5.0 * WARP_PX_PER_CM))
            cv2.rectangle(hud, (px - box_half, py - box_half), (px + box_half, py + box_half), (0, 0, 255), 2)
            # 5 cm clearance safety boundary
            clearance_radius = box_half + inflation_px
            cv2.circle(hud, (px, py), clearance_radius, (0, 165, 255), 1, cv2.LINE_AA)
            cv2.putText(hud, f"Obs {obs_id}", (px - 25, py - box_half - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

        # Draw Goal (ID 5)
        if telemetry["goal_pos"] is not None:
            gx_cm, gy_cm = telemetry["goal_pos"]
            g_px, g_py = self.transformer.cm_to_warp_px(gx_cm, gy_cm)
            cv2.circle(hud, (g_px, g_py), 18, (0, 255, 255), 3, cv2.LINE_AA)
            cv2.circle(hud, (g_px, g_py), 6, (0, 255, 255), -1)
            cv2.putText(hud, f"GOAL ({gx_cm:.1f}, {gy_cm:.1f})", (g_px - 40, g_py + 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)

        # Draw Robot (ID 4)
        if telemetry["robot_pose"] is not None:
            rx_cm, ry_cm, heading_rad = telemetry["robot_pose"]
            r_px, r_py = self.transformer.cm_to_warp_px(rx_cm, ry_cm)
            # Draw robot circular footprint
            cv2.circle(hud, (r_px, r_py), 16, (255, 100, 0), -1)
            cv2.circle(hud, (r_px, r_py), 18, (255, 255, 255), 2)
            # Draw forward heading arrow
            arrow_len = 35
            arrow_dx = int(round(arrow_len * math.cos(heading_rad)))
            arrow_dy = int(round(-arrow_len * math.sin(heading_rad)))
            cv2.arrowedLine(hud, (r_px, r_py), (r_px + arrow_dx, r_py + arrow_dy), (0, 255, 0), 3, tipLength=0.35)
            cv2.putText(hud, f"ROBOT ({rx_cm:.1f}, {ry_cm:.1f}) {math.degrees(heading_rad):.0f}deg",
                        (r_px - 60, r_py - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)

        # Draw Hand Alert Bounding Boxes
        if telemetry["hand_detected"]:
            for (hx, hy, hw, hh) in self.hand_boxes:
                cv2.rectangle(hud, (hx, hy), (hx + hw, hy + hh), (0, 140, 255), 2)
                cv2.putText(hud, "HAND DETECTED (PAUSED)", (hx, hy - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 2, cv2.LINE_AA)

        # Top Telemetry Status Banner
        banner_h = 45
        cv2.rectangle(hud, (0, 0), (hud.shape[1], banner_h), (30, 30, 30), -1)
        cv2.line(hud, (0, banner_h), (hud.shape[1], banner_h), (80, 80, 80), 1)

        arena_text = "LOCKED" if telemetry["arena_locked"] else "SEARCHING CORNERS 0-3"
        arena_color = (0, 255, 0) if telemetry["arena_locked"] else (0, 0, 255)
        cv2.putText(hud, f"Arena: {arena_text}", (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, arena_color, 2, cv2.LINE_AA)

        if telemetry["hand_detected"]:
            cv2.putText(hud, "[!] SAFETY: MOTION PAUSED (HAND IN ARENA)", (260, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 140, 255), 2, cv2.LINE_AA)
        elif telemetry["robot_pose"] is not None and telemetry["goal_pos"] is not None:
            dist = math.hypot(telemetry["goal_pos"][0] - telemetry["robot_pose"][0],
                              telemetry["goal_pos"][1] - telemetry["robot_pose"][1])
            cv2.putText(hud, f"Goal Dist: {dist:.1f} cm | Obs: {len(telemetry['obstacles'])}",
                        (260, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        return hud


class SyntheticMockCamera:
    """Generates an animated synthetic video feed for automated headless testing without a physical camera."""

    def __init__(self, width_px: int = 1280, height_px: int = 720):
        self.width_px = width_px
        self.height_px = height_px
        self.t0 = time.time()

        # Generate ArUco marker images
        from generate_markers import generate_aruco_image
        self.markers = {
            mid: generate_aruco_image(ARUCO_DICT, mid, 80)
            for mid in [0, 1, 2, 3, 4, 5, 10, 11]
        }

    def read(self) -> tuple[bool, np.ndarray]:
        frame = np.full((self.height_px, self.width_px, 3), 235, dtype=np.uint8)
        now = time.time() - self.t0

        # Draw a slight perspective trapezoid table surface
        arena_quad = np.array([
            [180, 620],  # BL (Corner 0)
            [1100, 620], # BR (Corner 1)
            [980, 120],  # TR (Corner 2)
            [300, 120],  # TL (Corner 3)
        ], dtype=np.int32)

        cv2.fillConvexPoly(frame, arena_quad, (215, 225, 220))
        cv2.polylines(frame, [arena_quad], True, (160, 170, 165), 2)

        def paste_marker(mid: int, center_pt: tuple[int, int], angle_deg: float = 0.0):
            marker_img = self.markers[mid]
            # Add white padding to prevent corner clipping during rotation
            pad_size = 25
            padded = cv2.copyMakeBorder(marker_img, pad_size, pad_size, pad_size, pad_size,
                                        cv2.BORDER_CONSTANT, value=255)
            ph, pw = padded.shape[:2]
            M = cv2.getRotationMatrix2D((pw / 2, ph / 2), angle_deg, 1.0)
            rotated = cv2.warpAffine(padded, M, (pw, ph), borderValue=255)
            rotated_bgr = cv2.cvtColor(rotated, cv2.COLOR_GRAY2BGR)

            cx, cy = center_pt
            x1, y1 = cx - pw // 2, cy - ph // 2
            x2, y2 = x1 + pw, y1 + ph
            if x1 >= 0 and y1 >= 0 and x2 < self.width_px and y2 < self.height_px:
                frame[y1:y2, x1:x2] = rotated_bgr

        # Paste 4 arena corners
        paste_marker(0, (arena_quad[0][0], arena_quad[0][1]))
        paste_marker(1, (arena_quad[1][0], arena_quad[1][1]))
        paste_marker(2, (arena_quad[2][0], arena_quad[2][1]))
        paste_marker(3, (arena_quad[3][0], arena_quad[3][1]))

        # Paste Goal (Top-Right inside arena)
        paste_marker(5, (860, 220))

        # Paste Obstacles (Middle of arena)
        paste_marker(10, (580, 380))
        paste_marker(11, (740, 420))

        # Paste Robot (moving forward-right smoothly)
        progress = (now * 0.15) % 1.0
        rx = int(320 + progress * 400)
        ry = int(520 - progress * 240)
        heading_deg = 35.0
        paste_marker(4, (rx, ry), heading_deg)

        # Simulate hand entering occasionally between t=8s and t=12s
        if 8.0 < (now % 20.0) < 13.0:
            # Draw synthetic hand skin patch over obstacle 10
            hand_patch = np.zeros_like(frame)
            cv2.ellipse(frame, (600, 360), (55, 35), 25, 0, 360, (140, 175, 220), -1) # BGR skin tone

        return True, frame


def main():
    parser = argparse.ArgumentParser(description="Overhead Vision Tracker for Robot Path Planning")
    parser.add_argument("--source", type=str, default="mock",
                        help="Camera source: 'mock', webcam index (0, 1), or URL (http://... / rtsp://...)")
    parser.add_argument("--width-cm", type=float, default=DEFAULT_ARENA_WIDTH_CM, help="Arena width in cm")
    parser.add_argument("--height-cm", type=float, default=DEFAULT_ARENA_HEIGHT_CM, help="Arena height in cm")
    parser.add_argument("--headless", action="store_true", help="Run without opening GUI windows")
    args = parser.parse_args()

    tracker = VisionTracker(width_cm=args.width_cm, height_cm=args.height_cm)

    # Initialize video capture
    if args.source.lower() == "mock":
        print("[Vision] Using Synthetic Mock Camera feed.")
        cap = SyntheticMockCamera()
    elif args.source.isdigit():
        print(f"[Vision] Opening USB Camera index {args.source}...")
        cap = cv2.VideoCapture(int(args.source))
    else:
        print(f"[Vision] Opening IP Video Stream from {args.source}...")
        cap = cv2.VideoCapture(args.source)

    print("\nVision tracking active. Press 'q' to quit.")
    frame_count = 0
    t0 = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("[Vision] Failed to read frame.")
                break

            annotated_frame, telemetry = tracker.process_frame(frame)
            frame_count += 1

            if frame_count % 30 == 0:
                fps = frame_count / (time.time() - t0)
                status_str = f"FPS: {fps:.1f} | Arena: {'LOCKED' if telemetry['arena_locked'] else 'SEARCHING'}"
                if telemetry['robot_pose']:
                    rx, ry, th = telemetry['robot_pose']
                    status_str += f" | Robot: ({rx:.1f}, {ry:.1f}) {math.degrees(th):.0f}°"
                if telemetry['hand_detected']:
                    status_str += " | [!] HAND ACTIVE (PAUSED)"
                print(f"[Vision Status] {status_str}")

            if not args.headless:
                cv2.imshow("Overhead Arena Tracker (Rectified)", annotated_frame)
                key = cv2.waitKey(20) & 0xFF
                if key == ord('q') or key == 27:
                    break

            if args.headless and frame_count >= 100:
                print(f"[Vision] Completed {frame_count} frames test in headless mode.")
                break

    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(cap, 'release'):
            cap.release()
        cv2.destroyAllWindows()
        print("[Vision] Tracker stopped cleanly.")


if __name__ == "__main__":
    main()
