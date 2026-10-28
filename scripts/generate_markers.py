#!/usr/bin/env python3
"""
ArUco Marker Generator for Autonomous Vision-Guided Quadruped Navigation.

Generates high-contrast printable ArUco markers (DICT_4X4_50):
- ID 0: Arena Origin / Bottom-Left (0, 0)
- ID 1: Arena Bottom-Right
- ID 2: Arena Top-Right
- ID 3: Arena Top-Left
- ID 4: Robot Base Tracker (with Forward Heading Arrow)
- ID 5: Goal Target Endpoint
- IDs 10, 11, 12, 13, 14: Dynamic Obstacle Blocks
- Composite Printable Sheets (A4 / Letter) for instant 1-click printing
"""

import os
import cv2
import numpy as np

# Use standard 4x4 dictionary with 50 markers (fast, high contrast, optimal for overhead tracking)
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

MARKER_DEFS = [
    {"id": 0, "name": "arena_corner_0_origin_BL", "label": "CORNER 0: ORIGIN (0, 0) - Bottom-Left", "size_px": 400},
    {"id": 1, "name": "arena_corner_1_BR", "label": "CORNER 1: Bottom-Right", "size_px": 400},
    {"id": 2, "name": "arena_corner_2_TR", "label": "CORNER 2: Top-Right", "size_px": 400},
    {"id": 3, "name": "arena_corner_3_TL", "label": "CORNER 3: Top-Left", "size_px": 400},
    {"id": 4, "name": "robot_tracker_id4", "label": "ROBOT TRACKER (ID 4) - Mount on Robot Top", "size_px": 400, "is_robot": True},
    {"id": 5, "name": "goal_target_id5", "label": "GOAL TARGET (ID 5) - Navigation Endpoint", "size_px": 400, "is_goal": True},
    {"id": 10, "name": "obstacle_block_id10", "label": "OBSTACLE 1 (ID 10) - Box Center", "size_px": 350},
    {"id": 11, "name": "obstacle_block_id11", "label": "OBSTACLE 2 (ID 11) - Box Center", "size_px": 350},
    {"id": 12, "name": "obstacle_block_id12", "label": "OBSTACLE 3 (ID 12) - Box Center", "size_px": 350},
    {"id": 13, "name": "obstacle_block_id13", "label": "OBSTACLE 4 (ID 13) - Box Center", "size_px": 350},
    {"id": 14, "name": "obstacle_block_id14", "label": "OBSTACLE 5 (ID 14) - Box Center", "size_px": 350},
]


def generate_aruco_image(dictionary, marker_id: int, size_px: int, border_bits: int = 1) -> np.ndarray:
    """Cross-version compatible ArUco marker generator (OpenCV 4.x & 5.x)."""
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, size_px, borderBits=border_bits)
    elif hasattr(cv2.aruco, 'drawMarker'):
        return cv2.aruco.drawMarker(dictionary, marker_id, size_px, borderBits=border_bits)
    else:
        raise RuntimeError("No recognized ArUco marker generation function found in cv2.aruco.")


def generate_single_marker(marker_info: dict, border_bits: int = 1) -> np.ndarray:
    marker_id = marker_info["id"]
    size_px = marker_info.get("size_px", 400)

    # 1. Generate core ArUco image
    marker_img = generate_aruco_image(ARUCO_DICT, marker_id, size_px, border_bits=border_bits)

    # Convert to BGR for annotations
    card_h = size_px + 140
    card_w = size_px + 80
    card = np.full((card_h, card_w, 3), 255, dtype=np.uint8)

    # Center marker in card
    offset_y = 60
    offset_x = 40
    card[offset_y:offset_y + size_px, offset_x:offset_x + size_px] = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)

    # 2. Add border line around marker
    cv2.rectangle(card, (offset_x - 1, offset_y - 1), (offset_x + size_px, offset_y + size_px), (0, 0, 0), 2)

    # 3. Label text header
    label = marker_info.get("label", f"ID {marker_id}")
    cv2.putText(card, label, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2, cv2.LINE_AA)

    # 4. Robot-specific annotations: forward heading arrow & orientation mark
    if marker_info.get("is_robot", False):
        arrow_start = (offset_x + size_px // 2, offset_y - 10)
        arrow_end = (offset_x + size_px // 2, 10)
        cv2.arrowedLine(card, arrow_start, arrow_end, (0, 0, 255), 4, tipLength=0.4)
        cv2.putText(card, "FORWARD (HEAD)", (offset_x + size_px // 2 - 60, offset_y - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

    # 5. Goal-specific annotations: bullseye badge
    if marker_info.get("is_goal", False):
        cv2.circle(card, (card_w - 40, 30), 16, (0, 180, 0), 3)
        cv2.circle(card, (card_w - 40, 30), 8, (0, 180, 0), -1)

    # 6. Footer calibration ruler (5 cm indicator guide)
    footer_y = card_h - 25
    cv2.line(card, (50, footer_y), (card_w - 50, footer_y), (100, 100, 100), 2)
    cv2.line(card, (50, footer_y - 8), (50, footer_y + 8), (100, 100, 100), 2)
    cv2.line(card, (card_w - 50, footer_y - 8), (card_w - 50, footer_y + 8), (100, 100, 100), 2)
    cv2.putText(card, f"DICT_4X4_50 | Marker ID: {marker_id} | Real Size Scale",
                (card_w // 2 - 140, footer_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1, cv2.LINE_AA)

    return card


def generate_composite_sheet(cards: list[np.ndarray], title: str, grid_rows: int = 2, grid_cols: int = 2) -> np.ndarray:
    """Combines individual cards onto a single printable grid sheet."""
    target_h = 420
    target_w = 420

    resized = [cv2.resize(c, (target_w, target_h), interpolation=cv2.INTER_AREA) for c in cards]

    total_cells = grid_rows * grid_cols
    while len(resized) < total_cells:
        resized.append(np.full((target_h, target_w, 3), 255, dtype=np.uint8))

    rows = []
    for r in range(grid_rows):
        row_cards = resized[r * grid_cols:(r + 1) * grid_cols]
        rows.append(np.hstack(row_cards))
    grid = np.vstack(rows)

    # Add header banner
    banner = np.full((70, grid.shape[1], 3), 255, dtype=np.uint8)
    cv2.putText(banner, title, (30, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.line(banner, (0, 68), (grid.shape[1], 68), (0, 0, 0), 2)

    return np.vstack([banner, grid])


def main():
    base_dirs = [
        os.path.join(os.path.expanduser("~"), "Documents", "MYSPACE", "RPP-Project-2026", "markers"),
        os.path.join(os.path.expanduser("~"), "Documents", "MYSPACE", "STUDY", "robot path planning", "markers"),
    ]

    for out_dir in base_dirs:
        os.makedirs(out_dir, exist_ok=True)

    print("Generating ArUco DICT_4X4_50 markers...")
    generated_cards = {}
    for m in MARKER_DEFS:
        card = generate_single_marker(m)
        generated_cards[m["id"]] = card
        for out_dir in base_dirs:
            file_path = os.path.join(out_dir, f"{m['name']}.png")
            cv2.imwrite(file_path, card)
        print(f" -> Generated: {m['name']}.png (ID {m['id']})")

    # Generate Sheet 1: Arena Corners (0, 1, 2, 3)
    corners = [generated_cards[0], generated_cards[1], generated_cards[2], generated_cards[3]]
    sheet_corners = generate_composite_sheet(corners, "SHEET 1: ARENA CORNER BOUNDARIES (IDs 0 - 3)")
    for out_dir in base_dirs:
        cv2.imwrite(os.path.join(out_dir, "print_sheet_1_arena_corners.png"), sheet_corners)
    print(" -> Generated printable: print_sheet_1_arena_corners.png")

    # Generate Sheet 2: Robot Tracker + Goal + Obstacles 10 & 11
    entities = [generated_cards[4], generated_cards[5], generated_cards[10], generated_cards[11]]
    sheet_entities = generate_composite_sheet(entities, "SHEET 2: ROBOT (ID 4), GOAL (ID 5), OBSTACLES (IDs 10, 11)")
    for out_dir in base_dirs:
        cv2.imwrite(os.path.join(out_dir, "print_sheet_2_robot_goal_obstacles.png"), sheet_entities)
    print(" -> Generated printable: print_sheet_2_robot_goal_obstacles.png")

    # Generate Sheet 3: Additional Obstacles (12, 13, 14)
    obs = [generated_cards[12], generated_cards[13], generated_cards[14]]
    sheet_obs = generate_composite_sheet(obs, "SHEET 3: ADDITIONAL OBSTACLES (IDs 12, 13, 14)")
    for out_dir in base_dirs:
        cv2.imwrite(os.path.join(out_dir, "print_sheet_3_more_obstacles.png"), sheet_obs)
    print(" -> Generated printable: print_sheet_3_more_obstacles.png")

    print("\nAll markers and 1-click printable sheets generated successfully!")


if __name__ == "__main__":
    main()
