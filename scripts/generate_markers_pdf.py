#!/usr/bin/env python3
"""
High-Precision A4 PDF Generator for ArUco Markers (DICT_4X4_50).

Generates print-ready vector PDF documents with EXACT real-world physical millimeter sizing:
- Page 1: 4 Arena Corners (IDs 0, 1, 2, 3) @ exactly 60 x 60 mm
- Page 2: Robot Tracker (ID 4) @ exactly 40 x 40 mm (with Forward Heading Arrow)
          Goal Target (ID 5) @ exactly 60 x 60 mm (with Bullseye)
          Obstacle 1 & 2 (IDs 10, 11) @ exactly 50 x 50 mm
- Page 3: Additional Obstacles (IDs 12, 13, 14) @ exactly 50 x 50 mm

Includes:
- Physical millimeter calibration ruler on every page for 100% scale verification.
- Dashed scissor cut lines and center alignment crosshairs.
- Orientation indicators and placement guides.
"""

from __future__ import annotations
import os
import io
import math
import numpy as np
import cv2
from PIL import Image

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas
from reportlab.lib import colors

# ArUco Dictionary
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def make_pure_aruco_image(marker_id: int, px_size: int = 800) -> Image.Image:
    """Generates sharp, unadulterated ArUco marker bitmap (OpenCV 4.x / 5.x compatible)."""
    if hasattr(cv2.aruco, 'generateImageMarker'):
        mat = cv2.aruco.generateImageMarker(ARUCO_DICT, marker_id, px_size, borderBits=1)
    elif hasattr(cv2.aruco, 'drawMarker'):
        mat = cv2.aruco.drawMarker(ARUCO_DICT, marker_id, px_size, borderBits=1)
    else:
        raise RuntimeError("No cv2.aruco generator found.")
    return Image.fromarray(mat)


def draw_calibration_ruler(c: canvas.Canvas, y_pos_mm: float = 278.0):
    """Draws an exact 50 mm physical calibration check bar at the top of the page."""
    y = y_pos_mm * mm
    x_start = 25 * mm
    ruler_len = 50 * mm  # exactly 50.0 mm (5.0 cm)

    c.saveState()
    c.setStrokeColor(colors.black)
    c.setFillColor(colors.black)
    c.setLineWidth(1.0)

    # Main bar
    c.line(x_start, y, x_start + ruler_len, y)

    # 10 mm tick marks
    for tick in range(6):
        tx = x_start + tick * 10 * mm
        tick_h = 4 * mm if (tick == 0 or tick == 5) else 2.5 * mm
        c.line(tx, y - tick_h / 2, tx, y + tick_h / 2)
        if tick < 5:
            c.setFont("Helvetica", 7)
            c.drawCentredString(tx + 5 * mm, y + 3 * mm, f"{tick + 1}cm")

    # Ruler verification text
    c.setFont("Helvetica-Bold", 8)
    c.drawString(x_start + ruler_len + 8 * mm, y - 2 * mm,
                 "SCALE CHECK: Measure this bar with a ruler. Must be exactly 50 mm (5.0 cm).")
    c.setFont("Helvetica", 7)
    c.setFillColor(colors.HexColor("#555555"))
    c.drawString(x_start + ruler_len + 8 * mm, y - 6 * mm,
                 "IMPORTANT: In your printer dialog, select 'Actual Size' or '100% Scale' (Do NOT use 'Fit to Page').")
    c.restoreState()


def draw_marker_card(c: canvas.Canvas, marker_id: int, cx_mm: float, cy_mm: float,
                     size_mm: float, title: str, subtitle: str,
                     is_robot: bool = False, is_goal: bool = False):
    """
    Renders an ArUco marker card at an exact physical millimeter position and size.
    cx_mm, cy_mm: center of marker in mm from bottom-left of A4 page.
    size_mm: exact physical width and height of the ArUco marker in mm.
    """
    half = (size_mm / 2.0) * mm
    cx = cx_mm * mm
    cy = cy_mm * mm

    c.saveState()

    # 1. Dashed scissor cut box around card (padded by 8 mm)
    pad = 8 * mm
    c.setStrokeColor(colors.HexColor("#777777"))
    c.setLineWidth(0.6)
    c.setDash(3, 3)
    c.rect(cx - half - pad, cy - half - pad, (size_mm * mm) + 2 * pad, (size_mm * mm) + 2 * pad + 14 * mm)
    c.setDash()  # reset dash

    # Scissor icon hint
    c.setFont("Helvetica", 7)
    c.setFillColor(colors.HexColor("#888888"))
    c.drawString(cx - half - pad + 3 * mm, cy + half + pad + 11 * mm, "[✂ Cut along dashed border]")

    # 2. Draw Title Header
    c.setFont("Helvetica-Bold", 9)
    c.setFillColor(colors.black)
    c.drawCentredString(cx, cy + half + 7 * mm, title)

    # 3. Insert exact-dimension pure ArUco bitmap
    img_pil = make_pure_aruco_image(marker_id, px_size=800)
    img_buffer = io.BytesIO()
    img_pil.save(img_buffer, format='PNG')
    img_buffer.seek(0)

    from reportlab.lib.utils import ImageReader
    reader = ImageReader(img_buffer)
    c.drawImage(reader, cx - half, cy - half, width=size_mm * mm, height=size_mm * mm)

    # 4. Solid boundary box around the ArUco marker itself
    c.setStrokeColor(colors.black)
    c.setLineWidth(1.0)
    c.rect(cx - half, cy - half, size_mm * mm, size_mm * mm)

    # 5. Center alignment crosshairs outside the marker
    cross_len = 4 * mm
    c.setStrokeColor(colors.HexColor("#0066cc"))
    c.setLineWidth(0.5)
    # Left, Right, Bottom, Top ticks
    c.line(cx - half - cross_len, cy, cx - half, cy)
    c.line(cx + half, cy, cx + half + cross_len, cy)
    c.line(cx, cy - half - cross_len, cx, cy - half)
    c.line(cx, cy + half, cx, cy + half + cross_len)

    # 6. Robot Forward Heading Arrow (if ID 4)
    if is_robot:
        arrow_y_start = cy + half + 1 * mm
        arrow_y_end = cy + half + 5.5 * mm
        c.setStrokeColor(colors.HexColor("#cc0000"))
        c.setFillColor(colors.HexColor("#cc0000"))
        c.setLineWidth(2.0)
        c.line(cx, arrow_y_start, cx, arrow_y_end)
        # Arrowhead
        p = c.beginPath()
        p.moveTo(cx, arrow_y_end + 2 * mm)
        p.lineTo(cx - 2 * mm, arrow_y_end)
        p.lineTo(cx + 2 * mm, arrow_y_end)
        p.close()
        c.drawPath(p, fill=1, stroke=1)
        c.setFont("Helvetica-Bold", 7)
        c.drawCentredString(cx, arrow_y_end + 3.5 * mm, "FORWARD (HEAD)")

    # 7. Goal Target Ring Indicator (if ID 5)
    if is_goal:
        c.setStrokeColor(colors.HexColor("#009900"))
        c.setLineWidth(1.2)
        c.circle(cx + half + pad - 6 * mm, cy + half + 9 * mm, 3.5 * mm)
        c.setFillColor(colors.HexColor("#009900"))
        c.circle(cx + half + pad - 6 * mm, cy + half + 9 * mm, 1.5 * mm, fill=1)

    # 8. Subtitle / Dimension Label
    c.setFont("Helvetica", 7.5)
    c.setFillColor(colors.HexColor("#333333"))
    c.drawCentredString(cx, cy - half - 5 * mm, subtitle)
    c.setFont("Helvetica-Oblique", 6.5)
    c.setFillColor(colors.HexColor("#666666"))
    c.drawCentredString(cx, cy - half - 7.5 * mm, f"Exact Size: {size_mm:.0f} mm x {size_mm:.0f} mm")

    c.restoreState()


def create_page_header(c: canvas.Canvas, page_num: int, total_pages: int, page_title: str):
    """Renders top banner on each A4 page."""
    c.saveState()
    # Title
    c.setFont("Helvetica-Bold", 14)
    c.setFillColor(colors.HexColor("#111111"))
    c.drawString(20 * mm, 287 * mm, "Sesame Quadruped Navigation: Printable ArUco Markers")

    # Sub-banner
    c.setFont("Helvetica-Bold", 10)
    c.setFillColor(colors.HexColor("#0055aa"))
    c.drawString(20 * mm, 281 * mm, page_title)

    # Page indicator
    c.setFont("Helvetica", 8)
    c.setFillColor(colors.HexColor("#666666"))
    c.drawRightString(190 * mm, 287 * mm, f"Page {page_num} of {total_pages} (A4 Standard)")

    draw_calibration_ruler(c, y_pos_mm=272.0)
    c.restoreState()


def generate_aruco_pdf(output_path: str):
    """Builds the complete 3-page true-to-scale A4 PDF document."""
    c = canvas.Canvas(output_path, pagesize=A4)
    c.setTitle("Sesame ArUco Navigation Markers - Exact Scale A4")
    c.setAuthor("Antigravity - Robotics Path Planning")

    # =========================================================================
    # PAGE 1: 4 ARENA CORNERS (IDs 0, 1, 2, 3) @ Exactly 60 x 60 mm
    # =========================================================================
    create_page_header(c, 1, 3, "SHEET 1: ARENA CORNER BOUNDARIES (IDs 0 - 3)")

    # Grid layout: 2 columns x 2 rows
    col1_x = 58.0   # mm from left
    col2_x = 152.0  # mm from left
    row1_y = 195.0  # Top row mm from bottom
    row2_y = 80.0   # Bottom row mm from bottom
    size_corner = 60.0  # 60 x 60 mm

    # Corner 3: Top-Left (0, 80 cm)
    draw_marker_card(c, ID_CORNER_TL := 3, col1_x, row1_y, size_corner,
                     "ARENA CORNER 3: TOP-LEFT",
                     "Place at Top-Left Corner (0, 80 cm)")

    # Corner 2: Top-Right (100, 80 cm)
    draw_marker_card(c, ID_CORNER_TR := 2, col2_x, row1_y, size_corner,
                     "ARENA CORNER 2: TOP-RIGHT",
                     "Place at Top-Right Corner (100, 80 cm)")

    # Corner 0: Origin Bottom-Left (0, 0 cm)
    draw_marker_card(c, ID_CORNER_BL := 0, col1_x, row2_y, size_corner,
                     "ARENA CORNER 0: ORIGIN (0, 0)",
                     "Place at Bottom-Left Origin (0, 0 cm)")

    # Corner 1: Bottom-Right (100, 0 cm)
    draw_marker_card(c, ID_CORNER_BR := 1, col2_x, row2_y, size_corner,
                     "ARENA CORNER 1: BOTTOM-RIGHT",
                     "Place at Bottom-Right Corner (100, 0 cm)")

    c.showPage()

    # =========================================================================
    # PAGE 2: ROBOT (ID 4) @ 40x40 mm, GOAL (ID 5) @ 60x60 mm, OBSTACLES 10 & 11 @ 50x50 mm
    # =========================================================================
    create_page_header(c, 2, 3, "SHEET 2: ROBOT TRACKER, GOAL TARGET & OBSTACLES (IDs 4, 5, 10, 11)")

    # Row 1: Robot Tracker (40x40 mm) & Goal Target (60x60 mm)
    draw_marker_card(c, 4, 58.0, 195.0, 40.0,
                     "ROBOT TRACKER (ID 4)",
                     "Mount flat on robot top cover (arrow = forward)",
                     is_robot=True)

    draw_marker_card(c, 5, 152.0, 195.0, 60.0,
                     "GOAL TARGET (ID 5)",
                     "Place anywhere as target destination",
                     is_goal=True)

    # Row 2: Obstacles 10 and 11 @ 50x50 mm
    draw_marker_card(c, 10, 58.0, 80.0, 50.0,
                     "OBSTACLE 1 (ID 10)",
                     "Center on top of 10x10 cm cardboard box")

    draw_marker_card(c, 11, 152.0, 80.0, 50.0,
                     "OBSTACLE 2 (ID 11)",
                     "Center on top of 10x10 cm cardboard box")

    c.showPage()

    # =========================================================================
    # PAGE 3: ADDITIONAL OBSTACLES (IDs 12, 13, 14) @ 50x50 mm
    # =========================================================================
    create_page_header(c, 3, 3, "SHEET 3: ADDITIONAL OBSTACLE BLOCKS (IDs 12, 13, 14)")

    draw_marker_card(c, 12, 58.0, 195.0, 50.0,
                     "OBSTACLE 3 (ID 12)",
                     "Center on top of 10x10 cm cardboard box")

    draw_marker_card(c, 13, 152.0, 195.0, 50.0,
                     "OBSTACLE 4 (ID 13)",
                     "Center on top of 10x10 cm cardboard box")

    draw_marker_card(c, 14, 105.0, 80.0, 50.0,
                     "OBSTACLE 5 (ID 14)",
                     "Center on top of 10x10 cm cardboard box")

    c.showPage()
    c.save()
    print(f"[PDF Generator] Successfully generated true-scale PDF: {output_path}")


def main():
    base_dirs = [
        os.path.join(os.path.expanduser("~"), "Documents", "MYSPACE", "RPP-Project-2026", "markers"),
        os.path.join(os.path.expanduser("~"), "Documents", "MYSPACE", "STUDY", "robot path planning", "markers"),
    ]

    for d in base_dirs:
        os.makedirs(d, exist_ok=True)
        pdf_path = os.path.join(d, "aruco_markers_a4.pdf")
        generate_aruco_pdf(pdf_path)


if __name__ == "__main__":
    main()
