#!/usr/bin/env python3
"""
Interactive Camera & ArUco Marker Diagnostic Tool.

Use this utility to test your video feed (DroidCam USB/Wi-Fi or IP Webcam)
before launching the autonomous navigator:
- Shows live resolution and real-time FPS.
- Detects and lists all visible ArUco markers (Corners 0-3, Robot 4, Goal 5, Obstacles 10+).
- Press 'h' to toggle the Hand Detection HSV skin mask to verify lighting calibration.
- Press 'w' to toggle perspective-warp test once all 4 corners are visible.
- Press 'q' or 'ESC' to exit.
"""

from __future__ import annotations
import time
import argparse
import numpy as np
import cv2

from vision_tracker import (
    detect_markers, MetricArenaTransformer, HandDetector,
    ID_CORNER_BL, ID_CORNER_BR, ID_CORNER_TR, ID_CORNER_TL,
    ID_ROBOT, ID_GOAL, OBSTACLE_IDS,
    DEFAULT_ARENA_WIDTH_CM, DEFAULT_ARENA_HEIGHT_CM
)

MARKER_NAMES = {
    ID_CORNER_BL: "Corner 0 (BL/Origin)",
    ID_CORNER_BR: "Corner 1 (BR)",
    ID_CORNER_TR: "Corner 2 (TR)",
    ID_CORNER_TL: "Corner 3 (TL)",
    ID_ROBOT: "Robot Tracker (ID 4)",
    ID_GOAL: "Goal Target (ID 5)",
    10: "Obstacle 1 (ID 10)",
    11: "Obstacle 2 (ID 11)",
    12: "Obstacle 3 (ID 12)",
    13: "Obstacle 4 (ID 13)",
    14: "Obstacle 5 (ID 14)",
}


def scan_usb_cameras(max_tested: int = 4) -> list[int]:
    """Probes local camera indices to report connected webcams."""
    available = []
    for idx in range(max_tested):
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                available.append(idx)
            cap.release()
    return available


def main():
    parser = argparse.ArgumentParser(description="Camera Feed & ArUco Diagnostic Tool")
    parser.add_argument("--source", type=str, default="0",
                        help="Camera source: webcam index ('0', '1'), 'mock', or IP Stream URL (http://...)")
    parser.add_argument("--scan", action="store_true", help="Scan and list all available USB camera indices")
    parser.add_argument("--width-cm", type=float, default=DEFAULT_ARENA_WIDTH_CM, help="Arena width in cm")
    parser.add_argument("--height-cm", type=float, default=DEFAULT_ARENA_HEIGHT_CM, help="Arena height in cm")
    parser.add_argument("--headless", action="store_true", help="Run without opening GUI window")
    args = parser.parse_args()

    if args.scan:
        print("[Scanner] Probing USB webcams...")
        cams = scan_usb_cameras()
        if cams:
            print(f"[Scanner] Found working camera indices: {cams}")
        else:
            print("[Scanner] No USB webcams detected.")
        return

    transformer = MetricArenaTransformer(width_cm=args.width_cm, height_cm=args.height_cm)
    hand_detector = HandDetector()

    if args.source.lower() == "mock":
        from vision_tracker import SyntheticMockCamera
        cap = SyntheticMockCamera()
        print("[Diagnostic] Testing Synthetic Mock Camera feed.")
    elif args.source.isdigit():
        cap = cv2.VideoCapture(int(args.source))
        print(f"[Diagnostic] Testing USB Camera index {args.source}...")
    else:
        cap = cv2.VideoCapture(args.source)
        print(f"[Diagnostic] Testing Network Stream: {args.source}...")

    show_skin_mask = False
    show_warped = False
    frame_count = 0
    t0 = time.time()

    print("\nControls:")
    print("  'h' - Toggle Hand Detection skin mask")
    print("  'w' - Toggle 4-corner perspective-warped view")
    print("  'q' / ESC - Quit\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("[Diagnostic] No frame received.")
                time.sleep(0.05)
                continue

            frame_count += 1
            h, w = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids = detect_markers(gray)

            detected_dict = {}
            if ids is not None and len(ids) > 0:
                for i, mid in enumerate(ids.flatten()):
                    pts = corners[i].reshape((4, 2))
                    detected_dict[int(mid)] = pts.mean(axis=0)

            # Update perspective
            corners_ready = transformer.update_corners(detected_dict)

            # Render display
            if show_warped and corners_ready:
                display_img = transformer.warp_frame(frame)
                cv2.putText(display_img, "MODE: TOP-DOWN WARPED", (15, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                display_img = frame.copy()
                # Draw detected ArUco markers on raw frame
                if ids is not None and len(ids) > 0:
                    for i, mid in enumerate(ids.flatten()):
                        pts = corners[i].reshape((4, 2)).astype(np.int32)
                        cv2.polylines(display_img, [pts], True, (0, 255, 0), 2)
                        cx, cy = int(detected_dict[int(mid)][0]), int(detected_dict[int(mid)][1])
                        name = MARKER_NAMES.get(int(mid), f"ID {mid}")
                        cv2.putText(display_img, name, (cx - 30, cy - 12),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            # Hand mask overlay if requested
            if show_skin_mask:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, hand_detector.lower_skin, hand_detector.upper_skin)
                mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                display_img = cv2.addWeighted(display_img, 0.7, mask_bgr, 0.3, 0)
                cv2.putText(display_img, "SKIN MASK ACTIVE ('h' to toggle)", (15, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)

            # Overlay telemetry banner
            fps = frame_count / (time.time() - t0)
            banner = f"FPS: {fps:.1f} | Res: {w}x{h} | Corners: {4 if corners_ready else len([k for k in (0,1,2,3) if k in detected_dict])}/4"
            cv2.putText(display_img, banner, (15, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

            if frame_count % 30 == 0:
                detected_names = [MARKER_NAMES.get(k, f"ID {k}") for k in detected_dict.keys()]
                print(f"[Diagnostic] {banner} | Detected: {detected_names}")

            if not args.headless:
                cv2.imshow("Camera & ArUco Diagnostic", display_img)
                key = cv2.waitKey(20) & 0xFF
                if key == ord('q') or key == 27:
                    break
                elif key == ord('h'):
                    show_skin_mask = not show_skin_mask
                    print(f"[Diagnostic] Skin mask display: {show_skin_mask}")
                elif key == ord('w'):
                    show_warped = not show_warped
                    print(f"[Diagnostic] Warped view display: {show_warped}")

            if args.headless and frame_count >= 60:
                print("[Diagnostic] Headless diagnostic test complete.")
                break

    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(cap, 'release'):
            cap.release()
        cv2.destroyAllWindows()
        print("[Diagnostic] Closed.")


if __name__ == "__main__":
    main()
