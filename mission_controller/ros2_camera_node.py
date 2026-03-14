#!/usr/bin/env python3

import argparse
import math
import os
import time
import cv2
import numpy as np
from datetime import datetime

SAVE_DIR = os.path.expanduser("~/drone_images")
WINDOW_NAME = "Drone Camera"



def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920, capture_height=1080,
    display_width=1280, display_height=720,
    framerate=30, flip_method=0,
):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} "
        f"exposuretimerange='13000 683709000' "
        f"gainrange='1 16' "
        f"ispdigitalgainrange='1 8' ! "
        f"video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, "
        f"framerate={framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width={display_width}, height={display_height}, format=BGRx ! "
        f"videoconvert ! video/x-raw, format=BGR ! appsink drop=1"
    )


def open_camera(source, flip=0):
    if source == "csi":
        pipeline = gstreamer_pipeline(flip_method=flip)
        print(f"[INFO] GStreamer: {pipeline}")
        return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    elif source == "test":
        return None
    else:
        cap = cv2.VideoCapture(int(source))
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        return cap


def generate_test_frame(frame_count, width=1280, height=720):
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    t = frame_count * 0.02

    for i in range(0, width, 80):
        for j in range(0, height, 80):
            b = int(127 + 127 * math.sin(t + i * 0.01))
            g = int(127 + 127 * math.sin(t + j * 0.01 + 2))
            r = int(127 + 127 * math.sin(t + (i + j) * 0.005 + 4))
            cv2.rectangle(frame, (i, j), (i + 80, j + 80), (b, g, r), -1)

    cx = int(width / 2 + 200 * math.sin(t * 0.5))
    cy = int(height / 2 + 100 * math.cos(t * 0.3))
    cv2.circle(frame, (cx, cy), 60, (255, 255, 255), 3)
    cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)
    cv2.putText(frame, "TEST PATTERN", (width // 2 - 150, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2)

    return frame


def save_frame(frame):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    path = os.path.join(SAVE_DIR, f"frame_{timestamp}.jpg")
    cv2.imwrite(path, frame)
    print(f"[SAVED] {path}")
    return path


def draw_hud(display, frame_count, auto_active, auto_interval, status_msg, status_time, source):
    h, w = display.shape[:2]

    overlay = display.copy()
    cv2.rectangle(overlay, (0, 0), (w, 36), (15, 30, 60), -1)
    cv2.addWeighted(overlay, 0.7, display, 0.3, 0, display)

    cv2.putText(display, "DRONE CAMERA", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 210, 106), 2)

    src_label = f"[{source.upper()}]"
    cv2.putText(display, src_label, (220, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

    cv2.putText(display, f"Frames: {frame_count}", (w - 180, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    if auto_active:
        cv2.putText(display, f"AUTO {auto_interval:.1f}s", (w - 350, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 230), 2)

    overlay2 = display.copy()
    cv2.rectangle(overlay2, (0, h - 30), (w, h), (15, 30, 60), -1)
    cv2.addWeighted(overlay2, 0.7, display, 0.3, 0, display)

    cv2.putText(display, "[S] Save  [A] Auto-capture  [Q] Quit", (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

    if status_msg and (time.time() - status_time) < 3.0:
        cv2.putText(display, status_msg, (w - 400, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 210, 106), 1)


def main():
    parser = argparse.ArgumentParser(description="Drone Camera Node")
    parser.add_argument("--source", default="csi",
                        help="'csi' for RPi HQ Camera on Jetson (default), USB index, or 'test'")
    parser.add_argument("--auto", type=float, default=0.0,
                        help="Auto-capture interval in seconds (0 = disabled)")
    parser.add_argument("--flip", type=int, default=0,
                        help="Flip method: 0=none, 1=ccw90, 2=180, 3=cw90, 4=h-flip, 5=v-flip")
    args = parser.parse_args()

    os.makedirs(SAVE_DIR, exist_ok=True)

    print(f"[INFO] Opening camera: {args.source}")
    cap = open_camera(args.source, flip=args.flip)
    test_mode = args.source == "test"

    if not test_mode and (cap is None or not cap.isOpened()):
        print(f"[WARN] Cannot open camera '{args.source}', falling back to test pattern")
        test_mode = True
        cap = None

    if test_mode:
        print("[INFO] Running in TEST PATTERN mode (no camera)")
    else:
        print(f"[INFO] Camera opened — {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")

    print(f"[INFO] Saving to: {SAVE_DIR}")
    print(f"[INFO] Keys: [S] save | [A] auto-capture | [Q] quit")

    frame_count = 0
    auto_active = args.auto > 0
    auto_interval = args.auto if args.auto > 0 else 2.0
    last_auto_save = time.time()
    status_msg = ""
    status_time = 0.0
    source_label = "test" if test_mode else args.source

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

    while True:
        if test_mode:
            frame = generate_test_frame(frame_count)
            ret = True
            time.sleep(0.033)
        else:
            ret, frame = cap.read()

        if not ret:
            print("[ERROR] Failed to read frame")
            break

        frame_count += 1

        if auto_active and (time.time() - last_auto_save) >= auto_interval:
            path = save_frame(frame)
            status_msg = f"Auto-saved: {os.path.basename(path)}"
            status_time = time.time()
            last_auto_save = time.time()

        display = frame.copy()
        draw_hud(display, frame_count, auto_active, auto_interval, status_msg, status_time, source_label)
        cv2.imshow(WINDOW_NAME, display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("s"):
            path = save_frame(frame)
            status_msg = f"Saved: {os.path.basename(path)}"
            status_time = time.time()
        elif key == ord("a"):
            auto_active = not auto_active
            state = "ON" if auto_active else "OFF"
            status_msg = f"Auto-capture {state} ({auto_interval:.1f}s)"
            status_time = time.time()
            last_auto_save = time.time()
            print(f"[INFO] Auto-capture: {state}")

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
