import json
import os
import sys
import cv2
import numpy as np


DEFAULTS = {
    "h1_lo": 0,   "h1_hi": 8,
    "h2_lo": 160, "h2_hi": 180,
    "s_lo":  40,  "s_hi": 140,
    "v_lo": 120,  "v_hi": 235,
    "kernel": 9,
    "min_area": 500,
    "circularity": 0.45,
    "aspect": 0.5,
    "solidity": 0.9,
}


def load_calibration():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")
    if not os.path.exists(path):
        return dict(DEFAULTS)
    with open(path) as f:
        loaded = json.load(f)
    calib = dict(DEFAULTS)
    calib.update(loaded)
    return calib


def detect(frame, calib, verbose=False):
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    m1 = cv2.inRange(hsv,
                     np.array([calib["h1_lo"], calib["s_lo"], calib["v_lo"]]),
                     np.array([calib["h1_hi"], calib["s_hi"], calib["v_hi"]]))
    if calib["h2_hi"] >= calib["h2_lo"]:
        m2 = cv2.inRange(hsv,
                         np.array([calib["h2_lo"], calib["s_lo"], calib["v_lo"]]),
                         np.array([calib["h2_hi"], calib["s_hi"], calib["v_hi"]]))
        mask = cv2.bitwise_or(m1, m2)
    else:
        mask = m1

    k = max(1, int(calib["kernel"]) | 1)
    kernel = np.ones((k, k), np.uint8)
    solid = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    solid = cv2.morphologyEx(solid, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(solid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area, best_stats = None, 0, None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < calib["min_area"] or area <= best_area:
            continue
        perim = cv2.arcLength(cnt, True)
        if perim == 0:
            continue
        circ = 4 * np.pi * area / (perim * perim)
        if circ < calib["circularity"]:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        aspect = min(w, h) / max(w, h) if max(w, h) else 0
        if aspect < calib["aspect"]:
            continue
        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        solidity = area / hull_area if hull_area else 0
        if solidity < calib["solidity"]:
            continue
        best, best_area = cnt, area
        best_stats = {"area": area, "circularity": circ, "aspect": aspect,
                      "solidity": solidity, "bbox": (x, y, w, h)}

    if best is not None:
        if len(best) >= 5:
            ellipse = cv2.fitEllipse(best)
            cv2.ellipse(frame, ellipse, (0, 255, 0), 2)
        else:
            x, y, w, h = best_stats["bbox"]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        M = cv2.moments(best)
        if M["m00"] > 0:
            cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
            cv2.circle(frame, (cx, cy), 4, (255, 0, 0), -1)
            best_stats["centroid"] = (cx, cy)
        if verbose:
            print(f"detected: area={best_stats['area']:.0f} "
                  f"circ={best_stats['circularity']:.2f} "
                  f"aspect={best_stats['aspect']:.2f} "
                  f"solidity={best_stats['solidity']:.2f} "
                  f"centroid={best_stats.get('centroid')}")
    elif verbose:
        print("no detection")

    return frame, solid, best_stats


def median_depth(depth_frame, cx, cy, half=4):
    """Median depth (meters) over a small window around (cx, cy), ignoring zeros."""
    w, h = depth_frame.get_width(), depth_frame.get_height()
    vals = []
    for y in range(max(0, cy - half), min(h, cy + half + 1)):
        for x in range(max(0, cx - half), min(w, cx + half + 1)):
            d = depth_frame.get_distance(x, y)
            if d > 0:
                vals.append(d)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def run_realsense(calib, verbose=False):
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    try:
        pipeline.start(config)
    except RuntimeError as e:
        print(f"Could not start RealSense pipeline: {e}")
        print("Check the camera is connected (run: rs-enumerate-devices).")
        sys.exit(1)

    align = rs.align(rs.stream.color)
    try:
        while True:
            frames = align.process(pipeline.wait_for_frames())
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            annotated, mask, stats = detect(frame, calib, verbose=verbose)

            if stats and stats.get("centroid"):
                cx, cy = stats["centroid"]
                dist = median_depth(depth_frame, cx, cy)
                if dist is not None:
                    label = f"{dist:.2f} m"
                    cv2.putText(annotated, label, (cx + 10, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    if verbose:
                        print(f"target distance: {dist:.2f} m at {(cx, cy)}")
                elif verbose:
                    print("target detected but no valid depth at centroid")

            cv2.imshow('Live Dry Target Detection', annotated)
            cv2.imshow('Mask', mask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


def main():
    raw = sys.argv[1:]
    verbose = "--verbose" in raw
    use_realsense = "--realsense" in raw
    raw = [a for a in raw if a != "--realsense"]

    cam_index = 0
    if "--cam" in raw:
        i = raw.index("--cam")
        cam_index = int(raw[i + 1])
        raw = raw[:i] + raw[i + 2:]

    args = [a for a in raw if a != "--verbose"]
    calib = load_calibration()

    if use_realsense:
        run_realsense(calib, verbose=verbose)
        return

    if args:
        img = cv2.imread(args[0])
        if img is None:
            print(f"Could not read {args[0]}")
            sys.exit(1)
        h, w = img.shape[:2]
        scale = min(1.0, 900 / max(h, w))
        if scale < 1.0:
            img = cv2.resize(img, (int(w * scale), int(h * scale)))
        annotated, mask, _ = detect(img, calib, verbose=True)
        cv2.imshow('Dry Target Detection', annotated)
        cv2.imshow('Mask', mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return

    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"Could not open camera (device {cam_index}). "
              f"Try a different index with --cam <N> (USB cameras are usually 1 or 2). "
              f"Also check no other app is using it and permissions are granted.")
        sys.exit(1)
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        annotated, mask, _ = detect(frame, calib, verbose=verbose)
        cv2.imshow('Live Dry Target Detection', annotated)
        cv2.imshow('Mask', mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
