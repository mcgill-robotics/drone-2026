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
    # Depth-gated size check (lighting-invariant). A real target must have an
    # apparent pixel size consistent with a physical diameter in this range.
    # Target is 5-30 cm; tolerance band widened to absorb depth/contour noise.
    "min_diameter_m": 0.03,
    "max_diameter_m": 0.45,
    # Temporal lock: once a target is found, prefer candidates within this many
    # pixels of last frame's centroid so the box does not flicker between
    # similar-looking objects.
    "track_radius": 120,
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


def depth_size_ok(depth_frame, fx, calib, cx, cy, area):
    """Reject blobs whose apparent size is inconsistent with the target's
    real-world diameter at the measured distance. Lighting-invariant.

    Returns (ok, distance, real_diameter_m). If depth is unavailable the check
    passes (ok=True) so the pipeline degrades to color+shape only.
    """
    if depth_frame is None or not fx:
        return True, None, None
    dist = median_depth(depth_frame, cx, cy)
    if dist is None or dist <= 0:
        return True, None, None
    # Equivalent-circle radius of the blob, projected to metres at this depth.
    radius_px = (area / np.pi) ** 0.5
    real_diameter = 2.0 * radius_px * dist / fx
    ok = calib["min_diameter_m"] <= real_diameter <= calib["max_diameter_m"]
    return ok, dist, real_diameter


def detect(frame, calib, verbose=False, depth_frame=None, fx=None,
           prev_centroid=None):
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
    # Collect every contour that passes all filters, then choose among them.
    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < calib["min_area"]:
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
        # Depth-gated size check: reject blobs whose apparent size is
        # inconsistent with a 5-30 cm target at the measured distance.
        M = cv2.moments(cnt)
        if M["m00"] <= 0:
            continue
        cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
        size_ok, dist, real_d = depth_size_ok(depth_frame, fx, calib, cx, cy, area)
        if not size_ok:
            if verbose:
                print(f"rejected by depth size gate: real_diam={real_d:.2f} m "
                      f"at {dist:.2f} m")
            continue
        candidates.append({
            "cnt": cnt, "area": area, "circularity": circ, "aspect": aspect,
            "solidity": solidity, "bbox": (x, y, w, h), "centroid": (cx, cy),
            "distance": dist, "real_diameter": real_d,
        })

    # Pick the target. Circularity is the strongest cue for a round target, so
    # rank on that rather than raw area (a background blob can be bigger).
    # If we had a detection last frame, restrict to candidates near it first so
    # the lock does not flicker between two valid-looking objects.
    best_stats = None
    if candidates:
        pool = candidates
        if prev_centroid is not None:
            px, py = prev_centroid
            gated = [c for c in candidates
                     if (c["centroid"][0] - px) ** 2 + (c["centroid"][1] - py) ** 2
                     <= calib["track_radius"] ** 2]
            if gated:
                pool = gated
        best_stats = max(pool, key=lambda c: (c["circularity"], c["area"]))
    best = best_stats["cnt"] if best_stats else None

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
        profile = pipeline.start(config)
    except RuntimeError as e:
        print(f"Could not start RealSense pipeline: {e}")
        print("Check the camera is connected (run: rs-enumerate-devices).")
        sys.exit(1)

    # Lock white balance so hue stays stable across indoor/outdoor lighting;
    # keep auto-exposure ON so the sensor adapts to the brightness range.
    color_sensor = profile.get_device().first_color_sensor()
    try:
        color_sensor.set_option(rs.option.enable_auto_white_balance, 0)
        color_sensor.set_option(rs.option.enable_auto_exposure, 1)
    except Exception as e:
        print(f"Warning: could not set color sensor options: {e}")

    # Color intrinsics: fx (focal length in pixels) drives the depth size gate.
    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    fx = color_profile.get_intrinsics().fx

    align = rs.align(rs.stream.color)
    prev_centroid = None   # last frame's target position, for the temporal lock
    misses = 0             # consecutive frames with no detection
    MAX_MISSES = 8         # drop the lock after this many empty frames
    try:
        while True:
            frames = align.process(pipeline.wait_for_frames())
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            annotated, mask, stats = detect(frame, calib, verbose=verbose,
                                            depth_frame=depth_frame, fx=fx,
                                            prev_centroid=prev_centroid)

            if stats and stats.get("centroid"):
                prev_centroid = stats["centroid"]
                misses = 0
            else:
                misses += 1
                if misses >= MAX_MISSES:
                    prev_centroid = None

            if stats and stats.get("centroid"):
                cx, cy = stats["centroid"]
                dist = stats.get("distance")
                if dist is not None:
                    label = f"{dist:.2f} m  ~{stats['real_diameter'] * 100:.0f} cm"
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
