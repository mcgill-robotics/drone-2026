import json
import os
import sys
import time
import cv2
import numpy as np


# Known-good values. calibration.json overrides any of these; if it is missing
# the tool must still work, so these are kept current with what actually works
# rather than left at loose initial guesses.
DEFAULTS = {
    # Detection colour mask: h1/h2 are the target hue bands (rose/magenta wraps
    # around 0 and 180); s_lo/v_lo are floors that, when adapt_floors is on,
    # serve only as fallbacks for the per-frame estimate.
    "h1_lo": 0,   "h1_hi": 10,
    "h2_lo": 160, "h2_hi": 180,
    "s_lo":  40,  "s_hi": 245,
    "v_lo": 120,  "v_hi": 255,
    "kernel": 9,
    "min_area": 3000,
    "circularity": 0.6,
    "aspect": 0.65,
    "solidity": 0.9,
    # Per-frame adaptation: derive s_lo/v_lo from the frame's own histogram so
    # the mask tracks ambient brightness instead of relying on fixed floors.
    "adapt_floors": 1,
    # Depth-gated size check (lighting-invariant). A real target must have an
    # apparent pixel size consistent with a physical diameter in this range.
    # Target is 5-30 cm; tolerance band widened to absorb depth/contour noise.
    "min_diameter_m": 0.03,
    "max_diameter_m": 0.45,
    # Temporal lock: once a target is found, prefer candidates within this many
    # pixels of last frame's centroid so the box does not flicker between
    # similar-looking objects.
    "track_radius": 120,
    # Wet/dry classification. A dry target shows its vivid rose/magenta dye;
    # a wet one has lost it and gone blue or pale grey. b_lo..b_hi is the blue
    # hue band, used only to help DETECT wet targets in the colour mask.
    # Classification is INDEPENDENT of the detection bands: it measures the
    # fraction of target pixels carrying the dye -- hue in the dry_h1/dry_h2
    # bands and saturated past dry_s_min -- and calls the target "dry" when
    # that fraction is at least dry_hue_frac.
    "b_lo": 90,
    "b_hi": 140,
    "dry_h1_lo": 0,   "dry_h1_hi": 10,
    "dry_h2_lo": 160, "dry_h2_hi": 180,
    "dry_s_min": 25,
    "dry_hue_frac": 0.5,
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


def depth_size_ok(depth_m, fx, calib, cx, cy, area):
    """Reject blobs whose apparent size is inconsistent with the target's
    real-world diameter at the measured distance. Lighting-invariant.

    depth_m is a metric float32 depth array (or None). Returns
    (ok, distance, real_diameter_m). If depth is unavailable the check passes
    (ok=True) so the pipeline degrades to color+shape only.
    """
    if depth_m is None or not fx:
        return True, None, None
    dist = median_depth(depth_m, cx, cy)
    if dist is None or dist <= 0:
        return True, None, None
    # Equivalent-circle radius of the blob, projected to metres at this depth.
    radius_px = (area / np.pi) ** 0.5
    real_diameter = 2.0 * radius_px * dist / fx
    ok = calib["min_diameter_m"] <= real_diameter <= calib["max_diameter_m"]
    return ok, dist, real_diameter


def _collect_candidates(contours, calib, depth_m, fx, verbose):
    """Filter contours down to plausible targets by shape, size and depth."""
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
        x, y, w, h = cv2.boundingRect(cnt)   # axis-aligned bbox, for drawing
        # Aspect from the minimum-area (rotated) rectangle, so a target seen
        # at an angle is not penalised the way an axis-aligned bbox would be.
        (rw, rh) = cv2.minAreaRect(cnt)[1]
        aspect = min(rw, rh) / max(rw, rh) if max(rw, rh) else 0
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
        size_ok, dist, real_d = depth_size_ok(depth_m, fx, calib, cx, cy, area)
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
    return candidates


def _merge_candidates(primary, extra):
    """Append `extra` candidates that are not duplicates of a `primary` one.

    Two candidates are the same target when their centroids are closer than
    half the primary blob's equivalent-circle radius. Primary (colour-mask)
    candidates are kept in preference to edge-fallback ones.
    """
    merged = list(primary)
    for e in extra:
        ex, ey = e["centroid"]
        dup = False
        for p in primary:
            px, py = p["centroid"]
            r = 0.5 * (p["area"] / np.pi) ** 0.5
            if (ex - px) ** 2 + (ey - py) ** 2 <= r * r:
                dup = True
                break
        if not dup:
            merged.append(e)
    return merged


def classify_wetness(hsv, contour, calib):
    """Label a detected target 'wet' or 'dry'.

    A dry target shows its vivid rose/magenta dye; a wet one has lost it and
    gone blue or pale grey. We measure the fraction of the target's pixels
    that still carry the dye -- hue inside the dry_h1/dry_h2 bands and
    saturated past dry_s_min so the hue is real and not grey-pixel noise. A
    high fraction means dry; a low one means the dye has washed out -> wet.

    The dry_* bands are deliberately separate from the h1/h2 detection bands
    so that widening detection does not change what counts as "dry".

    Returns (label, dry_fraction).
    """
    region = np.zeros(hsv.shape[:2], np.uint8)
    cv2.drawContours(region, [contour], -1, 255, thickness=cv2.FILLED)
    pix = hsv[region > 0]
    if len(pix) == 0:
        return "wet", 0.0
    h, s = pix[:, 0].astype(int), pix[:, 1]
    dry_hue = (((h >= calib["dry_h1_lo"]) & (h <= calib["dry_h1_hi"])) |
               ((h >= calib["dry_h2_lo"]) & (h <= calib["dry_h2_hi"])))
    dry = dry_hue & (s >= calib["dry_s_min"])
    frac = float(np.count_nonzero(dry)) / len(pix)
    return ("dry" if frac >= calib["dry_hue_frac"] else "wet"), frac


def annotate_target(frame, stats):
    """Draw the target outline, centroid and wet/dry label onto frame."""
    cnt = stats["cnt"]
    if len(cnt) >= 5:
        cv2.ellipse(frame, cv2.fitEllipse(cnt), (0, 255, 0), 2)
    else:
        x, y, w, h = stats["bbox"]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    if stats.get("centroid"):
        cv2.circle(frame, stats["centroid"], 4, (255, 0, 0), -1)
    wetness = stats.get("wetness", "")
    bx, by = stats["bbox"][0], stats["bbox"][1]
    wet_color = (255, 80, 0) if wetness == "wet" else (0, 200, 0)
    cv2.putText(frame, wetness.upper(), (bx, max(22, by - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, wet_color, 2)


def detect(frame, calib, verbose=False, depth_frame=None, fx=None,
           prev_centroid=None, draw=True):
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Convert the RealSense depth frame to a metric float32 array once, so the
    # depth size gate can slice it instead of calling get_distance per pixel.
    depth_m = None
    if depth_frame is not None:
        depth_m = (np.asanyarray(depth_frame.get_data()).astype(np.float32)
                   * depth_frame.get_units())

    # Saturation/value floors. With adapt_floors on, derive them from this
    # frame's own histogram -- but only ever LOWER them: in a dim or washed-out
    # frame the target dims too, so fixed floors would reject it. Never raise
    # them above the calibrated values (that could let background in).
    s_lo, v_lo = calib["s_lo"], calib["v_lo"]
    if calib.get("adapt_floors", 0):
        s_lo = int(min(s_lo, np.percentile(hsv[:, :, 1], 40)))
        v_lo = int(min(v_lo, np.percentile(hsv[:, :, 2], 30)))

    m1 = cv2.inRange(hsv,
                     np.array([calib["h1_lo"], s_lo, v_lo]),
                     np.array([calib["h1_hi"], calib["s_hi"], calib["v_hi"]]))
    if calib["h2_hi"] >= calib["h2_lo"]:
        m2 = cv2.inRange(hsv,
                         np.array([calib["h2_lo"], s_lo, v_lo]),
                         np.array([calib["h2_hi"], calib["s_hi"], calib["v_hi"]]))
        mask = cv2.bitwise_or(m1, m2)
    else:
        mask = m1

    # Also catch wet (blue) targets so they can be detected and then labelled.
    if calib["b_hi"] >= calib["b_lo"]:
        mb = cv2.inRange(hsv,
                         np.array([calib["b_lo"], s_lo, v_lo]),
                         np.array([calib["b_hi"], calib["s_hi"], calib["v_hi"]]))
        mask = cv2.bitwise_or(mask, mb)

    k = max(1, int(calib["kernel"]) | 1)
    kernel = np.ones((k, k), np.uint8)
    solid = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    solid = cv2.morphologyEx(solid, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(solid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Collect every colour-mask contour that passes all filters.
    candidates = _collect_candidates(contours, calib, depth_m, fx, verbose)

    # Always also run edge-based detection. The colour mask misses
    # low-saturation (grey) targets, and finding a colour false positive must
    # not stop us seeing a real grey target in the same frame. Edge detection
    # picks up more clutter, so edge candidates are held to tighter limits.
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    edges = cv2.dilate(edges, kernel)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    sc, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    strict = dict(calib)
    strict["circularity"] = max(calib["circularity"], 0.72)
    strict["solidity"] = max(calib["solidity"], 0.93)
    strict["min_area"] = max(calib["min_area"], 6000)
    edge_candidates = _collect_candidates(sc, strict, depth_m, fx, verbose)

    # Merge: keep all colour candidates, add edge candidates that are not
    # duplicates of one. Show both masks in the debug view.
    candidates = _merge_candidates(candidates, edge_candidates)
    solid = cv2.bitwise_or(solid, edges)
    if verbose and edge_candidates:
        print(f"edge fallback contributed {len(edge_candidates)} candidate(s)")

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
        M = cv2.moments(best)
        if M["m00"] > 0:
            cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
            best_stats["centroid"] = (cx, cy)
        # Wet vs dry: does the target still show its rose/magenta dye?
        wetness, dry_frac = classify_wetness(hsv, best, calib)
        best_stats["wetness"] = wetness
        best_stats["dry_fraction"] = dry_frac
        if draw:
            annotate_target(frame, best_stats)
        if verbose:
            print(f"detected: area={best_stats['area']:.0f} "
                  f"circ={best_stats['circularity']:.2f} "
                  f"aspect={best_stats['aspect']:.2f} "
                  f"solidity={best_stats['solidity']:.2f} "
                  f"centroid={best_stats.get('centroid')} "
                  f"wetness={wetness} (dry_frac={dry_frac:.2f})")
    elif verbose:
        print("no detection")

    return frame, solid, best_stats


def median_depth(depth_m, cx, cy, half=4):
    """Median depth (metres) in a small window around (cx, cy), ignoring zeros.

    depth_m is a float32 array of metric depth (0 = no reading). Vectorised --
    one array slice instead of per-pixel get_distance() calls.
    """
    h, w = depth_m.shape
    win = depth_m[max(0, cy - half):min(h, cy + half + 1),
                  max(0, cx - half):min(w, cx + half + 1)]
    valid = win[win > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid))


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

    # Let auto-exposure and auto-white-balance converge, then freeze BOTH.
    # A locked WB with auto-exposure still running lets brightness (and so the
    # apparent saturation) drift frame to frame -- that drift is what made the
    # colour mask need constant re-tuning. Freezing both makes a frame's HSV
    # reproducible without hard-coding scene-specific exposure values.
    color_sensor = profile.get_device().first_color_sensor()
    try:
        color_sensor.set_option(rs.option.enable_auto_exposure, 1)
        color_sensor.set_option(rs.option.enable_auto_white_balance, 1)
        for _ in range(30):
            pipeline.wait_for_frames()
        color_sensor.set_option(rs.option.enable_auto_exposure, 0)
        color_sensor.set_option(rs.option.enable_auto_white_balance, 0)
    except Exception as e:
        print(f"Warning: could not lock color sensor options: {e}")

    # Color intrinsics: fx (focal length in pixels) drives the depth size gate.
    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    fx = color_profile.get_intrinsics().fx

    align = rs.align(rs.stream.color)
    # Temporal lock, expressed in TIME not frame counts so behaviour does not
    # depend on the actual frame rate.
    CONFIRM_SECONDS = 0.3   # a candidate must persist this long to be trusted
    LOST_SECONDS = 0.3      # drop the lock after this long with no detection
    base_radius = calib["track_radius"]

    prev_centroid = None    # last frame's target position
    streak_start = None     # time the current stable run began
    last_seen = None        # time of the most recent detection
    confirmed = False       # True once a candidate persisted CONFIRM_SECONDS
    recent_motion = 0.0     # smoothed frame-to-frame centroid displacement (px)
    try:
        while True:
            frames = align.process(pipeline.wait_for_frames())
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            now = time.monotonic()
            frame = np.asanyarray(color_frame.get_data())
            # Detect without drawing: a candidate is annotated only once it has
            # persisted long enough, which rejects transient false positives
            # (a passing hand, a hoodie, background flicker).
            annotated, mask, stats = detect(frame, calib, verbose=verbose,
                                            depth_frame=depth_frame, fx=fx,
                                            prev_centroid=prev_centroid,
                                            draw=False)

            centroid = stats.get("centroid") if stats else None
            if centroid:
                if prev_centroid is None:
                    stable, motion = True, 0.0
                else:
                    dx = centroid[0] - prev_centroid[0]
                    dy = centroid[1] - prev_centroid[1]
                    motion = (dx * dx + dy * dy) ** 0.5
                    # The gate grows with recent motion, so on a moving drone
                    # -- where the target legitimately shifts a lot per frame
                    # -- a real target still counts as the same lock.
                    gate = base_radius + 2.0 * recent_motion
                    stable = motion <= gate
                if stable:
                    recent_motion = 0.7 * recent_motion + 0.3 * motion
                    if streak_start is None:
                        streak_start = now
                else:
                    streak_start = now      # treat as a fresh run
                    recent_motion = 0.0
                prev_centroid = centroid
                last_seen = now
                if now - streak_start >= CONFIRM_SECONDS:
                    confirmed = True
            else:
                streak_start = None
                if last_seen is not None and now - last_seen >= LOST_SECONDS:
                    prev_centroid = None
                    confirmed = False
                    recent_motion = 0.0

            if confirmed and centroid:
                annotate_target(annotated, stats)
                cx, cy = centroid
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
