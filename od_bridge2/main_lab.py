import json
import os
import sys
import time
import cv2
import numpy as np

# The temporal-confirmation state machine is pipeline-agnostic, so we reuse
# main.py's class rather than duplicate it. Keeps both paths in lockstep.
from main import TemporalLock


# Known-good values. calibration.json overrides any of these; if it is missing
# the tool must still work, so these are kept current with what actually works
# rather than left at loose initial guesses.
DEFAULTS = {
    # Detection colour mask: LAB space. OpenCV uint8 LAB ranges are:
    # L (0-255), a (0-255, 128 is neutral), b (0-255, 128 is neutral).
    # Magenta/Rose target: prominent positive 'a' (red/magenta).
    "tgt_L_lo": 80,  "tgt_L_hi": 255,
    "tgt_a_lo": 145, "tgt_a_hi": 255,
    "tgt_b_lo": 110, "tgt_b_hi": 170,
    "kernel": 9,
    "min_area": 3000,
    # circularity is only a fallback shape gate for contours too small to fit
    # an ellipse; the main gate is ellipticity (tilt-invariant -- a circle at
    # any viewing angle is still a clean ellipse).
    "circularity": 0.6,
    "ellipticity_min": 0.80,
    "ellipticity_max": 1.25,
    # aspect only rejects extreme slivers; 0.25 still allows a very steeply
    # tilted (~75 deg, foreshortened) circular target.
    "aspect": 0.25,
    "solidity": 0.9,
    # Per-frame adaptation: derive L_lo from the frame's own histogram so
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
    # Wet target (blue/grey fallback): low 'b', relatively neutral 'a'.
    # Used only to help DETECT wet targets in the colour mask.
    "wet_L_lo": 80,  "wet_L_hi": 255,
    "wet_a_lo": 110, "wet_a_hi": 150,
    "wet_b_lo": 0,   "wet_b_hi": 115,
    # Wet/dry classification. A dry pixel sits on the warm side of LAB
    # neutral (a >= dry_a_lo), is not strongly blue (b >= dry_b_lo), and is at
    # least about as red as it is yellow (a >= b - dry_a_minus_b). That last
    # condition is what separates a real rose pixel from a warm-light-on-grey
    # pixel that would otherwise look "dry" by raw a/b alone.
    "dry_a_lo": 128,
    "dry_b_lo": 115,
    "dry_a_minus_b": 0,
    # Minimum chroma (distance from neutral in a/b) for a pixel to count as
    # carrying the dye. Without this a near-neutral pixel under a warm light
    # passes the a/b box and gets called "dry" even though it is just grey.
    # Set high enough to reject your live wet target (whose centre measures
    # chroma ~6) and low enough to keep ordinary indoor dry rose targets
    # (chroma 10+).
    "dry_chroma_min": 8.0,
    "dry_hue_frac": 0.5,
}


def load_calibration():
    # Looks for a calibration_lab.json first, falls back to calibration.json if not found,
    # but uses the LAB DEFAULTS.
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration_lab.json")
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
        x, y, w, h = cv2.boundingRect(cnt)   # axis-aligned bbox, for drawing
        # Aspect from the minimum-area (rotated) rectangle.
        (rw, rh) = cv2.minAreaRect(cnt)[1]
        aspect = min(rw, rh) / max(rw, rh) if max(rw, rh) else 0
        # Shape gate: ellipse-fit quality, not raw circularity. A round target
        # viewed at an angle foreshortens into an ELLIPSE -- still a clean
        # ellipse, so ellipticity stays ~1.0 at any tilt where circularity
        # would collapse. ellipticity = blob area / best-fit-ellipse area.
        if len(cnt) >= 5:
            (erw, erh) = cv2.fitEllipse(cnt)[1]
            ell_area = np.pi * (erw / 2.0) * (erh / 2.0)
            ellipticity = area / ell_area if ell_area else 0.0
            if not (calib["ellipticity_min"] <= ellipticity
                    <= calib["ellipticity_max"]):
                continue
        else:
            ellipticity = 0.0
            if circ < calib["circularity"]:
                continue
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
            "ellipticity": ellipticity,
            "solidity": solidity, "bbox": (x, y, w, h), "centroid": (cx, cy),
            "distance": dist, "real_diameter": real_d,
        })
    return candidates


def _is_plausible_target(lab, contour, calib):
    """Veto candidates whose interior cannot plausibly be ANY kind of target.

    A target paper is rose/magenta (dry dye -> positive a), blue (wet ->
    low b), or near-neutral grey (washed-out wet -> a and b near 128).
    A region dominantly green / yellow / strongly warm is not a target no
    matter how circular -- this catches bogus shape matches on busy scenes
    where the LAB mask alone is too permissive.
    """
    region = np.zeros(lab.shape[:2], np.uint8)
    cv2.drawContours(region, [contour], -1, 255, thickness=cv2.FILLED)
    pix = lab[region > 0]
    if len(pix) == 0:
        return False
    a, b = pix[:, 1].astype(int), pix[:, 2].astype(int)
    rose = (a >= 130) & (b <= 160)
    blue = b <= 115
    grey = (np.abs(a - 128) <= 8) & (np.abs(b - 128) <= 8)
    plausible = rose | blue | grey
    return float(plausible.mean()) >= calib.get("plausibility_min", 0.5)


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


def classify_wetness(lab, contour, calib):
    """Label a detected target 'wet' or 'dry'.

    A dry target shows its vivid rose/magenta dye; a wet one has either gone
    blue or lost the dye and reads as grey/washed-out under whatever ambient
    light it is sitting in.

    A pixel counts as carrying the dye when ALL of:
      - a >= dry_a_lo: it is on the warm/red side of LAB neutral.
      - b >= dry_b_lo: it is NOT strongly blue (rules out the wet/blue case).
      - a >= b - dry_a_minus_b: it is at least about as red as it is yellow.
        A warm-light-on-grey pixel leans yellow so b ends up above a.
      - chroma (a,b distance from neutral) >= dry_chroma_min: the pixel
        actually carries enough colour to be a dye signal, not just lighting
        noise on a near-grey paper.

    Returns (label, dry_fraction).
    """
    region = np.zeros(lab.shape[:2], np.uint8)
    cv2.drawContours(region, [contour], -1, 255, thickness=cv2.FILLED)
    pix = lab[region > 0]
    if len(pix) == 0:
        return "wet", 0.0
    a = pix[:, 1].astype(int)
    b = pix[:, 2].astype(int)
    chroma = np.sqrt((a - 128) ** 2 + (b - 128) ** 2)
    dry = ((a >= calib["dry_a_lo"]) &
           (b >= calib["dry_b_lo"]) &
           (a >= b - calib["dry_a_minus_b"]) &
           (chroma >= calib["dry_chroma_min"]))
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
    lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)

    # Convert the RealSense depth frame to a metric float32 array once, so the
    # depth size gate can slice it instead of calling get_distance per pixel.
    depth_m = None
    if depth_frame is not None:
        depth_m = (np.asanyarray(depth_frame.get_data()).astype(np.float32)
                   * depth_frame.get_units())

    # Lightness floors. With adapt_floors on, derive them from this
    # frame's own histogram. Never raise them above the calibrated values.
    L_lo = calib["tgt_L_lo"]
    wet_L_lo = calib["wet_L_lo"]
    if calib.get("adapt_floors", 0):
        ambient_L = np.percentile(lab[:, :, 0], 30)
        L_lo = int(min(L_lo, ambient_L))
        wet_L_lo = int(min(wet_L_lo, ambient_L))

    m_tgt = cv2.inRange(lab,
                        np.array([L_lo, calib["tgt_a_lo"], calib["tgt_b_lo"]]),
                        np.array([calib["tgt_L_hi"], calib["tgt_a_hi"], calib["tgt_b_hi"]]))
    
    m_wet = cv2.inRange(lab,
                        np.array([wet_L_lo, calib["wet_a_lo"], calib["wet_b_lo"]]),
                        np.array([calib["wet_L_hi"], calib["wet_a_hi"], calib["wet_b_hi"]]))
    
    mask = cv2.bitwise_or(m_tgt, m_wet)

    k = max(1, int(calib["kernel"]) | 1)
    kernel = np.ones((k, k), np.uint8)
    solid = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    solid = cv2.morphologyEx(solid, cv2.MORPH_OPEN, kernel)

    # Mask-coverage sanity guard. When the LAB mask paints most of the frame
    # (typical of a low-saturation webcam scene), any "contour" is meaningless
    # -- skip the colour path and rely on the stricter edge fallback alone.
    # The displayed mask is blanked so the debug view reflects this.
    mask_coverage = float((solid > 0).mean())
    if mask_coverage > calib.get("mask_coverage_max", 0.4):
        if verbose:
            print(f"LAB mask too permissive ({mask_coverage:.0%} of frame); "
                  f"skipping colour candidates")
        contours = []
        solid = np.zeros_like(solid)
        cv2.putText(solid, "MASK SUPPRESSED", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, 255, 2)
    else:
        contours, _ = cv2.findContours(solid, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
    # Shape + depth filter, then a target-colour plausibility veto so a
    # circular non-target (round bright object, head, etc.) is rejected.
    candidates = _collect_candidates(contours, calib, depth_m, fx, verbose)
    candidates = [c for c in candidates if _is_plausible_target(lab, c["cnt"], calib)]

    # Always also run edge-based detection. The colour mask misses
    # low-saturation (grey) targets, and finding a colour false positive must
    # not stop us seeing a real grey target in the same frame. Edge detection
    # picks up more clutter, so edge candidates are held to tighter limits.
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    # Low Canny thresholds: a low-contrast target edge fades further at angle.
    # (50,150) leaves gaps; (30,90) keeps the loop closed.
    edges = cv2.Canny(gray, 30, 90)
    edges = cv2.dilate(edges, kernel)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    # RETR_LIST: the target's edge loop is often connected to card/background
    # edges, so its outer boundary is a giant blob -- but the target interior
    # is still an enclosed hole, which RETR_LIST returns as its own contour.
    sc, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    strict = dict(calib)
    strict["ellipticity_min"] = max(calib["ellipticity_min"], 0.85)
    strict["ellipticity_max"] = min(calib["ellipticity_max"], 1.18)
    strict["solidity"] = max(calib["solidity"], 0.93)
    strict["min_area"] = max(calib["min_area"], 6000)
    edge_candidates = _collect_candidates(sc, strict, depth_m, fx, verbose)
    edge_candidates = [c for c in edge_candidates
                       if _is_plausible_target(lab, c["cnt"], calib)]

    # Merge: keep all colour candidates, add edge candidates that are not
    # duplicates of one. The displayed mask is the COLOUR mask alone -- ORing
    # the edges in floods the debug view with white on busy scenes.
    candidates = _merge_candidates(candidates, edge_candidates)
    if verbose and edge_candidates:
        print(f"edge fallback contributed {len(edge_candidates)} candidate(s)")

    # Pick the target. Rank on how cleanly the blob matches an ellipse
    # (tilt-invariant, unlike circularity), then on area, so a background blob
    # cannot win just by being bigger.
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
        best_stats = max(pool, key=lambda c: (-abs(c["ellipticity"] - 1.0),
                                              c["area"]))
    best = best_stats["cnt"] if best_stats else None

    if best is not None:
        M = cv2.moments(best)
        if M["m00"] > 0:
            cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
            best_stats["centroid"] = (cx, cy)
        # Wet vs dry: does the target still show its rose/magenta dye?
        wetness, dry_frac = classify_wetness(lab, best, calib)
        best_stats["wetness"] = wetness
        best_stats["dry_fraction"] = dry_frac
        if draw:
            annotate_target(frame, best_stats)
        if verbose:
            print(f"detected: area={best_stats['area']:.0f} "
                  f"ellipticity={best_stats['ellipticity']:.2f} "
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
    # colour mask need constant re-tuning. Freezing both makes a frame's LAB
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
    lock = TemporalLock(calib["track_radius"])
    try:
        while True:
            frames = align.process(pipeline.wait_for_frames())
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            # Detect without drawing: the temporal lock decides when to draw,
            # which rejects transient false positives (passing hand, hoodie,
            # background flicker).
            annotated, mask, stats = detect(frame, calib, verbose=verbose,
                                            depth_frame=depth_frame, fx=fx,
                                            prev_centroid=lock.prev_centroid,
                                            draw=False)

            centroid = stats.get("centroid") if stats else None
            confirmed = lock.update(centroid, time.monotonic())

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

            cv2.imshow('Live Dry Target Detection LAB', annotated)
            cv2.imshow('Mask LAB', mask)
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
        cv2.imshow('Dry Target Detection LAB', annotated)
        cv2.imshow('Mask LAB', mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return

    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"Could not open camera (device {cam_index}). "
              f"Try a different index with --cam <N> (USB cameras are usually 1 or 2). "
              f"Also check no other app is using it and permissions are granted.")
        sys.exit(1)
    lock = TemporalLock(calib["track_radius"])
    save_counter = 0
    while True:
        ret, cap_frame = cap.read()
        if not ret:
            break
        # Detect without drawing; the temporal lock decides when to annotate.
        # This stops false positives flashing on screen every frame on a busy
        # webcam feed where no real target is present.
        annotated, mask, stats = detect(cap_frame, calib, verbose=verbose,
                                        prev_centroid=lock.prev_centroid,
                                        draw=False)
        centroid = stats.get("centroid") if stats else None
        if lock.update(centroid, time.monotonic()) and centroid:
            annotate_target(annotated, stats)
        cv2.imshow('Live Dry Target Detection LAB', annotated)
        cv2.imshow('Mask LAB', mask)
        key = cv2.waitKey(1) & 0xFF
        # Press 's' to dump the current raw frame so we can measure the target
        # LAB values offline -- much more useful than guessing thresholds.
        if key == ord('s'):
            save_counter += 1
            path = f"webcam_capture_{save_counter}.png"
            cv2.imwrite(path, cap_frame)
            print(f"saved {path}")
        if key == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
