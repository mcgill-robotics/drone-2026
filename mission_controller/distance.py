#!/usr/bin/env python3
"""
Click-to-measure 3D distance using a KITTI Raw (synced) camera image + Velodyne point cloud.

What this script does
- Loads a KITTI Raw frame image (default: image_02 left color camera).
- Loads the matching KITTI Velodyne point cloud (.bin).
- Loads KITTI Raw calibration files from the same sequence folder:
    - calib_velo_to_cam.txt
    - calib_cam_to_cam.txt
- Builds a single projection matrix P (3x4) mapping Velodyne XYZ1 -> image pixels.
- Projects LiDAR points into the image.
- UI: click two pixels; for each click, finds nearest projected LiDAR point and retrieves its 3D position.
- Outputs the 3D distance between the two chosen points (meters) and overlays it on the image.

Requirements
- Python 3.x
- opencv-python
- numpy

Example
Assuming:
  ./distance_gui_kitti.py
  ./2011_09_26/2011_09_26_drive_0005_sync/...

Run:
  python3 distance_gui_kitti.py --seq 2011_09_26/2011_09_26_drive_0005_sync --frame 0 --show_overlay
"""

import argparse
import os
from dataclasses import dataclass
from typing import Tuple, List, Dict, Optional

import cv2
import numpy as np


# -----------------------------
# Data structures
# -----------------------------
@dataclass
class Calibration:
    P: np.ndarray      # 3x4 velo->img
    Tr: np.ndarray     # 4x4 velo->cam (unrectified)
    R_rect4: np.ndarray # 4x4 rectification



# -----------------------------
# KITTI calibration loading
# -----------------------------
def _read_kitti_calib_file(path: str) -> Dict[str, np.ndarray]:
    """
    Reads KITTI calibration text file into dict of numpy arrays (flat).
    Lines look like:  "P_rect_02: 7.2e+02 ..."

    Returns dict key -> np.ndarray of floats (flat).
    """
    if not os.path.isfile(path):
        raise FileNotFoundError(f"Calibration file not found: {path}")

    data: Dict[str, np.ndarray] = {}
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            vals = np.fromstring(value.strip(), sep=" ")
            data[key.strip()] = vals
    return data


def load_kitti_raw_calibration(seq_dir: str, cam: int = 2) -> Calibration:
    """
    seq_dir example:
      ./2011_09_26/2011_09_26_drive_0005_sync

    Uses:
      - calib_velo_to_cam.txt (R, T)  => Velodyne -> cam0
      - calib_cam_to_cam.txt  (R_rect_00, P_rect_0X)

    Builds:
      P_velo_to_img = P_rect_0{cam} @ R_rect_00_4x4 @ Tr_velo_to_cam_4x4

    cam:
      0/1 = grayscale cams, 2/3 = color cams
      default cam=2 (image_02 left color).
    """
    velo_to_cam_path = os.path.join(seq_dir, "calib_velo_to_cam.txt")
    cam_to_cam_path = os.path.join(seq_dir, "calib_cam_to_cam.txt")

    vc = _read_kitti_calib_file(velo_to_cam_path)
    cc = _read_kitti_calib_file(cam_to_cam_path)

    # Velodyne -> cam0 (unrectified) transform
    R = vc["R"].reshape(3, 3)
    T = vc["T"].reshape(3, 1)

    Tr = np.eye(4, dtype=np.float64)
    Tr[:3, :3] = R
    Tr[:3, 3:4] = T

    # Rectification for cam0
    R_rect = cc["R_rect_00"].reshape(3, 3)
    R_rect4 = np.eye(4, dtype=np.float64)
    R_rect4[:3, :3] = R_rect

    # Projection matrix for selected camera
    key = f"P_rect_0{cam}"
    if key not in cc:
        raise KeyError(f"Could not find {key} in calib_cam_to_cam.txt")
    P_cam = cc[key].reshape(3, 4)

    P = P_cam @ (R_rect4 @ Tr)  # 3x4

    return Calibration(P=P, Tr=Tr, R_rect4=R_rect4)


# -----------------------------
# LiDAR loading
# -----------------------------
def load_lidar_points(path: str) -> np.ndarray:
    """
    Supported:
    - .npy: numpy array of shape (N,3) or (N,4) (XYZ or XYZ+intensity)
    - .bin: KITTI-style float32 (x,y,z,intensity) repeated (N,4)

    Returns Nx3 float64 (XYZ) in Velodyne (LiDAR) frame.
    """
    ext = os.path.splitext(path)[1].lower()

    if ext == ".npy":
        pts = np.load(path)
        if pts.ndim != 2 or pts.shape[1] < 3:
            raise ValueError("Expected .npy with shape (N,3) or (N,4)")
        pts = pts[:, :3]

    elif ext == ".bin":
        raw = np.fromfile(path, dtype=np.float32)
        if raw.size % 4 != 0:
            raise ValueError("Expected .bin with floats in groups of 4 (x,y,z,intensity)")
        pts = raw.reshape(-1, 4)[:, :3]

    else:
        raise ValueError(f"Unsupported LiDAR format: {ext}. Use .npy or .bin")

    return pts.astype(np.float64)


# -----------------------------
# Projection (KITTI)
# -----------------------------
def project_lidar_to_image(
    lidar_xyz: np.ndarray,
    calib: Calibration,
    image_shape: Tuple[int, int],
    min_depth: float = 0.1,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Returns:
    - uv: (M,2) projected pixels
    - Pc_kept: (M,3) corresponding 3D points in RECTIFIED camera frame (depth = Z)
    """
    H, W = image_shape[:2]

    if lidar_xyz.size == 0:
        return np.zeros((0, 2), dtype=np.float64), np.zeros((0, 3), dtype=np.float64)

    N = lidar_xyz.shape[0]
    xyz1 = np.hstack([lidar_xyz, np.ones((N, 1), dtype=np.float64)])  # Nx4

    # Camera-rectified 3D points (for depth + distance)
    Pc_all = (calib.R_rect4 @ (calib.Tr @ xyz1.T)).T[:, :3]  # Nx3
    Zc = Pc_all[:, 2]

    # Project to pixels using P (velo->img)
    proj = (calib.P @ xyz1.T).T  # Nx3
    depth = proj[:, 2]

    # Keep points in front (both should be positive; use camera Z as the real depth)
    front = (Zc > min_depth) & (depth > min_depth)

    Pc = Pc_all[front]
    proj = proj[front]
    depth = depth[front]

    if proj.shape[0] == 0:
        return np.zeros((0, 2), dtype=np.float64), np.zeros((0, 3), dtype=np.float64)

    u = proj[:, 0] / depth
    v = proj[:, 1] / depth

    in_img = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    uv = np.stack([u[in_img], v[in_img]], axis=1)

    Pc_kept = Pc[in_img]

    return uv.astype(np.float64), Pc_kept.astype(np.float64)



# -----------------------------
# Click lookup (fast)
# -----------------------------
def build_pixel_bucket(uv: np.ndarray) -> Dict[Tuple[int, int], List[int]]:
    """
    Buckets projected points by integer pixel coordinate for fast local search.
    Using round is usually nicer than floor for KITTI projections.
    """
    bucket: Dict[Tuple[int, int], List[int]] = {}
    uv_int = np.rint(uv).astype(np.int32)
    for i, (u, v) in enumerate(uv_int):
        key = (int(u), int(v))
        bucket.setdefault(key, []).append(i)
    return bucket


def find_nearest_projected_point(click_xy, uv, Pc, bucket, search_radius_px=15):
    cx, cy = click_xy
    best_i, best_d2, best_z = None, float("inf"), float("inf")

    for du in range(-search_radius_px, search_radius_px + 1):
        for dv in range(-search_radius_px, search_radius_px + 1):
            key = (cx + du, cy + dv)
            if key not in bucket:
                continue
            for i in bucket[key]:
                dx = float(uv[i,0] - cx); dy = float(uv[i,1] - cy)
                d2 = dx*dx + dy*dy
                z = float(Pc[i,2])   # depth in camera frame

                # primary: pixel closeness, secondary: closer depth
                if d2 < best_d2 - 1e-9 or (abs(d2 - best_d2) < 1e-9 and z < best_z):
                    best_i, best_d2, best_z = i, d2, z

    if best_i is None:
        return None, None
    return Pc[best_i].copy(), uv[best_i].copy()


# -----------------------------
# Drawing helpers
# -----------------------------
def draw_lidar_overlay(img: np.ndarray, uv: np.ndarray, P3: np.ndarray, max_points: int = 90000) -> np.ndarray:
    """
    Overlays projected LiDAR points on the image.
    Uses (approx) forward distance sqrt(x^2+y^2+z^2) for a simple depth-like cue.
    """
    out = img.copy()
    if uv.shape[0] == 0:
        return out

    if uv.shape[0] > max_points:
        idx = np.random.choice(uv.shape[0], size=max_points, replace=False)
        uv_s = uv[idx]
        P3_s = P3[idx]
    else:
        uv_s = uv
        P3_s = P3

    d = np.linalg.norm(P3_s, axis=1)
    dmin, dmax = float(np.percentile(d, 5)), float(np.percentile(d, 95))
    dmin = max(dmin, 0.1)
    dmax = max(dmax, dmin + 1e-6)
    dn = np.clip((d - dmin) / (dmax - dmin), 0.0, 1.0)

    for (u, v), a in zip(uv_s.astype(np.int32), dn):
        g = int(50 + 205 * (1.0 - float(a)))
        cv2.circle(out, (int(u), int(v)), 1, (0, g, 0), -1)

    return out


# -----------------------------
# Main UI
# -----------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seq", required=True,
                    help="Path to KITTI sequence folder, e.g. 2011_09_26/2011_09_26_drive_0005_sync")
    ap.add_argument("--frame", type=int, default=0, help="Frame index (e.g., 0)")
    ap.add_argument("--cam", type=int, default=2, choices=[0, 1, 2, 3],
                    help="Which camera folder to use: 0/1 grayscale, 2/3 color. Default 1 (image_01).")
    ap.add_argument("--search_radius", type=int, default=15,
                    help="Pixel radius to search for nearest projected LiDAR point")
    ap.add_argument("--min_depth", type=float, default=0.1,
                    help="Min projected depth to keep (positive, meters-ish).")
    ap.add_argument("--show_overlay", action="store_true", help="Draw LiDAR projection overlay points")
    args = ap.parse_args()

    seq = args.seq
    frame_str = f"{args.frame:010d}"

    img_path = os.path.join(seq, f"image_0{args.cam}", "data", f"{frame_str}.png")
    lidar_path = os.path.join(seq, "velodyne_points", "data", f"{frame_str}.bin")

    # Load image
    img = cv2.imread(img_path)
    if img is None:
        raise FileNotFoundError(f"Could not read image: {img_path}")

    # Load LiDAR
    lidar_pts = load_lidar_points(lidar_path)

    # Load KITTI calibration (builds projection to selected camera)
    calib = load_kitti_raw_calibration(seq, cam=args.cam)

    H, W = img.shape[:2]

    # Project
    uv, Pc = project_lidar_to_image(
        lidar_pts,
        calib,
        image_shape=(H, W),
        min_depth=args.min_depth,
    )

    if uv.shape[0] == 0:
        print("No LiDAR points projected into the image (check calibration, min_depth, frame alignment).")
        return

    bucket = build_pixel_bucket(uv)

    clicked: List[Dict[str, np.ndarray]] = []

    base = img.copy()
    if args.show_overlay:
        base = draw_lidar_overlay(base, uv, Pc)

    display = base.copy()

    def redraw():
        nonlocal display
        display = base.copy()

        for j, c in enumerate(clicked):
            u, v = c["pix"]
            cv2.circle(display, (int(u), int(v)), 7, (0, 255, 255), 2)
            cv2.putText(display, f"{j+1}", (int(u) + 10, int(v) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            P = c["P"]
            txt = f"LiDAR XYZ: {P[0]:.2f}, {P[1]:.2f}, {P[2]:.2f} m"
            cv2.putText(display, txt, (int(u) + 10, int(v) + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        if len(clicked) == 2:
            p1 = clicked[0]["pix"]
            p2 = clicked[1]["pix"]
            cv2.line(display, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (0, 255, 255), 2)

            P1 = clicked[0]["P"]
            P2 = clicked[1]["P"]
            dist_m = float(np.linalg.norm(P1 - P2))
            cv2.putText(display, f"3D distance: {dist_m:.3f} m", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

    def on_mouse(event, x, y, flags, param):
        nonlocal clicked
        if event == cv2.EVENT_LBUTTONDOWN:
            P, uvP = find_nearest_projected_point(
                (x, y),
                uv=uv,
                Pc=Pc,
                bucket=bucket,
                search_radius_px=args.search_radius,
            )

            if P is None:
                print(f"No projected LiDAR point near click ({x},{y}). Try increasing --search_radius.")
                return

            clicked.append({
                "pix": np.array([x, y], dtype=np.float64),
                "P": P
            })

            if len(clicked) > 2:
                clicked = clicked[-2:]

            print(f"Click {len(clicked)}: pixel=({x},{y}) -> camera XYZ={P} meters (Z=depth)")
            if len(clicked) == 2:
                d = float(np.linalg.norm(clicked[0]['P'] - clicked[1]['P']))
                print(f"Distance = {d:.6f} meters")

            redraw()

    win = "KITTI LiDAR->Image Click Measure  (R=reset, O=toggle overlay, ESC=quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, on_mouse)

    redraw()
    overlay_on = args.show_overlay

    while True:
        cv2.imshow(win, display)
        key = cv2.waitKey(10) & 0xFF

        if key == 27:  # ESC
            break

        if key in (ord('r'), ord('R')):
            clicked = []
            redraw()

        if key in (ord('o'), ord('O')):
            overlay_on = not overlay_on
            base = img.copy()
            if overlay_on:
                base = draw_lidar_overlay(base, uv, P3)
            redraw()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
