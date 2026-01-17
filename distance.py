#!/usr/bin/env python3
"""
Click-to-measure 3D distance using a camera image + Velodyne VLP-16 point cloud.

What this script does
- Loads an image (PNG/JPG) from the SIYI camera (or any camera frame).
- Loads a LiDAR point cloud (recommended: .npy with shape (N,3)).
- Loads calibration (camera intrinsics K + distortion, and LiDAR->camera extrinsics R,t).
- Projects LiDAR points into the image.
- UI: click two pixels. For each click, finds the nearest projected LiDAR point and gets its 3D position.
- Outputs the 3D distance between the two chosen points (meters) and overlays it on the image.

Requirements
- Python 3.x
- opencv-python
- numpy
Optional:
- open3d (only if you want to load .pcd/.ply)
"""

import argparse
import json
import os
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict

import cv2
import numpy as np


# Data structures
@dataclass
class Calibration:
    K: np.ndarray          # (3,3)
    dist: Optional[np.ndarray]  # (N,) or None
    R: np.ndarray          # (3,3) LiDAR->Camera
    t: np.ndarray          # (3,1) LiDAR->Camera


# Calibration loading
def load_calibration_json(path: str) -> Calibration:
    """
    JSON format expected:

    {
      "K": [[fx,0,cx],[0,fy,cy],[0,0,1]],
      "dist": [k1,k2,p1,p2,k3],   // optional; can be [] or omitted
      "R": [[...],[...],[...]],   // LiDAR->Camera rotation
      "t": [tx,ty,tz]             // LiDAR->Camera translation (meters)
    }
    """
    with open(path, "r") as f:
        data = json.load(f)

    K = np.array(data["K"], dtype=np.float64)
    if K.shape != (3, 3):
        raise ValueError("K must be 3x3")

    dist = data.get("dist", None)
    if dist is None or (isinstance(dist, list) and len(dist) == 0):
        dist_arr = None
    else:
        dist_arr = np.array(dist, dtype=np.float64).reshape(-1)

    R = np.array(data["R"], dtype=np.float64)
    if R.shape != (3, 3):
        raise ValueError("R must be 3x3")

    t = np.array(data["t"], dtype=np.float64).reshape(3, 1)

    return Calibration(K=K, dist=dist_arr, R=R, t=t)


# LiDAR loading
def load_lidar_points(path: str) -> np.ndarray:
    """
    Supported:
    - .npy: numpy array of shape (N,3) or (N,4) (XYZ or XYZ+intensity)
    - .bin: KITTI-style float32 (x,y,z,intensity) repeated (N,4)
    - .pcd/.ply: requires open3d (optional)

    Returns Nx3 float64 (XYZ) in LiDAR frame.
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

    elif ext in (".pcd", ".ply"):
        try:
            import open3d as o3d  # type: ignore
        except Exception as e:
            raise RuntimeError("To load .pcd/.ply, install open3d: pip install open3d") from e
        pcd = o3d.io.read_point_cloud(path)
        pts = np.asarray(pcd.points, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] != 3:
            raise ValueError("PCD/PLY did not produce Nx3 points")

    else:
        raise ValueError(f"Unsupported LiDAR format: {ext}. Use .npy, .bin, .pcd, or .ply")

    return pts.astype(np.float64)


# Projection
def project_lidar_to_image(
    lidar_xyz: np.ndarray,
    calib: Calibration,
    image_shape: Tuple[int, int],
    min_z: float = 0.1,
    undistort_image: bool = False,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Returns:
    - uv: (M,2) float64 projected pixels (in the display image coordinate system)
    - Pc: (M,3) float64 corresponding 3D points in CAMERA frame
    - mask_in_img: (M,) bool mask for points in image bounds (already applied)

    Note on distortion:
    - If undistort_image is False:
        We assume the image is the raw (distorted) frame.
        We project points with cv2.projectPoints using dist (if provided).
    - If undistort_image is True:
        We undistort the image first and then project points WITHOUT distortion (dist=None),
        and also use the undistorted camera matrix (newK) for consistency.
    """
    H, W = image_shape[:2]

    # Transform LiDAR -> Camera
    Pc_all = (calib.R @ lidar_xyz.T + calib.t).T  # (N,3)
    Zc = Pc_all[:, 2]
    front = Zc > min_z
    Pc = Pc_all[front]

    if Pc.shape[0] == 0:
        return np.zeros((0, 2), dtype=np.float64), np.zeros((0, 3), dtype=np.float64), np.zeros((0,), dtype=bool)

    # Decide projection model
    if undistort_image and calib.dist is not None:
        # Use an optimal new camera matrix for undistortion
        newK, _roi = cv2.getOptimalNewCameraMatrix(calib.K, calib.dist, (W, H), 1.0, (W, H))
        Kproj = newK
        distproj = None
    else:
        Kproj = calib.K
        distproj = calib.dist if calib.dist is not None else None

    # Use cv2.projectPoints (handles distortion if distproj not None)
    rvec = np.zeros((3, 1), dtype=np.float64)
    tvec = np.zeros((3, 1), dtype=np.float64)
    imgpts, _ = cv2.projectPoints(Pc.reshape(-1, 1, 3), rvec, tvec, Kproj, distproj)
    uv = imgpts.reshape(-1, 2).astype(np.float64)

    # Keep in-bounds
    in_img = (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)
    uv = uv[in_img]
    Pc = Pc[in_img]

    return uv, Pc, in_img


def undistort_if_requested(img: np.ndarray, calib: Calibration, undistort_image: bool) -> np.ndarray:
    if not undistort_image or calib.dist is None:
        return img
    H, W = img.shape[:2]
    newK, _roi = cv2.getOptimalNewCameraMatrix(calib.K, calib.dist, (W, H), 1.0, (W, H))
    return cv2.undistort(img, calib.K, calib.dist, None, newK)


# Click lookup
def build_pixel_bucket(uv: np.ndarray) -> Dict[Tuple[int, int], List[int]]:
    """
    Buckets projected points by integer pixel coordinate for fast local search.
    """
    bucket: Dict[Tuple[int, int], List[int]] = {}
    uv_int = np.floor(uv).astype(np.int32)
    for i, (u, v) in enumerate(uv_int):
        key = (int(u), int(v))
        bucket.setdefault(key, []).append(i)
    return bucket


def find_nearest_projected_point(
    click_xy: Tuple[int, int],
    uv: np.ndarray,
    Pc: np.ndarray,
    bucket: Dict[Tuple[int, int], List[int]],
    search_radius_px: int = 10,
    prefer_closest_depth: bool = True,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """
    Finds the best matching LiDAR point for a click.

    - Searches in a square window +/- search_radius_px around the click pixel.
    - Chooses the candidate with smallest pixel distance.
    - If prefer_closest_depth=True, breaks ties (or near-ties) by smaller Z (closer surface).

    Returns:
    - P (3,) in camera frame, and uvP (2,) in pixels
    """
    cx, cy = click_xy
    best_i = None
    best_d2 = float("inf")
    best_z = float("inf")

    for du in range(-search_radius_px, search_radius_px + 1):
        for dv in range(-search_radius_px, search_radius_px + 1):
            key = (cx + du, cy + dv)
            if key not in bucket:
                continue
            for i in bucket[key]:
                dx = float(uv[i, 0] - cx)
                dy = float(uv[i, 1] - cy)
                d2 = dx * dx + dy * dy

                if d2 < best_d2:
                    best_d2 = d2
                    best_i = i
                    best_z = float(Pc[i, 2])
                elif prefer_closest_depth and abs(d2 - best_d2) < 1e-6:
                    # tie-break on depth
                    z = float(Pc[i, 2])
                    if z < best_z:
                        best_i = i
                        best_z = z

    if best_i is None:
        return None, None
    return Pc[best_i].copy(), uv[best_i].copy()


# Drawing helpers
def draw_lidar_overlay(img: np.ndarray, uv: np.ndarray, Pc: np.ndarray, max_points: int = 80000) -> np.ndarray:
    """
    Overlays projected LiDAR points on the image.
    Uses depth to set point size/intensity (simple visualization).
    """
    out = img.copy()
    if uv.shape[0] == 0:
        return out

    # Subsample if huge
    if uv.shape[0] > max_points:
        idx = np.random.choice(uv.shape[0], size=max_points, replace=False)
        uv_s = uv[idx]
        Pc_s = Pc[idx]
    else:
        uv_s = uv
        Pc_s = Pc

    # Normalize depth for display
    z = Pc_s[:, 2]
    zmin, zmax = float(np.percentile(z, 5)), float(np.percentile(z, 95))
    zmin = max(zmin, 0.1)
    zmax = max(zmax, zmin + 1e-6)
    zn = np.clip((z - zmin) / (zmax - zmin), 0.0, 1.0)

    for (u, v), a in zip(uv_s.astype(np.int32), zn):
        # Closer -> brighter and slightly bigger (green channel)
        g = int(50 + 205 * (1.0 - float(a)))
        cv2.circle(out, (int(u), int(v)), 1, (0, g, 0), -1)

    return out


# Main UI
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True, help="Path to image (jpg/png)")
    ap.add_argument("--lidar", required=True, help="Path to LiDAR point cloud (.npy recommended; also supports .bin/.pcd/.ply)")
    ap.add_argument("--calib", required=True, help="Path to calibration JSON (K, dist optional, R, t)")
    ap.add_argument("--search_radius", type=int, default=10, help="Pixel radius to search for nearest projected LiDAR point")
    ap.add_argument("--min_z", type=float, default=0.1, help="Min Z in camera frame to keep (meters)")
    ap.add_argument("--undistort", action="store_true", help="Undistort image and project consistently (if dist is provided)")
    ap.add_argument("--show_overlay", action="store_true", help="Draw LiDAR projection overlay points")
    args = ap.parse_args()

    # Load
    img_raw = cv2.imread(args.image)
    if img_raw is None:
        raise FileNotFoundError(f"Could not read image: {args.image}")

    lidar_pts = load_lidar_points(args.lidar)
    calib = load_calibration_json(args.calib)

    # Optionally undistort
    img = undistort_if_requested(img_raw, calib, args.undistort)

    H, W = img.shape[:2]

    # Project
    uv, Pc, _ = project_lidar_to_image(
        lidar_pts,
        calib,
        image_shape=(H, W),
        min_z=args.min_z,
        undistort_image=args.undistort,
    )

    if uv.shape[0] == 0:
        print("No LiDAR points projected into the image (check calibration, min_z, time alignment).")
        return

    # Build lookup
    bucket = build_pixel_bucket(uv)

    # State
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

            # Show the matched 3D point
            P = c["P"]
            txt = f"X={P[0]:.2f} Y={P[1]:.2f} Z={P[2]:.2f} m"
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
                prefer_closest_depth=True,
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

            print(f"Click {len(clicked)}: pixel=({x},{y}) -> camera XYZ={P} meters")
            if len(clicked) == 2:
                d = float(np.linalg.norm(clicked[0]["P"] - clicked[1]["P"]))
                print(f"Distance = {d:.6f} meters")

            redraw()

    win = "LiDAR->Image Click Measure  (R=reset, O=toggle overlay, ESC=quit)"
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
                base = draw_lidar_overlay(base, uv, Pc)
            redraw()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
