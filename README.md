Datasets used for testing:
1. https://www.cvlibs.net/datasets/kitti/raw_data.php?type=city (Will have to make an account with school email to access datasets)
2. Go to any raw data category except calibration(city, residential, road, campus, person)
3. Download + extract synced+rectified data + calibration folders
4. place the calibration .txt files in the [images date]_synced directory
5. Run:
python[3] distance.py --seq [path to date_sync directory] --frame 0[or any other frame in the dataset] --show_overlay

# LiDAR → Image Click-to-Measure

You load:
- a camera frame (e.g., SIYI R1M image)
- a LiDAR point cloud captured at (roughly) the same time
- a calibration JSON (camera intrinsics + LiDAR→camera extrinsics)

Then:
- the tool overlays LiDAR points onto the image
- you click **two points** on the image
- the tool finds the corresponding **3D LiDAR points**
- it outputs the **3D distance** between them in meters

---

## Table of Contents

- [Mental model: what “mapping LiDAR onto an image” means](#mental-model-what-mapping-lidar-onto-an-image-means)
- [Algorithm / thought process](#algorithm--thought-process)
  - [1) Put LiDAR points into the camera coordinate system](#1-put-lidar-points-into-the-camera-coordinate-system)
  - [2) Project 3D camera points into 2D pixels](#2-project-3d-camera-points-into-2d-pixels)
  - [3) Build a fast lookup for clicking](#3-build-a-fast-lookup-for-clicking)
  - [4) Convert two clicks into two 3D points](#4-convert-two-clicks-into-two-3d-points)
  - [5) Compute and display the distance](#5-compute-and-display-the-distance)
- [Calibration file: how to think about `calib.json`](#calibration-file-how-to-think-about-calibjson)
- [How to run](#how-to-run)
- [Controls](#controls)
- [Troubleshooting](#troubleshooting)
- [Limitations](#limitations)

---

## Mental model: what “mapping LiDAR onto an image” means

Each LiDAR point is a 3D point in the LiDAR’s coordinate frame.

To draw it on an image, you must:
1) **Transform** the point into the **camera coordinate frame** (so the camera “sees” it)
2) **Project** it through the camera intrinsics to get pixel coordinates `(u, v)`

Once you can project points into the image, you can build a relationship:

> pixel `(u, v)` ↔ some 3D point `P = (X, Y, Z)` in camera coordinates

That relationship is the foundation for click-to-measure.

---

## Algorithm / thought process

### 1) Put LiDAR points into the camera coordinate system

**Goal:** express every LiDAR point in the same coordinate system as the camera.

LiDAR points are given in LiDAR frame:
- `P_lidar = [X_l, Y_l, Z_l]`

We apply a rigid transform (rotation + translation) that you get from calibration:

\[
P_{cam} = R \cdot P_{lidar} + t
\]

In code, this is done as:
- `Pc_all = (R @ lidar_xyz.T + t).T`

**Why this matters:** projection requires points in camera frame, because camera intrinsics (`K`) are defined in the camera frame.

**Filtering:** points behind the camera are invalid for projection, so the script removes points with:
- `Z_cam <= min_z` (default `min_z = 0.1m`)

---

### 2) Project 3D camera points into 2D pixels

**Goal:** for each valid 3D point in camera frame, compute where it lands on the image.

For a pinhole camera model:

\[
u = f_x \cdot \frac{X}{Z} + c_x,\quad
v = f_y \cdot \frac{Y}{Z} + c_y
\]

Where:
- `(X, Y, Z)` are camera-frame coordinates
- `K` contains `fx, fy, cx, cy`
- `dist` (optional) models lens distortion

The script uses OpenCV’s `cv2.projectPoints(...)` so distortion is handled correctly if provided.

After projection:
- points outside the image bounds are discarded

At this stage you have two aligned arrays:
- `uv[i] = (u, v)` pixel
- `Pc[i] = (X, Y, Z)` camera-frame 3D point

---

### 3) Build a fast lookup for clicking

**Goal:** clicking must be fast, so we pre-index projected points.

Projected LiDAR points are sparse and not aligned to every pixel.
If you click pixel `(x, y)`, the script searches a small radius around that click to find the nearest projected LiDAR point.

To make this efficient, it builds a **bucket map**:
- keys: integer pixel `(u_int, v_int)`
- values: list of indices of points projected to that pixel

So instead of scanning all points every click, it searches only the buckets in a window around the click.

---

### 4) Convert two clicks into two 3D points

**Goal:** map user clicks to real 3D points.

On each click:
1) read click `(x, y)`
2) search a window of pixels around `(x, y)` of size `±search_radius`
3) among all candidate points found, choose the one with:
   - smallest pixel distance to the click
   - (optional) tie-break by closest depth `Z` to prefer foreground surfaces

This gives you a 3D point `P = (X, Y, Z)` for that click.

When you have two clicks, you have two 3D points:
- `P1`, `P2`

---

### 5) Compute and display the distance

Distance is the standard Euclidean distance in 3D:

\[
d = \| P_1 - P_2 \| = \sqrt{(X_1-X_2)^2 + (Y_1-Y_2)^2 + (Z_1-Z_2)^2}
\]

The script prints the distance and overlays it on the display window.

---

## Calibration file: how to think about `calib.json`

`calib.json` encodes:
1) **Camera intrinsics** (`K`, `dist`)
2) **LiDAR→camera extrinsics** (`R`, `t`)

```json
{
  "K": [[fx, 0, cx],
        [0, fy, cy],
        [0,  0,  1]],

  "dist": [k1, k2, p1, p2, k3],

  "R": [[...],[...],[...]],
  "t": [tx, ty, tz]
}
