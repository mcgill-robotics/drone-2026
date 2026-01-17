#!/usr/bin/env python3
"""
Generates synthetic testing data for the LiDAR-Camera measurement script.

Outputs:
1. dummy_image.jpg: A dark image with projected points. Two large RED dots are targets.
2. dummy_lidar.npy: A 3D point cloud matching the dots in the image.
3. dummy_calib.json: The exact calibration needed to align them.

The Test Scenario:
We simulate a wall exactly 5.0 meters away from the camera along the Z-axis.
We place two target points on this wall that are exactly 1.0 meter apart horizontally.
If you click the two red dots in the measurement script, the result should be exactly 1.000m.
"""

import json
import cv2
import numpy as np
import os

# --- Configuration ---
OUTPUT_DIR = "test_data"
IMG_NAME = os.path.join(OUTPUT_DIR, "dummy_image.jpg")
LIDAR_NAME = os.path.join(OUTPUT_DIR, "dummy_lidar.npy")
CALIB_NAME = os.path.join(OUTPUT_DIR, "dummy_calib.json")

W, H = 1280, 720
WALL_DEPTH_M = 5.0
TARGET_SEPARATION_M = 1.0


def ensure_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)

def create_synthetic_scene():
    # 1. Define Camera Intrinsics (K) - Simple pinhole, no distortion
    fx, fy = 1000.0, 1000.0
    cx, cy = W / 2.0, H / 2.0
    K = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ], dtype=np.float64)
    # Zero distortion for this test to keep it mathematically perfect
    dist = np.zeros((5,), dtype=np.float64)

    # 2. Define Extrinsics (LiDAR -> Camera transformation R, t)
    # Let's define a setup where the LiDAR is rotated 90 degrees relative to camera
    # and offset by 10cm along the camera's Y-axis.
    # Rotate +90deg around X-axis
    R_vec_lc = np.array([np.pi / 2.0, 0, 0], dtype=np.float64)
    R_lc, _ = cv2.Rodrigues(R_vec_lc)
    # Translation in meters
    t_lc = np.array([[0.0], [0.1], [0.0]], dtype=np.float64)

    # 3. Create 3D Points in Camera Frame (Pc)
    # We build a grid on a wall at fixed depth Z.
    pc_list = []

    # A. The two "Golden Target" points (exactly 1.0m apart along X)
    # Left point
    pc_list.append([-TARGET_SEPARATION_M / 2.0, 0.0, WALL_DEPTH_M])
    # Right point
    pc_list.append([TARGET_SEPARATION_M / 2.0, 0.0, WALL_DEPTH_M])

    # B. Fill background grid points so it looks like a cloud
    for x in np.linspace(-3, 3, 50):
        for y in np.linspace(-2, 2, 40):
            # Add slight noise to Z so it's not a perfect plane, testing depth buffer
            z_noise = np.random.uniform(-0.05, 0.05)
            pc_list.append([x, y, WALL_DEPTH_M + z_noise])

    Pc = np.array(pc_list, dtype=np.float64) # Shape (N, 3)

    # 4. Generate Image (Project Pc -> Image)
    # Create a dark background image
    img = np.zeros((H, W, 3), dtype=np.uint8)

    # Project 3D camera points to 2D pixels
    imgpts, _ = cv2.projectPoints(Pc.reshape(-1, 1, 3), (0,0,0), (0,0,0), K, dist)
    uv = imgpts.reshape(-1, 2)

    # Draw points on image
    for i, (u, v) in enumerate(uv):
        u_int, v_int = int(round(u)), int(round(v))
        if 0 <= u_int < W and 0 <= v_int < H:
            # The first two points are our targets: make them big red circles
            if i < 2:
                cv2.circle(img, (u_int, v_int), 8, (0, 0, 255), -1)
            # The rest are background grid: small gray dots
            else:
                cv2.circle(img, (u_int, v_int), 2, (150, 150, 150), -1)

    # 5. Generate LiDAR Cloud (Transform Camera Frame -> LiDAR Frame)
    # If Pc = R_lc * Pl + t_lc
    # Then Pl = R_lc.T * (Pc - t_lc)  (since R is orthogonal, R inverse = R transpose)
    Pc_centered = Pc - t_lc.T
    Pl = (R_lc.T @ Pc_centered.T).T # Result is (N, 3)

    # 6. Save outputs
    ensure_dir(OUTPUT_DIR)

    # Save Image
    cv2.imwrite(IMG_NAME, img)
    print(f"Generated image: {IMG_NAME}")

    # Save Cloud
    np.save(LIDAR_NAME, Pl.astype(np.float32)) # Save as float32 to mimic typical sensor data
    print(f"Generated LiDAR cloud: {LIDAR_NAME} (shape {Pl.shape})")

    # Save Calibration
    calib_data = {
        "K": K.tolist(),
        "dist": dist.tolist(),
        "R": R_lc.tolist(),
        "t": t_lc.flatten().tolist()
    }
    with open(CALIB_NAME, "w") as f:
        json.dump(calib_data, f, indent=2)
    print(f"Generated calibration: {CALIB_NAME}")
    print("\n--- Test Data Generation Complete ---")
    print(f"Target distance between the two RED dots is exactly {TARGET_SEPARATION_M:.3f} meters.")


if __name__ == "__main__":
    create_synthetic_scene()