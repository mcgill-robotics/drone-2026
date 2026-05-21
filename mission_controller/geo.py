"""
Pixel + depth + drone telemetry -> target GPS coordinates.

Pipeline:
  pixel (px, py) + depth_z -> camera-frame point
  -> rotated into body frame by the gimbal's commanded yaw/pitch
  -> rotated into world (NED) by drone roll/pitch/yaw
  -> NED offset converted to lat/lon delta on the WGS-84 ellipsoid.

The flat-earth approximation is accurate to <0.1 m for offsets under ~1 km,
which is well beyond the range of our depth sensor.

Frame conventions:
  Camera  (OpenCV/RealSense): X right, Y down, Z forward.
  Body    (aircraft FRD):     X forward, Y right, Z down.
  World   (NED):              X north, Y east, Z down.

Gimbal angles:
  yaw_g    rotation about body-down axis. 0 = camera forward, + = right.
  pitch_g  rotation about gimbal-right axis. 0 = level, + = nose up.
"""

import math


WGS84_A = 6378137.0  # WGS-84 equatorial radius, metres.


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _matmul(A, B):
    return tuple(
        tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _matvec(A, v):
    return tuple(sum(A[i][k] * v[k] for k in range(3)) for i in range(3))


# Camera (X right, Y down, Z forward) -> body (X fwd, Y right, Z down) with
# the gimbal centred (yaw=0, pitch=0). Just an axis relabel.
_CAM_TO_BODY = (
    (0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
)


def pixel_to_gps(px, py, depth_m, fx, fy, ppx, ppy,
                 drone_lat, drone_lon,
                 drone_roll, drone_pitch, drone_yaw,
                 gimbal_yaw=0.0, gimbal_pitch=0.0):
    """Project a detected pixel to a target lat/lon.

    All drone/gimbal angles in radians. `depth_m` is the depth at the pixel as
    reported by the RealSense (perpendicular Z, not slant range). Returns
    (lat_deg, lon_deg) or None if depth is missing/invalid.
    """
    if depth_m is None or depth_m <= 0:
        return None

    # 1. Pixel -> camera-frame 3D point (metres).
    x_cam = (px - ppx) * depth_m / fx
    y_cam = (py - ppy) * depth_m / fy
    z_cam = float(depth_m)

    # 2. Camera -> body, with the gimbal centred.
    p_body_zero = _matvec(_CAM_TO_BODY, (x_cam, y_cam, z_cam))

    # 3. Apply the gimbal's yaw-then-pitch to express the same point as if the
    #    camera were rigidly mounted forward but the body had rotated. R_z(yaw)
    #    swings the aim about the body-down axis; R_y(pitch) tilts about the
    #    gimbal-right axis.
    R_gimbal = _matmul(_rz(gimbal_yaw), _ry(gimbal_pitch))
    p_body = _matvec(R_gimbal, p_body_zero)

    # 4. Body -> world (NED). Standard ZYX intrinsic: yaw, then pitch, then
    #    roll. With yaw=0 the body's forward axis points north.
    R_body_to_world = _matmul(_rz(drone_yaw),
                              _matmul(_ry(drone_pitch), _rx(drone_roll)))
    n, e, _d = _matvec(R_body_to_world, p_body)

    # 5. NED offset -> lat/lon delta. Flat-earth on WGS-84 equatorial radius;
    #    good to <0.1 m at the ranges this sensor reaches.
    lat_rad = math.radians(drone_lat)
    dlat_deg = math.degrees(n / WGS84_A)
    dlon_deg = math.degrees(e / (WGS84_A * math.cos(lat_rad)))

    return drone_lat + dlat_deg, drone_lon + dlon_deg
