#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


@dataclass
class Calibration:
    P: np.ndarray      # 3x4 camera projection matrix
    fx: float
    fy: float
    cx: float
    cy: float


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    # Standard quaternion -> rotation matrix
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


def transform_points_xyz(points_xyz: np.ndarray, transform_msg) -> np.ndarray:
    t = transform_msg.transform.translation
    q = transform_msg.transform.rotation

    R = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
    T = np.array([t.x, t.y, t.z], dtype=np.float64)

    return (R @ points_xyz.T).T + T


def build_calibration_from_camera_info(msg: CameraInfo) -> Calibration:
    P = np.array(msg.p, dtype=np.float64).reshape(3, 4)
    fx = P[0, 0]
    fy = P[1, 1]
    cx = P[0, 2]
    cy = P[1, 2]
    return Calibration(P=P, fx=fx, fy=fy, cx=cx, cy=cy)


def project_points_to_image(
    points_cam: np.ndarray,
    calib: Calibration,
    image_shape: Tuple[int, int],
    min_depth: float = 0.1,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    points_cam: Nx3 in camera optical frame
    Returns:
      uv: Mx2
      Pc_kept: Mx3
    """
    H, W = image_shape[:2]

    if points_cam.size == 0:
        return np.zeros((0, 2), dtype=np.float64), np.zeros((0, 3), dtype=np.float64)

    Z = points_cam[:, 2]
    front = Z > min_depth
    Pc = points_cam[front]

    if Pc.shape[0] == 0:
        return np.zeros((0, 2), dtype=np.float64), np.zeros((0, 3), dtype=np.float64)

    xyz1 = np.hstack([Pc, np.ones((Pc.shape[0], 1), dtype=np.float64)])
    proj = (calib.P @ xyz1.T).T
    depth = proj[:, 2]

    valid = depth > min_depth
    proj = proj[valid]
    Pc = Pc[valid]

    if proj.shape[0] == 0:
        return np.zeros((0, 2), dtype=np.float64), np.zeros((0, 3), dtype=np.float64)

    u = proj[:, 0] / depth[valid]
    v = proj[:, 1] / depth[valid]

    in_img = (u >= 0) & (u < W) & (v >= 0) & (v < H)

    uv = np.stack([u[in_img], v[in_img]], axis=1)
    Pc_kept = Pc[in_img]

    return uv.astype(np.float64), Pc_kept.astype(np.float64)


def build_pixel_bucket(uv: np.ndarray) -> Dict[Tuple[int, int], List[int]]:
    bucket: Dict[Tuple[int, int], List[int]] = {}
    uv_int = np.rint(uv).astype(np.int32)
    for i, (u, v) in enumerate(uv_int):
        bucket.setdefault((int(u), int(v)), []).append(i)
    return bucket


def find_best_3d_point(
    click_xy: Tuple[int, int],
    uv: np.ndarray,
    Pc: np.ndarray,
    calib: Calibration,
    bucket: Dict[Tuple[int, int], List[int]],
    search_radius_px: int = 10,
    max_search_radius_px: int = 40,
    min_candidates: int = 12,
    top_k: int = 8,
):
    cx, cy = click_xy

    candidates: List[int] = []
    radius = search_radius_px

    while radius <= max_search_radius_px:
        local = []
        for du in range(-radius, radius + 1):
            for dv in range(-radius, radius + 1):
                key = (cx + du, cy + dv)
                if key in bucket:
                    local.extend(bucket[key])

        if local:
            candidates = local

        if len(candidates) >= min_candidates:
            break

        radius *= 2

    if len(candidates) == 0:
        return None, None, None

    idx = np.array(candidates, dtype=np.int32)
    uv_c = uv[idx]
    Pc_c = Pc[idx]

    duv = uv_c - np.array([cx, cy], dtype=np.float64)
    pixel_d2 = np.sum(duv ** 2, axis=1)

    x = (cx - calib.cx) / calib.fx
    y = (cy - calib.cy) / calib.fy
    ray = np.array([x, y, 1.0], dtype=np.float64)
    ray /= np.linalg.norm(ray)

    proj_len = Pc_c @ ray
    closest = np.outer(proj_len, ray)
    perp = Pc_c - closest
    ray_dist = np.linalg.norm(perp, axis=1)

    score = pixel_d2 + 10.0 * (ray_dist ** 2)

    k = min(top_k, len(score))
    order = np.argsort(score)[:k]
    Pc_best = Pc_c[order]
    uv_best = uv_c[order]
    score_best = score[order]

    depths = Pc_best[:, 2]
    median_z = np.median(depths)
    mask = np.abs(depths - median_z) < 1.0

    if np.sum(mask) >= 3:
        Pc_best = Pc_best[mask]
        uv_best = uv_best[mask]
        score_best = score_best[mask]

    weights = 1.0 / (score_best + 1e-6)
    weights /= np.sum(weights)

    P_final = np.sum(Pc_best * weights[:, None], axis=0)
    uv_final = np.sum(uv_best * weights[:, None], axis=0)
    reproj_err_px = float(np.linalg.norm(uv_final - np.array([cx, cy], dtype=np.float64)))

    return P_final, uv_final, reproj_err_px


def draw_lidar_overlay(img: np.ndarray, uv: np.ndarray, Pc: np.ndarray, max_points: int = 50000) -> np.ndarray:
    out = img.copy()
    if uv.shape[0] == 0:
        return out

    if uv.shape[0] > max_points:
        idx = np.random.choice(uv.shape[0], size=max_points, replace=False)
        uv_s = uv[idx]
        Pc_s = Pc[idx]
    else:
        uv_s = uv
        Pc_s = Pc

    d = np.linalg.norm(Pc_s, axis=1)
    dmin, dmax = float(np.percentile(d, 5)), float(np.percentile(d, 95))
    dmin = max(dmin, 0.1)
    dmax = max(dmax, dmin + 1e-6)
    dn = np.clip((d - dmin) / (dmax - dmin), 0.0, 1.0)

    for (u, v), a in zip(uv_s.astype(np.int32), dn):
        g = int(50 + 205 * (1.0 - float(a)))
        cv2.circle(out, (int(u), int(v)), 1, (0, g, 0), -1)

    return out


class LidarImageDistanceGuiNode(Node):
    def __init__(self):
        super().__init__('lidar_image_distance_gui')

        self.declare_parameter('image_topic', '/camera/image_rect_color')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('cloud_topic', '/velodyne_points')
        self.declare_parameter('search_radius', 10)
        self.declare_parameter('max_search_radius', 40)
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('sync_slop', 0.08)
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('show_overlay', True)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        sync_slop = self.get_parameter('sync_slop').get_parameter_value().double_value

        self.search_radius = int(self.get_parameter('search_radius').value)
        self.max_search_radius = int(self.get_parameter('max_search_radius').value)
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.show_overlay = bool(self.get_parameter('show_overlay').value)

        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.image_sub = Subscriber(self, Image, image_topic)
        self.info_sub = Subscriber(self, CameraInfo, camera_info_topic)
        self.cloud_sub = Subscriber(self, PointCloud2, cloud_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.info_sub, self.cloud_sub],
            queue_size=queue_size,
            slop=sync_slop,
        )
        self.sync.registerCallback(self.synced_callback)

        self.latest_base: Optional[np.ndarray] = None
        self.latest_display: Optional[np.ndarray] = None
        self.latest_uv: Optional[np.ndarray] = None
        self.latest_Pc: Optional[np.ndarray] = None
        self.latest_bucket: Optional[Dict[Tuple[int, int], List[int]]] = None
        self.latest_calib: Optional[Calibration] = None

        self.clicked: List[Dict[str, np.ndarray]] = []

        self.window_name = 'ROS2 LiDAR->Image Click Measure  (R=reset, O=toggle overlay, ESC=quit)'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.on_mouse)

        self.get_logger().info('Node started.')

    def lookup_cloud_to_camera_transform(self, camera_frame: str, cloud_frame: str, stamp) -> Optional[object]:
        try:
            # tf2 listener buffers transforms and lookup_transform is the standard query interface.
            tf_msg = self.tf_buffer.lookup_transform(
                camera_frame,
                cloud_frame,
                Time.from_msg(stamp),
            )
            return tf_msg
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def synced_callback(self, image_msg: Image, info_msg: CameraInfo, cloud_msg: PointCloud2):
        try:
            img = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            return

        calib = build_calibration_from_camera_info(info_msg)

        # PointCloud2 reading through sensor_msgs_py.point_cloud2.read_points is the documented API.
        try:
            pts_struct = point_cloud2.read_points(
                cloud_msg,
                field_names=['x', 'y', 'z'],
                skip_nans=True,
            )
            pts_xyz = np.asarray(pts_struct, dtype=np.float64)
        except Exception as e:
            self.get_logger().error(f'PointCloud2 conversion failed: {e}')
            return

        if pts_xyz.ndim != 2 or pts_xyz.shape[1] != 3:
            self.get_logger().warn('Unexpected point cloud shape after conversion.')
            return

        tf_msg = self.lookup_cloud_to_camera_transform(
            camera_frame=image_msg.header.frame_id,
            cloud_frame=cloud_msg.header.frame_id,
            stamp=cloud_msg.header.stamp,
        )
        if tf_msg is None:
            return

        points_cam = transform_points_xyz(pts_xyz, tf_msg)

        uv, Pc = project_points_to_image(
            points_cam,
            calib=calib,
            image_shape=img.shape[:2],
            min_depth=self.min_depth,
        )

        if uv.shape[0] == 0:
            self.get_logger().warn('No projected LiDAR points landed in the image.')
            return

        bucket = build_pixel_bucket(uv)

        base = img.copy()
        if self.show_overlay:
            base = draw_lidar_overlay(base, uv, Pc)

        self.latest_base = base
        self.latest_display = base.copy()
        self.latest_uv = uv
        self.latest_Pc = Pc
        self.latest_bucket = bucket
        self.latest_calib = calib

        self.redraw()
        self.spin_gui_once()

    def redraw(self):
        if self.latest_base is None:
            return

        display = self.latest_base.copy()

        for j, c in enumerate(self.clicked):
            click_pix = c['click_pix']
            est_pix = c['est_pix']
            P = c['P']
            err_px = c['err_px']

            cv2.circle(display, (int(click_pix[0]), int(click_pix[1])), 7, (0, 255, 255), 2)
            cv2.putText(display, f'{j + 1}', (int(click_pix[0]) + 10, int(click_pix[1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            cv2.circle(display, (int(est_pix[0]), int(est_pix[1])), 5, (0, 0, 255), -1)
            cv2.line(display,
                     (int(click_pix[0]), int(click_pix[1])),
                     (int(est_pix[0]), int(est_pix[1])),
                     (255, 0, 0), 2)

            txt = f'XYZ: {P[0]:.2f}, {P[1]:.2f}, {P[2]:.2f} m | err: {err_px:.2f}px'
            cv2.putText(display, txt, (int(click_pix[0]) + 10, int(click_pix[1]) + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 2)

        if len(self.clicked) == 2:
            p1 = self.clicked[0]['click_pix']
            p2 = self.clicked[1]['click_pix']
            P1 = self.clicked[0]['P']
            P2 = self.clicked[1]['P']

            cv2.line(display, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (0, 255, 255), 2)
            dist_m = float(np.linalg.norm(P1 - P2))
            cv2.putText(display, f'3D distance: {dist_m:.3f} m', (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

        self.latest_display = display

    def on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if any(v is None for v in [self.latest_uv, self.latest_Pc, self.latest_bucket, self.latest_calib]):
            return

        P, uvP, err_px = find_best_3d_point(
            (x, y),
            uv=self.latest_uv,
            Pc=self.latest_Pc,
            calib=self.latest_calib,
            bucket=self.latest_bucket,
            search_radius_px=self.search_radius,
            max_search_radius_px=self.max_search_radius,
        )

        if P is None:
            self.get_logger().info(f'No projected LiDAR point near click ({x}, {y}).')
            return

        self.clicked.append({
            'click_pix': np.array([x, y], dtype=np.float64),
            'est_pix': uvP,
            'P': P,
            'err_px': np.array(err_px, dtype=np.float64),
        })

        if len(self.clicked) > 2:
            self.clicked = self.clicked[-2:]

        self.get_logger().info(
            f'Click {len(self.clicked)} pixel=({x},{y}) -> XYZ=[{P[0]:.3f}, {P[1]:.3f}, {P[2]:.3f}] m, reproj={err_px:.2f}px'
        )

        if len(self.clicked) == 2:
            d = float(np.linalg.norm(self.clicked[0]['P'] - self.clicked[1]['P']))
            self.get_logger().info(f'Distance = {d:.6f} m')

        self.redraw()
        self.spin_gui_once()

    def spin_gui_once(self):
        if self.latest_display is None:
            return

        cv2.imshow(self.window_name, self.latest_display)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # ESC
            self.get_logger().info('ESC pressed, shutting down.')
            rclpy.shutdown()
        elif key in (ord('r'), ord('R')):
            self.clicked = []
            self.redraw()
        elif key in (ord('o'), ord('O')):
            self.show_overlay = not self.show_overlay
            if self.latest_uv is not None and self.latest_Pc is not None and self.latest_display is not None:
                # Rebuild base from the most recent raw display source is not stored separately,
                # so we just redraw on next synchronized frame.
                self.get_logger().info(f'show_overlay={self.show_overlay}')

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarImageDistanceGuiNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.spin_gui_once()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
