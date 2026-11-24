#!/usr/bin/env python3
"""Shared RGB-D perception, 3D/ground projection, and map-geometry helpers.

Pure-NumPy utilities (no mandatory ROS imports) shared by:
  - doorway_navigator.py  (room floor classification & passive target mapping)
  - alpha_cleaning.py     (autonomous Room Alpha cleaning zone discovery)
  - beta_pick_place.py    (autonomous Room Beta box & drop-zone discovery)
"""
from collections import deque
import math
import numpy as np


# Fallback camera intrinsics matching turtlebot3_manipulation_gz.xacro
# (320x240, horizontal_fov = 1.047 rad)
DEFAULT_WIDTH = 320
DEFAULT_HEIGHT = 240
DEFAULT_HFOV = 1.047
DEFAULT_FX = (DEFAULT_WIDTH / 2.0) / math.tan(DEFAULT_HFOV / 2.0)
DEFAULT_FY = DEFAULT_FX
DEFAULT_CX = DEFAULT_WIDTH / 2.0
DEFAULT_CY = DEFAULT_HEIGHT / 2.0

OCCUPIED_THRESH = 50
FREE_MAX = 25


# ---------------------------------------------------------------------------
# Colour Segmentation
# ---------------------------------------------------------------------------
def classify_floor_color(rgb_img: np.ndarray, roi_top_frac: float = 0.55):
    """Classify the floor colour in the lower portion of an RGB image.

    Returns (label, yellow_ratio, green_ratio) where label is one of
    'YELLOW' (Room Alpha), 'GREEN' (Room Beta), or 'HALL'.
    """
    h, w, _ = rgb_img.shape
    roi = rgb_img[int(h * roi_top_frac):, int(w * 0.15):int(w * 0.85)].astype(np.float32)
    r, g, b = roi[:, :, 0], roi[:, :, 1], roi[:, :, 2]

    # Room Alpha yellow floor: RGB ~(255, 232, 122) lit, ~(140, 128, 67) shadow
    yellow_mask = (r > 125) & (g > 110) & (b < 195) & ((r - b) > 45) & ((g - b) > 35)
    # Room Beta mint/green floor: RGB ~(184, 240, 217) lit, ~(101, 132, 119) shadow
    green_mask = (g > 100) & ((g - r) > 15) & ((g - b) > 5)

    total = float(roi.shape[0] * roi.shape[1])
    if total <= 0.0:
        return 'HALL', 0.0, 0.0

    yr = float(np.count_nonzero(yellow_mask)) / total
    gr = float(np.count_nonzero(green_mask)) / total

    if yr > 0.15 and yr > gr:
        return 'YELLOW', yr, gr
    if gr > 0.15 and gr > yr:
        return 'GREEN', yr, gr
    return 'HALL', yr, gr


def classify_near_floor_color(rgb_img: np.ndarray):
    """Classify floor colour in the bottom 20% of the image (~0.29-0.43 m ahead).

    Used to confirm the robot has physically crossed a doorway into a room
    rather than peeking through a doorway from the hallway.
    """
    return classify_floor_color(rgb_img, roi_top_frac=0.80)


def segment_clean_zone(rgb_img: np.ndarray) -> np.ndarray:
    """Boolean (H, W) mask for the Room Alpha slate-blue cleaning zone mat.

    Material RGB (0.24, 0.50, 0.84) under linear or Ogre2 sRGB lighting:
      sRGB lit ~(133, 186, 236), sRGB shadow ~(101, 142, 180),
      linear lit ~(61, 128, 214), linear shadow ~(34, 70, 118).
    Restricted to the lower 55% of the frame (floor region below the horizon).
    """
    h, w, _ = rgb_img.shape
    rgb = rgb_img.astype(np.float32)
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]

    mask = (
        (b > 95)
        & ((b - r) > 50)
        & ((b - g) > 22)
        & (b > 1.40 * np.maximum(r, 1.0))
        & (g > r + 15)
    )
    mask[:int(h * 0.45), :] = False
    return mask


def segment_orange_cube(rgb_img: np.ndarray) -> np.ndarray:
    """Boolean (H, W) mask for the Room Beta orange pick cube.

    Material RGB (0.94, 0.62, 0.15) under Ogre2 Gazebo Harmonic rendering:
      shaded vertical face ~(118, 97, 47) [range R:91..118, G:76..97, B:47..58],
      lit face ~(230..248, 191..205, 100..108), linear lit ~(240, 158, 38).
    Distinguished from wood furniture (R/B <= 1.55) and yellow floor (R-G <= 12, R/B <= 1.4).
    """
    rgb = rgb_img.astype(np.float32)
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]

    return (
        (r > 88)
        & (g > 65)
        & (g < 225)
        & (b < 135)
        & ((r - b) > 40)
        & ((r - g) > 12)
        & ((g - b) > 25)
        & (r > 1.75 * np.maximum(b, 1.0))
    )


def segment_drop_spot(rgb_img: np.ndarray) -> np.ndarray:
    """Boolean (H, W) mask for the Room Beta crimson/magenta drop zone mat.

    Material RGB (0.84, 0.14, 0.42) under linear or Ogre2 sRGB lighting:
      sRGB lit ~(236, 104, 172), sRGB shadow ~(180, 79, 131),
      linear lit ~(214, 36, 107), linear shadow ~(118, 20, 59).
    Restricted to the lower 55% of the frame (floor region below the horizon).
    """
    h, w, _ = rgb_img.shape
    rgb = rgb_img.astype(np.float32)
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]

    mask = (
        (r > 95)
        & (g < 140)
        & ((r - g) > 65)
        & ((b - g) > 18)
        & (r > b + 15)
    )
    mask[:int(h * 0.45), :] = False
    return mask


# ---------------------------------------------------------------------------
# Rigid Transforms & 3D / Ground-Plane Projection
# ---------------------------------------------------------------------------
def quat_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Return 3x3 rotation matrix from unit quaternion (x, y, z, w)."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def make_transform_matrix(tx: float, ty: float, tz: float,
                          qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Build 4x4 homogeneous transform matrix from translation and quaternion."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_rotation_matrix(qx, qy, qz, qw)
    T[:3, 3] = [tx, ty, tz]
    return T


def transform_stamped_to_matrix(transform_stamped) -> np.ndarray:
    """Convert a geometry_msgs/TransformStamped into a 4x4 homogeneous matrix."""
    t = transform_stamped.transform.translation
    q = transform_stamped.transform.rotation
    return make_transform_matrix(t.x, t.y, t.z, q.x, q.y, q.z, q.w)


def intrinsics_from_camera_info(msg):
    """Extract (fx, fy, cx, cy) from sensor_msgs/CameraInfo (with safe fallback)."""
    if msg is None:
        return DEFAULT_FX, DEFAULT_FY, DEFAULT_CX, DEFAULT_CY
    k = msg.k
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if fx <= 0.0 or fy <= 0.0:
        return DEFAULT_FX, DEFAULT_FY, DEFAULT_CX, DEFAULT_CY
    return fx, fy, cx, cy


def project_pixels_with_depth(
    us: np.ndarray,
    vs: np.ndarray,
    depth_img: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    T_map_cam: np.ndarray,
    min_depth: float = 0.14,
    max_depth: float = 4.5,
) -> np.ndarray:
    """Project (u, v) pixels with aligned Z-depth into 3D `map` points (N, 3)."""
    if len(us) == 0 or depth_img is None:
        return np.empty((0, 3), dtype=np.float64)

    ds = depth_img[vs, us].astype(np.float64)
    valid = np.isfinite(ds) & (ds >= min_depth) & (ds <= max_depth)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float64)

    u_v = us[valid].astype(np.float64)
    v_v = vs[valid].astype(np.float64)
    d_v = ds[valid]

    x_cam = (u_v - cx) * d_v / fx
    y_cam = (v_v - cy) * d_v / fy
    z_cam = d_v

    pts_cam = np.stack([x_cam, y_cam, z_cam], axis=1)  # (N, 3)
    R = T_map_cam[:3, :3]
    t = T_map_cam[:3, 3]
    return pts_cam @ R.T + t


def project_pixels_to_ground(
    us: np.ndarray,
    vs: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    T_map_cam: np.ndarray,
    ground_z: float = 0.003,
    min_range: float = 0.15,
    max_range: float = 4.2,
    depth_img: np.ndarray = None,
    max_depth_discrepancy: float = 0.35,
) -> np.ndarray:
    """Intersect camera rays for (u, v) with z = ground_z in `map` frame -> (N, 2).

    If `depth_img` is provided, pixels whose measured depth is much closer than
    the ground intersection (indicating a vertical obstacle rather than floor)
    are rejected.
    """
    if len(us) == 0:
        return np.empty((0, 2), dtype=np.float64)

    u_f = us.astype(np.float64)
    v_f = vs.astype(np.float64)

    dirs_cam = np.stack([(u_f - cx) / fx, (v_f - cy) / fy, np.ones_like(u_f)], axis=1)
    R = T_map_cam[:3, :3]
    origin = T_map_cam[:3, 3]
    dirs_map = dirs_cam @ R.T  # (N, 3)

    # Rays must point downward toward the floor plane
    dz = dirs_map[:, 2]
    downward = dz < -1e-3
    if not np.any(downward):
        return np.empty((0, 2), dtype=np.float64)

    dirs_map = dirs_map[downward]
    dz = dz[downward]
    us_d = us[downward]
    vs_d = vs[downward]

    # Parameter s along optical Z-axis where z_map == ground_z
    s = (ground_z - origin[2]) / dz
    pts_map = origin[None, :] + s[:, None] * dirs_map
    horiz_dist = np.hypot(pts_map[:, 0] - origin[0], pts_map[:, 1] - origin[1])

    valid = (s >= min_range) & (horiz_dist >= min_range) & (horiz_dist <= max_range)

    if depth_img is not None:
        meas_d = depth_img[vs_d, us_d].astype(np.float64)
        finite_d = np.isfinite(meas_d)
        # Reject pixels that hit an obstacle well in front of the floor ray
        valid &= (~finite_d) | (np.abs(meas_d - s) <= max_depth_discrepancy)

    if not np.any(valid):
        return np.empty((0, 2), dtype=np.float64)
    return pts_map[valid, :2]


# ---------------------------------------------------------------------------
# High-Level Frame Detectors
# ---------------------------------------------------------------------------
def detect_floor_point_in_map(
    rgb_img: np.ndarray,
    depth_img: np.ndarray,
    intrinsics,
    T_map_cam: np.ndarray,
    target_label: str,
    min_pixels: int = 80,
    max_range: float = 3.8,
):
    """Project visible yellow (Room Alpha) or green (Room Beta) floor pixels into `map`.

    Returns (x_map, y_map) median point *inside* the coloured room floor (useful
    when the robot glimpses a room through a doorway without entering), or None.
    """
    h, w, _ = rgb_img.shape
    roi_top = int(h * 0.55)
    rgb = rgb_img.astype(np.float32)
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]

    if target_label == 'YELLOW':
        mask = (r > 125) & (g > 110) & (b < 195) & ((r - b) > 45) & ((g - b) > 35)
    elif target_label == 'GREEN':
        mask = (g > 100) & ((g - r) > 15) & ((g - b) > 5)
    else:
        return None

    mask[:roi_top, :] = False
    vs, us = np.nonzero(mask)
    if len(us) < min_pixels:
        return None
    us = us[::3]
    vs = vs[::3]

    fx, fy, cx, cy = intrinsics
    pts = project_pixels_to_ground(
        us, vs, fx, fy, cx, cy, T_map_cam,
        ground_z=0.001, min_range=0.40, max_range=max_range,
        depth_img=depth_img, max_depth_discrepancy=0.45,
    )
    if len(pts) < 15:
        return None
    med = np.median(pts, axis=0)
    return float(med[0]), float(med[1])


def detect_clean_zone_points(
    rgb_img: np.ndarray,
    depth_img: np.ndarray,
    intrinsics,
    T_map_cam: np.ndarray,
    min_pixels: int = 30,
    stride: int = 2,
    max_range: float = 4.0,
) -> np.ndarray:
    """Detect Room Alpha clean-zone mat pixels and project them onto map (x, y)."""
    mask = segment_clean_zone(rgb_img)
    vs, us = np.nonzero(mask)
    if len(us) < min_pixels:
        return np.empty((0, 2), dtype=np.float64)
    if stride > 1:
        us = us[::stride]
        vs = vs[::stride]
    fx, fy, cx, cy = intrinsics
    return project_pixels_to_ground(
        us, vs, fx, fy, cx, cy, T_map_cam,
        ground_z=0.003, min_range=0.18, max_range=max_range,
        depth_img=depth_img, max_depth_discrepancy=0.40,
    )


def detect_orange_cube_map(
    rgb_img: np.ndarray,
    depth_img: np.ndarray,
    intrinsics,
    T_map_cam: np.ndarray,
    min_pixels: int = 12,
    min_z: float = 0.005,
    max_z: float = 0.22,
    max_range: float = 4.5,
    cube_half_size: float = 0.075,
):
    """Detect the 3D orange cube and estimate its center (x, y) in `map` frame.

    Depth hits the *front face* of the 0.15 m cube, so we offset the median
    surface point along the camera->surface horizontal direction by
    `cube_half_size` to estimate the cube's true geometric centre.
    Returns (cx_map, cy_map, num_points) or None.
    """
    if depth_img is None:
        return None
    mask = segment_orange_cube(rgb_img)
    vs, us = np.nonzero(mask)
    if len(us) < min_pixels:
        return None

    fx, fy, cx, cy = intrinsics
    pts = project_pixels_with_depth(
        us, vs, depth_img, fx, fy, cx, cy, T_map_cam,
        min_depth=0.16, max_depth=max_range,
    )
    if len(pts) < min_pixels:
        return None

    # Height filter removes floor reflections and taller wood furniture
    z_ok = (pts[:, 2] >= min_z) & (pts[:, 2] <= max_z)
    pts = pts[z_ok]
    if len(pts) < max(10, min_pixels // 2):
        return None

    med = np.median(pts[:, :2], axis=0)
    dists = np.hypot(pts[:, 0] - med[0], pts[:, 1] - med[1])
    inliers = pts[dists <= 0.22]
    if len(inliers) < max(8, min_pixels // 2):
        return None

    # Compactness check: a 0.15 m cube should not span > 0.35 m
    span_x = float(np.percentile(inliers[:, 0], 95) - np.percentile(inliers[:, 0], 5))
    span_y = float(np.percentile(inliers[:, 1], 95) - np.percentile(inliers[:, 1], 5))
    if max(span_x, span_y) > 0.35:
        return None

    surf_xy = np.median(inliers[:, :2], axis=0)
    cam_xy = T_map_cam[:2, 3]
    ray = surf_xy - cam_xy
    ray_norm = float(np.hypot(ray[0], ray[1]))
    if ray_norm > 1e-4:
        center_xy = surf_xy + (ray / ray_norm) * cube_half_size
    else:
        center_xy = surf_xy

    return float(center_xy[0]), float(center_xy[1]), int(len(inliers))


def detect_drop_spot_map(
    rgb_img: np.ndarray,
    depth_img: np.ndarray,
    intrinsics,
    T_map_cam: np.ndarray,
    min_pixels: int = 25,
    max_range: float = 4.2,
):
    """Detect the flat crimson/magenta Room Beta drop mat and return (x, y, count) or None."""
    mask = segment_drop_spot(rgb_img)
    vs, us = np.nonzero(mask)
    if len(us) < min_pixels:
        return None

    fx, fy, cx, cy = intrinsics
    pts = project_pixels_to_ground(
        us, vs, fx, fy, cx, cy, T_map_cam,
        ground_z=0.003, min_range=0.18, max_range=max_range,
        depth_img=depth_img, max_depth_discrepancy=0.35,
    )
    if len(pts) < max(12, min_pixels // 2):
        return None

    med = np.median(pts, axis=0)
    dists = np.hypot(pts[:, 0] - med[0], pts[:, 1] - med[1])
    inliers = pts[dists <= 0.45]
    if len(inliers) < max(10, min_pixels // 2):
        return None

    center = np.median(inliers, axis=0)
    return float(center[0]), float(center[1]), int(len(inliers))


# ---------------------------------------------------------------------------
# Target Bounds & Centroid Estimators (Option B: Nominal Dimensions)
# ---------------------------------------------------------------------------
def estimate_cluster_centroid(obs_xy, max_cluster_radius: float = 0.40):
    """Robust centroid from a sequence of (x, y) observations."""
    if not obs_xy:
        return None
    pts = np.asarray(obs_xy, dtype=np.float64)
    if pts.ndim != 2 or len(pts) == 0:
        return None
    med = np.median(pts, axis=0)
    dists = np.hypot(pts[:, 0] - med[0], pts[:, 1] - med[1])
    inliers = pts[dists <= max_cluster_radius]
    if len(inliers) == 0:
        return float(med[0]), float(med[1])
    c = np.median(inliers, axis=0)
    return float(c[0]), float(c[1])


def _is_cell_occupied(grid: np.ndarray, r: int, c: int) -> bool:
    rows, cols = grid.shape
    if r < 0 or r >= rows or c < 0 or c >= cols:
        return True
    return int(grid[r, c]) > FREE_MAX


def _clamp_box_to_free_space(x_min, x_max, y_min, y_max, grid, origin_x, origin_y,
                             resolution, wall_margin_m: float = 0.25):
    """Shrink bounding box edges slightly if any edge intrudes into occupied cells."""
    if grid is None or resolution <= 0.0:
        return x_min, x_max, y_min, y_max

    rows, cols = grid.shape
    margin_cells = max(1, int(round(wall_margin_m / resolution)))
    occupied = grid > OCCUPIED_THRESH

    # Dilate occupied mask by wall_margin_m
    blocked = occupied.copy()
    for _ in range(margin_cells):
        g = blocked.copy()
        g[1:, :] |= blocked[:-1, :]
        g[:-1, :] |= blocked[1:, :]
        g[:, 1:] |= blocked[:, :-1]
        g[:, :-1] |= blocked[:, 1:]
        blocked = g

    def world_to_rc(x, y):
        return int((y - origin_y) / resolution), int((x - origin_x) / resolution)

    # Pull each side inward if its midpoint/samples sit in blocked space
    step = resolution
    for _ in range(int(1.0 / max(step, 0.01))):
        changed = False
        if (x_max - x_min) < 0.6 or (y_max - y_min) < 0.6:
            break
        ys = np.linspace(y_min + 0.1, y_max - 0.1, 5)
        xs = np.linspace(x_min + 0.1, x_max - 0.1, 5)

        def edge_blocked(pts):
            hits = 0
            for px, py in pts:
                r, c = world_to_rc(px, py)
                if 0 <= r < rows and 0 <= c < cols and blocked[r, c]:
                    hits += 1
            return hits >= 2

        if edge_blocked([(x_min, y) for y in ys]):
            x_min += step
            changed = True
        if edge_blocked([(x_max, y) for y in ys]):
            x_max -= step
            changed = True
        if edge_blocked([(x, y_min) for x in xs]):
            y_min += step
            changed = True
        if edge_blocked([(x, y_max) for x in xs]):
            y_max -= step
            changed = True
        if not changed:
            break

    return float(x_min), float(x_max), float(y_min), float(y_max)


def estimate_clean_zone_bounds(
    pts_xy,
    nominal_sx: float = 2.29,
    nominal_sy: float = 1.57,
    grid: np.ndarray = None,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
    resolution: float = 0.05,
    wall_margin_m: float = 0.25,
):
    """Fit an axis-aligned cleaning zone (x_min, x_max, y_min, y_max) from floor points.

    Uses robust 5th..95th percentiles to reject stray projections, then uses the
    known nominal zone size (Option B) to complete any partially clipped edge
    while clamping against occupied walls/furniture on the SLAM occupancy grid.
    """
    if pts_xy is None or len(pts_xy) < 15:
        return None
    pts = np.asarray(pts_xy, dtype=np.float64)
    med = np.median(pts, axis=0)
    max_r = math.hypot(nominal_sx, nominal_sy) * 0.85
    inliers = pts[np.hypot(pts[:, 0] - med[0], pts[:, 1] - med[1]) <= max_r]
    if len(inliers) < 15:
        return None

    x_lo, x_hi = float(np.percentile(inliers[:, 0], 5)), float(np.percentile(inliers[:, 0], 95))
    y_lo, y_hi = float(np.percentile(inliers[:, 1], 5)), float(np.percentile(inliers[:, 1], 95))

    cx = 0.5 * (x_lo + x_hi)
    cy = 0.5 * (y_lo + y_hi)
    obs_sx = x_hi - x_lo
    obs_sy = y_hi - y_lo

    # Option B: use nominal dimensions when observed span is smaller or noisy,
    # keeping observed span if it closely matches nominal.
    half_x = 0.5 * max(min(obs_sx, nominal_sx * 1.10), nominal_sx * 0.92)
    half_y = 0.5 * max(min(obs_sy, nominal_sy * 1.10), nominal_sy * 0.92)

    x_min, x_max = cx - half_x, cx + half_x
    y_min, y_max = cy - half_y, cy + half_y

    if grid is not None:
        x_min, x_max, y_min, y_max = _clamp_box_to_free_space(
            x_min, x_max, y_min, y_max, grid, origin_x, origin_y, resolution,
            wall_margin_m=wall_margin_m,
        )

    return float(x_min), float(x_max), float(y_min), float(y_max)


# ---------------------------------------------------------------------------
# Map-Based Approach Selection & Active Room Search Viewpoints
# ---------------------------------------------------------------------------
def _dilate_obstacles(grid: np.ndarray, clearance_cells: int) -> np.ndarray:
    """Return boolean mask of cells within `clearance_cells` of any obstacle."""
    blocked = grid > OCCUPIED_THRESH
    for _ in range(max(0, clearance_cells)):
        g = blocked.copy()
        g[1:, :] |= blocked[:-1, :]
        g[:-1, :] |= blocked[1:, :]
        g[:, 1:] |= blocked[:, :-1]
        g[:, :-1] |= blocked[:, 1:]
        g[1:, 1:] |= blocked[:-1, :-1]
        g[1:, :-1] |= blocked[:-1, 1:]
        g[:-1, 1:] |= blocked[1:, :-1]
        g[:-1, :-1] |= blocked[1:, 1:]
        blocked = g
    return blocked


def select_approach_pose(
    target_x: float,
    target_y: float,
    standoff_m: float,
    grid: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    robot_xy=None,
    clearance_m: float = 0.42,
    num_angles: int = 16,
    ignore_target_radius_m: float = 0.16,
):
    """Choose a collision-free approach pose (ax, ay, yaw) facing (target_x, target_y).

    Evaluates `num_angles` candidate positions around the target at distance
    `standoff_m`, ignoring obstacle cells belonging to the target itself (within
    `ignore_target_radius_m`). Tries `clearance_m` first (matched to Nav2's
    inflation radius) and gracefully relaxes clearance only if no candidate fits.
    """
    if grid is None or resolution <= 0.0:
        if robot_xy is not None:
            ang = math.atan2(robot_xy[1] - target_y, robot_xy[0] - target_x)
        else:
            ang = 0.0
        ax = target_x + standoff_m * math.cos(ang)
        ay = target_y + standoff_m * math.sin(ang)
        yaw = math.atan2(target_y - ay, target_x - ax)
        return float(ax), float(ay), float(yaw)

    rows, cols = grid.shape
    # Mask out occupied cells belonging to the target object itself (e.g. cube in SLAM map)
    clean_grid = grid.copy()
    if ignore_target_radius_m > 0.0:
        tr = int((target_y - origin_y) / resolution)
        tc = int((target_x - origin_x) / resolution)
        rad_cells = int(math.ceil(ignore_target_radius_m / resolution))
        r0, r1 = max(0, tr - rad_cells), min(rows, tr + rad_cells + 1)
        c0, c1 = max(0, tc - rad_cells), min(cols, tc + rad_cells + 1)
        for r in range(r0, r1):
            for c in range(c0, c1):
                wx = origin_x + (c + 0.5) * resolution
                wy = origin_y + (r + 0.5) * resolution
                if math.hypot(wx - target_x, wy - target_y) <= ignore_target_radius_m:
                    if clean_grid[r, c] > FREE_MAX:
                        clean_grid[r, c] = 0

    best = None
    best_score = -math.inf
    for try_clearance in (clearance_m, min(clearance_m, 0.34), min(clearance_m, 0.26)):
        clearance_cells = max(1, int(round(try_clearance / resolution)))
        blocked = _dilate_obstacles(clean_grid, clearance_cells)

        for i in range(num_angles):
            ang = 2.0 * math.pi * i / num_angles
            ax = target_x + standoff_m * math.cos(ang)
            ay = target_y + standoff_m * math.sin(ang)
            r = int((ay - origin_y) / resolution)
            c = int((ax - origin_x) / resolution)
            if not (0 <= r < rows and 0 <= c < cols):
                continue
            if clean_grid[r, c] < 0 or clean_grid[r, c] > FREE_MAX or blocked[r, c]:
                continue

            # Also check the midpoint between approach pose and target isn't blocked by a wall
            mx = target_x + 0.5 * standoff_m * math.cos(ang)
            my = target_y + 0.5 * standoff_m * math.sin(ang)
            mr = int((my - origin_y) / resolution)
            mc = int((mx - origin_x) / resolution)
            if not (0 <= mr < rows and 0 <= mc < cols) or clean_grid[mr, mc] > OCCUPIED_THRESH:
                continue

            # Score: prefer extra local free space and proximity to robot
            patch = clean_grid[max(0, r - 6):min(rows, r + 7), max(0, c - 6):min(cols, c + 7)]
            free_frac = float(np.mean((patch >= 0) & (patch <= FREE_MAX)))
            dist_penalty = 0.0
            if robot_xy is not None:
                dist_penalty = 0.15 * math.hypot(ax - robot_xy[0], ay - robot_xy[1])
            score = 2.0 * free_frac - dist_penalty
            if score > best_score:
                yaw = math.atan2(target_y - ay, target_x - ax)
                best = (float(ax), float(ay), float(yaw))
                best_score = score

        if best is not None:
            return best

    # Fallback: approach from direction of robot
    ang = math.atan2(robot_xy[1] - target_y, robot_xy[0] - target_x) if robot_xy else 0.0
    ax = target_x + standoff_m * math.cos(ang)
    ay = target_y + standoff_m * math.sin(ang)
    return float(ax), float(ay), float(math.atan2(target_y - ay, target_x - ax))


def sample_room_search_viewpoints(
    robot_xy,
    grid: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    max_radius_m: float = 6.0,
    min_clearance_m: float = 0.42,
    spacing_m: float = 1.6,
    max_points: int = 6,
    forward_hint_yaw: float = None,
):
    """Sample reachable, collision-free viewpoints near `robot_xy` for active room search.

    Uses BFS flood-fill from `robot_xy` over free cells cleared by `min_clearance_m`
    so viewpoints stay reachable and clear of furniture/walls, then greedily selects
    well-spaced points (nearest-first ordering) up to `max_points`.
    """
    if grid is None or resolution <= 0.0 or robot_xy is None:
        return []

    rows, cols = grid.shape
    sr = int((robot_xy[1] - origin_y) / resolution)
    sc = int((robot_xy[0] - origin_x) / resolution)
    if not (0 <= sr < rows and 0 <= sc < cols):
        return []

    clearance_cells = max(1, int(round(min_clearance_m / resolution)))
    blocked = _dilate_obstacles(grid, clearance_cells)
    free = (grid >= 0) & (grid <= FREE_MAX) & (~blocked)

    # If the robot's exact cell is inside clearance inflation, snap seed to nearest free cell
    if not free[sr, sc]:
        found = None
        for rad in range(1, 15):
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    rr, cc = sr + dr, sc + dc
                    if 0 <= rr < rows and 0 <= cc < cols and free[rr, cc]:
                        found = (rr, cc)
                        break
                if found:
                    break
            if found:
                break
        if found is None:
            return []
        sr, sc = found

    # BFS within max_radius_m (path-distance capped so we mostly stay in the current room)
    max_steps = int(round((max_radius_m * 1.4) / resolution))
    visited = np.zeros((rows, cols), dtype=bool)
    visited[sr, sc] = True
    queue = deque([(sr, sc, 0)])
    candidates = []

    while queue:
        r, c, step = queue.popleft()
        wx = origin_x + (c + 0.5) * resolution
        wy = origin_y + (r + 0.5) * resolution
        dist = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
        if dist >= 0.8 and dist <= max_radius_m:
            candidates.append((wx, wy, dist))
        if step >= max_steps:
            continue
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < rows and 0 <= cc < cols and not visited[rr, cc] and free[rr, cc]:
                wx2 = origin_x + (cc + 0.5) * resolution
                wy2 = origin_y + (rr + 0.5) * resolution
                if math.hypot(wx2 - robot_xy[0], wy2 - robot_xy[1]) <= max_radius_m:
                    visited[rr, cc] = True
                    queue.append((rr, cc, step + 1))

    if not candidates:
        return []

    # Sort candidates: prefer forward direction if hinted (entering a room), then medium distance
    def cand_key(item):
        wx, wy, dist = item
        ang_pen = 0.0
        if forward_hint_yaw is not None:
            ang = math.atan2(wy - robot_xy[1], wx - robot_xy[0])
            diff = abs(math.atan2(math.sin(ang - forward_hint_yaw),
                                  math.cos(ang - forward_hint_yaw)))
            # Strongly prefer points inside the room (in front of doorway entry)
            ang_pen = 2.0 * diff
        return dist + ang_pen

    candidates.sort(key=cand_key)

    chosen = []
    for wx, wy, _ in candidates:
        if all(math.hypot(wx - cx, wy - cy) >= spacing_m for cx, cy in chosen):
            chosen.append((float(wx), float(wy)))
            if len(chosen) >= max_points:
                break

    return chosen
