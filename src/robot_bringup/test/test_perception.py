"""Unit tests for robot_bringup.perception vision, projection, and map helpers."""
import math
import numpy as np

from robot_bringup.perception import (
    DEFAULT_CX,
    DEFAULT_CY,
    DEFAULT_FX,
    DEFAULT_FY,
    classify_floor_color,
    detect_clean_zone_points,
    detect_drop_spot_map,
    detect_floor_point_in_map,
    detect_orange_cube_map,
    estimate_clean_zone_bounds,
    project_pixels_to_ground,
    project_pixels_with_depth,
    sample_room_search_viewpoints,
    segment_clean_zone,
    segment_drop_spot,
    segment_orange_cube,
    select_approach_pose,
)


def _camera_pose_facing_east(x=0.0, y=0.0, z=0.094):
    """Build T_map_cam for camera optical frame looking along +X (East), level."""
    # Optical +X -> map -Y (right), Optical +Y -> map -Z (down), Optical +Z -> map +X (forward)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ], dtype=np.float64)
    T[:3, 3] = [x, y, z]
    return T


def _solid_image(rgb, h=240, w=320):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:, :] = rgb
    return img


def test_floor_classification_lit_and_shadow():
    # Yellow Room Alpha (lit and shadowed)
    assert classify_floor_color(_solid_image((255, 232, 122)))[0] == 'YELLOW'
    assert classify_floor_color(_solid_image((140, 128, 67)))[0] == 'YELLOW'
    # Mint Room Beta (lit and shadowed)
    assert classify_floor_color(_solid_image((184, 240, 217)))[0] == 'GREEN'
    assert classify_floor_color(_solid_image((105, 136, 122)))[0] == 'GREEN'
    # Hall lavender
    assert classify_floor_color(_solid_image((219, 217, 245)))[0] == 'HALL'


def test_color_masks_positive_and_negative():
    # Slate-blue clean zone (linear & Ogre2 sRGB lit/shadow) vs yellow floor & wood
    img_clean_lit = _solid_image((61, 128, 214))
    img_clean_shadow = _solid_image((34, 70, 118))
    img_clean_srgb_lit = _solid_image((133, 186, 236))
    img_clean_srgb_shadow = _solid_image((101, 142, 180))
    img_wood_lin = _solid_image((140, 97, 56))
    img_wood_srgb = _solid_image((194, 164, 128))
    img_yellow_srgb = _solid_image((255, 244, 183))
    img_yellow_shadow = _solid_image((140, 128, 67))
    assert np.count_nonzero(segment_clean_zone(img_clean_lit)) > 10000
    assert np.count_nonzero(segment_clean_zone(img_clean_shadow)) > 10000
    assert np.count_nonzero(segment_clean_zone(img_clean_srgb_lit)) > 10000
    assert np.count_nonzero(segment_clean_zone(img_clean_srgb_shadow)) > 10000
    assert np.count_nonzero(segment_clean_zone(img_wood_lin)) == 0
    assert np.count_nonzero(segment_clean_zone(img_wood_srgb)) == 0
    assert np.count_nonzero(segment_clean_zone(img_yellow_shadow)) == 0

    # Orange cube (linear, Ogre2 sRGB lit, and real Gazebo shaded vertical face)
    img_cube_lit = _solid_image((240, 158, 38))
    img_cube_shadow = _solid_image((132, 87, 21))
    img_cube_srgb_top = _solid_image((248, 205, 108))
    img_cube_srgb_vert = _solid_image((230, 191, 100))
    img_cube_gz_shaded = _solid_image((118, 97, 47))
    img_cube_gz_edge = _solid_image((91, 76, 47))
    img_wood_gz_shaded = _solid_image((90, 76, 59))
    img_drop_lit = _solid_image((214, 36, 107))
    img_drop_shadow = _solid_image((118, 20, 59))
    img_drop_srgb_lit = _solid_image((236, 104, 172))
    img_drop_srgb_shadow = _solid_image((180, 79, 131))
    assert np.count_nonzero(segment_orange_cube(img_cube_lit)) > 10000
    assert np.count_nonzero(segment_orange_cube(img_cube_shadow)) > 10000
    assert np.count_nonzero(segment_orange_cube(img_cube_srgb_top)) > 10000
    assert np.count_nonzero(segment_orange_cube(img_cube_srgb_vert)) > 10000
    assert np.count_nonzero(segment_orange_cube(img_cube_gz_shaded)) > 10000
    assert np.count_nonzero(segment_orange_cube(img_cube_gz_edge)) > 10000
    assert np.count_nonzero(segment_orange_cube(img_wood_srgb)) == 0
    assert np.count_nonzero(segment_orange_cube(img_wood_gz_shaded)) == 0
    assert np.count_nonzero(segment_orange_cube(img_yellow_srgb)) == 0
    assert np.count_nonzero(segment_orange_cube(img_drop_lit)) == 0
    assert np.count_nonzero(segment_orange_cube(img_drop_srgb_lit)) == 0

    # Crimson drop spot (linear & Ogre2 sRGB lit/shadow) vs orange cube, wood, mint floor
    img_mint = _solid_image((184, 240, 217))
    assert np.count_nonzero(segment_drop_spot(img_drop_lit)) > 10000
    assert np.count_nonzero(segment_drop_spot(img_drop_shadow)) > 10000
    assert np.count_nonzero(segment_drop_spot(img_drop_srgb_lit)) > 10000
    assert np.count_nonzero(segment_drop_spot(img_drop_srgb_shadow)) > 10000
    assert np.count_nonzero(segment_drop_spot(img_cube_lit)) == 0
    assert np.count_nonzero(segment_drop_spot(img_cube_srgb_top)) == 0
    assert np.count_nonzero(segment_drop_spot(img_wood_lin)) == 0
    assert np.count_nonzero(segment_drop_spot(img_wood_srgb)) == 0
    assert np.count_nonzero(segment_drop_spot(img_mint)) == 0


def test_ground_plane_and_depth_projection():
    T = _camera_pose_facing_east(x=1.0, y=2.0, z=0.10)
    # Center pixel at depth 2.0 m should map to (3.0, 2.0, 0.10)
    depth = np.full((240, 320), 2.0, dtype=np.float32)
    pts = project_pixels_with_depth(
        np.array([int(DEFAULT_CX)]), np.array([int(DEFAULT_CY)]),
        depth, DEFAULT_FX, DEFAULT_FY, DEFAULT_CX, DEFAULT_CY, T,
    )
    assert pts.shape == (1, 3)
    assert np.allclose(pts[0], [3.0, 2.0, 0.10], atol=1e-3)

    # Pixel below horizon hitting ground_z=0.0 at distance 1.0 m ahead:
    # dz/d = -0.10 / 1.0 => (v - cy) / fy = 0.10 => v = cy + 0.10 * fy
    v_ground = int(round(DEFAULT_CY + 0.10 * DEFAULT_FY))
    u_center = int(DEFAULT_CX)
    gpts = project_pixels_to_ground(
        np.array([u_center]), np.array([v_ground]),
        DEFAULT_FX, DEFAULT_FY, DEFAULT_CX, DEFAULT_CY, T, ground_z=0.0,
    )
    assert gpts.shape == (1, 2)
    assert math.isclose(gpts[0, 0], 2.0, abs_tol=0.05)
    assert math.isclose(gpts[0, 1], 2.0, abs_tol=0.01)


def test_detect_cube_and_drop_spot_synthetic():
    T = _camera_pose_facing_east(x=-8.0, y=7.42, z=0.094)
    intrinsics = (DEFAULT_FX, DEFAULT_FY, DEFAULT_CX, DEFAULT_CY)

    # Place orange cube front face 1.5 m ahead -> centre at x = -8.0 + 1.5 + 0.075 = -6.425
    rgb = _solid_image((184, 240, 217))
    depth = np.full((240, 320), 4.0, dtype=np.float32)
    rgb[110:135, 150:170] = (240, 158, 38)
    depth[110:135, 150:170] = 1.50
    res = detect_orange_cube_map(rgb, depth, intrinsics, T)
    assert res is not None
    bx, by, _ = res
    assert math.isclose(bx, -6.425, abs_tol=0.03)
    assert math.isclose(by, 7.42, abs_tol=0.03)

    # Place crimson drop spot on floor ~1.0 m ahead
    rgb2 = _solid_image((184, 240, 217))
    v_1m = int(round(DEFAULT_CY + 0.091 * DEFAULT_FY))
    rgb2[v_1m - 3:v_1m + 4, 145:175] = (214, 36, 107)
    res_drop = detect_drop_spot_map(rgb2, None, intrinsics, T)
    assert res_drop is not None
    dx, dy, _ = res_drop
    assert math.isclose(dx, -7.0, abs_tol=0.12)
    assert math.isclose(dy, 7.42, abs_tol=0.05)

    # Projecting green floor pixels through a doorway returns a point *ahead* inside the room
    floor_pt = detect_floor_point_in_map(rgb2, None, intrinsics, T, 'GREEN')
    assert floor_pt is not None
    assert floor_pt[0] > -7.5  # strictly ahead of camera at x=-8.0


def test_clean_zone_bounds_and_obstacle_clamping():
    # Observed points cover partial width (1.5 m of 2.29 m) -> expanded to nominal
    xs = np.linspace(-13.2, -11.7, 20)
    ys = np.linspace(7.8, 8.9, 15)
    pts = np.array([(x, y) for x in xs for y in ys], dtype=np.float64)

    # Synthetic free map with a wall along x = -11.2
    res = 0.05
    ox, oy = -15.0, 6.0
    grid = np.zeros((100, 120), dtype=np.int16)  # 6m x 5m free space
    wall_col = int((-11.2 - ox) / res)
    grid[:, wall_col:] = 100

    bounds = estimate_clean_zone_bounds(
        pts, nominal_sx=2.29, nominal_sy=1.57,
        grid=grid, origin_x=ox, origin_y=oy, resolution=res, wall_margin_m=0.25,
    )
    assert bounds is not None
    x_min, x_max, y_min, y_max = bounds
    # Must stay clear of wall at x=-11.2 (with 0.25m margin -> x_max <= -11.40)
    assert x_max <= -11.35
    assert (y_max - y_min) >= 1.35


def test_select_approach_pose_avoids_wall():
    # Target near an east wall (like beta_drop_spot next to bookshelf)
    res = 0.05
    ox, oy = -5.0, 5.0
    grid = np.zeros((100, 120), dtype=np.int16)
    wall_col = int((0.2 - ox) / res)
    grid[:, wall_col:] = 100  # Wall at x >= 0.2

    tx, ty = -0.20, 8.50
    ax, ay, yaw = select_approach_pose(
        tx, ty, standoff_m=0.60,
        grid=grid, origin_x=ox, origin_y=oy, resolution=res,
        robot_xy=(-2.0, 8.50), clearance_m=0.28, ignore_target_radius_m=0.0,
    )
    # Approach pose must be on the open West side (x < tx) facing roughly East (yaw ~ 0)
    assert ax < tx - 0.3
    assert abs(math.atan2(math.sin(yaw), math.cos(yaw))) < 0.8


def test_sample_room_search_viewpoints():
    res = 0.05
    ox, oy = 0.0, 0.0
    grid = np.zeros((120, 160), dtype=np.int16)  # 8m x 6m room
    # Outer walls
    grid[0:3, :] = 100
    grid[-3:, :] = 100
    grid[:, 0:3] = 100
    grid[:, -3:] = 100
    # Obstacle block in middle
    grid[40:70, 70:100] = 100

    vps = sample_room_search_viewpoints(
        (1.0, 1.0), grid, ox, oy, res,
        max_radius_m=5.5, min_clearance_m=0.40, spacing_m=1.5, max_points=5,
    )
    assert len(vps) >= 3
    for vx, vy in vps:
        r = int((vy - oy) / res)
        c = int((vx - ox) / res)
        assert grid[r, c] == 0
