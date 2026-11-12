#!/usr/bin/env python3
"""Save the live /map (from slam_toolbox) as a Nav2 map: <name>.pgm + <name>.yaml.

Used instead of nav2_map_server's map_saver_cli, which times out after
2 seconds when Gazebo runs slowly in WSL.

    python3 save_map.py ~/ros2_ws/src/intelligent-robotics-project/src/home_nav_sim/maps/home_map
"""
import math
import sys
from pathlib import Path

FREE, OCCUPIED, UNKNOWN = 254, 0, 205      # trinary map, same as map_saver_cli
FREE_THRESH, OCC_THRESH = 0.25, 0.65


def grid_to_pgm_bytes(width, height, data):
    """OccupancyGrid data (row 0 = bottom) -> binary PGM bytes (row 0 = top)."""
    rows = []
    for y in range(height - 1, -1, -1):
        row = bytearray(width)
        for x in range(width):
            v = data[y * width + x]
            if v < 0:
                row[x] = UNKNOWN
            elif v >= OCC_THRESH * 100:
                row[x] = OCCUPIED
            elif v <= FREE_THRESH * 100:
                row[x] = FREE
            else:
                row[x] = UNKNOWN
        rows.append(bytes(row))
    return f'P5\n{width} {height}\n255\n'.encode() + b''.join(rows)


def yaml_text(image_name, resolution, ox, oy, yaw):
    return (f'image: {image_name}\n'
            f'mode: trinary\n'
            f'resolution: {resolution:.3f}\n'
            f'origin: [{ox:.3f}, {oy:.3f}, {yaw:.3f}]\n'
            f'negate: 0\n'
            f'occupied_thresh: {OCC_THRESH}\n'
            f'free_thresh: {FREE_THRESH}\n')


def save(msg, out):
    info = msg.info
    q = info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    out.parent.mkdir(parents=True, exist_ok=True)
    pgm = out.with_suffix('.pgm')
    pgm.write_bytes(grid_to_pgm_bytes(info.width, info.height, list(msg.data)))
    out.with_suffix('.yaml').write_text(yaml_text(
        pgm.name, info.resolution, info.origin.position.x, info.origin.position.y, yaw))
    known = sum(1 for v in msg.data if v >= 0)
    print(f'Saved {pgm} ({info.width} x {info.height} cells, '
          f'{info.width * info.resolution:.1f} x {info.height * info.resolution:.1f} m, '
          f'{known} cells mapped) and {out.with_suffix(".yaml")}')


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    out = Path(sys.argv[1]).expanduser()
    if out.suffix in ('.pgm', '.yaml'):
        out = out.with_suffix('')

    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node('home_map_saver')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    got = []
    node.create_subscription(OccupancyGrid, '/map', got.append, qos)
    print('Waiting for /map from slam_toolbox (Ctrl+C to cancel)...')
    try:
        while rclpy.ok() and not got:
            rclpy.spin_once(node, timeout_sec=1.0)
        if got:
            save(got[-1], out)
    except KeyboardInterrupt:
        print('Cancelled, nothing saved.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
