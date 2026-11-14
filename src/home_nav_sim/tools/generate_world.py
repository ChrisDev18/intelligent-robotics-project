#!/usr/bin/env python3
"""Generate the home_nav_sim Gazebo Harmonic worlds.

Edit the lists below (all units in metres, origin = bottom-left outer corner
of the map, x to the east, y to the north) and run:

    python3 tools/generate_world.py

It writes:
    worlds/home.sdf             - everything, including the 3 walking humans
    worlds/home_no_humans.sdf   - same world without humans (use for SLAM mapping)
"""
from pathlib import Path

WALL_H = 2.5      # wall height
WALL_T = 0.2      # wall thickness

# (name, x1, y1, x2, y2) wall centre-line segments
WALLS = [
    ("wall_south", 0.0, 0.0, 20.0, 0.0),          # entrance closed
    ("wall_north", 0.0, 12.0, 20.0, 12.0),
    ("wall_west", 0.0, 0.0, 0.0, 12.0),
    ("wall_east", 20.0, 0.0, 20.0, 12.0),
    # wall between hall and rooms, with Door 1 (x 4.0-5.2) and Door 2 (x 14.8-16.0)
    ("wall_mid_a", 0.0, 7.0, 4.0, 7.0),
    ("wall_mid_b", 5.2, 7.0, 14.8, 7.0),
    ("wall_mid_c", 16.0, 7.0, 20.0, 7.0),
    # wall between Room 1 (Alpha) and Room 2 (Beta)
    ("wall_divider", 10.0, 7.0, 10.0, 12.0),
]

# Coloured floors (visual only): name, cx, cy, sx, sy, rgb
FLOORS = [
    ("floor_alpha_room1", 5.0, 9.5, 10.0, 5.0, (1.00, 0.91, 0.48)),   # yellow
    ("floor_beta_room2", 15.0, 9.5, 10.0, 5.0, (0.72, 0.94, 0.85)),   # mint
    ("floor_hall", 10.0, 3.5, 20.0, 7.0, (0.86, 0.85, 0.96)),         # lavender
]

WOOD = (0.55, 0.38, 0.22)
DARK_WOOD = (0.40, 0.27, 0.16)
FABRIC = (0.35, 0.40, 0.55)
WHITE = (0.92, 0.92, 0.90)
BLACK = (0.15, 0.15, 0.15)

# Box furniture: name, cx, cy, sx, sy, height, rgb, yaw
BOXES = [
    # Room 1 - Alpha (bedroom)
    ("bed", 1.36, 10.84, 2.00, 1.61, 0.50, WHITE, 0.0),
    ("nightstand", 2.82, 11.39, 0.50, 0.50, 0.55, WOOD, 0.0),
    ("wardrobe", 8.54, 11.39, 2.21, 0.64, 2.00, DARK_WOOD, 0.0),
    ("bedroom_desk", 8.96, 7.68, 1.36, 0.64, 0.75, WOOD, 0.0),
    # Room 2 - Beta (office)
    ("office_desk_1", 11.54, 11.04, 1.64, 0.79, 0.75, WOOD, 0.0),
    ("office_desk_2", 17.25, 11.04, 1.64, 0.79, 0.75, WOOD, 0.0),
    ("bookshelf", 19.59, 9.57, 0.46, 3.71, 1.80, DARK_WOOD, 0.0),
    ("meeting_table", 17.86, 8.14, 2.14, 1.00, 0.75, WOOD, 0.0),
    # Hall
    ("sofa_long", 1.71, 1.57, 2.57, 0.86, 0.80, FABRIC, 0.0),
    ("sofa_side", 0.82, 3.54, 0.79, 2.36, 0.80, FABRIC, 0.0),
    ("coffee_table", 2.43, 3.39, 1.43, 0.79, 0.45, WOOD, 0.0),
    ("dining_table", 10.00, 3.50, 2.57, 1.43, 0.75, WOOD, 0.0),   # centre of hall
    ("bench", 14.11, 0.57, 2.50, 0.57, 0.45, DARK_WOOD, 0.0),
]

# Cylinder furniture: name, cx, cy, radius, height, rgb
CYLINDERS = [
    ("bedroom_chair", 8.96, 8.32, 0.20, 0.90, BLACK),
    ("office_chair_1", 11.54, 10.29, 0.22, 0.90, BLACK),
    ("office_chair_2", 17.25, 10.29, 0.22, 0.90, BLACK),
    ("dining_chair_1", 9.36, 4.46, 0.20, 0.90, DARK_WOOD),
    ("dining_chair_2", 10.64, 4.46, 0.20, 0.90, DARK_WOOD),
    ("dining_chair_3", 9.36, 2.54, 0.20, 0.90, DARK_WOOD),
    ("dining_chair_4", 10.64, 2.54, 0.20, 0.90, DARK_WOOD),
    ("dining_chair_5", 8.46, 3.50, 0.20, 0.90, DARK_WOOD),
    ("dining_chair_6", 11.54, 3.50, 0.20, 0.90, DARK_WOOD),
    ("plant_1", 0.71, 6.29, 0.29, 0.80, (0.20, 0.55, 0.25)),
    ("plant_2", 19.29, 6.29, 0.29, 0.80, (0.20, 0.55, 0.25)),
    ("plant_3", 8.43, 0.64, 0.29, 0.80, (0.20, 0.55, 0.25)),
    ("plant_4", 11.57, 0.64, 0.29, 0.80, (0.20, 0.55, 0.25)),
]

# Task markers
CLEAN_ZONE = (6.79, 9.14, 2.29, 1.57)       # Alpha: cx, cy, sx, sy (visual only)
PICK_ITEM = (12.86, 8.21, 0.15)             # Beta: cx, cy, cube size (movable)
DROP_SPOT = (18.98, 9.30, 0.54)             # Beta: cx, cy, size (visual only)

ROBOT_SPAWN = (19.21, 0.79, 1.5708)         # x, y, yaw - used by the launch file

WALK_DAE = "https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae"

# Humans walking back and forth: name, (x_a, y_a), (x_b, y_b), seconds per leg
HUMANS = [
    ("human_alpha", (3.50, 10.43), (6.86, 10.43), 4.0),
    ("human_beta", (15.89, 11.14), (15.89, 8.07), 4.0),
    ("human_hall", (7.21, 6.29), (7.21, 3.00), 4.0),
]


def rgba(c, a=1.0):
    return f"{c[0]} {c[1]} {c[2]} {a}"


def material(c, a=1.0):
    return (f"<material><ambient>{rgba(c, a)}</ambient><diffuse>{rgba(c, a)}</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular></material>")


def static_box(name, cx, cy, sx, sy, h, color, yaw=0.0, collide=True, z=None, alpha=1.0):
    z = h / 2 if z is None else z
    geom = f"<geometry><box><size>{sx} {sy} {h}</size></box></geometry>"
    col = f"<collision name='collision'>{geom}</collision>" if collide else ""
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{cx} {cy} {z} 0 0 {yaw}</pose>
      <link name='link'>
        {col}
        <visual name='visual'>{geom}{material(color, alpha)}</visual>
      </link>
    </model>"""


def static_cylinder(name, cx, cy, r, h, color):
    geom = f"<geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>"
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{cx} {cy} {h / 2} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>{geom}</collision>
        <visual name='visual'>{geom}{material(color)}</visual>
      </link>
    </model>"""


def wall(name, x1, y1, x2, y2):
    import math
    length = math.hypot(x2 - x1, y2 - y1) + WALL_T  # overlap corners
    yaw = math.atan2(y2 - y1, x2 - x1)
    return static_box(name, (x1 + x2) / 2, (y1 + y2) / 2, round(length, 3), WALL_T,
                      WALL_H, (0.93, 0.93, 0.91), round(yaw, 4))


def pick_item():
    cx, cy, s = PICK_ITEM
    m = 0.2
    i = m * s * s / 6
    geom = f"<geometry><box><size>{s} {s} {s}</size></box></geometry>"
    return f"""
    <model name='beta_pick_item'>
      <pose>{cx} {cy} {s / 2} 0 0 0</pose>
      <link name='link'>
        <inertial><mass>{m}</mass>
          <inertia><ixx>{i:.6f}</ixx><iyy>{i:.6f}</iyy><izz>{i:.6f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name='collision'>{geom}</collision>
        <visual name='visual'>{geom}{material((0.94, 0.62, 0.15))}</visual>
      </link>
    </model>"""


def actor(name, a, b, leg):
    import math
    yaw_ab = round(math.atan2(b[1] - a[1], b[0] - a[0]), 4)
    yaw_ba = round(math.atan2(a[1] - b[1], a[0] - b[0]), 4)
    t1, t2, t3, t4 = leg, leg + 1.0, 2 * leg + 1.0, 2 * leg + 2.0
    wps = [
        (0.0, a, yaw_ab), (t1, b, yaw_ab), (t2, b, yaw_ba), (t3, a, yaw_ba), (t4, a, yaw_ab),
    ]
    wp_xml = "".join(
        f"<waypoint><time>{t}</time><pose>{p[0]} {p[1]} 1.0 0 0 {y}</pose></waypoint>"
        for t, p, y in wps)
    return f"""
    <actor name='{name}'>
      <skin><filename>{WALK_DAE}</filename><scale>1.0</scale></skin>
      <animation name='walk'><filename>{WALK_DAE}</filename><interpolate_x>true</interpolate_x></animation>
      <script>
        <loop>true</loop>
        <delay_start>0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id='0' type='walk' tension='0.6'>{wp_xml}</trajectory>
      </script>
    </actor>"""


def build(with_humans):
    parts = []
    for w in WALLS:
        parts.append(wall(*w))
    for n, cx, cy, sx, sy, c in FLOORS:
        parts.append(static_box(n, cx, cy, sx, sy, 0.002, c, collide=False, z=0.001))
    for b in BOXES:
        parts.append(static_box(*b))
    for c in CYLINDERS:
        parts.append(static_cylinder(*c))
    cx, cy, sx, sy = CLEAN_ZONE
    parts.append(static_box("alpha_clean_zone", cx, cy, sx, sy, 0.002, (0.11, 0.62, 0.46),
                            collide=False, z=0.003, alpha=0.6))
    parts.append(pick_item())
    dx, dy, ds = DROP_SPOT
    parts.append(static_box("beta_drop_spot", dx, dy, ds, ds, 0.002, (0.73, 0.46, 0.09),
                            collide=False, z=0.003, alpha=0.7))
    if with_humans:
        for h in HUMANS:
            parts.append(actor(*h))

    return f"""<?xml version="1.0"?>
<!-- Generated by tools/generate_world.py - edit that file, not this one. -->
<sdf version="1.9">
  <world name="empty">
    <physics name="2ms" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <!-- Sensors and IMU systems are loaded by the robot model itself. -->

    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.8 0.8 0.8 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>10 6 20 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -1.0</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>
        </visual>
      </link>
    </model>
{''.join(parts)}
  </world>
</sdf>
"""


if __name__ == "__main__":
    out = Path(__file__).resolve().parent.parent / "worlds"
    out.mkdir(exist_ok=True)
    (out / "home.sdf").write_text(build(True))
    (out / "home_no_humans.sdf").write_text(build(False))
    print(f"Wrote {out / 'home.sdf'} and {out / 'home_no_humans.sdf'}")
