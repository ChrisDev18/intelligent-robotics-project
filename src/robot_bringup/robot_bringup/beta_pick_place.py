#!/usr/bin/env python3
"""Entry-point wrapper so `ros2 run robot_bringup beta_pick_place` runs the Phase D node."""
import importlib.util
import os
from ament_index_python.packages import get_package_prefix


def main(args=None):
    script_path = os.path.join(
        get_package_prefix('home_nav_sim'), 'lib', 'home_nav_sim', 'beta_pick_place.py'
    )
    spec = importlib.util.spec_from_file_location('beta_pick_place', script_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.main(args=args)


if __name__ == '__main__':
    main()
