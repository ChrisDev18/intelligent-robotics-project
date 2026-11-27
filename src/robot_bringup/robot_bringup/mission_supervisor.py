#!/usr/bin/env python3
"""Phase E: mission supervisor for the full autonomous run.

Supports both interleaved room-discovery execution (Room Alpha or Room Beta in
whichever order the robot enters them first) and sequential execution:

    explore  <->  enter first room  ->  /room_X/ready -> task X -> /room_X/done -> resume explore
             <->  enter second room ->  /room_Y/ready -> task Y -> /room_Y/done -> finish map
             ->   /exploration_done ->  doorway_navigator drives home -> done

If a task still fails after its retry, the supervisor publishes its /room_X/done
itself so the mission carries on instead of waiting forever, and records the
failure.

Publishes /mission_status (std_msgs/String, latched): exploring, cleaning_alpha,
beta_pick_place, finishing_map, returning_home, done (or done_with_failures).
Appends one row per run to log/mission_results.csv with stage times and results.

Parameters
  max_attempts      tries per task, including the first (default 2)
  task_timeout      seconds before a task attempt is stopped (default 900)
  discover_targets  pass discover_targets:=true to task nodes (default True)
  home_x, home_y    where doorway_navigator parks at the end, map frame (default 0.5, 0.5)
  home_radius       how close counts as home, metres (default 0.7)
  results_csv       where to append results (default log/mission_results.csv)
"""
import csv
import math
import os
import subprocess
import threading
import time
from datetime import datetime

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

TASKS = {
    'room_alpha': {'executable': 'alpha_cleaning', 'status': 'cleaning_alpha', 'label': 'Room Alpha cleaning'},
    'room_beta': {'executable': 'beta_pick_place', 'status': 'beta_pick_place', 'label': 'Room Beta pick-and-place'},
}


class MissionSupervisor(Node):
    def __init__(self):
        super().__init__('mission_supervisor')
        self.max_attempts = int(self.declare_parameter('max_attempts', 2).value)
        self.task_timeout = float(self.declare_parameter('task_timeout', 900.0).value)
        self.discover_targets = bool(self.declare_parameter('discover_targets', True).value)
        self.home_xy = (float(self.declare_parameter('home_x', 0.5).value),
                        float(self.declare_parameter('home_y', 0.5).value))
        self.home_radius = float(self.declare_parameter('home_radius', 0.7).value)
        self.results_csv = self.declare_parameter('results_csv', 'log/mission_results.csv').value

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/mission_status', latched)
        self.done_pubs = {room: self.create_publisher(Bool, f'/{room}/done', latched)
                          for room in TASKS}
        self.create_subscription(Bool, '/exploration_done', self.on_exploration_done, latched)
        for room in TASKS:
            self.create_subscription(Bool, f'/{room}/ready',
                                     lambda msg, r=room: self.on_ready(r, msg), latched)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.t0 = time.monotonic()
        self.marks = {}                     # stage -> seconds since start (real time)
        self.results = {room: {'ok': None, 'attempts': 0, 'seconds': None} for room in TASKS}
        self.task_order = []
        self.started = set()
        self.exploration_done = False
        self.finished = False
        self.set_status('exploring')
        self.create_timer(2.0, self.check_home)
        self.get_logger().info(
            f'Mission supervisor ready (discover_targets={self.discover_targets}): '
            'waiting for room triggers or exploration completion.'
        )

    # --- helpers ----------------------------------------------------------
    def elapsed(self):
        return time.monotonic() - self.t0

    def set_status(self, status):
        self.status_pub.publish(String(data=status))
        self.get_logger().info(f'[mission] {status}  (t = {self.elapsed():.0f} s)')

    def all_tasks_finished(self):
        return all(r['ok'] is not None for r in self.results.values())

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time(),
                                                timeout=Duration(seconds=0.2))
        except Exception:
            return None
        return t.transform.translation.x, t.transform.translation.y

    # --- events -----------------------------------------------------------
    def on_exploration_done(self, msg):
        if msg.data and not self.exploration_done:
            self.exploration_done = True
            self.marks['exploration'] = self.elapsed()
            self.get_logger().info(f'Exploration finished after {self.marks["exploration"]:.0f} s')
            if self.all_tasks_finished() and not self.finished:
                self.set_status('returning_home')

    def on_ready(self, room, msg):
        if not msg.data or room in self.started:
            return
        self.started.add(room)
        self.task_order.append(room)
        threading.Thread(target=self.run_task, args=(room,), daemon=True).start()

    def run_task(self, room):
        task = TASKS[room]
        self.set_status(task['status'])
        start = self.elapsed()
        ok = False
        for attempt in range(1, self.max_attempts + 1):
            self.results[room]['attempts'] = attempt
            self.get_logger().info(f'{task["label"]}: attempt {attempt}/{self.max_attempts}')
            cmd = [
                'ros2', 'run', 'robot_bringup', task['executable'],
                '--ros-args', '-p', f'discover_targets:={str(self.discover_targets).lower()}',
            ]
            try:
                ok = subprocess.run(cmd, timeout=self.task_timeout).returncode == 0
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f'{task["label"]}: attempt {attempt} timed out')
                ok = False
            if ok:
                break
            self.get_logger().warn(f'{task["label"]}: attempt {attempt} failed')
        self.results[room].update(ok=ok, seconds=self.elapsed() - start)
        if ok:
            self.get_logger().info(f'{task["label"]}: SUCCESS in {self.results[room]["seconds"]:.0f} s')
        else:
            # The task node only publishes /room_X/done on success; publish it here so
            # doorway_navigator moves on instead of waiting forever.
            self.get_logger().error(f'{task["label"]}: FAILED after {self.max_attempts} attempts; '
                                    'continuing the mission')
            self.done_pubs[room].publish(Bool(data=True))

        if self.all_tasks_finished():
            if self.exploration_done:
                self.set_status('returning_home')
            else:
                self.set_status('finishing_map')
        elif not self.exploration_done:
            self.set_status('exploring')

    def check_home(self):
        if self.finished or not self.exploration_done or not self.all_tasks_finished():
            return
        pos = self.robot_xy()
        if pos is None:
            return
        if math.hypot(pos[0] - self.home_xy[0], pos[1] - self.home_xy[1]) <= self.home_radius:
            self.finished = True
            self.marks['total'] = self.elapsed()
            all_ok = all(r['ok'] for r in self.results.values())
            self.set_status('done' if all_ok else 'done_with_failures')
            self.write_results(all_ok)

    def write_results(self, all_ok):
        row = {
            'date': datetime.now().isoformat(timespec='seconds'),
            'task_order': '->'.join(self.task_order),
            'exploration_s': round(self.marks.get('exploration', float('nan')), 1),
            'alpha_ok': self.results['room_alpha']['ok'],
            'alpha_attempts': self.results['room_alpha']['attempts'],
            'alpha_s': round(self.results['room_alpha']['seconds'] or float('nan'), 1),
            'beta_ok': self.results['room_beta']['ok'],
            'beta_attempts': self.results['room_beta']['attempts'],
            'beta_s': round(self.results['room_beta']['seconds'] or float('nan'), 1),
            'total_s': round(self.marks['total'], 1),
            'mission_success': all_ok,
        }
        os.makedirs(os.path.dirname(self.results_csv) or '.', exist_ok=True)
        new_file = not os.path.exists(self.results_csv)
        with open(self.results_csv, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(row))
            if new_file:
                writer.writeheader()
            writer.writerow(row)
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'MISSION {"COMPLETE" if all_ok else "FINISHED WITH FAILURES"} '
                               f'in {row["total_s"]:.0f} s (real time) | order={row["task_order"]}')
        self.get_logger().info(f'  exploration {row["exploration_s"]} s | '
                               f'Alpha {"OK" if row["alpha_ok"] else "FAILED"} ({row["alpha_s"]} s, '
                               f'{row["alpha_attempts"]} attempt(s)) | '
                               f'Beta {"OK" if row["beta_ok"] else "FAILED"} ({row["beta_s"]} s, '
                               f'{row["beta_attempts"]} attempt(s))')
        self.get_logger().info(f'  results appended to {self.results_csv}')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
