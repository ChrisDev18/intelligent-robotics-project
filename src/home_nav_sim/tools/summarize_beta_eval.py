#!/usr/bin/env python3
"""Summarize Beta Pick & Place evaluation CSV logs into separate pick/place metrics.

Examples:
    python3 src/home_nav_sim/tools/summarize_beta_eval.py
    python3 src/home_nav_sim/tools/summarize_beta_eval.py log/beta_pick_place_eval.csv --by-scenario
    python3 src/home_nav_sim/tools/summarize_beta_eval.py log/beta_pick_place_eval.csv --scenario humans_true
    python3 src/home_nav_sim/tools/summarize_beta_eval.py log/beta_pick_place_eval.csv --pick-thresh 0.10 --place-thresh 0.20
"""
import argparse
from collections import Counter, defaultdict
import csv
import os
import statistics
import sys


def _to_bool(val) -> bool:
    return str(val).strip().lower() in ('true', '1', 'yes')


def _to_float(val):
    if val is None:
        return None
    s = str(val).strip()
    if s == '':
        return None
    try:
        return float(s)
    except ValueError:
        return None


def _eval_row(row: dict, pick_thresh=None, place_thresh=None, lift_thresh: float = 0.08, max_z: float = 0.15):
    """Return (pick_ok, place_ok, mission_ok) for a CSV row, optionally overriding thresholds."""
    use_gt = row.get('pose_source', '').strip() == 'gz_ground_truth'

    if pick_thresh is None:
        pick_ok = _to_bool(row.get('pick_success'))
    else:
        pk_err = _to_float(row.get('pickup_distance_error_m'))
        dist_ok = pk_err is not None and pk_err <= pick_thresh
        lift_dz = _to_float(row.get('cube_lift_delta_z'))
        lift_ok = (lift_dz is not None and lift_dz >= lift_thresh) if use_gt else True
        pick_ok = dist_ok and lift_ok

    if place_thresh is None:
        place_ok = _to_bool(row.get('place_success'))
    else:
        pl_err = _to_float(row.get('placement_distance_error_m'))
        dist_ok = pl_err is not None and pl_err <= place_thresh
        final_z = _to_float(row.get('cube_final_z'))
        floor_ok = (final_z is not None and final_z <= max_z) if use_gt else True
        place_ok = dist_ok and floor_ok

    if pick_thresh is None and place_thresh is None:
        mission_ok = _to_bool(row.get('mission_success'))
    else:
        mission_ok = pick_ok and place_ok

    return pick_ok, place_ok, mission_ok


def _fmt_stats(values, unit: str = 'm') -> str:
    if not values:
        return 'N/A'
    mean_v = statistics.mean(values)
    med_v = statistics.median(values)
    std_v = statistics.stdev(values) if len(values) > 1 else 0.0
    min_v = min(values)
    max_v = max(values)
    return (
        f'mean={mean_v:.4f} {unit}, median={med_v:.4f} {unit}, '
        f'std={std_v:.4f} {unit}, min={min_v:.4f} {unit}, max={max_v:.4f} {unit} (n={len(values)})'
    )


def summarize_rows(rows: list, title: str, pick_thresh=None, place_thresh=None):
    total = len(rows)
    if total == 0:
        print(f'{title}: no trials found.')
        return

    n_pick = 0
    n_place = 0
    n_place_given_pick = 0
    n_mission = 0
    pick_errs = []
    pick_est_errs = []
    place_errs = []
    place_est_errs = []
    pick_times = []
    place_times = []
    total_times = []
    sources = Counter()
    failures = Counter()

    for r in rows:
        pk_ok, pl_ok, ms_ok = _eval_row(r, pick_thresh=pick_thresh, place_thresh=place_thresh)
        n_pick += int(pk_ok)
        n_place += int(pl_ok)
        if pk_ok and pl_ok:
            n_place_given_pick += 1
        n_mission += int(ms_ok)

        for key, bucket in (
            ('pickup_distance_error_m', pick_errs),
            ('pickup_estimated_error_m', pick_est_errs),
            ('placement_distance_error_m', place_errs),
            ('placement_estimated_error_m', place_est_errs),
            ('pick_time_sec', pick_times),
            ('place_time_sec', place_times),
            ('total_time_sec', total_times),
        ):
            v = _to_float(r.get(key))
            if v is not None:
                bucket.append(v)

        sources[r.get('pose_source', 'unknown')] += 1
        reason = (r.get('failure_reason') or '').strip()
        if reason:
            for part in reason.split(';'):
                part = part.strip()
                if part:
                    failures[part] += 1

    cond_str = (
        f'{n_place_given_pick}/{n_pick} ({100.0 * n_place_given_pick / n_pick:.1f}%)'
        if n_pick > 0 else 'N/A'
    )

    print('=' * 72)
    print(f'{title}  |  Trials: {total}  |  Pose sources: {dict(sources)}')
    if pick_thresh is not None or place_thresh is not None:
        print(
            f'Threshold overrides: pick_thresh={pick_thresh}, place_thresh={place_thresh}'
        )
    print('-' * 72)
    print(f'Pick Success Rate       : {n_pick:3d}/{total:<3d} ({100.0 * n_pick / total:5.1f}%)')
    print(f'Place Success Rate      : {n_place:3d}/{total:<3d} ({100.0 * n_place / total:5.1f}%)')
    print(f'Place | Pick Rate       : {cond_str}')
    print(f'Overall Mission Success : {n_mission:3d}/{total:<3d} ({100.0 * n_mission / total:5.1f}%)')
    print('-' * 72)
    print(f'Pickup Distance Error   : {_fmt_stats(pick_errs, "m")}')
    if pick_est_errs:
        print(f'Pickup Est. Error (ROS) : {_fmt_stats(pick_est_errs, "m")}')
    print(f'Placement Distance Err  : {_fmt_stats(place_errs, "m")}')
    if place_est_errs:
        print(f'Placement Est. Err (ROS): {_fmt_stats(place_est_errs, "m")}')
    print('-' * 72)
    print(f'Pick Duration           : {_fmt_stats(pick_times, "s")}')
    print(f'Place Duration          : {_fmt_stats(place_times, "s")}')
    print(f'Total Duration          : {_fmt_stats(total_times, "s")}')
    if failures:
        print('-' * 72)
        print('Failure Reasons         : ' + ', '.join(f'{k}={v}' for k, v in failures.most_common()))
    print('=' * 72)


def main():
    parser = argparse.ArgumentParser(
        description='Summarize Beta Pick & Place pickup/placement distance error logs.'
    )
    parser.add_argument(
        'csv_path',
        nargs='?',
        default='log/beta_pick_place_eval.csv',
        help='Path to evaluation CSV log (default: log/beta_pick_place_eval.csv)',
    )
    parser.add_argument(
        '--scenario',
        default=None,
        help='Filter trials to a specific scenario label.',
    )
    parser.add_argument(
        '--by-scenario',
        action='store_true',
        help='Also print a separate breakdown for each scenario.',
    )
    parser.add_argument(
        '--pick-thresh',
        type=float,
        default=None,
        help='Re-evaluate pick success using this pickup distance error threshold (metres).',
    )
    parser.add_argument(
        '--place-thresh',
        type=float,
        default=None,
        help='Re-evaluate place success using this placement distance error threshold (metres).',
    )
    args = parser.parse_args()

    if not os.path.isfile(args.csv_path):
        print(f'Evaluation log not found: {args.csv_path}', file=sys.stderr)
        sys.exit(1)

    with open(args.csv_path, 'r', newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))

    if args.scenario is not None:
        rows = [r for r in rows if (r.get('scenario') or '').strip() == args.scenario]
        summarize_rows(
            rows,
            f'Beta Pick & Place Evaluation (scenario="{args.scenario}")',
            pick_thresh=args.pick_thresh,
            place_thresh=args.place_thresh,
        )
        return

    summarize_rows(
        rows,
        f'Beta Pick & Place Evaluation ({args.csv_path})',
        pick_thresh=args.pick_thresh,
        place_thresh=args.place_thresh,
    )

    if args.by_scenario:
        grouped = defaultdict(list)
        for r in rows:
            grouped[(r.get('scenario') or '(default)').strip() or '(default)'].append(r)
        for scen, srows in sorted(grouped.items()):
            summarize_rows(
                srows,
                f'Scenario: {scen}',
                pick_thresh=args.pick_thresh,
                place_thresh=args.place_thresh,
            )


if __name__ == '__main__':
    main()
