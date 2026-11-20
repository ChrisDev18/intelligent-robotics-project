#!/usr/bin/env python3
"""Summarize Alpha Cleaning evaluation CSV logs into coverage and mission metrics.

Examples:
    python3 src/home_nav_sim/tools/summarize_alpha_eval.py
    python3 src/home_nav_sim/tools/summarize_alpha_eval.py log/alpha_cleaning_eval.csv --by-scenario
    python3 src/home_nav_sim/tools/summarize_alpha_eval.py log/alpha_cleaning_eval.csv --cov-thresh 0.90
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


def _eval_row(row: dict, cov_thresh=None):
    cov_ratio = _to_float(row.get('coverage_ratio'))
    if cov_thresh is None:
        mission_ok = _to_bool(row.get('mission_success'))
    else:
        cov_ok = cov_ratio is not None and cov_ratio >= cov_thresh
        dirt_ratio = _to_float(row.get('dirt_ratio'))
        dirt_ok = dirt_ratio is None or dirt_ratio >= 0.80
        mission_ok = cov_ok and dirt_ok
    return mission_ok, cov_ratio


def summarize_rows(rows: list, label: str = 'Overall', cov_thresh=None):
    total = len(rows)
    if total == 0:
        print(f'{label}: no records found.')
        return

    evals = [_eval_row(r, cov_thresh) for r in rows]
    mission_ok = [e[0] for e in evals]
    cov_ratios = [e[1] for e in evals if e[1] is not None]
    dirt_ratios = [_to_float(r.get('dirt_ratio')) for r in rows if _to_float(r.get('dirt_ratio')) is not None]
    clean_times = [_to_float(r.get('cleaning_time_sec')) for r in rows if _to_float(r.get('cleaning_time_sec')) is not None]
    total_times = [_to_float(r.get('total_time_sec')) for r in rows if _to_float(r.get('total_time_sec')) is not None]

    n_mission = sum(mission_ok)
    mission_rate = (n_mission / total) * 100.0

    def _stats(vals):
        if not vals:
            return 'N/A'
        mean = statistics.mean(vals)
        std = statistics.stdev(vals) if len(vals) > 1 else 0.0
        return f'{mean:.3f} ± {std:.3f} (min={min(vals):.3f}, max={max(vals):.3f})'

    print('=' * 72)
    print(f' Alpha Cleaning Summary: {label} (N = {total} trials)')
    print('=' * 72)
    print(f'  Mission Success Rate     : {n_mission}/{total} ({mission_rate:.1f}%)')
    print(f'  Coverage Ratio           : {_stats(cov_ratios)}')
    print(f'  Dirt Removal Ratio       : {_stats(dirt_ratios)}')
    print(f'  Cleaning Time (s)        : {_stats(clean_times)}')
    print(f'  Total Mission Time (s)   : {_stats(total_times)}')

    failures = Counter()
    for r in rows:
        reason = r.get('failure_reason', '').strip()
        if reason:
            for part in reason.split(';'):
                p = part.strip()
                if p:
                    failures[p] += 1
    if failures:
        print('  Failure Reasons:')
        for reason, count in failures.most_common():
            print(f'    - {reason}: {count}')
    print('=' * 72)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'csv_path',
        nargs='?',
        default='log/alpha_cleaning_eval.csv',
        help='Path to evaluation CSV log',
    )
    parser.add_argument('--by-scenario', action='store_true', help='Group stats by scenario')
    parser.add_argument('--scenario', type=str, default=None, help='Filter to a specific scenario')
    parser.add_argument('--cov-thresh', type=float, default=None, help='Override coverage threshold')
    args = parser.parse_args()

    if not os.path.isfile(args.csv_path):
        print(f'Error: CSV file not found at {args.csv_path}', file=sys.stderr)
        sys.exit(1)

    with open(args.csv_path, 'r', newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))

    if args.scenario:
        rows = [r for r in rows if r.get('scenario', '').strip() == args.scenario.strip()]

    if args.by_scenario:
        grouped = defaultdict(list)
        for r in rows:
            sc = r.get('scenario', '').strip() or '(default)'
            grouped[sc].append(r)
        for sc, sc_rows in sorted(grouped.items()):
            summarize_rows(sc_rows, label=f'Scenario: {sc}', cov_thresh=args.cov_thresh)
            print()
    else:
        summarize_rows(rows, cov_thresh=args.cov_thresh)


if __name__ == '__main__':
    main()
