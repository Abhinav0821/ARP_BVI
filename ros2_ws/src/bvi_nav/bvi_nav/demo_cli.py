"""Standalone terminal demo of the planner - no ROS required.

python3 -m bvi_nav.demo_cli --seed 42 --show-values

Prints the map as ASCII, runs Backward Value Iteration, extracts the optimal
path and runs the assignment's verification checklist. Useful for step 3 of the
build plan and as a fallback if RViz2 misbehaves during the demo.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

from .bvi import backward_value_iteration, extract_path, policy_field, verify
from .grid_map import GridMap


def parse_cell(text: str):
    x, y = (int(v) for v in text.replace('(', '').replace(')', '').split(','))
    return [x, y]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Backward Value Iteration demo (no ROS)')
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--size', type=int, default=30)
    ap.add_argument('--ratio', type=float, default=0.20)
    ap.add_argument('--start', type=parse_cell, default=None, help='e.g. --start 1,1')
    ap.add_argument('--goal', type=parse_cell, default=None, help='e.g. --goal 28,28')
    ap.add_argument('--show-values', action='store_true',
                    help='print the full V table as a numeric grid')
    ap.add_argument('--show-policy', action='store_true',
                    help='print the optimal action (N/S/E/W) for every cell')
    args = ap.parse_args(argv)

    grid = GridMap(width=args.size, height=args.size,
                   obstacle_ratio=args.ratio, seed=args.seed)
    start, goal = grid.pick_start_goal(args.start, args.goal)
    result = backward_value_iteration(grid, goal)
    path = extract_path(grid, result, start)

    cells = grid.width * grid.height
    print(f'Grid            : {grid.width} x {grid.height} ({cells} cells)')
    print(f'Seed            : {grid.seed}')
    print(f'Obstacles       : {grid.obstacle_count()} '
          f'({100.0 * grid.obstacle_count() / cells:.1f}%)')
    print(f'Start / Goal    : {start} / {goal}')
    print(f'BVI sweeps      : {result.iterations} ({result.updates} value updates)')
    print(f'Reachable states: {result.reachable} of {len(grid.free_cells())} free cells')
    print(f'V(start)        : {result.v(start):.0f}   max finite V: {result.max_finite():.0f}')
    print(f'Path length     : {len(path) - 1} moves')

    print('\nMap  (# obstacle, . free, o path, S start, G goal; y increases upwards)')
    print(grid.to_ascii(start, goal, path))

    if args.show_values:
        print('\nCost-to-go V(s)  ("##" = obstacle, " ." = unreachable)')
        for y in reversed(range(grid.height)):
            row = []
            for x in range(grid.width):
                if grid.grid[y][x]:
                    row.append(' ##')
                elif not np.isfinite(result.value[y][x]):
                    row.append('  .')
                else:
                    row.append(f'{int(result.value[y][x]):3d}')
            print(''.join(row))

    if args.show_policy:
        print('\nOptimal policy  ("#" obstacle, "G" goal, "." unreachable)')
        policy = policy_field(grid, result)
        for y in reversed(range(grid.height)):
            row = []
            for x in range(grid.width):
                if grid.grid[y][x]:
                    row.append('#')
                elif (x, y) == goal:
                    row.append('G')
                else:
                    row.append(policy.get((x, y), '.'))
            print(''.join(row))

    print('\nVerification checklist')
    failed = 0
    for name, ok, detail in verify(grid, result, start, path):
        failed += not ok
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}{'  -> ' + detail if detail else ''}")
    print(f"\n{'ALL CHECKS PASSED' if not failed else str(failed) + ' CHECK(S) FAILED'}")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
