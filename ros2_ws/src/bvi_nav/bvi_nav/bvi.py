"""Backward Value Iteration (BVI) and optimal path extraction.

Pure Python + NumPy, no ROS imports (assignment guide, S3), so the algorithm can
be exercised and verified from a plain interpreter.

The dynamic programming recursion solved here is the shortest-path / cost-to-go
form of Bellman's equation for a deterministic system with unit action costs:

    V*(goal) = 0
    V*(s)    = min over actions a of [ cost(s, a) + V*(f(s, a)) ]   for s != goal

with ``cost(s, a) == 1`` for every N/S/E/W move. It is solved *backwards* from the
goal by repeated synchronous-in-place sweeps over the free state space until no
value changes (delta == 0), which is exactly Backward Value Iteration.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .grid_map import Cell, GridMap, MOVE_NAMES, MOVES

INF = float('inf')
STEP_COST = 1.0


@dataclass
class BVIResult:
    """Everything the planner produces, ready for visualisation."""

    value: np.ndarray                      # value[y][x], float, inf = unreachable
    goal: Cell
    iterations: int                        # number of sweeps performed
    updates: int                           # total value updates across all sweeps
    reachable: int = field(default=0)      # count of free cells with finite V

    # -------------------------------------------------------------- helpers
    def v(self, cell: Cell) -> float:
        return float(self.value[cell[1]][cell[0]])

    def is_reachable(self, cell: Cell) -> bool:
        return np.isfinite(self.value[cell[1]][cell[0]])

    def max_finite(self) -> float:
        finite = self.value[np.isfinite(self.value)]
        return float(finite.max()) if finite.size else 0.0


def backward_value_iteration(grid: GridMap, goal: Cell,
                             max_iterations: int = 100_000) -> BVIResult:
    """Compute the optimal cost-to-go V(s) to ``goal`` for every free state.

    Obstacles keep the sentinel value ``inf`` and are never updated. Free cells
    that stay ``inf`` after convergence are unreachable from the goal.
    """
    if not grid.is_free(goal):
        raise ValueError(f'goal {goal} must be a free cell')

    value = np.full((grid.height, grid.width), INF, dtype=float)
    value[goal[1]][goal[0]] = 0.0

    free_cells = [c for c in grid.free_cells() if c != goal]

    iterations = 0
    updates = 0
    while iterations < max_iterations:
        delta = 0.0
        iterations += 1
        for x, y in free_cells:
            best = INF
            for dx, dy in MOVES:                     # fixed N, S, E, W order
                nx, ny = x + dx, y + dy
                if not grid.is_free((nx, ny)):       # also rejects out-of-bounds
                    continue
                candidate = STEP_COST + value[ny][nx]
                if candidate < best:
                    best = candidate
            if best < value[y][x]:
                delta = max(delta, value[y][x] - best)
                value[y][x] = best
                updates += 1
        if delta == 0.0:                             # no value changed -> converged
            break
    else:
        raise RuntimeError('value iteration did not converge')

    reachable = int(np.count_nonzero(np.isfinite(value)))
    return BVIResult(value=value, goal=goal, iterations=iterations,
                     updates=updates, reachable=reachable)


def extract_path(grid: GridMap, result: BVIResult, start: Cell) -> List[Cell]:
    """Greedily follow the steepest descent of V from ``start`` to the goal.

    Ties are broken with the fixed N, S, E, W primitive order, so the path is
    deterministic. Raises if the start cannot reach the goal.
    """
    if not grid.is_free(start):
        raise ValueError(f'start {start} must be a free cell')
    if not result.is_reachable(start):
        raise ValueError(f'start {start} cannot reach goal {result.goal} '
                         '(V(start) is infinite)')

    path: List[Cell] = [start]
    s = start
    budget = int(result.v(start)) + 1
    while s != result.goal:
        if len(path) > budget + 1:                   # guards against a bad V table
            raise RuntimeError('path extraction exceeded V(start) steps')
        x, y = s
        best: Optional[Cell] = None
        best_v = result.v(s)
        for dx, dy in MOVES:
            n = (x + dx, y + dy)
            if not grid.is_free(n):
                continue
            if result.v(n) < best_v:
                best_v = result.v(n)
                best = n
        if best is None:
            raise RuntimeError(f'stuck at {s}: no neighbour with a lower value')
        s = best
        path.append(s)
    return path


def policy_field(grid: GridMap, result: BVIResult) -> Dict[Cell, str]:
    """Optimal action (one of 'N', 'S', 'E', 'W') for every reachable free cell."""
    policy: Dict[Cell, str] = {}
    for x, y in grid.free_cells():
        if (x, y) == result.goal or not result.is_reachable((x, y)):
            continue
        best_v, best_name = result.v((x, y)), None
        for (dx, dy), name in zip(MOVES, MOVE_NAMES):
            n = (x + dx, y + dy)
            if grid.is_free(n) and result.v(n) < best_v:
                best_v, best_name = result.v(n), name
        if best_name:
            policy[(x, y)] = best_name
    return policy


def verify(grid: GridMap, result: BVIResult, start: Cell,
           path: Sequence[Cell]) -> List[Tuple[str, bool, str]]:
    """Run the assignment's correctness checklist. Returns (name, passed, detail)."""
    checks: List[Tuple[str, bool, str]] = []

    def add(name: str, ok: bool, detail: str = '') -> None:
        checks.append((name, bool(ok), detail))

    expected = int(round(grid.obstacle_ratio * grid.width * grid.height))
    add('obstacle count is exactly round(ratio*cells)',
        grid.obstacle_count() == expected,
        f'{grid.obstacle_count()} obstacles (expected {expected})')

    add('start and goal are free and distinct',
        grid.is_free(start) and grid.is_free(result.goal) and start != result.goal,
        f'start={start} goal={result.goal}')

    add('V(goal) == 0', result.v(result.goal) == 0.0, f'V(goal)={result.v(result.goal)}')

    add('V(start) is finite', result.is_reachable(start), f'V(start)={result.v(start)}')

    # Bellman consistency: every reachable non-goal cell equals 1 + min over neighbours.
    bellman_ok, bad = True, ''
    for x, y in grid.free_cells():
        if (x, y) == result.goal or not result.is_reachable((x, y)):
            continue
        best = min((result.v(n) for n in grid.neighbors((x, y))), default=INF)
        if result.v((x, y)) != best + STEP_COST:
            bellman_ok, bad = False, f'cell {(x, y)}'
            break
    add('Bellman optimality holds for every reachable cell', bellman_ok, bad)

    # Neighbouring reachable free cells differ by at most 1.
    smooth_ok, bad = True, ''
    for x, y in grid.free_cells():
        if not result.is_reachable((x, y)):
            continue
        for n in grid.neighbors((x, y)):
            if result.is_reachable(n) and abs(result.v((x, y)) - result.v(n)) > 1.0:
                smooth_ok, bad = False, f'{(x, y)} vs {n}'
                break
        if not smooth_ok:
            break
    add('neighbouring free values differ by at most 1', smooth_ok, bad)

    add('path length == V(start)',
        len(path) - 1 == result.v(start),
        f'len(path)-1={len(path) - 1}, V(start)={result.v(start)}')

    add('path starts at start and ends at goal',
        bool(path) and path[0] == start and path[-1] == result.goal,
        f'{path[0] if path else None} -> {path[-1] if path else None}')

    add('path stays on free, in-bounds cells',
        all(grid.is_free(c) for c in path), '')

    add('consecutive path cells are unit N/S/E/W moves',
        all(abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1
            for a, b in zip(path, path[1:])), '')

    return checks
