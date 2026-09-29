"""Grid map generation and connectivity helpers.

Pure Python + NumPy on purpose: this module must stay free of ROS imports so it
can be unit-tested with a plain ``python3`` interpreter (assignment guide, S3).

Convention used everywhere in this package:
    grid[y][x]   0 = free cell, 1 = obstacle
    x -> column index (East),  y -> row index (North)
"""

from __future__ import annotations

import random
from typing import Iterator, List, Optional, Sequence, Tuple

import numpy as np

FREE = 0
OBSTACLE = 1

Cell = Tuple[int, int]  # (x, y)

# Fixed motion primitive order: N, S, E, W. Ties are broken in this order so the
# extracted path is deterministic.
MOVES: Tuple[Cell, ...] = (
    (0, 1),    # N  (+y)
    (0, -1),   # S  (-y)
    (1, 0),    # E  (+x)
    (-1, 0),   # W  (-x)
)
MOVE_NAMES = ('N', 'S', 'E', 'W')


class GridMap:
    """A 2-D occupancy grid with N/S/E/W connectivity."""

    def __init__(self, width: int = 30, height: int = 30,
                 obstacle_ratio: float = 0.20, seed: int = 42) -> None:
        if width <= 0 or height <= 0:
            raise ValueError('width and height must be positive')
        if not 0.0 <= obstacle_ratio < 1.0:
            raise ValueError('obstacle_ratio must be in [0, 1)')

        self.width = int(width)
        self.height = int(height)
        self.obstacle_ratio = float(obstacle_ratio)
        self.seed = int(seed)

        self.n_obstacles = int(round(self.obstacle_ratio * self.width * self.height))
        self.grid = np.zeros((self.height, self.width), dtype=np.int8)
        self._place_obstacles()

    # ------------------------------------------------------------------ map
    def _place_obstacles(self) -> None:
        """Place exactly ``n_obstacles`` distinct obstacles using a fixed seed."""
        rng = random.Random(self.seed)
        all_cells = [(x, y) for y in range(self.height) for x in range(self.width)]
        for x, y in rng.sample(all_cells, self.n_obstacles):
            self.grid[y][x] = OBSTACLE

    # ------------------------------------------------------------- queries
    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and self.grid[cell[1]][cell[0]] == FREE

    def neighbors(self, cell: Cell) -> Iterator[Cell]:
        """Yield the free in-bounds N/S/E/W neighbours of ``cell``, in NSEW order."""
        x, y = cell
        for dx, dy in MOVES:
            n = (x + dx, y + dy)
            if self.is_free(n):
                yield n

    def free_cells(self) -> List[Cell]:
        ys, xs = np.nonzero(self.grid == FREE)
        return [(int(x), int(y)) for x, y in zip(xs, ys)]

    def obstacle_count(self) -> int:
        return int(np.count_nonzero(self.grid == OBSTACLE))

    # -------------------------------------------------------- connectivity
    def connected_component(self, source: Cell) -> set:
        """Breadth-first flood fill of the free cells reachable from ``source``."""
        if not self.is_free(source):
            return set()
        seen = {source}
        frontier = [source]
        while frontier:
            nxt: List[Cell] = []
            for cell in frontier:
                for n in self.neighbors(cell):
                    if n not in seen:
                        seen.add(n)
                        nxt.append(n)
            frontier = nxt
        return seen

    def are_connected(self, a: Cell, b: Cell) -> bool:
        return b in self.connected_component(a)

    # ------------------------------------------------------ start and goal
    def pick_start_goal(self,
                        start: Optional[Sequence[int]] = None,
                        goal: Optional[Sequence[int]] = None) -> Tuple[Cell, Cell]:
        """Return a (start, goal) pair of free, distinct, mutually reachable cells.

        Any coordinate the caller supplies is honoured (and validated); the rest is
        chosen automatically. Automatic choices prefer cells near opposite corners
        (bottom-left for the start, top-right for the goal) inside the largest
        connected component, so the optimal path is long and visually interesting.
        """
        fixed_start = self._validate(start, 'start')
        fixed_goal = self._validate(goal, 'goal')

        if fixed_start is not None and fixed_goal is not None:
            if fixed_start == fixed_goal:
                raise ValueError('start and goal must be distinct cells')
            if not self.are_connected(fixed_start, fixed_goal):
                raise ValueError(
                    f'start {fixed_start} and goal {fixed_goal} are not connected; '
                    'change the seed or pick different cells')
            return fixed_start, fixed_goal

        component = self._largest_component()
        if len(component) < 2:
            raise RuntimeError('map has no connected component with two free cells')

        if fixed_start is not None:
            component = self.connected_component(fixed_start)
        elif fixed_goal is not None:
            component = self.connected_component(fixed_goal)
        if len(component) < 2:
            raise RuntimeError('the requested cell has no reachable neighbour')

        s = fixed_start if fixed_start is not None else self._closest_to(component, (0, 0))
        g = fixed_goal if fixed_goal is not None else self._closest_to(
            component, (self.width - 1, self.height - 1), exclude=s)
        if s == g:
            raise ValueError('start and goal must be distinct cells')
        return s, g

    def _validate(self, cell: Optional[Sequence[int]], label: str) -> Optional[Cell]:
        if cell is None:
            return None
        c = (int(cell[0]), int(cell[1]))
        if not self.in_bounds(c):
            raise ValueError(f'{label} {c} is outside the {self.width}x{self.height} grid')
        if not self.is_free(c):
            raise ValueError(f'{label} {c} is an obstacle cell')
        return c

    def _largest_component(self) -> set:
        unvisited = set(self.free_cells())
        best: set = set()
        while unvisited:
            comp = self.connected_component(next(iter(unvisited)))
            unvisited -= comp
            if len(comp) > len(best):
                best = comp
        return best

    @staticmethod
    def _closest_to(component: set, corner: Cell, exclude: Optional[Cell] = None) -> Cell:
        cx, cy = corner
        candidates = (c for c in component if c != exclude)
        # Manhattan distance to the corner; (y, x) breaks ties deterministically.
        return min(candidates, key=lambda c: (abs(c[0] - cx) + abs(c[1] - cy), c[1], c[0]))

    # ------------------------------------------------------------- display
    def to_ascii(self, start: Optional[Cell] = None, goal: Optional[Cell] = None,
                 path: Optional[Sequence[Cell]] = None) -> str:
        """ASCII art of the map with y increasing upwards (matches RViz)."""
        chars = [['#' if self.grid[y][x] else '.' for x in range(self.width)]
                 for y in range(self.height)]
        for cell in path or ():
            x, y = cell
            chars[y][x] = 'o'
        if start:
            chars[start[1]][start[0]] = 'S'
        if goal:
            chars[goal[1]][goal[0]] = 'G'
        return '\n'.join(''.join(row) for row in reversed(chars))
