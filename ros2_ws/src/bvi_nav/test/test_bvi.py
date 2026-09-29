"""Unit tests for the ROS-free planner modules."""

from bvi_nav.bvi import backward_value_iteration, extract_path, policy_field, verify
from bvi_nav.grid_map import GridMap
import numpy as np
import pytest


@pytest.fixture(scope='module')
def solved():
    grid = GridMap(width=30, height=30, obstacle_ratio=0.20, seed=42)
    start, goal = grid.pick_start_goal()
    result = backward_value_iteration(grid, goal)
    path = extract_path(grid, result, start)
    return grid, start, goal, result, path


def test_map_has_exactly_180_obstacles(solved):
    grid = solved[0]
    assert grid.obstacle_count() == 180
    assert grid.grid.shape == (30, 30)


def test_map_is_reproducible_for_a_seed():
    a = GridMap(seed=42).grid
    b = GridMap(seed=42).grid
    c = GridMap(seed=7).grid
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)


def test_start_and_goal_are_free_distinct_and_connected(solved):
    grid, start, goal = solved[0], solved[1], solved[2]
    assert grid.is_free(start) and grid.is_free(goal)
    assert start != goal
    assert grid.are_connected(start, goal)


def test_goal_value_is_zero_and_start_is_finite(solved):
    _, start, goal, result, _ = solved
    assert result.v(goal) == 0.0
    assert np.isfinite(result.v(start))


def test_obstacles_keep_infinite_value(solved):
    grid, _, _, result, _ = solved
    obstacles = grid.grid == 1
    assert np.all(np.isinf(result.value[obstacles]))


def test_path_length_equals_value_at_start(solved):
    _, start, _, result, path = solved
    assert len(path) - 1 == result.v(start)


def test_path_is_a_legal_nsew_walk_over_free_cells(solved):
    grid, start, goal, _, path = solved
    assert path[0] == start and path[-1] == goal
    assert all(grid.is_free(c) for c in path)
    assert all(abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1 for a, b in zip(path, path[1:]))


def test_bvi_matches_a_reference_bfs(solved):
    """Uniform costs make BVI equivalent to a BFS from the goal."""
    grid, _, goal, result, _ = solved
    dist = {goal: 0}
    frontier = [goal]
    while frontier:
        nxt = []
        for cell in frontier:
            for n in grid.neighbors(cell):
                if n not in dist:
                    dist[n] = dist[cell] + 1
                    nxt.append(n)
        frontier = nxt
    for cell in grid.free_cells():
        expected = dist.get(cell, float('inf'))
        assert result.v(cell) == expected, f'mismatch at {cell}'


def test_every_verification_check_passes(solved):
    grid, start, _, result, path = solved
    for name, ok, detail in verify(grid, result, start, path):
        assert ok, f'{name}: {detail}'


def test_policy_is_defined_for_every_reachable_non_goal_cell(solved):
    grid, _, goal, result, _ = solved
    policy = policy_field(grid, result)
    for cell in grid.free_cells():
        if cell != goal and result.is_reachable(cell):
            assert cell in policy


@pytest.mark.parametrize('seed', [0, 1, 7, 42, 123, 2024])
def test_planner_works_across_seeds(seed):
    grid = GridMap(seed=seed)
    start, goal = grid.pick_start_goal()
    result = backward_value_iteration(grid, goal)
    path = extract_path(grid, result, start)
    assert grid.obstacle_count() == 180
    assert len(path) - 1 == result.v(start)


def test_open_grid_value_is_manhattan_distance():
    grid = GridMap(width=5, height=5, obstacle_ratio=0.0, seed=1)
    result = backward_value_iteration(grid, (0, 0))
    for x in range(5):
        for y in range(5):
            assert result.v((x, y)) == x + y


def test_unreachable_cells_stay_infinite():
    """A cell walled off from the goal must keep V = inf."""
    grid = GridMap(width=5, height=5, obstacle_ratio=0.0, seed=1)
    for y in range(5):            # wall along x = 2 isolates x in {3, 4}
        grid.grid[y][2] = 1
    result = backward_value_iteration(grid, (0, 0))
    assert np.isinf(result.v((4, 4)))
    assert result.v((1, 1)) == 2.0
    with pytest.raises(ValueError):
        extract_path(grid, result, (4, 4))


def test_explicit_start_and_goal_are_honoured():
    grid = GridMap(width=10, height=10, obstacle_ratio=0.0, seed=3)
    start, goal = grid.pick_start_goal([1, 1], [8, 8])
    assert start == (1, 1) and goal == (8, 8)


def test_obstacle_cell_rejected_as_goal():
    grid = GridMap(seed=42)
    obstacle = next((x, y) for y in range(30) for x in range(30) if grid.grid[y][x])
    with pytest.raises(ValueError):
        grid.pick_start_goal(None, list(obstacle))
