# ARP Assignment 1 — Backward Value Iteration in ROS 2

Backward Value Iteration (BVI) on a 30 × 30 grid world with 20 % randomly placed
obstacles, visualised end-to-end in RViz2: the map, the optimal cost-to-go over
every reachable state, the optimal start→goal path, and an animated robot that
follows that path.

| Part | Marks | Deliverable | Where it is |
|------|-------|-------------|-------------|
| 1 | 8 | ROS 2 installed; map built, robot at the start position | `/map` + `/start_goal` + `/robot_marker` |
| 2 | 4 | Cost-to-go values visualised over the map | `/cost_heatmap` (colour) + `/cost_labels` (numbers) |
| 3 | 4 | Optimal start→goal path | `/optimal_path` + `/optimal_path_line` |
| 4 | 4 | RViz2 animation of the robot following the path | `/robot_marker` + `/robot_trail`, 4 Hz timer |

**Environment used:** ROS 2 Jazzy Jalisco, installed with RoboStack (conda) on
Apple-Silicon macOS. Python 3.12, `rclpy`.

---

## 1. Layout

```
ARP_A1/
├── README.md
├── scripts/
│   ├── setup_ros2_macos.sh        # reproducible ROS 2 install (idempotent)
│   └── run_demo.sh                # one-command demo
├── docs/
│   ├── verification_output.txt    # saved ASCII map, V table, policy, checklist
│   └── unit_test_output.txt       # saved pytest run
└── ros2_ws/
    └── src/bvi_nav/
        ├── package.xml, setup.py, setup.cfg, resource/bvi_nav
        ├── bvi_nav/
        │   ├── grid_map.py        # map generation + connectivity   (no ROS imports)
        │   ├── bvi.py             # value iteration + path + checks (no ROS imports)
        │   ├── bvi_node.py        # ROS node: publishers, timers, animation
        │   └── demo_cli.py        # terminal demo / fallback if RViz misbehaves
        ├── launch/bvi_demo.launch.py
        ├── rviz/bvi.rviz          # pre-configured: all six displays, top-down view
        └── test/test_bvi.py       # 20 unit tests, incl. a BFS cross-check
```

`grid_map.py` and `bvi.py` contain **no ROS imports**, so the algorithm is
testable with a plain interpreter and reusable by the multi-robot follow-up
assignment. The node is parameterised on `frame_id` / `robot_frame` and uses
relative topic names, so several instances can be run under different namespaces.

---

## 2. Algorithm

**Map** (`grid_map.py`) — `grid[y][x]`, `0` free / `1` obstacle. Exactly
`round(0.20 × 900) = 180` obstacles are placed with `random.Random(seed).sample`,
so the same seed always produces the same map. Start and goal are chosen as the
free cells nearest to opposite corners **within the largest connected component**,
which guarantees `V(start)` is finite and makes the path long and interesting.
With the default seed 42 this gives start `(0, 0)` and goal `(29, 29)`.

**BVI** (`bvi.py`) — the shortest-path form of Bellman's equation

```
V*(goal) = 0
V*(s)    = min over a in {N,S,E,W} of [ 1 + V*(f(s,a)) ]      s ≠ goal
```

is solved backwards from the goal by repeated in-place sweeps over the free state
space until no value changes (`delta == 0`). Obstacles keep the sentinel `inf`
and are never updated; free cells still `inf` after convergence are unreachable
and are deliberately left out of the heatmap and the labels. With seed 42 it
converges in **59 sweeps / 719 updates**, and all 720 free cells are reachable.

**Path extraction** — greedy steepest descent of `V` from the start, ties broken
in the fixed `N, S, E, W` order, asserting `len(path) - 1 == V(start)`
(58 moves for seed 42).

---

## 3. Install ROS 2 (already done on this machine)

```bash
./scripts/setup_ros2_macos.sh
```

The script installs a native arm64 Miniforge into `~/miniforge3`, creates the
`ros_env` environment, pins the `robostack-jazzy` + `conda-forge` channels and
installs `ros-jazzy-ros-base`, `ros-jazzy-rviz2`, the demo nodes,
`colcon-common-extensions` and `numpy`.

Two version pins matter and the script applies them:

* `setuptools < 80` — setuptools 80 removed `setup.py develop --editable`, which
  `colcon build --symlink-install` still uses. Without the pin the build fails
  with `error: option --editable not recognized`.
* `pytest < 8` — ROS 2 Jazzy's `launch_testing` pytest plugin uses an old hook
  signature and crashes newer pytest during collection.

Verify the install (section 2.4 of the guide):

```bash
source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros_env
python -c "import rclpy, nav_msgs, visualization_msgs, numpy; print('ok')"
ros2 run demo_nodes_cpp talker      # terminal 1
ros2 run demo_nodes_py listener     # terminal 2
rviz2                               # a window must open
```

---

## 4. Build and run

```bash
source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros_env
cd ~/Desktop/Github/ARP_A1/ros2_ws
colcon build --symlink-install
source install/setup.zsh      # use install/setup.bash if your shell is bash
ros2 launch bvi_nav bvi_demo.launch.py
```

or simply:

```bash
./scripts/run_demo.sh
```

> **Shell note:** the default macOS shell is **zsh**. Sourcing
> `install/setup.bash` from zsh silently fails to put the package on
> `AMENT_PREFIX_PATH` (`ros2 run` then reports "Package 'bvi_nav' not found").
> Use `install/setup.zsh` in zsh.

> **Console noise:** Fast DDS prints
> `Problem to set affinity of thread ... Protocol family not supported` on macOS.
> It is harmless and does not affect the demo.

### Launch arguments

```bash
ros2 launch bvi_nav bvi_demo.launch.py seed:=7
ros2 launch bvi_nav bvi_demo.launch.py start:="[1, 1]" goal:="[28, 28]"
ros2 launch bvi_nav bvi_demo.launch.py step_rate:=8.0 loop:=false
ros2 launch bvi_nav bvi_demo.launch.py show_labels:=false     # heatmap only
ros2 launch bvi_nav bvi_demo.launch.py label_stride:=2        # thin out labels
ros2 launch bvi_nav bvi_demo.launch.py rviz:=false            # node only
```

| Parameter | Default | Meaning |
|---|---|---|
| `seed` | `42` | RNG seed for obstacle placement |
| `grid_size` | `30` | Grid width = height |
| `obstacle_ratio` | `0.20` | Fraction of cells that are obstacles |
| `start`, `goal` | `[-1, -1]` | `[x, y]`; `[-1, -1]` chooses automatically |
| `step_rate` | `4.0` | Animation steps per second |
| `loop` | `true` | Restart the animation after reaching the goal |
| `loop_pause` | `1.5` | Seconds held at the goal before restarting |
| `show_labels` | `true` | Numeric `V` labels |
| `label_stride` | `1` | Draw every n-th label |
| `publish_tf` | `true` | Broadcast `map → base_link` |
| `frame_id`, `robot_frame` | `map`, `base_link` | Frame names (for multi-robot reuse) |

---

## 5. Topics

| Topic | Type | Contents |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | 30×30, `resolution = 1.0`, origin `(0,0,0)`, row-major `index = y*width + x`, `100` occupied / `0` free |
| `/cost_heatmap` | `visualization_msgs/Marker` | `CUBE_LIST`, 720 cubes, per-point colour green (low `V`) → yellow → red (high `V`) |
| `/cost_labels` | `visualization_msgs/MarkerArray` | 720 `TEXT_VIEW_FACING` markers with integer `V`, unique id per cell |
| `/start_goal` | `visualization_msgs/MarkerArray` | Green start cylinder + `START`, red goal cylinder + `GOAL` |
| `/optimal_path` | `nav_msgs/Path` | 59 `PoseStamped` at cell centres |
| `/optimal_path_line` | `visualization_msgs/Marker` | Thick blue `LINE_STRIP` of the same path |
| `/robot_marker` | `visualization_msgs/Marker` | Magenta `SPHERE`, moved one cell per tick |
| `/robot_trail` | `visualization_msgs/Marker` | Yellow `LINE_STRIP` growing behind the robot |
| `/tf` | `tf2_msgs/TFMessage` | `map → base_link` at the robot's cell |

All static topics use **reliable + transient_local** QoS (depth 1) and are
republished at 1 Hz, so RViz2 receives them whenever it connects. Every cell
`(x, y)` is drawn at its centre `(x + 0.5, y + 0.5)`, so markers, path and map
align exactly.

---

## 6. Verifying the result

### a. Algorithm only, no ROS

```bash
cd ros2_ws/src/bvi_nav
python -m bvi_nav.demo_cli --seed 42 --show-values --show-policy
```

Prints the ASCII map with the path drawn on it, the full `V` table, the optimal
policy field, and the assignment's checklist. A saved copy is in
[docs/verification_output.txt](docs/verification_output.txt).

### b. Unit tests

```bash
cd ros2_ws/src/bvi_nav && python -m pytest test/test_bvi.py -v
```

20 tests, all passing. They check the obstacle count, reproducibility per seed,
`V(goal) == 0`, obstacles staying `inf`, `len(path) - 1 == V(start)`, that the
path is a legal N/S/E/W walk over free cells, that unreachable cells stay `inf`,
that an open grid yields exactly the Manhattan distance, and — the strongest
check — that **the whole `V` table equals a reference BFS from the goal**, which
is the ground truth for unit action costs. Saved output:
[docs/unit_test_output.txt](docs/unit_test_output.txt).

### c. Self-check printed by the node

The node runs the same checklist at start-up and logs a `[PASS]`/`[FAIL]` line
per item, so the TA can see correctness in the terminal next to the RViz window:

```
[INFO] [bvi_node]: Grid 30x30, seed=42, obstacles=180 (20.0%)
[INFO] [bvi_node]: Start=(0, 0)  Goal=(29, 29)
[INFO] [bvi_node]: BVI converged in 59 sweeps (719 value updates)
[INFO] [bvi_node]: Reachable states: 720 of 720 free cells
[INFO] [bvi_node]: V(start) = 58, max finite V = 58
[INFO] [bvi_node]: Optimal path length: 58 moves
[INFO] [bvi_node]: [PASS] obstacle count is exactly round(ratio*cells) - 180 obstacles (expected 180)
... 9 more [PASS] lines ...
```

### d. Live topic inspection

```bash
ros2 topic list
ros2 topic echo /map --field info --once            # 30x30, resolution 1.0
ros2 topic echo /optimal_path --field poses --once | head
ros2 topic hz /robot_marker                         # ~4 Hz while animating
ros2 param list /bvi_node
```

---

## 7. What to look for in RViz2

The saved config opens a top-down orthographic view centred on `(15, 15)` with
`Fixed Frame = map` and all displays enabled:

1. **Map** — black obstacles on a light background, 180 black cells.
2. **CostHeatmap** — green near the goal, grading through yellow to red at the
   far corner; each cube sits exactly on one free cell.
3. **CostLabels** — the integer cost-to-go on every reachable cell (zoom in to
   read them; toggle off or use `label_stride:=2` if cluttered).
4. **StartGoal** — green `START` cylinder at `(0, 0)`, red `GOAL` at `(29, 29)`.
5. **OptimalPath / OptimalPathLine** — a thick blue line of 58 moves; values
   along it decrease by exactly 1 per step.
6. **RobotMarker / RobotTrail** — a magenta sphere stepping cell by cell at 4 Hz
   with a yellow trail, restarting after a short pause at the goal.

If a display shows nothing: confirm `Fixed Frame = map` and that the display's
**Durability** is `Transient Local` (the Map display in particular).
