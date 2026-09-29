# Submission checklist — ARP Assignment 1

## Demo script for the TAs (about 4 minutes)

Open one terminal:

```bash
source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros_env
cd ~/Desktop/Github/ARP_A1/ros2_ws
source install/setup.zsh
ros2 launch bvi_nav bvi_demo.launch.py
```

Then walk the TA through the four parts, in order:

1. **Part 1 (8 marks) — ROS 2 installed, map built, robot at the start.**
   Point at the terminal: `Grid 30x30, seed=42, obstacles=180 (20.0%)`. In RViz,
   the Map display shows the 180 black obstacle cells and the green `START`
   cylinder with the magenta robot sphere on it at `(0, 0)`.
   Mention that ROS 2 Jazzy is installed via RoboStack and `rviz2` is running
   from the same launch file.
2. **Part 2 (4 marks) — cost-to-go over the map.**
   The heatmap grades green (near the goal) → yellow → red (far), and every
   reachable cell carries its integer `V` as a text label. Zoom in to read a few
   and show that adjacent cells differ by exactly 1.
3. **Part 3 (4 marks) — optimal path.**
   The thick blue line from `(0, 0)` to `(29, 29)`, 58 moves, equal to
   `V(start) = 58`; the terminal prints the `[PASS] path length == V(start)` check.
4. **Part 4 (4 marks) — animation.**
   The magenta robot steps cell by cell at 4 Hz along that path with a yellow
   trail, reaches the goal and restarts.

If asked how BVI was implemented, show `ros2_ws/src/bvi_nav/bvi_nav/bvi.py`
(sweep loop until `delta == 0`) and note that no Nav2, SLAM or external planner
is used.

Backup if RViz misbehaves on the demo machine:

```bash
cd ~/Desktop/Github/ARP_A1/ros2_ws/src/bvi_nav
python -m bvi_nav.demo_cli --seed 42 --show-values --show-policy
```

## Screenshots to capture beforehand (backup evidence)

Take these with Cmd-Shift-4 and keep them in `docs/screenshots/`:

1. `part1_map_start.png` — map with obstacles and the robot at START
   (run with `show_labels:=false` for a clean shot).
2. `part2_heatmap.png` — the cost-to-go heatmap over the whole map.
3. `part2_labels_zoom.png` — zoomed in far enough to read the numeric `V` values.
4. `part3_path.png` — the full optimal path from START to GOAL.
5. `part4_robot_midpath.png` — the robot part way along the path with its trail.
6. `terminal_checks.png` — the terminal showing the ten `[PASS]` lines.

## What to hand in

| Item | Path |
|---|---|
| Source package | `ros2_ws/src/bvi_nav/` (all of it) |
| Report / README | `README.md` |
| Saved algorithm output | `docs/verification_output.txt` |
| Saved test output | `docs/unit_test_output.txt` |
| Screenshots | `docs/screenshots/` |
| Setup + run scripts | `scripts/` |

Zip the repository **without** the build artefacts:

```bash
cd ~/Desktop/Github
zip -r ARP_A1_<rollno>.zip ARP_A1 \
  -x "ARP_A1/ros2_ws/build/*" "ARP_A1/ros2_ws/install/*" "ARP_A1/ros2_ws/log/*" \
     "*/__pycache__/*" "*.pyc" "*/.pytest_cache/*" "*/.DS_Store"
```

The grader can rebuild with `colcon build --symlink-install` and run the single
launch command; nothing else is needed.
