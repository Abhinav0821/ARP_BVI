#!/usr/bin/env bash
# One-command demo: activate the environment, build if needed, launch node + RViz2.
#   ./scripts/run_demo.sh                 # defaults (seed 42)
#   ./scripts/run_demo.sh seed:=7         # any launch argument is forwarded
# RoboStack activation hooks reference unset vars, so no "set -u" here.
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../ros2_ws" && pwd)"
source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate ros_env

cd "$WS"
[ -d install ] || colcon build --symlink-install
# shellcheck disable=SC1091
source install/setup.bash
exec ros2 launch bvi_nav bvi_demo.launch.py "$@"
