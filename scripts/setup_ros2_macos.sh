#!/usr/bin/env bash
# Reproducible ROS 2 Jazzy setup on Apple-Silicon macOS via RoboStack (conda).
# Safe to re-run: every step is skipped if it is already done.
set -euo pipefail

MINIFORGE="$HOME/miniforge3"
ENV_NAME="ros_env"

if [ ! -x "$MINIFORGE/bin/conda" ]; then
  echo ">>> Installing Miniforge (native arm64) into $MINIFORGE"
  TMP="$(mktemp -d)"
  curl -fsSL -o "$TMP/Miniforge3.sh" \
    "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-arm64.sh"
  bash "$TMP/Miniforge3.sh" -b -p "$MINIFORGE"
  rm -rf "$TMP"
else
  echo ">>> Miniforge already present at $MINIFORGE"
fi

CONDA="$MINIFORGE/bin/conda"

if ! "$CONDA" env list | grep -qE "^${ENV_NAME}\s"; then
  echo ">>> Creating conda environment '$ENV_NAME' (python 3.12)"
  "$CONDA" create -n "$ENV_NAME" python=3.12 -y
else
  echo ">>> Environment '$ENV_NAME' already exists"
fi

echo ">>> Pinning RoboStack channels for the environment"
cat > "$MINIFORGE/envs/$ENV_NAME/.condarc" <<'RC'
channels:
  - robostack-jazzy
  - conda-forge
channel_priority: strict
RC

echo ">>> Installing ROS 2 Jazzy (this is the long step)"
"$CONDA" install -n "$ENV_NAME" -y --override-channels \
  -c robostack-jazzy -c conda-forge \
  ros-jazzy-ros-base ros-jazzy-rviz2 \
  ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py \
  colcon-common-extensions numpy

# setuptools >= 80 removed "setup.py develop --editable", which colcon's
# --symlink-install still uses; pytest >= 8 breaks ROS's launch_testing plugin.
echo ">>> Pinning build/test tool versions that ROS 2 Jazzy needs"
"$CONDA" install -n "$ENV_NAME" -y --override-channels -c conda-forge \
  "setuptools<80" "pytest<8"

echo
echo ">>> Done. Verify with:"
echo "    source \"$MINIFORGE/etc/profile.d/conda.sh\" && conda activate $ENV_NAME"
echo "    python -c 'import rclpy, nav_msgs, visualization_msgs, numpy; print(\"ok\")'"
echo "    rviz2"
