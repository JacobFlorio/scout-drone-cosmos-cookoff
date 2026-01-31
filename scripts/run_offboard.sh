#!/usr/bin/env bash
set -eo pipefail

WS="${WS:-$HOME/autonomous-security-drone/ros_ws}"
PKG="${1:-asd_px4}"
NODE="${2:-offboard_commander_node}"

echo "[run_offboard] Sourcing ROS + workspace..."

# ROS setup scripts can reference unset vars; avoid set -u while sourcing
set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u 2>/dev/null || true

echo "[run_offboard] Running: ros2 run ${PKG} ${NODE}"
echo "  (Ctrl+C to stop)"
exec ros2 run "${PKG}" "${NODE}" -- "${@:3}"
