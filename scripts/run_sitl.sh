#!/usr/bin/env bash
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/autonomous-security-drone/sim/px4/PX4-Autopilot}"
MODEL="${1:-gz_x500}"

cd "${PX4_DIR}"
echo "[run_sitl] PX4 SITL: make px4_sitl ${MODEL}"
echo "  (Ctrl+C to stop)"
make px4_sitl "${MODEL}"
