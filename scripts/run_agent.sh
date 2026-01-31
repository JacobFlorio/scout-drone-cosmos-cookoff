#!/usr/bin/env bash
set -euo pipefail

PORT="${1:-8888}"

echo "[run_agent] Starting Micro XRCE-DDS Agent on UDP port ${PORT}"
echo "  (Ctrl+C to stop)"
MicroXRCEAgent udp4 -p "${PORT}"
