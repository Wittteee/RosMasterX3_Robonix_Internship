#!/usr/bin/env bash
# Offline check used by the Robonix package catalog lifecycle.
# Does not build the Jetson catkin workspace and does not touch the robot.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if command -v python >/dev/null 2>&1; then
  PYTHON=python
elif command -v python3 >/dev/null 2>&1; then
  PYTHON=python3
else
  echo "python is required to run scripts/self_check.py" >&2
  exit 1
fi

"$PYTHON" -X utf8 scripts/self_check.py
