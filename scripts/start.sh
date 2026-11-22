#!/usr/bin/env bash
# Catalog start hook. Preview only: no rosbridge, no goal, no /cmd_vel.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if command -v python >/dev/null 2>&1; then
  PYTHON=python
elif command -v python3 >/dev/null 2>&1; then
  PYTHON=python3
else
  echo "python is required to run the dry-run preview" >&2
  exit 1
fi

echo "rgbd_target_navigation: dry-run PREPARE only; the robot is not commanded."
exec "$PYTHON" -X utf8 scripts/run_mission.py --dry-run --distance 0.30
