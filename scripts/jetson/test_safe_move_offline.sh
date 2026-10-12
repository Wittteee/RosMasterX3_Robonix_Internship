#!/usr/bin/env bash
set -uo pipefail
export LC_ALL=C.UTF-8
task_python="$HOME/miniforge3/envs/robonix/bin/python"
cd "$HOME/refs/robonix-rosmaster-x3-safe-move/jetson"
task_failed=0
for task_suite in stage1_unit_test.py stage2_unit_test.py x3_bridge_navigation_unit_test.py tests/move_base_cli_unit_test.py; do
    echo "TEST: $task_suite"
    "$task_python" "$task_suite" || task_failed=1
done
exit "$task_failed"
