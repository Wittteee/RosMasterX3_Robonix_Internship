#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C.UTF-8
task_log="$HOME/rosmaster_x3_deploy/logs/official_cli_dry_run.log"
set +e
"$HOME/rosmaster_x3_deploy/scripts/move_base" --dry-run >"$task_log" 2>&1
task_result=$?
set -e
cat "$task_log"
echo "CLI exit code: $task_result"
# With no navigation nodes, the real CLI should reject before selection.
test "$task_result" -eq 2
grep -q 'nav_not_ready' "$task_log"
