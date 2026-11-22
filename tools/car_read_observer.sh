set -e
TASK_DIR=/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm
printf 'Latest independent observation\n'
cat "$TASK_DIR/latest_status.json" || true
printf '\nObserver log\n'
tail -n 8 "$TASK_DIR/observer.log"
printf 'Runtime messages\n'
tail -n 30 "$TASK_DIR/runtime.log"
