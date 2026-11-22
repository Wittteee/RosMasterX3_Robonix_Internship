set -e
export LC_ALL=C.UTF-8
TASK_DIR=/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm
mkdir -p "$TASK_DIR"
if [ ! -e "$TASK_DIR/source_before_deploy.tar.gz" ]; then
  tar -czf "$TASK_DIR/source_before_deploy.tar.gz" -C /home/jetson/yahboomcar_ws/src x3_target_navigation
fi
sha256sum "$TASK_DIR/source_before_deploy.tar.gz"
printf 'Backup ready\n'
