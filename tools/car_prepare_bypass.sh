set -e
TASK=/home/jetson/rosmaster_x3_runtime/obstacle2_20261121
mkdir -p "$TASK/build_ws/src/x3_target_navigation"
if [ ! -f "$TASK/function1_verified_source.tar.gz" ]; then
  tar -czf "$TASK/function1_verified_source.tar.gz" -C /home/jetson/yahboomcar_ws/src x3_target_navigation
  sha256sum /home/jetson/yahboomcar_ws/devel/lib/x3_target_navigation/target_task_manager > "$TASK/function1_binary.sha256"
fi
printf 'Isolated feature2 workspace ready\n'
printf 'Function1 original binary checksum\n'
cat "$TASK/function1_binary.sha256"
