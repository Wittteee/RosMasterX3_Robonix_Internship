set -e
TASK_DIR=/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm
OLD=/home/jetson/yahboomcar_ws/build/test_results/x3_target_navigation/rosunit-navigation_behavior.xml
if [ -f "$OLD" ]; then
  stat "$OLD"
  mv "$OLD" "$TASK_DIR/preexisting-rosunit-navigation_behavior.xml"
fi
