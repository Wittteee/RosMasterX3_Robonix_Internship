set -e
set -o pipefail
export LC_ALL=C.UTF-8
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
export X3_TRIAL_DIR=/home/jetson/rosmaster_x3_runtime/obstacle2_20261121/hardware_5cm_01
source /opt/ros/melodic/setup.bash
source /home/jetson/rosmaster_x3_runtime/obstacle2_20261121/build_ws/devel/setup.bash
python -u "$X3_TRIAL_DIR/live_trial.py" 2>&1 | tee "$X3_TRIAL_DIR/trial.log"
