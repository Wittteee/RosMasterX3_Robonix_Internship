set -e
export LC_ALL=C.UTF-8
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
set -o pipefail
python -u /home/jetson/rosmaster_x3_runtime/obstacle1_20261121_raised_trial/car_live_trial.py 2>&1 | tee /home/jetson/rosmaster_x3_runtime/obstacle1_20261121_raised_trial/trial.log
