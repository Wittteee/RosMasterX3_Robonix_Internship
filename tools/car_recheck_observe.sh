set -e
export LC_ALL=C.UTF-8
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
export X3_TRIAL_DIR=/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_raised_trial
nohup python -u "$X3_TRIAL_DIR/car_observe.py" > "$X3_TRIAL_DIR/observer.log" 2>&1 < /dev/null &
echo "$!" > "$X3_TRIAL_DIR/observer.pid"
echo "Observer started $!"
