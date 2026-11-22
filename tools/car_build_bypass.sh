set -e
set -o pipefail
export LC_ALL=C.UTF-8
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
TASK=/home/jetson/rosmaster_x3_runtime/obstacle2_20261121
cd "$TASK/build_ws"
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python2 -j2 -l2 2>&1 | tee "$TASK/build.log"
