set -e
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
rospack find global_planner
cat /home/jetson/yahboomcar_ws/src/yahboomcar_nav/param/common/local_costmap_params.yaml
cat /home/jetson/yahboomcar_ws/src/yahboomcar_nav/param/common/dwa_local_planner_params_X3.yaml
