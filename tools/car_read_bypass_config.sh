set -e
export LC_ALL=C.UTF-8
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
NAV=$(rospack find yahboomcar_nav)
for file in global_costmap_params.yaml local_costmap_params.yaml move_base_params.yaml costmap_common_params_X3.yaml dwa_local_planner_params_X3.yaml; do
  printf '\n%s\n' "$file"
  sed -n '1,180p' "$NAV/param/common/$file"
done
printf '\nExisting car node status\n'
export ROS_MASTER_URI=http://127.0.0.1:11311
timeout 4 rostopic echo -n 1 /target_navigation/state || true
printf '\nVendor velocity subscription\n'
grep -n -A 8 -B 3 'Subscriber.*cmd_vel\|def cmd_vel_callback' /home/jetson/yahboomcar_ws/src/yahboomcar_bringup/scripts/Mcnamu_driver.py
