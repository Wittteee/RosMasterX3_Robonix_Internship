set -e
export LC_ALL=C.UTF-8
printf 'Workspace links and package paths\n'
readlink -f /home/jetson/yahboomcar_ws
readlink -f /home/jetson/yahboomcar_ws/src/x3_target_navigation
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
rospack find x3_target_navigation
rospack find astra_camera || true
rospack find yahboomcar_nav
printf 'Build cache\n'
grep -E 'OpenCV_DIR|CATKIN_WHITELIST_PACKAGES|PYTHON_EXECUTABLE|CMAKE_BUILD_TYPE' /home/jetson/yahboomcar_ws/build/CMakeCache.txt || true
printf 'Map and calibration\n'
ls -l /home/jetson/yahboomcar_ws/src/yahboomcar_nav/maps/final_site_20261107.*
ls -l /home/jetson/.ros/camera_info/camera.yaml || true
printf 'Current last known pose files\n'
find /home/jetson/rosmaster_x3_runtime /home/jetson/rosmaster_x3_deploy -maxdepth 4 -type f \( -name '*pose*' -o -name '*amcl*' -o -name '*metrics*' \) | head -35
printf 'Git/source state\n'
ls -l /home/jetson/yahboomcar_ws/src/x3_target_navigation/src/target_task_manager.cpp
printf 'Overlay setup\n'
cat /home/jetson/yahboomcar_ws/devel/.catkin
cat /home/jetson/software/library_ws/devel/.catkin
ls -ld /home/jetson/software/world_canvas/devel
