# x3_target_navigation

ROS Melodic C++ package for a static, unique RGB-D target:

```text
Astra registered RGB-D → YOLOX → depth deprojection → timestamped TF to map
→ multi-frame fusion → standoff parking pose → make_plan
→ explicit ROS service confirmation → move_base → visual arrival verification
```

The package never publishes `cmd_vel`. The vendor base driver, AMCL, costmaps and
`move_base` retain motion control.

## Build on Jetson

Copy this directory to `~/yahboomcar_ws/src/x3_target_navigation`, then:

```bash
source /opt/ros/melodic/setup.bash
cd ~/yahboomcar_ws
catkin_make
source devel/setup.bash
```

Jetson's OpenCV 4.1 needs the compatibility copy produced from the verified
OpenCV Zoo model (the original remains unchanged):

```bash
python scripts/convert_yolox_opencv41.py \
  models/object_detection_yolox_2022nov.onnx \
  models/object_detection_yolox_2022nov_opencv41.onnx
```

The package applies the removed YOLOX Focus operation before inference
(`external_focus: true`). The target label must be one of the model's COCO
classes.

## Start safely

First start the base and navigation stacks for the current map. Do not start a
second camera or chassis driver. `start_astra:=true` starts Astra depth through
OpenNI and RGB through `usb_cam`; it intentionally does not use the vendor
libuvc RGB node, which conflicts with Jetson's kernel UVC driver.

The Astra Pro profile is limited to one static target. OpenCV 4.1 CPU inference
on the current Jetson takes about 2.3--3.2 seconds per frame, so the configured
multi-frame window is not intended for dynamic following.

Real-car entry:

```bash
source /opt/ros/melodic/setup.bash
source ~/software/library_ws/devel/setup.bash
source ~/yahboomcar_ws/devel/setup.bash
export ROBOT_TYPE=X3
roslaunch x3_target_navigation target_navigation.launch \
  start_bringup:=true \
  start_astra:=true \
  start_navigation:=true \
  map:=final_site_20261107 \
  target_label:=bottle \
  execute:=true
```

`target_label` is any COCO class the model knows (`bottle` by default, or `cup`).
`execute` stays false unless this argument is true. Devices are `/dev/x3_base`,
`/dev/x3_lidar` and `/dev/x3_camera_rgb`. Close MakerControl first; a second
chassis driver aborts the launch. `/target_navigation/confirm` accepts one goal
only after chassis version, voltage, lidar, RGB, depth, TF and AMCL all pass.

This launch starts the corrected chassis driver on `/dev/x3_base`, the RPLIDAR
on `/dev/x3_lidar`, Astra depth, RGB on `/dev/x3_camera_rgb`, the November 7
demo map and the existing ROS1 navigation stack. RViz stays off for SSH use.

Inspect:

```bash
rostopic echo /target_navigation/state
rostopic echo /target_navigation/preview_goal
```

Only after camera calibration, TF, localization, depth and planning have been
validated in the current environment, restart with `execute:=true`. A fresh
preview is consumed exactly once by:

```bash
rosservice call /target_navigation/confirm
```

Cancel an active task with:

```bash
rosservice call /target_navigation/cancel
```

`move_base` success is not enough: the node waits for a new fused target and
checks a fresh nearest-surface depth and the measured clearance from the robot
footprint. RGB-only recognition cannot finish the task. `parking_clearance_m`
is the gap from the physical front footprint to the nearest target/obstacle
surface; it is not the target-centre distance. The required base-footprint
standoff also includes the measured footprint front, depth error and braking
distance. The default safety interval is 0.10--0.20 m.

If the forward laser sees an obstacle between the robot and the target, the
candidate is conservatively clipped before the obstacle. A reachable clipped
goal finishes as `FAILED: BLOCKED_NO_SAFE_PATH` after action termination and
zero-velocity confirmation; it is never reported as `SUCCEEDED`. If a cancel
or timeout cannot be followed by a terminal action state and zero velocity in
the configured window, the task is locked as `FAILED: STOP_UNCONFIRMED` and a
new confirmation is rejected.

The launch keeps `execute:=false` by default. Use `target_label:=bottle` for
the current experiment. Before an attended run, measure the bottle/obstacle
surface with a ruler, initialize AMCL, and verify RGB-D/TF, laser and
`make_plan`. The first physical acceptance must record three terminal cases:
measured clearance, braking slip and heading error. The following scripts are
read-only or attended test helpers:

```bash
python scripts/ros1/capture_rgbd_calibration.py --expected-distance 0.8
python scripts/ros1/record_navigation_metrics.py --output trial.json
python scripts/ros1/run_target_trials.py --trials 3 --execute
python scripts/ros1/test_target_faults.py --cancel-after 2 --execute
```

Offline coverage is limited to geometry, depth, fusion and preview safety;
real clearance, braking slip, side bypass and a sealed corridor still require
separate attended experiments. The target is static and unique; multiple
same-class boxes are rejected.
