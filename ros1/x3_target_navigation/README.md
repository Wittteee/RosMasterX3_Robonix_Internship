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

Run preview-only mode:

```bash
roslaunch x3_target_navigation target_navigation.launch \
  model_path:=/absolute/path/object_detection_yolox_2022nov_opencv41.onnx \
  start_bringup:=true \
  start_astra:=true \
  start_navigation:=true \
  map:=final_site_20261107 \
  execute:=false
```

On the validated vehicle this single launch starts the vendor chassis/TF stack,
depth-to-scan, Astra depth, `usb_cam` RGB, the November 7 demo map and the
existing ROS1 navigation stack. It keeps RViz disabled for headless SSH use.

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
checks target drift and final standoff distance. Failure to re-observe is a
task failure. This first version intentionally does not follow moving targets.
