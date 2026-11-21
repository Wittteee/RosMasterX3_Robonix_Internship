# x3_target_navigation

ROS Melodic C++ package for a static, unique RGB-D target:

```text
Astra registered RGB-D → YOLOX → depth deprojection → timestamped TF to map
→ multi-frame fusion → bottle preview → explicit ROS service confirmation
→ straight approach → 5 cm front-gap obstacle stop (latched)
```

Function 1 publishes `/target_navigation/approach_velocity` to the corrected
base driver, which enforces the motion permission heartbeat and speed limits.
Ordinary `move_base` navigation remains available with
`obstacle_stop_approach:=false`; it uses the original costmaps and footprint.

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
demo map and AMCL. Function 1 supplies straight approach commands; ordinary
mode additionally starts `move_base`. RViz stays off for SSH use.

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

In ordinary navigation (`obstacle_stop_approach:=false`), `move_base` success
is not enough: the node waits for a new fused target and
checks a fresh nearest-surface depth and the measured clearance from the robot
footprint. RGB-only recognition cannot finish the task. `parking_clearance_m`
is the gap from the physical front footprint to the nearest target/obstacle
surface; it is not the target-centre distance. The required base-footprint
standoff also includes the measured footprint front, depth error and braking
distance. The default safety interval is 0.10--0.20 m.

In that ordinary mode, if the forward laser sees an obstacle between the robot and the target, the
candidate is conservatively clipped before the obstacle. A reachable clipped
goal finishes as `FAILED: BLOCKED_NO_SAFE_PATH` after action termination and
zero-velocity confirmation; it is never reported as `SUCCEEDED`. If a cancel
or timeout cannot be followed by a terminal action state and zero velocity in
the configured window, the task is locked as `FAILED: STOP_UNCONFIRMED` and a
new confirmation is rejected.

### Obstacle stop during bottle approach (function 1)

The real-car entry defaults to `obstacle_stop_approach:=true`. After a stable
bottle preview and one `/target_navigation/confirm`, the manager follows the
confirmed straight line at up to 0.04 m/s, with small heading corrections and
no lateral bypass. It slows to 0.015 m/s for the last 15 cm before the obstacle
stop threshold. The nominal bottle point is frozen for this attempt, so an
occluding obstacle cannot redirect the task to a new detection.

Every `/scan` and the 20 Hz task timer check the route corridor. Laser points
are transformed into `base_footprint` using their acquisition timestamp. For a
point in the route corridor, the measured front gap is
`point.x - footprint_front_m` (default front offset 0.117 m). An obstacle farther
than `obstacle_stop_distance_m: 0.05` does not trigger a stop. The first valid
measurement **at or below 0.05 m** commands zero linear and angular velocity
and inhibits the base driver. No braking distance, depth error or costmap
inflation is added to this trigger. This specifies when to send the command;
the physical resting gap may be smaller because of sampling and braking slip.
Returns beyond the frozen bottle or inside its configured depth-error band
are not treated as intervening obstacles. Obstacles outside the route corridor
are ignored; any intruding return is handled conservatively, without requiring
an estimate of whether an alternative route exists.

To prevent `move_base` goal clipping, inflation, no-path failures or replanning
from stopping this experiment early, function 1 uses its own approach
controller. The launch retains AMCL but does not start `move_base`, and remaps
the base input exclusively to `/target_navigation/approach_velocity`. It does
not weaken the ordinary navigation costmaps or shrink the footprint. With
`obstacle_stop_approach:=false`, ordinary move_base navigation remains available.

On the 5 cm trigger, state becomes `STOPPING: BLOCKED_NO_SAFE_PATH`, then
`FAILED: BLOCKED_NO_SAFE_PATH` once fresh odometry confirms a stop. It stays
latched after the obstacle is removed. Restart the launch and explicitly
confirm a new preview to try again. Missing/stale scans, missing TF, lost
localization/odometry, excessive path deviation, maximum travel and timeout
still cause immediate stops; they do not wait for a 5 cm measurement. A
lost visual bottle may be occluded: the frozen target remains usable while a
fresh laser blocker is tracked. Unconfirmed braking remains
`FAILED: STOP_UNCONFIRMED`.

Use the real-car entry with `start_bringup:=true`: it enables the base driver's
required 20 Hz motion permission heartbeat. Missing permission for 0.4 s forces
zero motion even if commands continue. A reused base must be the corrected
driver, with `require_navigation_permission:=true` and
`velocity_topic:=/target_navigation/approach_velocity` in `safe_bringup.launch`.

Offline regression covers the 5.1/5.0/4.9 cm boundary, an obstacle appearing
during motion, front-plane rather than radial distance, the production C++
controller and the actual base callback/watchdog with simulated messages.
After deployment, record the gap at the first zero command, the resting gap,
`/driver_node/applied_cmd_vel`, `/odom` and `STOPPING`/`FAILED` transitions.
Removing the obstacle must not restart motion. Laser blind zones and obstacles
outside its scan plane require on-car validation.

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

## Partial obstruction bypass (function 2)

Function 1 remains the default. `bottle_bypass.launch` opts into function 2;
it defaults to preview-only and still consumes exactly one explicit confirm:

```bash
roslaunch x3_target_navigation bottle_bypass.launch execute:=true
rosservice call /target_navigation/confirm
```

The scan is checked against the full body width plus a 5 cm clearance, rather
than just a centre ray. When an obstruction intrudes into the route within
5 cm of the front, the manager first sends zero velocity and inhibits the
base. It requires three fresh stopped odometry samples before making a plan.
The car approaches the confirmed bottle goal, not an obstacle waypoint. The
trigger is the first valid measured front gap at or below 0.05 m; a gap
above 0.05 m does not initiate a bypass. Sensor/localization failures can still
stop earlier. Sampling and braking can make the physical resting gap smaller.
Once braking has started, a 3 cm release band prevents tiny brake recoil or
measurement noise from resuming forward motion. A changed bypass side is
rejected instead of silently switching direction while stopped. The first
trigger remains 5 cm. Nearby points on the obstacle face are used to classify
its side even when only its leading edge has crossed the trigger boundary.
An intrusion on the right selects a left bypass; an intrusion on the left
selects a right bypass. A wall spanning the body width, or an ambiguous central
obstruction, remains stopped instead of choosing an arbitrary side.

The candidate has three segments: shift to the clear side, pass the obstacle,
then approach a new parking pose facing the **same frozen bottle**. The last
pose is computed from the clear side; it need not coincide with the original
straight-line parking point. Every segment must have a valid move_base plan.
Plans on the blocked side, initial segments that advance instead of shifting,
pass segments that cut back toward the obstacle, and segments whose swept
footprint touches any observed obstacle are rejected. The first lateral step
keeps its starting heading and uses the exact rectangular swept footprint;
later segments use the conservative circumscribed footprint, including turns.
A plan is checked again after each segment and stopped-odometry confirmation.

Function 2 loads its own low-speed holonomic DWA profile: maximum total
translation 0.04 m/s and lateral velocity +/-0.03 m/s, with lateral sampling
enabled. Both costmaps retain the real rectangular footprint with additional
padding; the global planner forbids unknown cells. The existing function 1
profile is unchanged. move_base publishes only to
`/target_navigation/bypass_nav_velocity`, which is **not** the base input.
The manager relays commands to the existing interlocked approach topic only
while a bypass action is active. Non-finite commands, missing sensors,
unexpected side/direction, and a predicted footprint collision stop and latch
the task. The same heartbeat and base command watchdog remain active.

If no safe route exists on the required side, another obstacle appears during
the bypass, the action fails, or stopping cannot be confirmed, the car remains
stopped with a specific failure reason. It does not switch to the blocked side
or bypass the safety checks. A completed bypass still requires fresh RGB-D
arrival verification before success.

Offline tests cover left/right selection, a clear centre with a blocked body
edge, full blockage, side clearance and swept-corner collision. Isolated ROS
tests exercise the actual manager with simulated odometry, scans and move_base
actions for both directions, no-path/blocked-side failures, a newly appearing
obstacle, delayed action-server startup and rejected wrong-side commands.
`bypass_real_planner.test` additionally runs the real GlobalPlanner, costmaps
and DWA with a simulated holonomic base on an isolated ROS master:

```bash
ROBOT_TYPE=X3 rostest x3_target_navigation bypass_real_planner.test scenario:=real_right
ROBOT_TYPE=X3 rostest x3_target_navigation bypass_real_planner.test scenario:=real_left
```

The bypass profile uses centred global-path coordinates, a 2 cm local grid
and a three-second trajectory horizon to distinguish low-speed lateral
progress. It also uses a 2 cm oscillation-progress distance appropriate to
short manoeuvres. These tests do not validate physical lateral calibration.
Attended left- and right-side experiments are required
before treating function 2 as physically validated. As with function 1, the
obstacle must intersect the lidar plane (about 19.2 cm above base_footprint on
the inspected car), and the bottle must first be visually recognized.
