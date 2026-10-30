#!/usr/bin/env bash
set -eo pipefail
root="$HOME/rosmaster_x3_deploy"
bash "$root/scripts/finish_demo_check.sh"
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
python - <<'PY'
import rospy
from std_msgs.msg import Bool
rospy.init_node('identify_chassis_only')
pub=rospy.Publisher('/Buzzer',Bool,queue_size=1)
deadline=rospy.get_time()+4
while pub.get_num_connections()==0 and rospy.get_time()<deadline: rospy.sleep(.1)
pub.publish(Bool(data=True))
rospy.sleep(.5)
pub.publish(Bool(data=False))
rospy.sleep(.5)
print('Controller identification beep sent; no motion command.')
PY
