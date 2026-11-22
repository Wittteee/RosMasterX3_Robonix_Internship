set -e
export LC_ALL=C.UTF-8
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
python - <<'PY'
import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
rospy.init_node('obstacle1_camera_capture', anonymous=True)
message = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=10)
data = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
image = data[:, :message.width*3].reshape(message.height, message.width, 3)
if message.encoding == 'rgb8':
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
elif message.encoding != 'bgr8':
    raise RuntimeError('Unexpected encoding '+message.encoding)
filename = '/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm/camera_latest.png'
assert cv2.imwrite(filename, image)
print(filename, message.header.stamp.to_sec(), image.shape)
PY
