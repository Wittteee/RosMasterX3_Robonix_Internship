set -e
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
python - <<'PY'
import rospy, numpy as np, tf2_ros, json
from sensor_msgs.msg import Image, CameraInfo
from tf.transformations import quaternion_matrix
rospy.init_node('obstacle1_depth_check', anonymous=True)
buffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(buffer)
info=rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo, timeout=10)
image=rospy.wait_for_message('/camera/depth_registered/image_raw', Image, timeout=10)
dtype=np.uint16 if image.encoding=='16UC1' else np.float32
depth=np.frombuffer(image.data,dtype=dtype).reshape(image.height,image.step//np.dtype(dtype).itemsize)[:,:image.width].astype(float)
if image.encoding=='16UC1': depth*=.001
result={'encoding':image.encoding,'frame':image.header.frame_id,'width':image.width,'height':image.height,'fx':info.K[0], 'fy':info.K[4], 'regions':{}}
for name,roi in [('bottle',(285,345,120,230)),('tissue',(245,420,250,330)),('floor',(245,420,350,420))]:
    x1,x2,y1,y2=roi
    values=depth[y1:y2,x1:x2]
    valid=values[np.isfinite(values)&(values>.1)&(values<3)]
    result['regions'][name]={'count':int(valid.size),'median_depth_m':float(np.median(valid)) if valid.size else None}
tf=buffer.lookup_transform('base_footprint',image.header.frame_id,image.header.stamp,rospy.Duration(2))
q=tf.transform.rotation
matrix=quaternion_matrix([q.x,q.y,q.z,q.w])
matrix[:3,3]=[tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z]
points=[]
for v in range(200,400,4):
    for u in range(180,460,4):
        z=depth[v,u]
        if not np.isfinite(z) or not .1<z<2: continue
        p=matrix.dot([(u-info.K[2])*z/info.K[0],(v-info.K[5])*z/info.K[4],z,1])
        if p[0]>0 and abs(p[1])<.2 and .015<p[2]<.3:
            points.append([float(p[0]-.117),float(p[0]),float(p[1]),float(p[2]),u,v])
points.sort()
result['nearest_depth_body_corridor']=points[:12]
path='/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm/static_depth_check.json'
with open(path,'w') as out: json.dump(result,out,indent=2)
print(json.dumps(result,indent=2))
PY
