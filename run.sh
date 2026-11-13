cd ~/uav/learn
conda activate uav_px4
source ~/uav/learn/ws/install/setup.bash


./qgroundcontrol/build/QGroundControl-x86_64.AppImage # 开启QGC

cd PX4-Autopilot PX4_GZ_WORLD=default2 make px4_sitl gz_x500_depth
MicroXRCEAgent udp4 -p 8888

RGB_TOPIC='/world/default2/model/x500_depth_0/link/camera_link/sensor/IMX214/image'

ros2 run ros_gz_bridge parameter_bridge \
  "${RGB_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image" \
  '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' \
  '/world/default2/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  --ros-args \
  -r "${RGB_TOPIC}:=/camera/image_raw" \
  -r /depth_camera:=/camera/depth/image_raw \
  -r /depth_camera/points:=/camera/depth/points \
  -r /world/default2/clock:=/clock
