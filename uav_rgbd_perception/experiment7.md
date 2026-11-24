# UAV RGB-D Experiment 7: YOLO + AABB + World State

## Files
- `uav_rgbd_perception/obj_detector_node.py`: YOLO COCO detector; publishes the legacy mask/BBox/center topics.
- `uav_rgbd_perception/pointcloud_processor_node.py`: your existing Open3D pipeline with a safe DBSCAN fallback.
- `uav_rgbd_perception/depth_processor_node.py`: your existing center-pixel RGB-D cross-check node.
- `uav_rgbd_perception/world_state_node.py`: transforms the camera-frame AABB to map_enu and publishes JSON.
- `launch/experiment7.launch.py`, `config/experiment7.yaml`.

## Important assumptions to verify
1. `/camera/depth/points` and `/red_box/position` use `camera_link` with Gazebo FLU axes (+X forward, +Y left, +Z up).
2. PX4 `VehicleOdometry.pose_frame == 1` is NED and quaternion order is [w,x,y,z], with child body FRD.
3. `camera_link` is parallel to `base_link`, and its fixed translation in Gazebo body FLU is [0.12, 0.03, 0.242].
4. RGB and depth optical axes are co-located/parallel. The existing pointcloud mask registration is a pinhole-only mapping; nonzero RGB-depth extrinsics require calibrated reprojection.
5. The current AABB is the percentile AABB of the visible point-cloud cluster, not necessarily the complete physical object dimensions.

The world-state node transforms all eight corners of the camera AABB and recomputes a world-axis-aligned AABB. The JSON `position_world` is the center of that world AABB.

## Install YOLO dependency
In the same Python environment used by ROS 2:
```bash
python3 -m pip install ultralytics
```
The first run of `yolo11n.pt` may download the model weights. You can instead set `model_path` to an absolute local `.pt` path.

## Build
Copy the package files into your existing package:
```bash
cp uav_rgbd_perception/obj_detector_node.py ~/uav/learn/ws/src/uav_rgbd_perception/uav_rgbd_perception/
cp uav_rgbd_perception/world_state_node.py ~/uav/learn/ws/src/uav_rgbd_perception/uav_rgbd_perception/
cp uav_rgbd_perception/pointcloud_processor_node.py ~/uav/learn/ws/src/uav_rgbd_perception/uav_rgbd_perception/
cp launch/experiment7.launch.py ~/uav/learn/ws/src/uav_rgbd_perception/launch/
cp config/experiment7.yaml ~/uav/learn/ws/src/uav_rgbd_perception/config/
cp setup.py ~/uav/learn/ws/src/uav_rgbd_perception/setup.py
```
Keep your original `depth_processor_node.py` unchanged.

Then:
```bash
cd ~/uav/learn/ws
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_rgbd_perception --symlink-install
source install/setup.bash
```

## Run
Start PX4 SITL with the custom ball world and `gz_x500_depth` first. In another terminal:
```bash
ros2 launch uav_rgbd_perception experiment7.launch.py
```

## Check
```bash
ros2 topic echo /red_box/bbox --once
ros2 topic echo /red_box/position --once
ros2 topic echo /red_box/size --once
ros2 topic echo /uav/world_state
```

`/uav/world_state` is `std_msgs/msg/String`; `data` contains JSON.
