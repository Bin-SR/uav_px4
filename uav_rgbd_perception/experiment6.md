# UAV RGB-D Experiment 6

本目录提供三个 ROS 2 Python 节点、参数文件、launch 文件和一个扩展后的 Gazebo `default2` world。

## 1. 节点职责

- `rgb_detector_node.py`
  - HSV 红色分割
  - 发布 `/red_box/center_pixel` (`geometry_msgs/PointStamped`)
  - 发布 `/red_box/bbox` (`geometry_msgs/PolygonStamped`)，四点顺序：左上、右上、右下、左下
  - 发布 `/red_box/mask` (`sensor_msgs/Image`, `mono8`)
- `depth_processor_node.py`
  - 保留中心像素 RGB-D 深度中值估计
  - 发布 `/red_box/rgbd_position`，用于与 PointCloud 结果交叉验证
  - 越界时跳过当前帧，不退出节点
- `pointcloud_processor_node.py`
  - 使用 MASK 从 organized PointCloud2 中提取目标点集合 P
  - 对完整场景执行 Voxel Downsampling + Statistical Outlier Removal + RANSAC 主平面（通常是地面）去除
  - 对目标点执行滤波 + DBSCAN，选择最大有效簇
  - 对全场景点云执行 DBSCAN，发布聚类包围盒 MarkerArray
  - 发布 `/red_box/position`、`/red_box/size`、`/red_box/marker`、`/red_box/points`

## 2. 坐标和标定假设（务必先读）

本版本的目标点云坐标直接沿用 `/camera/depth/points` 的 `header.frame_id`。
点云有效性按三维欧氏距离筛选，不假设前向轴一定是 Z（你的现有结果中 PointCloud 更接近 X 前向、Z 向上）。
`/red_box/position` 是三维包围盒中心，不是世界坐标；实验7再做 Camera -> UAV/World 坐标变换。
中心 RGB-D 对照结果先按 frame_id 处理：若深度图 frame_id 以 `camera_link` 结尾，则将光学坐标 `(右,下,前)` 转为机体相机坐标 `(前,左,上)`；否则保留光学坐标。后续实验7会用 TF 取代这种约定式转换。

MASK 到深度点云的像素映射使用两组 CameraInfo 的内参做 pinhole 映射，假设 RGB 与 Depth 光心基本重合、光轴平行。
若两相机存在明显平移/旋转，仅用内参映射不准确，需要使用 RGB-depth 外参进行 3D reprojection。
PointCloud2 必须是 organized cloud（height > 1，width/height 对应深度图像像素布局）。如果 `height=1`，需改用无组织点云的投影/索引策略。

## 3. 安装依赖

Ubuntu 22.04 / ROS 2 Humble 环境中，先确认 ROS Python 解释器可导入依赖：

```bash
source /opt/ros/humble/setup.bash
python3 -c "import rclpy, cv2, cv_bridge, open3d, sensor_msgs_py; print('deps ok')"
```

如果系统 Python 缺 Open3D，可在实际运行节点的 Python 环境中安装兼容版本；不要只在另一个 Conda 环境安装后就假设 ROS 节点能导入。

## 4. 集成到现有工作空间

如果你的现有 `~/uav/learn/ws/src/uav_rgbd_perception` 已经存在，不要覆盖整个包。将三个脚本分别复制/替换到现有 Python 包目录中：

```bash
cp uav_rgbd_perception/rgb_detector_node.py ~/uav/learn/ws/src/uav_rgbd_perception/uav_rgbd_perception/
cp uav_rgbd_perception/depth_processor_node.py ~/uav/learn/ws/src/uav_rgbd_perception/uav_rgbd_perception/
cp uav_rgbd_perception/pointcloud_processor_node.py ~/uav/learn/ws/src/uav_rgbd_perception/uav_rgbd_perception/
```

将 `config/experiment6.yaml` 复制到包的 `config/`，将 `launch/experiment6.launch.py` 复制到包的 `launch/`。
在现有 `setup.py` 的 `entry_points['console_scripts']` 中加入：

```python
'rgb_detector = uav_rgbd_perception.rgb_detector_node:main',
'depth_processor = uav_rgbd_perception.depth_processor_node:main',
'pointcloud_processor = uav_rgbd_perception.pointcloud_processor_node:main',
```

并确保 `setup.py` 的 `data_files` 安装 `launch/*.launch.py` 和 `config/*.yaml`。`package.xml` 需具有本包实际使用的 ROS 依赖。不要直接用本模板覆盖你已有的 setup.py/package.xml，以免丢失原有配置。

## 5. 编译与运行

```bash
cd ~/uav/learn/ws
source /opt/ros/humble/setup.bash
# 如果该工作空间依赖你的 Conda 环境，先激活实际运行节点所用环境
colcon build --packages-select uav_rgbd_perception --symlink-install
source install/setup.bash
```

先启动 PX4/Gazebo，并确保以下话题存在：

```bash
ros2 topic list | grep -E 'camera/image_raw|camera/depth/image_raw|camera/depth/points|camera_info'
```

使用扩展 world（推荐先保留原 world 备份）：

```bash
# 在 PX4-Autopilot 仓库中定位 worlds 目录
find Tools -path '*gz*' -type d -name worlds

# 将本包 worlds/default2_experiment6.sdf 复制到 PX4 实际使用的 worlds 目录
# 然后运行
PX4_GZ_WORLD=default2_experiment6 make px4_sitl gz_x500_depth
```

如果你希望继续使用 `PX4_GZ_WORLD=default2`，先备份 PX4 worlds 目录中的原 `default2.sdf`，再用本包的 `worlds/default2.sdf` 替换。你上传的原始 `default2.sdf` 没有额外 world 插件配置；如果你本地实际版本已经添加过插件，请合并模型部分，不要覆盖掉本地插件。

启动节点：
```bash
ros2 launch uav_rgbd_perception experiment6.launch.py
```

如果 launch 暂时没有配置好，也可以分三个终端运行：
```bash
ros2 run uav_rgbd_perception rgb_detector
ros2 run uav_rgbd_perception depth_processor
ros2 run uav_rgbd_perception pointcloud_processor
```

## 6. 验收命令

```bash
ros2 topic echo /red_box/bbox --once
ros2 topic echo /red_box/center_pixel --once
ros2 topic echo /red_box/position --once
ros2 topic echo /red_box/size --once
ros2 topic echo /red_box/rgbd_position --once
ros2 topic info /red_box/points
ros2 topic info /pointcloud/filtered
ros2 topic echo /pointcloud/clusters --once
```

RViz2 添加：
- PointCloud2: `/red_box/points`
- PointCloud2: `/pointcloud/filtered`
- Marker: `/red_box/marker`
- MarkerArray: `/pointcloud/clusters`

RViz 的 Fixed Frame 先设置为 `/camera/depth/points` 的 `header.frame_id`（通常是相机 optical frame；以实际消息为准）。

## 7. 调参建议

- 目标点云断裂、DBSCAN 没有簇：适当增大 `target_dbscan_eps`（如 0.12~0.18）。
- 红色箱子和背景粘连：减小 `target_dbscan_eps`，并检查 MASK 与深度是否注册。
- 场景点云过密、Open3D 卡顿：增大 `voxel_size`（如 0.05~0.08），降低 `process_hz`。
- 点云被滤得太少：增大 `statistical_std_ratio` 或减小 `statistical_nb_neighbors`。
- 地面误删/未删干净：调整 `plane_distance_threshold`；若场景主平面不是地面，先关闭 `remove_ground_plane` 再检查。
- `min_target_points` 是过滤门限，不是 DBSCAN 参数；根据距离和分辨率调整。

## 8. 当前版本的边界

1. 目标检测仍然是 HSV 红色分割，只发布最大红色连通轮廓。
2. 目标 3D AABB 使用点云 XYZ 的 2%~98% 分位数，降低边缘离群点影响；`size.x/y/z` 对应点云 frame 的三个轴向范围。
3. `/red_box/position`、`/red_box/size`、Marker 仍处于点云 header frame，不是 world frame。
4. 场景 DBSCAN 聚类是对当前相机可见点云的聚类，不等同于完整场景地图或持久化物体跟踪。
5. RGB/Depth 的 pinhole 映射仅适用于相机近似共光心、共轴的情形。实验7会进一步处理 TF、相机外参和世界坐标变换。
