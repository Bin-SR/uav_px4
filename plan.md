

| 实验 | 名称 | 主要目标 | 技术 |
|---|---|---|---|
| 5 | PointCloud2 三维验证 | 验证 RGB-D 三维坐标 | PointCloud2、Open3D |
| 6.1 | 障碍物三维点定位 | 获取障碍物表面三维点 | RGB-D、针孔模型 |
| 6.2 | 障碍物三维区域提取 | 获取箱子的长宽高和边界 | HSV、Depth、NumPy |
| 6.3 | 三维障碍物模型发布 | 发布三维中心和 Bounding Box | ROS 2、Marker |
| 7.1 | 相机与无人机坐标系关系 | 确定相机安装外参 | TF2、静态变换 |
| 7.2 | 无人机动态位姿获取 | 获取无人机在世界中的位置和姿态 | PX4、ROS 2 |
| 7.3 | 相机坐标到世界坐标 | 得到障碍物世界坐标 | TF2、齐次变换 |
| 7.4 | 坐标变换验证 | 无人机移动时障碍物世界坐标保持稳定 | Gazebo、RViz |  
6.1 获取箱子的二维边界框bbox, 新建话题`/red_box/bbox`  
使用`geometry_msgs/msg/PolygonStamped`  
```
(x_min, y_min)
(x_max, y_min)
(x_max, y_max)
(x_min, y_max)
```  
6.2/3 获取对应框的深度区域, (先把RGB的点映射到Depth中)通过RGBD解算得到区域中的每个点$P_i$, 从而得到得到三维点集合$P=\{P_1, ..., P_n\}$  
6.4 计算三维边界框，问题是：RGB-D 相机只能看到箱子朝向相机的表面，无法直接看到箱子的背面。  
所以，单帧点云计算出来的 Bounding Box，可能只能准确描述箱子的正面宽度和高度，而不能准确得到它真实的厚度。

实验6的结果：发布话题  
| Topic | 消息类型 | 内容 |
|---|---|---|
| `/red_box/bbox` | `geometry_msgs/PolygonStamped` | RGB 图像中的四个角点 |
| `/red_box/position` | `geometry_msgs/PointStamped` | 三维障碍物中心 |
| `/red_box/size` | `geometry_msgs/Vector3Stamped` | 三维障碍物长宽高 |
| `/red_box/marker` | `visualization_msgs/Marker` | RViz 三维边界框 |
| /red_box/pointcloud | sensor_msgs/PointCloud2 | 提取出来的障碍物三维点集合| 

| 顺序 | 工作 | 需要修改的文件 |
|---|---|---|
| 1 | RGB 节点发布二维 Bounding Box | rgb_detector_node.py |
| 2 | Depth 节点提取整个箱子的深度区域 | depth_processor_node.py |
| 3 | 反投影得到三维点集合 | 新建 obstacle_geometry_node.py |
| 4 | 计算三维 Bounding Box | obstacle_geometry_node.py |
| 5 | RViz 显示障碍物三维边界框 | obstacle_geometry_node.py |
| 6 | 检查当前 TF 树 | ROS 2 TF2 |
| 7 | 获取 PX4 无人机动态位姿 | PX4 ROS 2 Topics |
| 8 | 建立无人机与相机坐标变换 | TF2 |
| 9 | 计算障碍物世界坐标 | 新建 obstacle_world_node.py |
| 10 | 无人机移动时验证世界坐标稳定性 | Gazebo + RViz |