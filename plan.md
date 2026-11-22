

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



**走VLM的线路**  
| 阶段 | 核心任务 | 最终成果 |
|---|---|---|
| 阶段1 | RGB-D 数据融合与三维场景理解 | 得到完整的三维目标信息 |
| 阶段2 | 引入 VLM | 能够理解图像、识别目标、回答场景问题 |
| 阶段3 | VLM + RGB-D 融合 | 能够理解物体的语义和三维位置 |
| 阶段4 | Agent + 任务规划 | 能够根据自然语言生成无人机任务 |
| 阶段5 | VLA / 闭环控制 | 根据视觉和语言指令自主执行任务 |  

## 阶段1  
**同实验6/7的相关内容**  
先把RGBD和点云处理完善：无人机要从箱子上方飞过，仅仅知道箱子的中心坐标是不够的，还需要知道箱子的尺寸、空间范围，以及周围是否存在其他障碍物  
### 1.1 能够发布这几个话题：    
```
/red_box/bbox      # 角点
/red_box/position
/red_box/size
/red_box/marker
```  

### 1.2 点云滤波和聚类    
了解open3d的一些功能：  
| 算法 | 作用 |
|---|---|
| Voxel Downsampling | 点云降采样 |
| Statistical Outlier Removal | 统计离群点去除 |
| Radius Outlier Removal | 半径离群点去除 |
| RANSAC Plane Segmentation | 平面提取 |
| DBSCAN Clustering | 点云聚类 |
| Oriented Bounding Box | 计算有方向的三维包围盒 |  
其中DBSCAN, 通过聚类将不同物体对应的点云分离, 实现不同物体的分类  

PointCloud Processor Node的改进版本：  
1.订阅 RGB Detector 发布的完整 MASK  
2.订阅 PointCloud2。  
3.根据 MASK 提取目标对应的整片点云。  
4.对点云进行滤波。  
5.对目标点云进行 DBSCAN 聚类。  
6.计算目标三维包围盒。  
7.对完整场景进行滤波和聚类，提取其他障碍物  

实验6的最终系统结构如下：
```  
                    Gazebo
                       |
                 RGB-D Camera
                       |
           +-----------+-----------+
           |                       |
           v                       v
      RGB Image               Depth Image
           |                       |
           v                       v
    RGB Detector Node       Depth Processor Node
           |                       |
     +-----+------+                |
     |            |                |
     v            v                v
  center_pixel   bbox             RGB-D
     |          mask              Position
     |            |
     |            |
     |            v
     |      PointCloud Processor Node <--- PointCloud2
     |            |
     |      +-----+------+
     |      |            |
     |      v            v
     |   Target ROI    Full Scene
     |      |            |
     |      v            v
     |   DBSCAN      Voxel Filter
     |      |        Statistical Filter
     |      v            |
     |   3D BBox         v
     |      |          DBSCAN
     |      |            |
     |      +------+-----+
     |             |
     |             v
     |       ROS2 Publishers
     |             |
     |    +--------+---------+
     |    |        |         |
     |    v        v         v
     | position   size     marker
     |    |
     +----+
```  

消息：  
| Topic | 消息类型 | 作用 |
|---|---|---|
| `/red_box/bbox` | PolygonStamped | 二维 Bounding Box 四角 |
| `/red_box/center_pixel` | PointStamped | 二维中心坐标 |
| `/red_box/mask` | Image | 红色目标分割结果 |
| `/red_box/position` | PointStamped | 三维包围盒中心 |
| `/red_box/size` | Vector3Stamped | 三维包围盒长宽高 |
| `/red_box/marker` | Marker | RViz 三维立方体 |
| `/red_box/points` | PointCloud2 | 提取出的目标点集合 |
| `/pointcloud/filtered` | PointCloud2 | 滤波后的完整点云 |
| `/pointcloud/clusters` | MarkerArray | 场景聚类结果 |  


实验6的初步运行：
`PX4_GZ_WORLD=default2_experiment6 make px4_sitl gz_x500_depth`  
`ros2 launch uav_rgbd_perception experiment6.launch.py`  
终端打印数据分析：  
```
在default2_experiment6.sdf描述世界的文件中，定义红色箱子<size>0.85 0.85 1.20</size>, 即标准长宽高为0.85*0.85*1.20
地面时，高度为零。
分析：此时正面朝向红色箱子，也就是只能看到宽和高
[pointcloud_processor-3] [INFO] [1795359154.108614510] [pointcloud_processor_node]: Target points=2410, center=(4.47, -0.06, 0.34), size=(0.01, 0.81, 1.12), filtered scene points=4107
[pointcloud_processor-3] [INFO] [1795359155.155563661] [pointcloud_processor_node]: Target points=2410, center=(4.47, -0.06, 0.34), size=(0.01, 0.81, 1.12), filtered scene points=4065

起飞3m,看不到红色箱子。
分析：由于几乎看不到，所以Target points数量锐减，并且size也只有 宽 基本吻合
[pointcloud_processor-3] [INFO] [1795359181.056678778] [pointcloud_processor_node]: Target points=88, center=(5.19, 0.03, -1.99), size=(0.06, 0.78, 0.01), filtered scene points=7019
[pointcloud_processor-3] [INFO] [1795359182.633600159] [pointcloud_processor_node]: Target points=100, center=(5.22, -0.04, -2.00), size=(0.06, 0.81, 0.01), filtered scene points=7103

高度2m, 能看到部分红色箱子。
分析：可以看到一半，通过顶部的面，可以分析到长和宽，但是高就砍半
[pointcloud_processor-3] [INFO] [1795359206.217717894] [pointcloud_processor_node]: Target points=2161, center=(4.81, -0.02, -1.36), size=(0.74, 0.82, 0.68), filtered scene points=7043
[pointcloud_processor-3] [INFO] [1795359207.363920725] [pointcloud_processor_node]: Target points=2101, center=(4.79, -0.04, -1.38), size=(0.78, 0.82, 0.64), filtered scene points=7048
```  
要优化的，  
1.判断点云数量有效，判断尺寸是否有效，加入目标状态有效话题/red_box/valid，消息类型std_msgs/Bool  

| 改进项 | 当前情况 | 下一步 |
|---|---|---|
| 目标有效性判断 | 点云只有几十个点仍可能发布 | 增加点数、尺寸和观测质量判断 |
| DBSCAN 聚类 | 已经实现，但没有充分展示聚类效果 | 不同簇赋予不同颜色，显示各簇包围盒 |
| 三维包围盒 | 目前是可见表面 AABB | 明确区分可见表面包围盒与完整物体包围盒 |  

| 等级 | 验收要求 | 当前进度 |
|---|---|---|
| L1：二维感知 | 检测、MASK、四角点发布 | 基本完成 |
| L2：三维目标重建 | 点云提取、滤波、聚类、三维包围盒 | 已实现，需完善可靠性 |
| L3：动态感知 | 无人机移动时，能够正确判断目标可见性和三维观测质量 | 正在验证 |

### 1.3 相机坐标系到无人机世界坐标系的转换  
无人机需要目标物体在世界坐标系中的位置  
一种简单的转换关系：$P_{world}=T_{world}^{camera}P_{camera}$, 其中 $T_{world}^{camera}=T_{world}^{body}T_{body}^{camera}$  
验收标准：无人机在Gazebo中运动，目标在世界坐标系下的位置应该保持不变  

## 阶段2：使用VLM  
### 2.1 Qwen3-VL  
### 2.2 Grounding DINE + SAM2  


## 阶段3：VLM + RGB-D，真正实现多模态场景理解  
**注意：** 不要直接把 RGB、Depth 和 PointCloud 全部扔给 VLM，而是先使用几何算法**提取结构化信息**，再交给 VLM 进行语义推理  


## 阶段4：引入 Agent，让无人机能够自主完成任务  
之前的VLM解决的问题是：无人机看到了什么？理解环境  
Agent要解决：用户的指令要我做什么？我应该采取哪些步骤？调用哪些工具？...  
| 技术 | 学习内容 | 在项目中的作用 |
|---|---|---|
| Prompt Engineering | 指令设计 | 将用户需求转化为任务 |
| Function Calling | 函数调用 | 调用感知、定位、导航模块 |
| ReAct | 推理与行动 | 根据环境反馈决定下一步 |
| MCP | 工具协议 | 统一管理外部工具 |
| LangGraph | 工作流编排 | 管理复杂任务执行流程 |  


## 阶段5： VLA  
可以以几个标准的VLA模型，做一些微调，来适配无人机的人物