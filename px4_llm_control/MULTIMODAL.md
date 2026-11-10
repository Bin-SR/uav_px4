# multimodal_model_control

这是包内独立的多模态感知节点；原DeepSeek飞行节点继续保留。支持自然语言指定目标、
RGB视觉定位、重复观测目标关联，以及条件满足时的RGBD测距和三维目标发布。
**本阶段没有接通自主飞行跟踪、局部避障或自主导航。** navigation_request是给后续规划器的意图接口，
始终executable=false；节点没有PX4/Offboard发布器，不会自动起飞、追踪或修改已有飞行任务。
Gazebo配置方案单独见GAZEBO_PLAN.md，本次未改Gazebo模型或world。

## 已实现的数据流

```text
自然语言目标 + RGB -> Qwen API / Rex-Omni -> 归一化目标框
                            |
                    保守IoU关联（连续重检测）
                            |
          同步、配准、校正RGBD + P内参 -> 光学坐标目标表面点
                            |
                   图像时刻的TF -> 世界坐标点（可选）
                            |
             navigation_request -> 后续三维局部规划器（待实现）

深度 / LaserScan / XYZ点云 -> 前向距离观测 -> safety诊断
```

VLM调用在单工作线程中，ROS回调不等待模型；图像只保留最新RGB和有限深度缓存，不积压推理。
新任务、取消和时间戳回退会使旧请求的迟到结果失效。后台本地GPU推理不能强制中断，
取消只阻止它的结果被采用，下一任务需等当前请求返回。

## 构建

```bash
conda activate uav_px4
source /opt/ros/humble/setup.bash
cd /home/si/uav/learn/ws
export PYTHONNOUSERSITE=1
colcon build --packages-select px4_llm_control
source install/setup.bash
```

依赖：ROS sensor_msgs、geometry_msgs、tf2_ros、tf2_geometry_msgs，以及numpy、Pillow。
不使用cv_bridge，避免Conda numpy与系统cv_bridge的ABI冲突。
模型权重和Torch/vLLM/Rex依赖没有自动安装或下载。

## Qwen2.5-VL：推荐独立模型服务环境

在单独的模型环境部署支持图像输入的OpenAI兼容服务，例如已安装vLLM时：

```bash
vllm serve Qwen/Qwen2.5-VL-7B-Instruct --host 127.0.0.1 --port 8000
```

该命令需要匹配的CUDA/显存/模型依赖，可能下载权重；不是ROS节点自动执行的命令。
可以改为远程模型服务。运行ROS节点的uav_px4环境只需轻量客户端：

```bash
ros2 run px4_llm_control multimodal_model_control --ros-args \
  --params-file /home/si/uav/learn/ws/src/px4_llm_control/config/multimodal.yaml
```

若服务需要密钥，在启动节点的同一终端设置VLM_API_KEY（不使用DEEPSEEK_API_KEY）：

```bash
read -rsp 'VLM API Key: ' VLM_API_KEY; echo
export VLM_API_KEY
```

base_url须包括服务实际使用的/v1或其他兼容前缀；节点追加/chat/completions。
请求是image_url内嵌JPEG+自然语言目标，模型必须返回规定JSON归一化bbox。
未知坐标格式、非法框、截断JSON均拒绝；不把模型猜测的距离作为测距数据。
远程服务会接收到相机图片和目标文本；默认地址是本地127.0.0.1。

## Rex-Omni：官方Python包装器

在已经安装官方rex_omni与对应模型依赖、且能导入ROS的Python环境运行：

```bash
ros2 run px4_llm_control multimodal_model_control --ros-args \
  -p backend:=rex_omni \
  -p rex_model_path:=/absolute/path/to/Rex-Omni \
  -p rex_backend:=transformers
```

节点在首次请求时加载RexOmniWrapper，通过task=detection、categories=[自然语言目标]调用，
将extracted_predictions中的绝对像素框转换为统一归一化框。
也可使用rex_backend:=vllm，但需要官方相应运行时。
Rex主要用来做grounding，不冒充通用飞行任务规划器。若其依赖与ROS环境冲突，
后续将Rex独立为推理服务并保留该输出协议；本次未编写或部署Rex HTTP服务器。

## 发布任务

以下命令前先source ROS及工作区；必须已有相机ROS话题，否则节点等待图像。

```bash
# 单次观察和自然语言选目标
ros2 topic pub --once /px4_vlm/command std_msgs/msg/String "{data: '寻找左边的红色箱子'}"

# 持续重检测与目标关联；此时不会自动控制飞机
ros2 topic pub --once /px4_vlm/command std_msgs/msg/String "{data: '跟踪红色汽车'}"

# 参数化意图，目标描述仍是自然语言
ros2 topic pub --once /px4_vlm/command std_msgs/msg/String \
  '{data: "{\"mode\":\"navigate\",\"target\":\"蓝色箱子\",\"standoff_m\":3.0}"}'

# 取消；不影响原llm_control正在执行的飞行任务
ros2 topic pub --once /px4_vlm/cancel std_msgs/msg/String "{data: 'cancel'}"
```

plain text以跟踪/导航到/观察/寻找开头分别映射到track/navigate/observe/observe，其他文本默认observe。
这是目标任务入口，不是任意复合飞行指令解析器；“跟踪红车并穿过门再降落”需后续任务编排层。
observe只推理一次；track/navigate按inference_period间隔重复处理最新帧，一次最多一个推理请求。
多个匹配对象时输出ambiguous而不随意选取。请进一步说明“左边的红箱子”等条件。

## ROS接口

| 话题 | 类型 | 方向/意义 |
|---|---|---|
| /px4_vlm/command | std_msgs/String | 输入任务文本或JSON |
| /px4_vlm/cancel | std_msgs/String | 任意内容取消当前视觉任务 |
| /camera/image_raw | sensor_msgs/Image | RGB输入，可配置；rgb8/bgr8/rgba8/bgra8/mono8 |
| /camera/aligned_depth_to_color/image_raw | sensor_msgs/Image | 已配准深度输入；32FC1米或16UC1毫米 |
| /camera/camera_info | sensor_msgs/CameraInfo | 校正相机内参P |
| /lidar/scan | sensor_msgs/LaserScan | 可选前向扫描距离 |
| /lidar/points | sensor_msgs/PointCloud2 | 可选已标定坐标系的xyz点云 |
| /px4_vlm/detections | std_msgs/String | 检测JSON；归一化xyxy、原始尺寸、时间戳、选中索引 |
| /px4_vlm/annotated_image | sensor_msgs/Image | 原尺寸RGB叠框图，绿色为选中对象 |
| /px4_vlm/target | std_msgs/String | 当前目标有效性、框、关联次数、光学/世界点（可为空） |
| /px4_vlm/target_point | geometry_msgs/PointStamped | 目标表面估计点，optical坐标右/下/前，米 |
| /px4_vlm/target_point_world | geometry_msgs/PointStamped | 存在正确TF时的世界点，坐标系见header |
| /px4_vlm/navigation_request | std_msgs/String | track/navigate意图，含目标和standoff，不可直接执行 |
| /px4_vlm/safety | std_msgs/String | 10Hz前向距离诊断；blocked/clear_observed/unknown |
| /px4_vlm/status | std_msgs/String | 任务/缺传感器/模型错误/丢失/歧义等事件 |

```bash
ros2 topic echo /px4_vlm/target
ros2 topic echo /px4_vlm/safety
ros2 topic echo /px4_vlm/status
```

目标JSON携带image_stamp、observation_age_s、max_age_s；消费者必须检查时间，
不能永久复用最后一个PointStamped。超时/取消会另发valid=false，但PointStamped本身没有撤销消息。
默认只接收3秒内的结果；大模型太慢会输出stale_result，不给过期目标点。
IoU关联只适合相邻帧位移有限的基线演示，没有外观重识别、运动预测、遮挡恢复或快速目标保证。
目标丢失后不自动切换到无重叠的同类目标；重新发指令可重新选择。

## 三维测距条件

默认depth_registered=false、images_rectified=false，只做二维观测。
启用前必须实际具备：RGB校正图、对齐到RGB像素网格的深度、匹配的CameraInfo P，
全部分辨率及光学frame一致，深度与RGB时间差不超过sync_slop（默认0.1秒）。
支持标准零skew/零投影平移的RGB校正投影矩阵，RGB光学frame需显式配置。

```bash
# 仅在上述条件确实满足后添加这些参数
-p depth_registered:=true -p images_rectified:=true -p optical_frame:=camera_optical_frame
```

程序在目标框中心区域取有效深度中位数，反投影到光学坐标；这是可见表面估计，
不是物体精确中心、完整三维包围盒或可飞到的无碰撞目标位姿。透明/反光/遮挡/框跨背景可能测错。
光学坐标右下前、机体ROS坐标前左上、PX4机体前右下以及世界ENU/NED必须通过标定和TF区分，
不能把相机xyz直接填入PX4的NED setpoint。

## 障碍物观测的边界

深度仅检查中央图像区域，需先设置depth_forward_facing=true确认相机朝向。
LaserScan需匹配scan_frame，angle=scan_forward_angle_deg表示前向；点云需预先转换到points_frame，
前向x、左右y、上下z，检查±0.75米截面的前向走廊，不自动推断激光/雷达外参。
有xyz的雷达点云可接同一接口；专有雷达消息需另写驱动适配器，本次未实现雷达仿真物理。

默认2米内的有效回波输出blocked。缺失/过期/无有效回波为unknown；
只有required_obstacle_sources全部有有效数据才可输出clear_observed。
它仅表示当前观测区域未发现过近障碍，**不表示整条航线无碰撞，也不会自动制动或绕行**。
后续需速度相关制动距离、完整视场检查、三维占据/ESDF和局部规划器，并把故障保护接入唯一飞行执行器。

官方参考：
- https://huggingface.co/Qwen/Qwen2.5-VL-7B-Instruct
- https://github.com/IDEA-Research/Rex-Omni
