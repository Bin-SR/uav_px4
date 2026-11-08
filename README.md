## make px4_sitl gz_x500,编译报错：
```
CMake Error at /usr/lib/x86_64-linux-gnu/cmake/jsoncpp/jsoncppConfig.cmake:2 (cmake_policy):
  Compatibility with CMake < 3.5 has been removed from CMake.

  Update the VERSION argument <min> value.  Or, use the <min>...<max> syntax
  to tell CMake that the project requires at least <min> but has been updated
  to work with policies introduced by <max> or earlier.

  Or, add **-DCMAKE_POLICY_VERSION_MINIMUM=3.5** to try configuring anyway.
Call Stack (most recent call first):
  /usr/share/cmake/gz-cmake3/cmake3/FindJSONCPP.cmake:34 (find_package)
  /usr/lib/x86_64-linux-gnu/cmake/gz-fuel_tools9/gz-fuel_tools9-config.cmake:94 (find_package)
  /usr/lib/x86_64-linux-gnu/cmake/gz-sim8/gz-sim8-config.cmake:97 (find_package)
  src/modules/simulation/gz_plugins/CMakeLists.txt:38 (find_package)


-- Configuring incomplete, errors occurred!
Error: /home/si/uav/learn/PX4-Autopilot/build/px4_sitl_default is not a directory
make: *** [Makefile:250：px4_sitl] 错误 1
```
### 解决：
`export CMAKE_POLICY_VERSION_MINIMUM=3.5`，之后再重新`make px4_sitl gz_x500`

## 使用
### conda环境：
`conda activate uav_px4`
### 打开地面基站GCS：
运行文件：`/qgroundcontrol/build/Debug/QGroundControl` 或者
### 先导入环境变量：
`export CMAKE_POLICY_VERSION_MINIMUM=3.5`
### 启动uav的gz仿真：
`make px4_sitl gz_x500`
### 启动MicroXRCEAgent工具：
`MicroXRCEAgent udp4 -p 8888`，建立px4与ROS2的通信，可以查看相关话题的发布  
详见`Step2`
### 关闭系统
在`make`的那个终端窗口里面，直接`ctrl + c`退出，否则下次启动`make`就会出现`问题解决`中的:  
`1.启动make px4_sitl gz_x500后，没有打开Gazebo的UI界面`
## 目标：
利用大模型作为无人机集群的高层智能决策中枢，实现多架四旋翼无人机的任务理解、任务分配、自主导航、路径规划、动态避障和协同控制<br>
单机飞行 → ROS2 控制 → 自主导航 → 多机协同 → 大模型集群决策

## 学习规划
### 记录 & tech-stack
**tech-stack** : PX4 SITL, Gazebo Harmonic, Micro XRCE-DDS  

**NED坐标**： North East Down, 北东地坐标系  
NE坐标面与地球表面相切，D方向垂直当地水平面指向地心  
即，如果飞机起飞，高度2m，则z坐标等于 **-2m**  
|名称|解释|
|---|---|
|Commander|飞行模式、解锁和安全检查|
|Navigator|任务导航|
|EKF2|状态估计|
|Multicopter Position Control|多旋翼位置控制|
|Multicopter Attitude Control|姿态控制|
|Control Allocator|控制分配|
|uORB | PX4 内部消息通信机制
|Micro XRCE-DDS|ROS2与PX4的通信|    

| 通信方案 | 主要用途 |
|---|---|
| MAVLink | 地面站、遥控、机载计算机与飞控之间通信 |
| uXRCE-DDS | PX4 与 ROS2 之间进行原生 DDS 消息通信 |

| 功能 | 使用技术 |
|---|---|
| 飞行控制 | PX4 |
| 仿真环境 | Gazebo |
| 地面站监控 | QGroundControl |
| 地面站与飞控通信 | MAVLink |
| ROS2 与飞控通信 | uXRCE-DDS |
| 自主导航 | ROS2 |
| 路径规划 | A*、RRT、MPC |
| 视觉感知 | OpenCV、YOLO、VLM |
| 集群任务规划 | LLM |
| 多机协同 | ROS2 + MARL |  


### Step1: PX4 + Gazebo 基础
- [x] 利用QGroundControl来控制无人机，解锁、起飞、降落等简单动作
### Step2: ROS2 + PX4 Offboard 控制
不再使用 QGroundControl 手动控制无人机，而是通过自己编写的 ROS2 程序控制无人机。  
ROS2获取UAV状态，位姿，发送Offboard控制指令，控制UAV解锁，按指令飞行  
| ROS2 Topic | 作用 |
|---|---|
| /fmu/in/offboard_control_mode | 声明 Offboard 控制模式 |
| /fmu/in/trajectory_setpoint | 发布位置、速度或加速度设定值 |
| /fmu/in/vehicle_command| 发送解锁、切换模式等命令 |
| /fmu/out/vehicle_status_v5| 获取飞控状态 |
| /fmu/out/vehicle_local_position_v1| 获取无人机局部位置 |
| /fmu/out/vehicle_attitude | 获取无人机姿态 |
#### ROS2与PX4通信  
| ROS2 Topic | 作用 |
|---|---|
| /fmu/out/vehicle_status_v5| 获取飞控状态 |
| /fmu/out/vehicle_local_position_v1| 获取无人机局部位置 |
| /fmu/out/vehicle_attitude | 获取无人机姿态 |  

PX4仓库中的/PX4-Autopilot/src/modules/uxrce_dds_client,  MicroXRCEAgent,  px4_msg  

| 组件 | 运行位置 | 作用 |
|---|---|---|
| uxrce_dds_client | PX4 飞控内部 | 将 PX4 的 uORB 消息桥接到 DDS |
| MicroXRCEAgent | Ubuntu 主机 | 接收 PX4 Client 的连接，并桥接到 ROS2 DDS |
| px4_msgs | ROS2 工作空间 | 提供 PX4 消息对应的 ROS2 消息类型 |  

uxrce_dds_client是在PX4仓库编译时，已经顺带编译好的，可以验证：`pxh> uxrce_dds_client status`,来显示状态  
##### 1.1 MicroXRCEAgent工具，需要再单独安装, 再对MicroXRCEAgent进行编译：  
```Plain
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd uav/learn/Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
which MicroXRCEAgent # 验证
```
##### 1.2 启动PX4+Gazebo和QGC后，启动Micro XRCE-DDS Agent  
在`make px4_sitl gz_x500`的打印日志中有：`INFO  [uxrce_dds_client] init UDP agent IP:127.0.0.1, port:8888`  
启动Micro XRCE-DDS Agent: `MicroXRCEAgent udp4 -p 8888`    

##### 1.3 开启新终端，查看ros2与px4之间的话题 
加载ROS, 查看相关话题：  
```
source /opt/ros/humble/setup.bash
ros2 topic list
```
应该能看到
```
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command

/fmu/out/vehicle_attitude
...
```  
source新安装编译的px4_msgs: `source ~/uav/learn/ws/install/setup.bash`  
假如获取无人机位置：`ros2 topic echo /fmu/out/vehicle_local_position`  
```
！！！注意，ros2的topic显示，需要后台`MicroXRCEAgent udp4 -p 8888`不能停，否则会报错，找不到发布者  
WARNING: topic [/fmu/out/vehicle_attitude] does not appear to be published yet  
Could not determine the type for the passed topic
```
#### 通过ROS2给PX4发送指令,控制无人机  
##### 2.1 写一个ros2的pkg，通过订阅px4话题，获取uav的状态  
在conda环境中，禁止Python加载用户目录~/.local/lib/python3.10/site-packages中的第三方包：`export PYTHONNOUSERSITE=1`  
通过`/fmu/out/vehicle_local_position_v1, ../vehicle_attitude, ../vehicle_status`，分别获取xyz坐标， rpy角度， 以及飞行状态  
使用：在/ws/src中创建了名为px4_state_monitor的pkg，可以通过`ros2 pkg list`查看是否存在，若不存在，则需要source  
`ros2 run px4_state_monitor state_monitor`  
##### 2.2 ROS2通过offboard控制无人机  
| ROS2 Topic | 作用 |
|---|---|
| /fmu/in/offboard_control_mode | 声明 Offboard 控制模式 |
| /fmu/in/trajectory_setpoint | 发布位置、速度或加速度设定值 |
| /fmu/in/vehicle_command| 发送解锁、切换模式等命令 |  

`/fmu/in/offboard_control_mode`话题： 是PX4的控制模式  
`/fmu/in/trajectory_setpoint`话题： 有float32[3] position，通过发送NED坐标，给UAV指令  
`/fmu/in/vehicle_command`话题： 是PX4的命令，如解锁、起飞、降落等    

##### 2.3 offboard控制的整体架构(pipeline)  
1.等待PX4局部位置有效(状态：WAIT_FOR_ESTIMATOR)  
2.以20Hz发送offboardmode和trajectory话题持续2秒以上(状态：STREAM_SETPOINTS)  
3.请求进入offboard(状态：REQUEST_OFFBOARD)  
切换到(**REQUEST_ARM**)的判断：检查CommandAck的command, 检查status的nav_state是否变为NAVIGATION_STATE_OFFBOARD  
4.请求解锁(状态：REQUEST_ARM)  
切换到(**TAKEOFF**)的判断：检查CommandAck的command, 检查status的arming_state是否变为ARMING_STATE_ARMED  
5.起飞到指定高度，保持偏航角(状态：TAKEOFF)
6.悬停指定秒数(状态：HOVER)  
7.发送降落命令(状态：LAND)  
检测降落，PX4自动上锁，若没有上锁，则进入状态：REQUEST_DISARM  
8.手动上锁(状态：REQUEST_DISARM)  
再判断是否上锁，从而进入FINISH
9.FINISH/ERROR  

### Step3: 无人机自主导航与轨迹规划
UAV根据目标位置，自主规划路线
| 算法 | 研究内容 |
|---|---|
| A* | 栅格地图中的路径搜索 |
| Dijkstra | 最短路径搜索 |
| RRT | 随机采样路径规划 |
| RRT* | 渐近最优路径规划 |
| Minimum Snap | 四旋翼平滑轨迹生成 |
| MPC | 模型预测控制 |
### Step4: 环境感知与自主避障
理解并使用各种传感器： RGBD, LiDar, IMU, GPS等  
算法： 点云处理，深度图处理，障碍物检测，栅格地图， 障碍距离估计
| 避障算法 | 用途 |
|---|---|
| VFH | 基于方向直方图的避障 |
| Artificial Potential Field | 人工势场法 |
| DWA | 动态窗口法 |
| MPC | 考虑动力学约束的局部规划 |
| ESDF | 基于距离场的轨迹优化 |
后续可研究EGO-Planner无人机自主飞行规划框架
### Step5: 多无人机仿真与集群协同
...  
**！！！现在还不太懂多机怎么通信，怎么实现信息共享**
### Step6: 大模型介入
在单机上先测试，通过自然语言指令，LLM/VLM进行任务理解与规划  
**假设场景：** 派遣三架无人机搜索指定区域，发现目标后，两架无人机负责跟踪，另一架无人机负责返回基地报告目标位置。  
大模型需要完成：
1. 理解自然语言任务。
2. 将任务拆解为多个子任务。
3. 判断需要多少架无人机。
4. 分配每架无人机的任务。
5. 生成结构化任务描述。
6. 根据无人机反馈调整任务。
假如输出：  
```
{  
  "mission": "search_and_track",  
  "uavs": [  
    {"id": 1, "task": "search"," area":"A},  
    {"id": 2, "task": "track",      "target":"detected_target"},  
    {"id": 3, "task": "return_and_report", "destination": "base"}  
  ]  
}
```
, 然后由任务执行器将这些任务转化为具体的 ROS2 指令。
## 问题解决：
 ### 1.启动make px4_sitl gz_x500后，没有打开Gazebo的UI界面：
  新开启一个终端，输入：`gz sim -g`，如果正常打开，那就是有残留进程  
  查看残留进程：`pgrep -af 'gz|gazebo'`, 清理：`kill <PID号>`  
 ### 2.ROS2的话题，echo无效：
指令：`ros2 topic echo /fmu/out/vehicle_global_position `  
输出：`The message type 'px4_msgs/msg/VehicleGlobalPosition' is invalid`  
解决：在`~/uav/learn/`中创建`ws`工作空间，克隆px4_msgs源码再编译  
先检查PX4-Autopilot的分支，选择对应分支的px4_msgs进行克隆  
```
git clone -b main https://github.com/PX4/px4_msgs.git
cd px4_msgs
git branch --show-current
cd ~/uav/learn/ws                                 # 切换到工作空间中，准备编译
source /opt/ros/humble/setup.bash
colcon build
source ~/uav/learn/ws/install/setup.bash          # source一下新的px4_msgs
ros2 pkg list | grep px4_msgs                     # 验证消息类型
ros2 interface show px4_msgs/msg/VehicleAttitude
ros2 topic echo /fmu/out/vehicle_attitude
```


