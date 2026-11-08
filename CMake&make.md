# 什么是MAVLink
**MAVLink**(Micro Air Vehicle Link)是一种专门为无人机和机器人设计的轻量级通信协议  
通常一架UAV会涉及：飞控PX4, 地面站(QGControl), 机载计算机(eg. Jetson), 遥控器, GPS等传感器  
| 发送方 | 接收方 | 传递的信息 |
|---|---|---|
| PX4 | QGC | 无人机位置、姿态、电池电量 |
| QGC | PX4 | 解锁、起飞、降落指令 |
| Jetson | PX4 | 位置、速度、姿态控制指令 |
| PX4 | Jetson | IMU、GPS、飞行状态 |
## 例子
假设你希望无人机起飞到 5 米高度  
GC 可以向 PX4 发送一条 MAVLink 消息：
```Plain
COMMAND_LONG
    command: MAV_CMD_NAV_TAKEOFF
    param7: 5.0
```  
QGC 收到消息后，就可以在界面中显示无人机已经连接、处于什么飞行模式

## MAVLink的通信方式
| 通信方式 | 应用场景 |
|---|---|
| UART 串口 | 飞控与机载计算机通信 |
| USB | 飞控与电脑连接 |
| UDP | 仿真、网络通信 |
| TCP | 部分网络通信场景 |
| 无线数传电台 | 无人机与地面站通信 |  
在 Gazebo 仿真中，通常可以使用 UDP 进行 MAVLink 通信  
  
# 什么是QGC
QGroundControl(简称 QGC)是一个开源的无人机地面控制站
## 职责
### 1.无人机状态监控
GPS 位置, 飞行高度, Roll、Pitch、Yaw, 电池电压, 飞行模式, 卫星数量, 飞行速度
### 2.无人机参数配置
PID参数, 最大飞行速度高度, 返航高度, 失联保护等, 这些参数会被配置到 PX4 中
###  3.飞行任务规划
在QGC地图上选点，规划路径
### 4.人工控制无人机
包括解锁, 起飞, 降落等

