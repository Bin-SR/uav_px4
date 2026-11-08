# px4_llm_control

ROS 2 Humble / PX4 SITL 自然语言控制。DeepSeek 生成 JSON 计划，本地控制器校验和执行。
仅用于当前 gz_x500 仿真，没有视觉/避障能力。不要与 px4_offboard_controller 同时运行；
px4_state_monitor 可同时运行。节点需在飞机着陆、未解锁时开始控制，不接管空中飞机。

## 编译

所有 ROS 终端使用你原有的 ROS/Conda 初始化方式，例如：

```bash
conda activate uav_px4
source /opt/ros/humble/setup.bash
cd /home/si/uav/learn/ws
PYTHONNOUSERSITE=1 /usr/bin/colcon build --symlink-install --packages-select px4_llm_control
source install/setup.bash
```

本机用户目录的 setuptools/packaging 存在版本冲突，上述构建命令隔离用户 Python 包；已构建成功。
无需安装 openai SDK，HTTP 使用 Python 标准库。API 密钥只从环境变量读取，不写在源码中。

## 启动仿真及通信

分别在终端运行（已启动的进程无需重复启动）：

```bash
cd /home/si/uav/learn/PX4-Autopilot
make px4_sitl gz_x500
```

```bash
MicroXRCEAgent udp4 -p 8888
```

```bash
/home/si/uav/learn/qgroundcontrol/build/QGroundControl-x86_64.AppImage
```

启动控制节点的终端：

```bash
conda activate uav_px4
source /opt/ros/humble/setup.bash
source /home/si/uav/learn/ws/install/setup.bash
read -rsp 'DeepSeek API Key: ' DEEPSEEK_API_KEY; echo
export DEEPSEEK_API_KEY
ros2 run px4_llm_control llm_control
```

默认模型 deepseek-chat，参数可改：`--ros-args -p model:=deepseek-chat -p api_timeout:=30.0`。
建议先用 `--ros-args -p dry_run:=true`，只推理和校验，不发送飞行命令。
如果平台调整模型名，用 model 参数指定账户当前可用的对话模型。

## 另一终端发布指令

同样激活环境并 source ROS 与工作区后：

```bash
ros2 topic echo /px4_llm/status
# 在另一终端查看模型输出
ros2 topic echo /px4_llm/plan
```

```bash
ros2 topic pub --once /px4_llm/command std_msgs/msg/String "{data: '起飞到2米，悬停5秒，向前飞2米，右转90度，再向前飞1米，最后降落'}"
```

先完成一个任务再发下一条。没有要求降落时，任务结束持续悬停：

```bash
ros2 topic pub --once /px4_llm/command std_msgs/msg/String "{data: '起飞到2米并悬停3秒'}"
# 等待 completed，再发布：
ros2 topic pub --once /px4_llm/command std_msgs/msg/String "{data: '向右飞1米，上升1米，悬停5秒'}"
```

独立本地控制接口，绕过 DeepSeek；land 优先取消任务和迟到的推理结果：

```bash
ros2 topic pub --once /px4_llm/emergency std_msgs/msg/String "{data: 'stop'}"
ros2 topic pub --once /px4_llm/emergency std_msgs/msg/String "{data: 'land'}"
```

stop 在本节点控制且遥测有效时保持当前位置；降落阶段不会被 stop 中断。
land 请求 PX4 AUTO_LAND，着陆并自动上锁后回到 idle。退出节点时若仍由本节点控制会请求一次降落，须在 QGC 确认。

## 接口和执行规则

| 话题 | 类型 | 方向 |
|---|---|---|
| /px4_llm/command | std_msgs/msg/String | 输入自然语言 |
| /px4_llm/emergency | std_msgs/msg/String | 输入 stop / land |
| /px4_llm/plan | std_msgs/msg/String | 输出已通过格式校验的 JSON；仍可能因飞行状态或边界被拒绝 |
| /px4_llm/status | std_msgs/msg/String | 输出 JSON 事件：planning/executing/completed/rejected/error/ack 等 |

PX4 话题沿用已有包：输入 offboard_control_mode、trajectory_setpoint、vehicle_command_v1；
输出 vehicle_local_position_v1、vehicle_status_v5、vehicle_land_detected、vehicle_command_ack_v2。
可用 ROS `-r old_topic:=new_topic` 适配不同消息版本，消息定义本身也必须匹配固件。

- 起飞高度0.5–10米，单步移动最长10米，距起飞点水平半径20米，最多16步。
- 前/右/下按每一步开始时飞机朝向计算；NED 为北/东/下。上升对应负 dz，右转为正角度。
- 悬停最多120秒，转向单步最多180度。位置与航向实际到达且速度降低后才进入下一步。
- 无指令时不自动解锁。起飞前先发送2秒20 Hz心跳，确认 Offboard 后请求解锁。
- API 在后台运行，失败不执行新计划；已有悬停持续。忙碌时拒绝新指令，不排队。
- 遥测超过2秒无更新、估计器重置、failsafe或动作超时会取消任务并请求降落。
- 离开 Offboard 后释放控制，不自动抢回模式或重新解锁。QGC 可切换模式接管。
- 起飞/解锁最长15秒，单动作最长180秒，降落最长90秒。ACK 会发布到状态话题；
  飞行是否成功以实际状态为准。降落超时需通过 QGC 接管。
- 不生成/执行任意 Python、shell 或底层电机命令；完整任务及每步实际目标均校验边界。
  边界校验不是物理围栏，不限制瞬时速度，也不替代 PX4 failsafe；默认速度由 PX4 参数决定。
- 推理请求会向 DeepSeek 发送自然语言和局部位置/航向状态；未提供完整历史，跨指令请说清动作。

API 参考：https://api-docs.deepseek.com/guides/json_mode/

## 本地测试

```bash
cd /home/si/uav/learn/ws/src/px4_llm_control
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test -q
```

单元测试不需要 API 或 ROS，覆盖非法输出、坐标转换和累计边界。
真实 API 和端到端飞行需要你自己的密钥及运行中的仿真。
