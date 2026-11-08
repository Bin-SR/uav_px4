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
export PYTHONNOUSERSITE=1
colcon build --packages-select px4_llm_control
source install/setup.bash
```

本机用户目录的 setuptools/packaging 存在版本冲突，上述构建命令隔离用户 Python 包，使用普通安装；修改Python脚本后需要重新构建并重启节点。
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

默认模型 deepseek-chat，参数可改：`--ros-args -p model:=deepseek-chat -p api_timeout:=60.0`。
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
| /px4_llm/feedback | std_msgs/msg/String | 输出2Hz位置、速度、航向、实际俯仰与轨迹进度 |
| /px4_llm/status | std_msgs/msg/String | 输出 JSON 事件：planning/executing/completed/rejected/error/ack 等 |

PX4 话题沿用已有包：输入 offboard_control_mode、trajectory_setpoint、vehicle_command_v1；
输出 vehicle_local_position_v1、vehicle_status_v5、vehicle_land_detected、vehicle_command_ack_v2、vehicle_attitude。
可用 ROS `-r old_topic:=new_topic` 适配不同消息版本，消息定义本身也必须匹配固件。

- 起飞高度0.5–10米，单步移动最长10米，距起飞点水平半径20米；动作数组无固定步数上限。
- 前/右/下按每一步开始时飞机朝向计算；NED 为北/东/下。上升对应负 dz，右转为正角度。
- 悬停时间非负，无固定120秒上限；转向单步最多180度。位置与航向实际到达且速度降低后才进入下一步。
- 无指令时不自动解锁。起飞前先发送2秒20 Hz心跳，确认 Offboard 后请求解锁。
- API 在后台运行，失败不执行新计划；已有悬停持续。忙碌时拒绝新指令，不排队。
- 遥测超过2秒无更新、估计器重置、failsafe或动作超时会取消任务并请求降落。
- 离开 Offboard 后释放控制，不自动抢回模式或重新解锁。QGC 可切换模式接管。
- 起飞/解锁最长15秒；运动动作超时为轨迹预计时长+120秒，悬停为要求时长+120秒；降落最长90秒。ACK 会发布到状态话题；
  飞行是否成功以实际状态为准。降落超时需通过 QGC 接管。
- 不生成/执行任意 Python、shell 或底层电机命令；完整任务及每步实际目标均校验边界。
  边界校验不是物理围栏，也不替代 PX4 failsafe；参考速度不等于实际速度保证，PX4参数也会限速。
- 推理请求会向 DeepSeek 发送自然语言和局部位置/航向状态；未提供完整历史，跨指令请说清动作。

API 参考：https://api-docs.deepseek.com/guides/json_mode/

## 本地测试

```bash
cd /home/si/uav/learn/ws/src/px4_llm_control
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test -q
```

轨迹/规划测试不需要API或ROS；执行器测试需source ROS和工作区，但使用合成遥测，不建立DDS连接，也不发送飞行指令。
真实 API 和端到端飞行需要你自己的密钥及运行中的仿真。

## 连续轨迹与复杂任务（新增）

```bash
ros2 topic pub --once /px4_llm/command std_msgs/msg/String "{data: '起飞到2米，以起飞前的位置为中心，绕边长6米的正方形一圈，巡航速度3米每秒，机头始终朝向中心。保持高度，俯仰尽量水平，允许加减速和转弯时必要的倾斜；顶点允许减速。完成后悬停。'}"
```

无需手写分段坐标。完整模型输出例子见 `examples/square_plan.json`。
该任务没有要求降落，因此默认持续悬停；如需降落在data末尾明确加“最后降落”。

```bash
ros2 topic pub --once /px4_llm/command std_msgs/msg/String "{data: '起飞到2米，以起飞点为中心绕半径6米的圆两圈，速度2米每秒，机头持续朝中心，之后升到5米，悬停10秒并降落。'}"
ros2 topic echo /px4_llm/feedback
```

新增动作：

| 动作 | 参数 | 说明 |
|---|---|---|
| square | side, height, speed | 中心正方形，默认从东北角开始、顺时针，边平行北东轴 |
| circle | radius, height, speed | 解析圆周轨迹，平滑绕行 |
| polygon | sides, side, height, speed | 正多边形，边数3到64；更多边可用circle/path |
| path | points, speed | 任意多个相对起飞点的[北,东,高度]航点 |
| takeoff / move | 原参数，加可选speed | 起飞默认1m/s，最多2m/s；move默认1m/s |

几何动作可选：`center`=`takeoff`/`current`/相对起飞点的`[北,东]`；
`heading`=`center`/`tangent`/`hold`；`laps`为正整数（无固定圈数上限）；
`clockwise`布尔值；`rotation_deg`为顺时针旋转角。
path也支持heading和center，但center只支持takeoff或[北,东]。

控制器以20Hz生成时间参数化的位置、速度和加速度设定点，速度/加速度作为PX4位置控制前馈。
默认最大参考加速度2m/s²，默认最大航向参考角速度90°/s：

```bash
ros2 run px4_llm_control llm_control --ros-args \
  -p max_acceleration:=2.0 -p max_yaw_rate_deg:=90.0 -p max_tokens:=8192
```

- speed范围0.1到5m/s。正方形、polygon、path保持直边，在每个顶点减速到零后再加速，
  不承诺全程恒定3m/s；边太短也可能无法达到指定巡航速度。需要不停顿转弯应使用圆形或另行实现圆角路径。
- 首先以最多1m/s连接到几何路径起点，此段不计入绕行圈数。square边长6米时，顶点是中心偏移(±3,±3)。
- circle按向心加速度预算自动限速：参考速度最多sqrt(max_acceleration*radius/2)，
  切向加速度最多max_acceleration/2。实际使用速度见trajectory事件。
- heading=center每个控制周期根据飞机实际位置更新指向中心的航向，并限制航向变化速率；
  起始位置恰在中心时朝向没有定义，暂保持原航向。实际航向仍有飞控跟踪误差。
- 参考跟踪误差超过1.5米或航向滞后40°时暂停轨迹进度，保持心跳等待跟上；
  位置误差超过4米请求降落。暂停时会降低参考速度，实际仍由PX4完成制动。
- 姿态约束：可选顶层`attitude: {"pitch_deg":0,"strict":false}`仅表达水平飞行意图并输出警告；
  **不会直接设置或强制俯仰角**。多旋翼产生水平加速度需要倾斜，正方形拐角也不能瞬间改变速度方向。
  `strict:true`或非零指定俯仰会明确拒绝，避免输出实际上无法兑现的保证。
- feedback输出actual_speed_m_s、yaw_actual_deg、pitch_actual_deg等实测量，
  pitch_enforced始终为false；姿态遥测不可用时pitch_actual_deg为null。
- 没有固定动作步数/圈数上限，但一次请求仍受API上下文、max_tokens和计算机内存限制。
  大量绕行用laps表达，不在内存中展开每一圈。模型输出截断会拒绝整个计划，不执行半份任务。

参考：https://docs.px4.io/main/en/flight_modes/offboard （位置设定点与速度/加速度前馈）。
