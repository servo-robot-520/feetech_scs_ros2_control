# feetech_scs_ros2_control

[English](README.md) | [简体中文](README.zh-CN.md)

`ros2_control` 硬件接口，面向飞特 **SCS** 系列总线舵机，支持 **SC215** 与 **SC225**（电位器、10 位编码器、TTL 半双工总线）。

驱动通过 POSIX 串口直接实现 SCSCL 协议：

- 8N1 原始串口，支持任意波特率（默认 1 Mbaud）；
- 0..1023 tick 位置，出厂中位为 511；
- 16 位寄存器高字节在前；
- READ / WRITE / SYNC WRITE、扭矩与 EEPROM 锁控制；
- 逐关节顺序读取反馈（SCS 协议手册未定义 SYNC READ）。

## 构建

```bash
colcon build --packages-select feetech_scs_ros2_control
colcon test  --packages-select feetech_scs_ros2_control
```

### 普通 CMake / CLion

在已 source ROS 2 环境的情况下，本包也可以作为普通 CMake 工程配置。独立构建时需要先构建/安装 `common_cpp`（例如 `colcon build --packages-select common_cpp`）：

```bash
source /opt/ros/humble/setup.bash
cmake -S src/feetech_scs_ros2_control -B build-pkg \
  -DCMAKE_CXX_COMPILER=g++-13 \
  -DCMAKE_PREFIX_PATH="$PWD/install/common_cpp:/opt/ros/humble"
cmake --build build-pkg -j
```

使用 CLion 时直接打开工作区根目录，选择 GCC 13 并加载 ROS 2 环境。根 CMake 会设置`ROS2_WS_SOURCE_BUILD=ON`，通过 `add_subdirectory` 聚合各功能包；本包此时链接构建树中的 `common_cpp` target，而不是已安装版本。若还需要构建测试目标，请在 CLion 的 CMake选项中添加 `-DBUILD_TESTING=ON`。

依赖：ROS 2 Humble（`rclcpp`、`rclcpp_lifecycle`、`hardware_interface`、`pluginlib`）、`common_cpp` 与 `yaml-cpp`。`common_cpp` 需要支持 C++23 的标准库（GCC 13+），因此请使用 GCC 13 构建工作区。串口设备需要可访问，通常：

```bash
sudo usermod -aG dialout "$USER"
```

## ros2_control 配置

```xml
<ros2_control name="feetech_scs" type="system">
  <hardware>
    <plugin>feetech_scs_ros2_control/FeetechScsHardwareInterface</plugin>
    <param name="usb_port">/dev/ttyUSB0</param>
    <param name="baud_rate">1000000</param>
    <param name="joint_config_file">$(find my_robot_bringup)/config/scs_joints.yaml</param>
    <param name="calibration_file">~/.config/duck_ros2/calibration.yaml</param>
  </hardware>

  <joint name="shoulder_pan_joint">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="load"/>
    <state_interface name="voltage"/>
    <state_interface name="temperature"/>
    <state_interface name="raw_tick"/>
    <state_interface name="position_lower_limit"/>
    <state_interface name="position_upper_limit"/>
    <param name="id">1</param>
    <param name="range_min_tick">100</param>
    <param name="range_max_tick">923</param>
    <param name="p_coefficient">16</param>
    <param name="d_coefficient">16</param>
  </joint>
</ros2_control>
```

只有 URDF 中声明的状态接口才会导出。支持的状态接口：`position`、`velocity`、`load`、`voltage`、`temperature`、`current`、`raw_tick`、`position_lower_limit`、`position_upper_limit`。支持的命令接口：`position`（rad）与 `velocity`（单位 tick/s，直接写入 GoalSpeed 寄存器；同一周期的速度命令会覆盖该关节的 `speed_tick`）。

其中 `position_lower_limit` / `position_upper_limit` 为关节**行程上下限**（rad），由 `min_tick/max_tick` 换算并扣除零点偏移：`limit_rad = tick_to_radians(tick, range_deg) - offset`。它们供上层控制器（如 `duck_control`）读取并做软限位钳位，不参与驱动自身运动。

### 硬件参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `usb_port` | — | 串口设备。可在此处或关节配置文件的 `usb_port` 中提供。 |
| `baud_rate` | `1000000` | 总线波特率。关节配置文件会覆盖该值。 |
| `joint_config_file` | — | 可选 YAML 文件，包含总线设置与逐关节参数。 |
| `calibration_file` | — | 可选 YAML 文件，包含逐关节零点 `calibration_tick`。缺省则不偏移。见[标定文件](#标定文件)。 |
| `response_timeout_ms` | `10` | 单个舵机响应超时。 |
| `range_deg` | `300` | 舵机在 0..1023 tick 范围内的机械行程。关节级 `range_deg` 可覆盖。 |
| `default_speed_tick` | `1000` | 关节未设置 `speed_tick` 时的命令速度，单位 tick/s。 |
| `strict_model_check` | `false` | 拒绝型号不是 SC215/SC225 的舵机。 |
| `write_angle_limits_on_start` | `false` | 为 `true` 时，启动配置阶段把各关节的 `range_min_tick/range_max_tick` 检查后写入舵机 EEPROM 角度限位寄存器（读当前值相同则跳过）。默认 `false` 不写 EEPROM，仅用于软件钳位与状态导出，以保护 EEPROM 擦写寿命。 |

串口设备与波特率也可以通过关节配置文件提供（顶层 `usb_port` / `baud_rate`）；配置文件优先于 URDF 参数。

### 启动校验

`on_configure` 只访问 URDF / 关节配置文件中定义的舵机 id，不再扫描整条总线。对每个已配置关节：

1. ping 该 id；舵机无应答时输出错误日志并中止启动；
2. 读取并打印固件主/次版本（寄存器 0/1）、舵机主/次版本（寄存器 3/4）及其推导出的型号；
3. 打印该关节配置的全部参数；
4. 逐项读取寄存器当前值，不一致时打印 `current=... configured=... -> update` 并写入新值。

总线使用配置文件中设置的波特率，因此不再打印波特率。

### 日志

诊断信息使用 `common_cpp` 日志后端（spdlog 控制台/文件 sink 与 ROS 桥接）。它是进程级后端，在 `on_init` 时从硬件参数初始化：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `log.log_level` | `info` | `trace`、`debug`、`info`、`warn`、`error`、`critical`、`off`。 |
| `log.log_path` | `./log` | 日志目录，文件名为 `log_<name>_<timestamp>.log`。 |
| `log.async` | `true` | 使用 spdlog 异步队列。 |
| `log.logger_name` | `feetech_scs_ros2_control` | 日志器名称，同时用于 ROS 适配器。 |
| `log.node_name` | `feetech_scs_ros2_control` | 日志行中显示的前缀。 |
| `log.filter.black` | 空 | 逗号分隔的过滤规则（`start-`、`end-`、`inc-`、`equal-`）。 |

### 关节参数

| 参数 | 寄存器 | 范围 | 说明 |
|---|---|---|---|
| `id` | — | 0..253 | **必填**。总线上必须唯一。 |
| `range_min_tick` | 9（2 字节） | 0..1023 | 命令/EEPROM 下限，单位 tick。 |
| `range_max_tick` | 11（2 字节） | 0..1023 | 命令/EEPROM 上限，单位 tick。 |
| `range_deg` | — | 0.1..360 | 用于 tick/弧度换算的机械行程。 |
| `speed_tick` | — | 1..1000 | 命令速度，单位 tick/s。 |
| `reserved_address` | 7 | 0..255 | 预留地址（新固件命名；旧资料曾标注返回延时）。 |
| `p_coefficient` | 21 | 0..254 | 位置环 P 增益。 |
| `d_coefficient` | 22 | 0..254 | 位置环 D 增益。 |
| `cw_dead_tick` | 26 | 0..32 | 顺时针死区，单位 tick。 |
| `ccw_dead_tick_tick` | 27 | 0..32 | 逆时针死区，单位 tick。 |
| `max_temperature` | 13 | 0..100 | 最高工作温度，单位 °C。 |
| `max_input_voltage` | 14 | 0..254 | 最高输入电压，单位 0.1 V（90 = 9.0 V）。 |
| `min_input_voltage` | 15 | 0..254 | 最低输入电压，单位 0.1 V（45 = 4.5 V）。 |
| `max_torque` | 16（2 字节） | 0..1000 | 最大扭矩，1000 = 100% 堵转扭矩。 |
| `unloading_condition` | 19 | 0..254 | 卸载条件位域（满足哪些故障时释放扭矩）。 |
| `led_alarm_condition` | 20 | 0..254 | LED 报警条件位域。 |
| `protection_torque_percent` | 37 | 0..100 | 过载保护后的输出扭矩，单位 %。 |
| `protection_time_ms` | 38 | 0..254 | 过载保持时间，单位 40 ms。 |
| `overload_torque_percent` | 39 | 0..100 | 过载阈值，单位为最大扭矩的百分比。 |

`min_input_voltage` 不得大于 `max_input_voltage`。

EEPROM 参数仅在舵机当前值不同时写入，写入前关闭扭矩并解锁 EEPROM。STS 舵机存在但 SCS电位器舵机不存在的参数会直接报错，而不是静默忽略：`homing_offset`、`offset`、`i_coefficient`、`acceleration`、`protection_current`、`max_torque_limit`。

## 协议库

`feetech_scs_driver` 库不依赖 ROS，可供独立工具链接。除传输层（`ScsSerialPort`）与 READ / WRITE / SYNC WRITE 客户端（`ScsProtocol`）外，还提供：

- `ScsProtocol::scan()` —— 总线扫描（PING 遍历）。
- `ScsProtocol::read_identity()` —— 固件版本与型号读取。
- `ScsProtocol::read_state()` / `decode_feedback()` —— 反馈块解码（位置、速度、负载、电压、温度、电流、移动标志）。
- `ScsProtocol::write_word()` / `write_eeprom()` —— 16 位寄存器写入与 EEPROM安全流程（关扭矩 → 解锁 → 写入 → 回锁）。
- `scs_params.hpp` —— 参数描述表（地址、字宽、范围、单位、存储区），与插件的配置校验共用同一份元数据。

## 关节配置文件

```yaml
usb_port: /dev/ttyUSB0
baud_rate: 1000000

joints:
  shoulder_pan_joint:
    id: 1
    p_coefficient: 16
    d_coefficient: 16
    range_min_tick: 100
    range_max_tick: 923
    range_deg: 300
    speed_tick: 800
    max_temperature: 70
    max_input_voltage: 100
    min_input_voltage: 50
    unloading_condition: 32
    led_alarm_condition: 37
    protection_torque_percent: 20
    protection_time_ms: 100
    overload_torque_percent: 80
  elbow_joint:
    id: 2
    range_min_tick: 20
    range_max_tick: 1003
```

条目按**舵机 id** 匹配 URDF 关节；YAML 中的关节名只是标签。相同 id 下 YAML 值覆盖URDF `<param>` 值。

## 角度约定

位置与速度使用配置的机械行程：

```
radians = (ticks - 511) * range_deg * pi / 180 / 1024
```

`range_deg` 默认 300（SC215）。SC225 有 180° 与 300° 两种规格，请按实际舵机为该关节设置 `range_deg`。`range_min_tick`/`range_max_tick` 为原始 tick 值（0..1023），直接写入舵机 EEPROM 角度限制寄存器，并用于命令钳位；上位机或内存表读到的 tick 可直接填入配置，无需换算。命令会四舍五入到 tick，并钳位到 `range_min_tick..range_max_tick` 后发送。

## 标定文件

可选参数 `calibration_file` 设置每个关节的零点。它由标定流程写入（ros2_control 仅在启动时读取，从不写回）：

```yaml
version: 1
joints:
  shoulder_pan_joint:
    calibration_tick: 531
  elbow_joint:
    calibration_tick: 508
```

`calibration_tick` 是机器人保持参考/零位姿态时观测到的原始计数值（0..1023）。启动时驱动计算 `offset = tick_to_radians(calibration_tick, range_deg)`，并对称地应用：

- 状态：`position_rad = tick_to_radians(raw_tick) - offset`
- 命令：`goal_tick = radians_to_tick(command_rad + offset)`

`raw_tick` 状态接口始终上报未经修正的原始计数，因此标定服务可直接读取当前 tick而不受偏移影响。tick ↔ 弧度常量（中位 511、1024 步）保留在驱动层内部；标定文件只存原始 tick。未出现在文件中的关节偏移为零。匹配按关节**名称**。

## 状态接口

| 接口 | 单位 | 来源 |
|---|---|---|
| `position` | rad | 当前位置减去零点偏移，标定 tick 处 = 0 rad |
| `raw_tick` | tick | 原始位置计数（0..1023），不施加偏移 |
| `velocity` | rad/s | 当前速度（符号位 15），单位 tick/s |
| `load` | -1..1 | 当前负载（符号位 10），1000 = 100%；每周期读取 |
| `voltage` | V | 当前电压，单位 0.1 V |
| `temperature` | °C | 当前温度 |
| `current` | A | 当前电流（0x45），单位 6.5 mA |

## 生命周期

- `on_init`：解析并校验硬件与关节配置（含 EEPROM 参数取值范围），不访问总线。
- `on_configure`：打开总线，校验已配置的舵机，并写入显式配置的 EEPROM 参数（关闭扭矩、解锁 EEPROM，写完后再上锁）。
- `on_activate`：读取当前位置，把命令值锁存到当前位置，然后为带 position 命令接口的关节使能扭矩。
- `read`：每个关节在 `0x38` 起始处顺序读取 15 字节（`0x38..0x46`），包含负载与电流。
- `write`：对 position 关节在 `0x2A`（GoalPosition）发送一次 SYNC WRITE，并对velocity 关节 / 速度覆盖在 `0x2E`（GoalSpeed）再发送一次 SYNC WRITE。
- `on_deactivate`：关闭扭矩。
- `on_cleanup` / `on_shutdown`：必要时关闭扭矩并关闭串口。

舵机在 `response_timeout_ms` 内未应答时，`read`/`write` 返回 `ERROR`。

## 限制

- 不支持 SYNC READ：反馈逐关节顺序读取，每周期每个关节一次事务。
- SCS 电位器舵机没有 EEPROM 回中偏移、I 项与加速度寄存器，因此这些 STS 参数不可用。
- 舵机必须对 WRITE 指令应答（`Response Level` 寄存器 8 = 1，出厂默认）。
- 已通过协议级 pty 模拟器与实机 SC225 验证；SC215 仍需实机验证。

## 示例（urdf / config / launch）

包内提供一个两关节的最小示例，便于在没有整机的情况下起一条总线：

```
urdf/feetech_scs_ros2_control.urdf.xacro   # 可复用宏：feetech_scs_joint / feetech_scs_hardware
urdf/feetech_scs_example.urdf.xacro        # 自包含的两关节示例机器人
config/feetech_example_joints.yaml         # joint_config_file 示例（总线 + 按 id 的参数）
config/feetech_example_calibration.yaml    # calibration_file 示例（按关节名的零位 tick）
config/feetech_controllers.yaml            # joint_state_broadcaster + joint_trajectory_controller
launch/feetech_scs_example.launch.py       # robot_state_publisher + ros2_control_node + spawner
```

快速起控（SCS 总线在 `/dev/ttyUSB0`）：

```bash
source install/setup.bash   # 或本工程提供的环境脚本
ros2 launch feetech_scs_ros2_control feetech_scs_example.launch.py \
    usb_port:=/dev/ttyUSB0 baud_rate:=1000000
```

launch 会拉起 `joint_state_broadcaster` 与 `joint_trajectory_controller`，控制`shoulder_pan_joint` / `elbow_joint`，通过
`/joint_trajectory_controller/follow_joint_trajectory` 下发 `trajectory_msgs/msg/JointTrajectory` 目标。

`joint_config_file` 按**舵机 id** 与 URDF 关节匹配（每条必须含 `id`）；`calibration_file`按**关节名**匹配，记录零位姿态读到的原始 tick（`offset=(calibration_tick-511)*range_deg*pi/180/1024`）。两个示例文件都是模板，请替换为实测值。

> velocity【命令】接口单位为 **rad/s**（与状态接口一致），驱动内部换算成 GoalSpeed(tick/s)。
> 仅在“按位置移动时覆盖速度”或纯速度关节时需要；常规位置控制/策略驱动不导出
> （`cmd_velocity=false`）。

### 在自己的机器人中复用宏

先 include `feetech_scs_ros2_control.urdf.xacro`，再在自定义的`<ros2_control type="system">` 内用 `feetech_scs_hardware` 生成硬件块，并为每个舵机写一个 `feetech_scs_joint`：

```xml
<xacro:include filename="$(find feetech_scs_ros2_control)/urdf/feetech_scs_ros2_control.urdf.xacro"/>
<ros2_control name="feetech_scs" type="system">
  <xacro:feetech_scs_hardware usb_port="/dev/ttyUSB0" baud_rate="1000000"
                              joint_config_file="$(find my_robot)/config/scs_joints.yaml"/>
  <xacro:feetech_scs_joint name="servo_a" id="1" range_deg="300"/>
  <xacro:feetech_scs_joint name="servo_b" id="2" range_deg="300"/>
</ros2_control>
```

`microduck_rev` 工程正是这样在 `duck_bringup/urdf/duck_ros2.urdf.xacro` 中接入15 个舵机的。`joint_config_file` 中的 `joints.<name>` 只是标签，实际按 `id`与 URDF 关节匹配。
