# feetech_scs_ros2_control

[English](README.md) | [简体中文](README.zh-CN.md)

`ros2_control` hardware interface for Feetech **SCS** series bus servos, targeting
the **SC215** and **SC225** (potentiometer, 10-bit encoder, TTL half-duplex bus).

The driver speaks the SCSCL protocol directly over a POSIX serial port:

- 8N1 raw serial, arbitrary baud rates (1 Mbaud by default),
- 0..1023 tick positions with the factory midpoint at 511,
- high-byte-first 16-bit register encoding,
- READ / WRITE / SYNC WRITE, torque and EEPROM lock control,
- sequential feedback reads (the SCS protocol manual does not define SYNC READ).

## Build

```bash
colcon build --packages-select feetech_scs_ros2_control
colcon test  --packages-select feetech_scs_ros2_control
```

### Plain CMake / CLion

The package also configures as a normal CMake project when the ROS 2 environment
is sourced. For a standalone build, `common_cpp` must be built/installed first
(for example with `colcon build --packages-select common_cpp`):

```bash
source /opt/ros/humble/setup.bash
cmake -S src/feetech_scs_ros2_control -B build-pkg \
  -DCMAKE_CXX_COMPILER=g++-13 \
  -DCMAKE_PREFIX_PATH="$PWD/install/common_cpp:/opt/ros/humble"
cmake --build build-pkg -j
```

For CLion, open the workspace root, select GCC 13 and the sourced ROS 2
environment. The root CMake sets `ROS2_WS_SOURCE_BUILD=ON` and aggregates the
workspace packages with `add_subdirectory`; this package then links the in-tree
`common_cpp` targets instead of the installed ones. Add `-DBUILD_TESTING=ON` to
the CLion CMake options if the test targets should be built as well.

Dependencies: ROS 2 Humble (`rclcpp`, `rclcpp_lifecycle`, `hardware_interface`,
`pluginlib`), `common_cpp` and `yaml-cpp`. `common_cpp` requires a C++23-capable
standard library (GCC 13+), so build the workspace with GCC 13. The serial device
must be accessible, typically:

```bash
sudo usermod -aG dialout "$USER"
```

## ros2_control configuration

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
    <param name="id">1</param>
    <param name="range_min_tick">100</param>
    <param name="range_max_tick">923</param>
    <param name="p_coefficient">16</param>
    <param name="d_coefficient">16</param>
  </joint>
</ros2_control>
```

Only state interfaces declared in the URDF are exported. Supported state
interfaces: `position`, `velocity`, `load`, `voltage`, `temperature`, `current`,
`raw_tick`. Supported command interfaces: `position` (rad) and `velocity`
(tick/s, written to the GoalSpeed register; a velocity command overrides the
joint's `speed_tick` for that cycle).

### Hardware parameters

| Parameter | Default | Description |
|---|---|---|
| `usb_port` | — | Serial device. Required either here or as `usb_port` in the joint config file. |
| `baud_rate` | `1000000` | Bus baud rate. The joint config file overrides this value. |
| `joint_config_file` | — | Optional YAML file with bus settings and per-joint parameters. |
| `calibration_file` | — | Optional YAML file with per-joint zero-pose `calibration_tick`. Missing → no offset. See [Calibration file](#calibration-file). |
| `response_timeout_ms` | `10` | Timeout for one servo response. |
| `range_deg` | `300` | Mechanical travel of the servo across 0..1023 ticks. Per-joint `range_deg` overrides it. |
| `default_speed_tick` | `1000` | Command speed in ticks/s when a joint does not set `speed_tick`. |
| `strict_model_check` | `false` | Reject servos whose model is not SC215/SC225. |

The serial device and baud rate may also be provided by the joint config file
(top-level `usb_port` / `baud_rate`); the config file takes precedence over the
URDF parameters.

### Startup verification

`on_configure` only contacts the servo ids configured in the URDF / joint config
file; there is no full-bus scan. For each configured joint it:

1. pings the id and aborts initialization with an error log if the servo does not
   answer;
2. reads and prints the firmware major/minor version (registers 0/1), the servo
   major/minor version (registers 3/4) and the model derived from them;
3. prints every configured parameter for the joint;
4. compares each register-backed parameter with the servo value and logs
   `current=... configured=... -> update` before writing a different value.

The configured baud rate is the one used on the bus, so no baud rate is printed.

### Logging

Diagnostics use the `common_cpp` logging backend (spdlog console/file sinks with
the ROS bridge). It is a process-global backend, initialized from the hardware
parameters on `on_init`:

| Parameter | Default | Description |
|---|---|---|
| `log.log_level` | `info` | `trace`, `debug`, `info`, `warn`, `error`, `critical`, `off`. |
| `log.log_path` | `./log` | Directory for `log_<name>_<timestamp>.log`. |
| `log.async` | `true` | Asynchronous spdlog queue. |
| `log.logger_name` | `feetech_scs_ros2_control` | Logger name, also used by the ROS adapter. |
| `log.node_name` | `feetech_scs_ros2_control` | Prefix shown in log lines. |
| `log.filter.black` | empty | Comma-separated filter rules (`start-`, `end-`, `inc-`, `equal-`). |

### Joint parameters

| Parameter | Register | Range | Notes |
|---|---|---|---|
| `id` | — | 0..253 | **Required**. Must be unique on the bus. |
| `range_min_tick` | 9 (2 B) | 0..1023 | Lower command/EEPROM limit, in ticks. |
| `range_max_tick` | 11 (2 B) | 0..1023 | Upper command/EEPROM limit, in ticks. |
| `range_deg` | — | 0.1..360 | Mechanical travel used for the tick/radian conversion. |
| `speed_tick` | — | 1..1000 | Command speed in ticks/s. |
| `reserved_address` | 7 | 0..255 | Reserved address (newer firmware; older docs label it return delay). |
| `p_coefficient` | 21 | 0..254 | Position loop P gain. |
| `d_coefficient` | 22 | 0..254 | Position loop D gain. |
| `cw_dead_tick` | 26 | 0..32 | Clockwise dead zone, ticks. |
| `ccw_dead_tick_tick` | 27 | 0..32 | Counter-clockwise dead zone, ticks. |
| `max_temperature` | 13 | 0..100 | Maximum operating temperature, °C. |
| `max_input_voltage` | 14 | 0..254 | Maximum input voltage, 0.1 V units (90 = 9.0 V). |
| `min_input_voltage` | 15 | 0..254 | Minimum input voltage, 0.1 V units (45 = 4.5 V). |
| `max_torque` | 16 (2 B) | 0..1000 | Maximum torque, 1000 = 100 % stall torque. |
| `unloading_condition` | 19 | 0..254 | Unloading-condition bit field (which faults release torque). |
| `led_alarm_condition` | 20 | 0..254 | LED-alarm condition bit field. |
| `protection_torque_percent` | 37 | 0..100 | Torque output after overload protection, percent. |
| `protection_time_ms` | 38 | 0..254 | Overload hold time, 40 ms units. |
| `overload_torque_percent` | 39 | 0..100 | Overload threshold, percent of maximum torque. |

`min_input_voltage` must not exceed `max_input_voltage`.

EEPROM parameters are written only when the servo value differs, with torque
disabled and the EEPROM unlocked. Parameters that exist on STS servos but not on
SCS potentiometer servos are rejected with an error instead of being silently
ignored: `homing_offset`, `offset`, `i_coefficient`, `acceleration`,
`protection_current`, `max_torque_limit`.

## Protocol library

The `feetech_scs_driver` library is independent of ROS and can be linked by
standalone tools. In addition to the transport (`ScsSerialPort`) and the
READ / WRITE / SYNC WRITE client (`ScsProtocol`) it provides:

- `ScsProtocol::scan()` — ping sweep to discover online servos on the bus.
- `ScsProtocol::read_identity()` — firmware version and model.
- `ScsProtocol::read_state()` / `decode_feedback()` — decoded feedback block
  (position, velocity, load, voltage, temperature, current, moving flag).
- `ScsProtocol::write_word()` / `write_eeprom()` — word writes and the safe
  EEPROM flow (torque off, unlock, write, lock).
- `scs_params.hpp` — the parameter descriptor table (address, width, range,
  unit, storage area), shared with the plugin's configuration validation.

## Joint config file

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

Entries are matched to URDF joints by **servo id**; the YAML joint name is only
a label. YAML values override URDF `<param>` values for the same id.

## Angle convention

Positions and velocities use the configured mechanical range:

```
radians = (ticks - 511) * range_deg * pi / 180 / 1024
```

`range_deg` defaults to 300 (SC215). SC225 has 180° and 300° variants, so set
`range_deg` for that joint to match the actual servo. `range_min_tick`/`range_max_tick` are
raw tick values (0..1023), written directly to the servo EEPROM angle-limit
registers and used for command clamping. This lets values read back from the
official tool or memory table be copied into the configuration without
conversion. Commands are rounded to ticks and clamped to `range_min_tick..range_max_tick`
before being sent.

## Calibration file

The optional `calibration_file` sets each joint's zero pose. It is written by
the calibration pipeline (ros2_control reads it at startup; it never writes it):

```yaml
version: 1
joints:
  shoulder_pan_joint:
    calibration_tick: 531
  elbow_joint:
    calibration_tick: 508
```

`calibration_tick` is the raw counter value (0..1023) observed when the robot is
held at its reference/zero pose. At startup the driver computes
`offset = tick_to_radians(calibration_tick, range_deg)` and applies it
symmetrically:

- state: `position_rad = tick_to_radians(raw_tick) - offset`
- command: `goal_tick = radians_to_tick(command_rad + offset)`

The `raw_tick` state interface always reports the uncorrected counter, so the
calibration service can discover the current tick without the offset. Tick ↔
radian constants (midpoint 511, 1024 steps) stay inside this driver; the
calibration file stores only raw ticks. Joints absent from the file keep zero
offset. Matching is by joint **name**.

## State interfaces

| Interface | Unit | Source |
|---|---|---|
| `position` | rad | present position minus zero-pose offset, tick at calibration = 0 rad |
| `raw_tick` | tick | raw present-position counter (0..1023), no offset applied |
| `velocity` | rad/s | present speed (sign bit 15), ticks/s |
| `load` | -1..1 | present load (sign bit 10), 1000 = 100 %; read every cycle |
| `voltage` | V | present voltage, 0.1 V units |
| `temperature` | °C | present temperature |
| `current` | A | present current (0x45), 6.5 mA units |

## Lifecycle

- `on_init`: parses and validates the hardware/joint configuration, including
  EEPROM parameter ranges; it does not access the bus.
- `on_configure`: opens the bus, verifies the configured servos and writes
  explicit EEPROM settings (torque off, EEPROM unlocked, then locked again).
- `on_activate`: reads current positions, latches commands to them, then enables
  torque on joints with a position command interface.
- `read`: one sequential 15-byte read per joint at `0x38` (`0x38..0x46`), including load and current.
- `write`: one SYNC WRITE at `0x2A` (GoalPosition) for position joints, plus a
  SYNC WRITE at `0x2E` (GoalSpeed) for velocity joints / speed overrides.
- `on_deactivate`: disables torque.
- `on_cleanup` / `on_shutdown`: disable torque if needed and close the bus.

A servo that does not answer within `response_timeout_ms` makes `read`/`write`
return `ERROR`.

## Limitations

- No SYNC READ: feedback is read sequentially, one transaction per joint per
  cycle.
- SCS potentiometer servos have no EEPROM homing offset, no I term and no
  acceleration register, so those STS-oriented parameters are unavailable.
- The servo must answer WRITE commands (`Response Level` register 8 = 1, the
  factory default).
- Tested against a protocol-faithful pty simulator and a physical SC225;
  SC215 hardware validation is still required.
