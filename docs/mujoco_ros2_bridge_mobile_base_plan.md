# `mujoco_ros2_bridge` 通用移动底盘与传感器扩展方案

## 背景与目标

当前 `mujoco_ros2_bridge` 主要覆盖 joint-level 的 `ros2_control` 硬件绑定，适合固定底座机械臂，但对以下能力缺少系统支持：

- IMU
- Camera
- Lidar
- 通用 ROS 2 `mobile_base`

本次扩展的目标不是面向单一机器人型号，而是让 bridge 的内部数据模型与 ROS 2 常见移动机器人接口兼容：

- 坐标语义遵循 REP-105：`map -> odom -> base_link`
- 底盘结构对齐 `ros2_control` 常见控制器族：
  - `diff_drive`
  - `ackermann`
  - `tricycle`
  - `mecanum`
  - `omni`

`mobile_fr3_duo_v0_2` 只是验收样例，不是设计中心。

## 非目标

本轮不做以下能力：

- 不在 bridge 内重写官方底盘控制器
- 不实现 Nav2 完整链路
- 不实现通用 `cmd_vel -> wheel command` 运动学控制层
- 不实现 GPIO / `cartesian_velocity`
- 不实现 floating-base / `freejoint` 控制语义

## 通用架构

bridge 内部按三层组织：

### 1. Simulation Layer

`MuJoCoSimulation` 负责：

- `mjModel` / `mjData`
- physics thread
- official MuJoCo viewer
- `/clock`
- pause / reset / step

### 2. Hardware Binding Layer

`MujocoSystemInterface` 负责：

- joint binding
- IMU binding
- camera binding
- lidar binding
- mobile base binding

这一层只做通用硬件抽象，不承载机器人专用控制策略。

### 3. Device Worker Layer

独立 worker 负责异步设备任务：

- `MujocoCameras`
- `MujocoLidar`
- `ImuBinding`

其中：

- IMU 在 `read()` 周期内同步读取
- Camera / Lidar 通过后台线程采样与发布

## Mobile Base 设计

新增内部抽象：

```cpp
enum class MobileBaseType {
  None,
  DifferentialDrive,
  Ackermann,
  Tricycle,
  Mecanum,
  Omni,
  CustomJointGroup,
};
```

```cpp
struct MobileBaseBinding {
  MobileBaseType type;
  std::string base_frame_id;
  std::string odom_frame_id;
  std::string feedback_mode;
  std::vector<std::string> traction_joint_names;
  std::vector<std::string> steering_joint_names;
  std::vector<std::string> passive_joint_names;
};
```

```cpp
enum class JointRole {
  Manipulator,
  MobileTraction,
  MobileSteering,
  Passive,
};
```

设计约束：

- bridge 当前只做 joint-level hardware binding
- 不直接提供标准底盘控制器
- 内部 joint 组织方式要与 ROS 2 官方/通用移动底盘控制器兼容
- 未来若接 `diff_drive_controller` 或 steering controller，不需要推倒当前数据模型

## 配置约定

统一使用 `hardware_interface::HardwareInfo` 作为配置源。

### Mobile Base 参数

新增 hardware parameters：

- `mobile_base.type`
- `mobile_base.base_frame_id`
- `mobile_base.odom_frame_id`
- `mobile_base.feedback_mode`
- `mobile_base.traction_joints`
- `mobile_base.steering_joints`
- `mobile_base.passive_joints`

默认值：

- `mobile_base.type = none`
- `mobile_base.base_frame_id = base_link`
- `mobile_base.odom_frame_id = odom`
- `mobile_base.feedback_mode = position`

当前字符串列表参数采用逗号分隔，例如：

```text
mobile_base.traction_joints=left_wheel_joint,right_wheel_joint
```

### IMU 参数

通过 `sensor` 定义：

- `mujoco_type=imu`
- `mujoco_orientation_sensor`
- `mujoco_gyro_sensor`
- `mujoco_accel_sensor`

导出的 state interfaces：

- `orientation.x/y/z/w`
- `angular_velocity.x/y/z`
- `linear_acceleration.x/y/z`

### Camera 参数

通过 `sensor` 定义：

- `mujoco_type=camera`
- `mujoco_camera_name`

可选参数：

- `frame_name`
- `image_topic`
- `camera_info_topic`
- `depth_topic`
- `publish_rate`

默认值：

- `frame_name = <sensor_name>`
- `image_topic = /<sensor_name>/image_raw`
- `camera_info_topic = /<sensor_name>/camera_info`
- `depth_topic = /<sensor_name>/depth/image_raw`
- `publish_rate = 5.0`

### Lidar 参数

通过 `sensor` 定义：

- `mujoco_type=lidar`
- `mujoco_sensor_prefix`
- `frame_name`
- `angle_min`
- `angle_max`
- `angle_increment`
- `range_min`
- `range_max`

可选参数：

- `scan_topic`
- `publish_rate`

beam 命名规则：

- MuJoCo `rangefinder` 传感器名匹配 `<prefix>-<index>`
- `index` 从 0 开始
- 接受零填充和非零填充格式

## `mobile_fr3_duo_v0_2` 适配说明

`mobile_fr3_duo_v0_2` 作为通用方案实例，当前建议映射为：

- `tmrv0_2_joint_0`、`tmrv0_2_joint_2` -> steering joints
- `tmrv0_2_joint_1`、`tmrv0_2_joint_3` -> traction joints
- `rocker_arm_joint` -> passive joint
- `caster_*` joints -> passive joints
- `franka_spine_vertical_joint` -> passive joint

若 MuJoCo 模型没有显式满足 Ackermann 几何约束，则先将其归类为 `CustomJointGroup`，而不是过度声明为 `Ackermann`。

本轮不实现其 GPIO `cartesian_velocity`。

## 实现边界

- `MujocoSystemInterface` 先构建 mobile base binding，再绑定剩余 joints
- 支持同一系统内混合存在：
  - manipulator `position`
  - manipulator `velocity`
  - manipulator `effort`
  - base steering `position`
  - base traction `velocity`
  - passive state-only joints
- 没有 command interface 的 passive joints 允许无 actuator
- 需要 command interface 的 joints 若找不到 actuator，仍然视为配置错误

## 验收标准

满足以下条件即视为完成：

- bridge 内新增通用 `mobile_base` 抽象
- `mobile_fr3_duo_v0_2` 这类底盘 + 双臂混合系统可绑定
- IMU state interfaces 可用
- Camera / Lidar topics 可用
- step/reset/viewer 行为无回归
- 不引入机器人型号硬编码分支作为主设计路径
