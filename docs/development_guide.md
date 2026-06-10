# franka_mujoco Development Guide

`franka_mujoco` is the repository name. The reusable MuJoCo backend lives in `mujoco_simulation`. The ROS 2 `ros2_control` hardware plugin lives in `franka_hardware`.

## Package Boundaries

| Package | Responsibility |
| --- | --- |
| `mujoco_simulation` | MuJoCo backend runtime, device abstractions, official viewer integration, and RoboCasa MJCF generation helpers |
| `franka_hardware` | Standard `hardware_interface::SystemInterface` plugin that adapts `mujoco_simulation` to `ros2_control` |

Do not add ROS bridge nodes, custom simulation services, or robot-specific control logic under `mujoco_simulation`. Robot-specific bringup, controller configs, MoveIt, Nav2, and hardware-specific wrappers stay outside the backend library.

## Runtime Architecture

The control chain is:

`MoveIt 2 / application`
→ `FollowJointTrajectory`
→ `JointTrajectoryController`
→ `ros2_control`
→ `franka_hardware::FrankaHardwareInterface`
→ `mujoco_simulation::MuJoCoSimulation`

`MuJoCoSimulation` owns `mjModel`, `mjData`, the physics thread, viewer lifecycle, and the public joint / IMU access API. `franka_hardware` consumes only `mujoco_simulation/mujoco_simulation.hpp`; it must not reach into backend internals such as `HardwareManager`.

## Viewer

`mujoco_simulation` vendors the official MuJoCo `mujoco/simulate` source under:

```text
mujoco_simulation/src/viewer/simulate/
```

The vendored files are copied from the repository's `mujoco/simulate` directory and should not be edited for project-specific behavior. Custom logic belongs in wrapper code inside `mujoco_simulation`.

The internal CMake target is:

```text
mujoco_official_simulate
```

Runtime mode is selected with `render_mode`:

- `headless`: no viewer window is started.
- `viewer`: uses the official MuJoCo simulate UI integration.

`franka_hardware` currently supports:

- joint command/state interfaces
- IMU state interfaces
- camera topic publishing via `sensor_msgs/Image` and `sensor_msgs/CameraInfo`
- lidar topic publishing via `sensor_msgs/LaserScan`

It does not provide custom ROS services, `/clock`, pause/reset/step APIs, or a `controller_manager` host node.

Current constraint:

- camera publishing requires `render_mode:=viewer`, because the camera path still depends on MuJoCo's viewer render context.

## RoboCasa

RoboCasa Python helpers are installed as:

```python
from mujoco_simulation.robocasa import SceneConfig, SceneGenerator
```

This package generates and adapts MJCF XML. It does not run the MuJoCo simulation and does not provide ROS 2 runtime bindings.

The old import path is intentionally removed:

```python
from mujoco_ros2_bridge.robocasa import SceneGenerator  # unsupported
```

## Build And Test

Python-only validation:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 -m unittest discover -s tests
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 -m compileall -q mujoco_simulation tests
```

ROS 2 validation:

```bash
colcon build --packages-select mujoco_simulation franka_hardware
colcon test --packages-select mujoco_simulation --event-handlers console_direct+
```

## Development Rules

- Keep `mujoco_simulation` generic and model-agnostic.
- Keep `franka_hardware` as a standard `SystemInterface`, not a bridge node.
- Use standard `ros2_control` controllers instead of custom action/topic shims.
- Keep RoboCasa XML generation in `mujoco_simulation.robocasa`.
- Keep vendored MuJoCo simulate source synchronized by copying from `mujoco/simulate`; do not hand-edit it.
