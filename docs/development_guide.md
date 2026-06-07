# franka_mujoco Development Guide

`franka_mujoco` is the repository name. The reusable MuJoCo/ROS 2 integration code lives in the ROS 2 package `mujoco_ros2_bridge`.

## Package Boundaries

| Package | Responsibility |
| --- | --- |
| `mujoco_ros2_bridge` | Generic MuJoCo runtime, ROS 2 node, `ros2_control` hardware plugin, official MuJoCo viewer integration, and RoboCasa MJCF generation helpers |
| `mujoco_ros2_bridge_msgs` | Lightweight service interfaces shared by runtime nodes and tests |

Do not add MuJoCo runtime code under a `franka_mujoco` Python package. Robot-specific bringup, controller configs, MoveIt, Nav2, or Franka-specific wrappers should stay outside the generic bridge.

## Runtime Architecture

The C++ runtime follows the `mujoco_ros2_control` architecture:

- `MuJoCoSimulation` owns `mjModel`, `mjData`, the physics thread, pause/reset/step behavior, and `/clock`.
- `MujocoSystemInterface` implements `hardware_interface::SystemInterface` and maps ROS 2 control joint interfaces to MuJoCo joints and actuators.
- `mujoco_ros2_bridge_node` runs `controller_manager` and the control update loop.
- Services are provided under `/mujoco/*` through `mujoco_ros2_bridge_msgs`.

The bridge is generic. It should not assume a Franka, Stretch, Unitree, or RoboCasa robot model.

## Viewer

`mujoco_ros2_bridge` vendors the official MuJoCo `mujoco/simulate` source under:

```text
mujoco_ros2_bridge/third_party/mujoco_simulate/
```

The vendored files are copied from the repository's `mujoco/simulate` directory and should not be edited for project-specific behavior. Custom logic belongs in wrapper code inside `mujoco_ros2_bridge`.

The internal CMake target is:

```text
mujoco_official_simulate
```

Runtime mode is selected with `render_mode`:

- `headless`: no viewer window is started.
- `viewer`: uses the official MuJoCo simulate UI integration.

## ROS 2 Interfaces

Service definitions live in `mujoco_ros2_bridge_msgs`:

| Service | Purpose |
| --- | --- |
| `SetPause.srv` | Pause or resume physics |
| `ResetWorld.srv` | Reset the model, optionally to a keyframe |
| `StepSimulation.srv` | Advance a fixed number of steps |

Runtime service names:

```text
/mujoco/set_pause
/mujoco/reset_world
/mujoco/step_simulation
```

## RoboCasa

RoboCasa Python helpers are installed as:

```python
from mujoco_ros2_bridge.robocasa import SceneConfig, SceneGenerator
```

This package generates and adapts MJCF XML. It does not run the MuJoCo simulation and does not provide ROS 2 runtime bindings.

The old import path is intentionally removed:

```python
from franka_mujoco.robocasa import SceneGenerator  # unsupported
```

## Build And Test

Python-only validation:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=mujoco_ros2_bridge python3 -m unittest discover -s tests
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=mujoco_ros2_bridge python3 -m compileall -q mujoco_ros2_bridge/mujoco_ros2_bridge tests
```

ROS 2 validation:

```bash
colcon build --packages-select mujoco_ros2_bridge mujoco_ros2_bridge_msgs
colcon test --packages-select mujoco_ros2_bridge mujoco_ros2_bridge_msgs --event-handlers console_direct+
```

## Development Rules

- Keep runtime code generic and model-agnostic.
- Use standard `ros2_control` controllers instead of robot-specific action/topic shims in the bridge.
- Keep message definitions in `mujoco_ros2_bridge_msgs`.
- Keep RoboCasa XML generation in `mujoco_ros2_bridge.robocasa`.
- Keep vendored MuJoCo simulate source synchronized by copying from `mujoco/simulate`; do not hand-edit it.
