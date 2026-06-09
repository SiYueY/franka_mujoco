"""RoboCasa integration helpers for mujoco_ros2_bridge.

This package treats RoboCasa as a kitchen scene generator. It creates a
RoboCasa / robosuite task scene, adapts the generated MJCF, replaces the
placeholder RoboCasa robot with the project robot MJCF, and returns a MuJoCo
model plus scene metadata.
"""

from mujoco_ros2_bridge.robocasa.scene_config import (
    SceneConfig,
    config_from_mapping,
    load_config,
)
from mujoco_ros2_bridge.robocasa.scene_data import (
    GeneratedScene,
    ObjectPlacement,
    SceneMetadata,
    SpawnPose,
)
from mujoco_ros2_bridge.robocasa.scene_generator import SceneGenerator

__all__ = [
    "GeneratedScene",
    "ObjectPlacement",
    "SceneConfig",
    "SceneGenerator",
    "SceneMetadata",
    "SpawnPose",
    "config_from_mapping",
    "load_config",
]
