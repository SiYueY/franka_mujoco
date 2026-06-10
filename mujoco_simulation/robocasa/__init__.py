"""RoboCasa integration helpers for ``mujoco_simulation``.

This package treats RoboCasa as a kitchen scene generator. It creates a
RoboCasa / robosuite task scene, adapts the generated MJCF, replaces the
placeholder RoboCasa robot with the project robot MJCF, and returns a MuJoCo
model plus scene metadata.
"""

from typing import Any

from mujoco_simulation.robocasa.scene_config import (
    SceneConfig,
    config_from_mapping,
    load_config,
)
from mujoco_simulation.robocasa.scene_data import (
    GeneratedScene,
    ObjectPlacement,
    SceneMetadata,
    SpawnPose,
)

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


def __getattr__(name: str) -> Any:
    """Lazily import heavy helpers."""

    if name == "SceneGenerator":
        from mujoco_simulation.robocasa.scene_generator import SceneGenerator

        return SceneGenerator

    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
