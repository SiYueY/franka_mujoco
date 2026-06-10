from __future__ import annotations

from pathlib import Path
import re
import unittest


class RoboCasaImportsTest(unittest.TestCase):
    def test_package_imports(self) -> None:
        import mujoco_simulation.robocasa as robocasa

        for name in (
            "GeneratedScene",
            "ObjectPlacement",
            "SceneConfig",
            "SceneGenerator",
            "SceneMetadata",
            "SpawnPose",
            "config_from_mapping",
            "load_config",
        ):
            with self.subTest(name=name):
                self.assertTrue(hasattr(robocasa, name))

        for name in (
            "GeneratedRoboCasaScene",
            "RoboCasaObjectInfo",
            "RoboCasaSceneConfig",
            "RoboCasaSceneGenerator",
            "RoboCasaSceneInfo",
            "RobotSpawnPose",
            "load_scene_config",
            "scene_config_from_mapping",
        ):
            with self.subTest(name=name):
                self.assertFalse(hasattr(robocasa, name))

    def test_old_franka_mujoco_import_path_is_removed(self) -> None:
        with self.assertRaises(ModuleNotFoundError):
            __import__("franka_mujoco.robocasa")

    def test_old_mujoco_ros2_bridge_import_path_is_removed(self) -> None:
        source_root = (
            Path(__file__).parents[1]
            / "mujoco_simulation"
            / "robocasa"
        )

        offenders = []
        for path in source_root.glob("*.py"):
            if "robot_mujoco.robocasa" in path.read_text(encoding="utf-8"):
                offenders.append(path.name)

        self.assertEqual(offenders, [])

    def test_old_public_type_names_are_removed(self) -> None:
        source_root = (
            Path(__file__).parents[1]
            / "mujoco_simulation"
            / "robocasa"
        )
        old_names = (
            "GeneratedRoboCasaScene",
            "RoboCasaObjectInfo",
            "RoboCasaSceneConfig",
            "RoboCasaSceneGenerator",
            "RoboCasaSceneInfo",
            "RobotSpawnPose",
        )

        offenders = []
        for path in source_root.glob("*.py"):
            if path.name == "exceptions.py":
                continue

            text = path.read_text(encoding="utf-8")
            for old_name in old_names:
                if re.search(rf"\b{old_name}\b(?!Error)", text):
                    offenders.append(f"{path.name}:{old_name}")

        self.assertEqual(offenders, [])


if __name__ == "__main__":
    unittest.main()
