from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

from mujoco_simulation.robocasa.exceptions import RoboCasaSceneConfigError
from mujoco_simulation.robocasa.scene_config import config_from_mapping


class SceneConfigTest(unittest.TestCase):
    def test_parses_mapping_defaults_and_relative_paths(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            base_dir = Path(tmpdir)
            robot_xml = base_dir / "robot.xml"
            robot_xml.write_text("<mujoco/>", encoding="utf-8")

            config = config_from_mapping(
                {"robot": {"xml_path": "robot.xml"}},
                base_dir=base_dir,
            )
            config.validate()

        self.assertEqual(config.task_name, "PickPlaceCounterToCabinet")
        self.assertEqual(config.layout_id, 2)
        self.assertEqual(config.style_id, 2)
        self.assertEqual(config.robot_xml_path, str(robot_xml.resolve()))

    def test_non_mapping_section_raises(self) -> None:
        with self.assertRaises(RoboCasaSceneConfigError):
            config_from_mapping({"robocasa": []})

    def test_invalid_int_values_raise_config_error(self) -> None:
        for key in ("layout_id", "style_id", "control_freq"):
            with self.subTest(key=key):
                with self.assertRaises(RoboCasaSceneConfigError):
                    config_from_mapping({"robocasa": {key: "bad"}})

    def test_missing_generated_xml_path_when_write_enabled_raises(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            robot_xml = Path(tmpdir) / "robot.xml"
            robot_xml.write_text("<mujoco/>", encoding="utf-8")

            config = config_from_mapping(
                {
                    "robot": {"xml_path": str(robot_xml)},
                    "output": {"write_generated_xml": True},
                }
            )

            with self.assertRaises(RoboCasaSceneConfigError):
                config.validate()

    def test_missing_robot_xml_raises(self) -> None:
        config = config_from_mapping({"robot": {"xml_path": "/missing/robot.xml"}})

        with self.assertRaises(RoboCasaSceneConfigError):
            config.validate()


if __name__ == "__main__":
    unittest.main()
