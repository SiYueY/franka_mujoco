from __future__ import annotations

from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from mujoco_ros2_bridge.robocasa.exceptions import RoboCasaMjcfAdaptationError
from mujoco_ros2_bridge.robocasa.mjcf_adapter import adapt_mjcf
from mujoco_ros2_bridge.robocasa.scene_data import ObjectPlacement, SpawnPose


RAW_XML = """
<mujoco model="kitchen">
  <option timestep="0.002"/>
  <actuator><motor name="old_motor"/></actuator>
  <sensor><touch name="old_sensor"/></sensor>
  <worldbody>
    <body name="robot0_base" pos="1 2 3"/>
    <body name="robot_base"/>
    <body name="cup_main" pos="0 0 0" quat="1 0 0 0"/>
    <geom name="debug" rgba="1 0 0 1"/>
  </worldbody>
</mujoco>
"""


class RoboCasaMjcfAdapterTest(unittest.TestCase):
    def test_adapts_scene_xml(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            robot_xml = Path(tmpdir) / "robot.xml"
            robot_xml.write_text("<mujoco/>", encoding="utf-8")

            xml = adapt_mjcf(
                RAW_XML,
                robot_xml_path=robot_xml,
                object_placements=[
                    ObjectPlacement(
                        name="cup",
                        category="mug",
                        pos=(0.1, 0.2, 0.3),
                        quat=(1.0, 0.0, 0.0, 0.0),
                    )
                ],
                spawn_pose=SpawnPose(
                    pos=(1.0, 2.0, 0.0),
                    quat=(0.70710678, 0.0, 0.0, 0.70710678),
                ),
                robot_body_name="robot_base",
            )

        root = ET.fromstring(xml)

        self.assertIsNone(root.find("option"))
        self.assertIsNone(root.find("actuator"))
        self.assertIsNone(root.find("sensor"))
        self.assertIsNone(_body(root, "robot0_base"))

        include = root.find("include")
        self.assertIsNotNone(include)
        self.assertTrue(Path(include.get("file", "")).is_absolute())

        cup = _required_body(root, "cup_main")
        self.assertEqual(cup.get("pos"), "0.1 0.2 0.3")
        self.assertEqual(cup.get("quat"), "1 0 0 0")

        robot = _required_body(root, "robot_base")
        self.assertEqual(robot.get("pos"), "1 2 0")
        self.assertEqual(robot.get("quat"), "0.70710678 0 0 0.70710678")

        debug = root.find(".//geom[@name='debug']")
        self.assertEqual(debug.get("rgba"), "1 1 1 0")

    def test_missing_robot_xml_raises(self) -> None:
        with self.assertRaises(RoboCasaMjcfAdaptationError):
            adapt_mjcf(RAW_XML, robot_xml_path="/missing/robot.xml")

    def test_missing_required_placeholder_raises(self) -> None:
        raw_xml = '<mujoco><worldbody><body name="other"/></worldbody></mujoco>'

        with tempfile.TemporaryDirectory() as tmpdir:
            robot_xml = Path(tmpdir) / "robot.xml"
            robot_xml.write_text("<mujoco/>", encoding="utf-8")

            with self.assertRaises(RoboCasaMjcfAdaptationError):
                adapt_mjcf(raw_xml, robot_xml_path=robot_xml)

    def test_invalid_xml_raises(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            robot_xml = Path(tmpdir) / "robot.xml"
            robot_xml.write_text("<mujoco/>", encoding="utf-8")

            with self.assertRaises(RoboCasaMjcfAdaptationError):
                adapt_mjcf("<not-xml", robot_xml_path=robot_xml)


def _body(root: ET.Element, name: str) -> ET.Element | None:
    return root.find(f".//body[@name='{name}']")


def _required_body(root: ET.Element, name: str) -> ET.Element:
    body = _body(root, name)
    if body is None:
        raise AssertionError(f"Missing body: {name}")
    return body


if __name__ == "__main__":
    unittest.main()
