from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from mujoco_simulation.robocasa.scene_cli import SceneConfig, choose_option, main


class SceneCliTest(unittest.TestCase):
    def test_choose_option_retries_invalid_input(self) -> None:
        prompts: list[str] = []
        outputs: list[str] = []
        result = choose_option(
            {1: "A", 2: "B"},
            "layout",
            input_func=lambda prompt: prompts.append(prompt) or ("bad" if len(prompts) == 1 else "2"),
            output_func=outputs.append,
        )

        self.assertEqual(result, 2)
        self.assertTrue(any("Invalid input" in message for message in outputs))

    def test_main_generates_scene_with_explicit_values(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            output_path = Path(tmpdir) / "scene.xml"
            config = SceneConfig(robot_xml_path=str(output_path), generated_xml_path=None)
            scene = mock.Mock(xml="<mujoco/>")

            stdout = StringIO()
            with redirect_stdout(stdout), mock.patch(
                "mujoco_simulation.robocasa.scene_cli.load_config",
                return_value=config,
            ), mock.patch(
                "mujoco_simulation.robocasa.scene_generator.SceneGenerator"
            ) as scene_generator:
                scene_generator.return_value.generate.return_value = scene
                result = main(
                    [
                        "--config",
                        "base.yaml",
                        "--output",
                        str(output_path),
                        "--task",
                        "OpenDrawer",
                        "--layout",
                        "3",
                        "--style",
                        "4",
                    ]
                )

        self.assertEqual(result, 0)
        self.assertEqual(stdout.getvalue().strip(), str(output_path.resolve()))
        generated_config = scene_generator.return_value.generate.call_args.args[0]
        self.assertEqual(generated_config.task_name, "OpenDrawer")
        self.assertEqual(generated_config.layout_id, 3)
        self.assertEqual(generated_config.style_id, 4)
        self.assertTrue(generated_config.write_generated_xml)

    def test_main_prompts_for_missing_values(self) -> None:
        stdout = StringIO()
        config = SceneConfig(robot_xml_path="/tmp/robot.xml")
        scene = mock.Mock(xml="<mujoco/>")
        with redirect_stdout(stdout), mock.patch(
            "mujoco_simulation.robocasa.scene_cli.load_config",
            return_value=config,
        ), mock.patch(
            "mujoco_simulation.robocasa.scene_cli.choose_option",
            side_effect=[3, 4],
        ), mock.patch(
            "mujoco_simulation.robocasa.scene_cli.get_style_choices",
            return_value={4: "Style 4"},
        ), mock.patch(
            "mujoco_simulation.robocasa.scene_generator.SceneGenerator"
        ) as scene_generator:
            scene_generator.return_value.generate.return_value = scene
            result = main(
                ["--config", "base.yaml", "--output", "/tmp/scene.xml", "--task", "OpenDrawer"]
            )

        self.assertEqual(result, 0)
        generated_config = scene_generator.return_value.generate.call_args.args[0]
        self.assertEqual(generated_config.layout_id, 3)
        self.assertEqual(generated_config.style_id, 4)


if __name__ == "__main__":
    unittest.main()
