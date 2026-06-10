"""CLI for generating MuJoCo-ready RoboCasa scenes."""

from __future__ import annotations

import argparse
from collections import OrderedDict
from dataclasses import replace
from pathlib import Path
import sys
from typing import Callable

from .exceptions import RoboCasaIntegrationError
from .scene_config import SceneConfig, load_config


TASK_CHOICES = OrderedDict(
    [
        ("PickPlaceCounterToCabinet", "Pick and place from counter to cabinet"),
        ("OpenDrawer", "Open drawer"),
        ("CloseDrawer", "Close drawer"),
        ("OpenFridgeDrawer", "Open fridge drawer"),
        ("CloseFridgeDrawer", "Close fridge drawer"),
        ("TurnOnMicrowave", "Turn on microwave"),
        ("TurnOffMicrowave", "Turn off microwave"),
        ("TurnOnSinkFaucet", "Turn on sink faucet"),
        ("TurnOffSinkFaucet", "Turn off sink faucet"),
        ("TurnOnStove", "Turn on stove"),
        ("TurnOffStove", "Turn off stove"),
        ("TurnOnElectricKettle", "Turn on electric kettle"),
    ]
)

LAYOUT_CHOICES = OrderedDict(
    [
        (1, "One wall"),
        (2, "One wall w/ island"),
        (3, "L-shaped"),
        (4, "L-shaped w/ island"),
        (5, "Galley"),
        (6, "U-shaped"),
        (7, "U-shaped w/ island"),
        (8, "G-shaped"),
        (9, "G-shaped (large)"),
        (10, "Wraparound"),
    ]
)


def get_style_choices() -> OrderedDict[int, str]:
    """Return available RoboCasa style ids."""

    try:
        from robocasa.models.scenes.scene_registry import StyleType
    except ImportError as exc:  # pragma: no cover - covered by CLI failure path
        raise RoboCasaIntegrationError(
            "robocasa is required to enumerate style choices."
        ) from exc

    return OrderedDict(
        (item.value, item.name.title().replace("Style", "Style "))
        for item in StyleType
        if item.value > 0
    )


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI parser."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="Base RoboCasa YAML config path.")
    parser.add_argument("--output", required=True, help="Output MJCF XML path.")
    parser.add_argument("--task", help="RoboCasa task name.")
    parser.add_argument("--layout", type=int, help="RoboCasa layout id.")
    parser.add_argument("--style", type=int, help="RoboCasa style id.")
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Prompt for missing task/layout/style values in the terminal.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the CLI."""

    args = build_parser().parse_args(argv)

    try:
        config = load_config(args.config)
        interactive = args.interactive or any(
            value is None for value in (args.task, args.layout, args.style)
        )

        task = args.task
        layout = args.layout
        style = args.style
        if interactive:
            task = task or choose_option(TASK_CHOICES, "task")
            layout = layout if layout is not None else choose_option(LAYOUT_CHOICES, "layout")
            style = style if style is not None else choose_option(get_style_choices(), "style")

        if task is None or layout is None or style is None:
            raise RoboCasaIntegrationError(
                "task, layout, and style must all be specified when not using interactive mode."
            )

        config = replace(
            config,
            task_name=task,
            layout_id=layout,
            style_id=style,
            write_generated_xml=True,
            generated_xml_path=str(Path(args.output).expanduser().resolve()),
        )

        from .scene_generator import SceneGenerator

        scene = SceneGenerator().generate(config)
        output_path = Path(config.generated_xml_path).expanduser().resolve()
        if not output_path.exists():
            output_path.write_text(scene.xml, encoding="utf-8")

        print(output_path)
        return 0
    except RoboCasaIntegrationError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    except Exception as exc:  # noqa: BLE001
        print(f"Unexpected RoboCasa scene generation failure: {exc}", file=sys.stderr)
        return 1


def choose_option(
    options: OrderedDict[object, str],
    option_name: str,
    *,
    input_func: Callable[[str], str] = input,
    output_func: Callable[[str], None] = print,
) -> object:
    """Prompt for one option and return its key."""

    items = list(options.items())
    output_func(f"{option_name.capitalize()}s:")
    for index, (key, label) in enumerate(items, start=1):
        output_func(f"[{index}] {key}: {label}")

    prompt = f"Choose {option_name} 1-{len(items)} [default 1]: "
    while True:
        raw = input_func(prompt).strip()
        if raw == "":
            return items[0][0]
        try:
            choice = int(raw)
        except ValueError:
            output_func("Invalid input. Enter a number.")
            continue
        if 1 <= choice <= len(items):
            return items[choice - 1][0]
        output_func("Choice out of range.")


if __name__ == "__main__":
    raise SystemExit(main())
