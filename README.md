# franka_mujoco

Franka MuJoCo

## Setup

### 1. Initialize submodules

This repository vendors `robosuite` and `robocasa` under `third_party` as git submodules.

```bash
git submodule update --init --recursive
```

Verify:

```bash
git submodule status
```

### 2. Create a Python virtual environment with `uv`

To avoid polluting the system Python environment, create and use a local virtual environment.
Use Python `3.10.12` so the environment matches the Python version used by ROS 2:

```bash
uv venv --python 3.10.12 .venv
source .venv/bin/activate
```

If `uv` is not installed yet:

```bash
python3 -m pip install --user uv
```

If Python `3.10.12` is not available locally yet:

```bash
uv python install 3.10.12
```

### 3. Install `robosuite` and `robocasa`

Install the vendored submodules in editable mode:

```bash
uv pip install --upgrade pip setuptools wheel
uv pip install -e "robosuite@third_party/robosuite"
uv pip install -e "robocasa@third_party/robocasa"
```

### 4. Initialize robosuite and RoboCasa private macros

Both `robosuite` and `robocasa` expect private macro files for machine-local settings.
Create them once after installation:

```bash
uv run third_party/robosuite/robosuite/scripts/setup_macros.py
uv run third_party/robocasa/robocasa/scripts/setup_macros.py
```

This creates:

- `third_party/robosuite/robosuite/macros_private.py`
- `third_party/robocasa/robocasa/macros_private.py`

You can later edit these files for machine-local settings such as dataset paths or SpaceMouse IDs.

### 5. Download RoboCasa kitchen assets

RoboCasa environments are not complete until the kitchen assets are downloaded.
The full asset package is about 10 GB.

Download all assets:

```bash
uv run third_party/robocasa/robocasa/scripts/download_kitchen_assets.py --type all
```

If you only want specific subsets, the available asset groups are:

- `tex`
- `tex_generative`
- `fixtures_lw`
- `objs_objaverse`
- `objs_aigen`
- `objs_lw`

Example:

```bash
uv run third_party/robocasa/robocasa/scripts/download_kitchen_assets.py --type tex objs_objaverse
```

### 6. Validate Python imports

```bash
uv run python - <<'PY'
import robosuite
import robocasa

print("robosuite:", robosuite.__file__)
print("robocasa:", robocasa.__file__)
PY
```

### 7. Validate RoboCasa environment creation

Run a minimal environment creation check:

```bash
uv run python - <<'PY'
import gymnasium as gym
import robocasa

env = gym.make(
    "robocasa/PickPlaceCounterToCabinet",
    split="pretrain",
    seed=0,
)
print("Environment created:", type(env))
env.close()
PY
```

Ignore non-fatal warnings during the first setup unless the import or environment creation step actually fails.
