# Tinker decision module

ROS 2 behaviour-tree orchestration for Tinker's RoboCup@Home missions.

The public command surface is intentionally small. Each task family has one
unsuffixed command, and that command points at the current competition tree.
Historical variants and development probes are tests or Git history, not
installed executables.

## Commands

Competition tasks:

```text
doing-laundry
follow-person
gpsr
help-me-carry
hri
inspection
pick-and-place
receptionist
restaurant
serve-breakfast
store-groceries
```

Operator tools:

```text
draw
fetch-points
verify-task-endpoints
```

Examples:

```bash
source ~/tk25_ws/src/tk25_decision/.venv_decision/bin/activate
source ~/tk25_ws/install/setup.zsh

ros2 run behavior_tree restaurant
ros2 run behavior_tree hri
ros2 run behavior_tree pick-and-place --place-policy vlm
ros2 run behavior_tree gpsr
```

The five competition cutovers are:

| Command | Current implementation |
|---|---|
| `restaurant` | `Restaurant.restaurant_v2` |
| `hri` | `HRI.hri_2026` |
| `pick-and-place` | `PickAndPlace.pick_and_place_rulebook` |
| `gpsr` | `GPSR.gpsr_orchestrator` |
| `doing-laundry` | `DoingLaundry.laundry` |

## Package architecture

Dependencies flow in one direction:

```text
core
  ↓
interfaces
  ↓
nodes
  ↓
components
  ↓
task packages
  ↓
tools / CLI adapters
```

- `core/` owns configuration, packaged-resource loading, runtime lifecycle,
  visualization, input routing, and shared protocol constants.
- `interfaces/` is the only selection point for real versus mock ROS message,
  service, and action types. Subsystem facades keep imports focused.
- `nodes/` contains task-neutral behaviour-tree leaves.
- `components/` contains reusable multi-node behaviours such as person
  following, grocery perception/grasping, and person-introduction nodes.
- Task directories compose shared pieces and own task-specific constants.
- `tools/` contains the three retained operator utilities.

Task packages must not import other task packages. Shared layers must not reach
into task packages. `test/test_architecture_boundaries.py` enforces both rules.

Task `__init__.py` files are side-effect free. Entry modules import mission
factories lazily, so command discovery does not initialize ROS clients or load
an unrelated mission.

## Configuration and resources

Task constants are package resources and work from both the source tree and an
installed ROS package:

```python
from behavior_tree.core.resources import read_json

constants = read_json("behavior_tree.Restaurant")
```

Do not add workspace-absolute constants paths.

Mock configuration defaults to `behavior_tree/mock_config.json`. Override it
with:

```bash
export BT_MOCK_CONFIG=/path/to/mock_config.json
export BT_MOCK_MODE=true   # or false
```

GPSR reads credentials from the process environment. The launcher exports
`~/tk25_ws/.env` when present. At minimum:

```bash
python -m pip install -r requirements-gpsr.txt
export OPENROUTER_API_KEY=...
```

Optional overrides include `GPSR_LLM_MODEL`, `GPSR_FALLBACK_MODEL`,
`BT_GPSR_CMD`, `BT_GPSR_NUM_COMMANDS`, and `BT_GPSR_PLAN_DIR`.

## Operator tools

Render any canonical tree:

```bash
ros2 run behavior_tree draw restaurant
ros2 run behavior_tree draw gpsr
```

Capture navigation poses. The writable output defaults to
`./constants_basic.json`; set `BT_POINTS_FILE` or the ROS `points_file`
parameter for another location:

```bash
BT_POINTS_FILE=~/tk25_ws/task_points.json \
  ros2 run behavior_tree fetch-points
```

Check live ROS endpoints:

```bash
ros2 run behavior_tree verify-task-endpoints --task all
ros2 run behavior_tree verify-task-endpoints --task hri --json
```

## Development

Build:

```bash
cd ~/tk25_ws
colcon build --packages-select behavior_tree
source install/setup.zsh
```

Run the functional and architecture suite:

```bash
cd ~/tk25_ws/src/tk25_decision/src/behavior_tree
ROS_LOG_DIR=/tmp/behavior_tree_test_logs \
PYTHONPATH=. \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
../../.venv_decision/bin/python -m pytest -q test
```

The broad legacy flake8 check is explicitly skipped while old task modules are
incrementally formatted. New architectural constraints and command contracts
remain mandatory.

When adding a mission, add one `cli.py`, one canonical entry in both
`core/entrypoints.py` and `setup.py`, package its resources, and extend the
architecture/entrypoint tests. Do not add versioned or test console scripts.

## Competition launchers

Robot-side launchers live in `tk25_basic/src/scripts`. They all call:

```bash
tmux_decision.sh <canonical-task>
```

The helper validates the task name, activates the decision environment, loads
the workspace environment, and reuses the tmux `decision` window.
