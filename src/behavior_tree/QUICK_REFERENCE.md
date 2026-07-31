# Decision quick reference

```bash
source ~/tk25_ws/src/tk25_decision/.venv_decision/bin/activate
source ~/tk25_ws/install/setup.zsh
```

Run a task:

```bash
ros2 run behavior_tree restaurant
ros2 run behavior_tree hri
ros2 run behavior_tree pick-and-place
ros2 run behavior_tree gpsr
ros2 run behavior_tree doing-laundry
```

Other canonical tasks are `follow-person`, `help-me-carry`, `inspection`,
`receptionist`, `serve-breakfast`, and `store-groceries`.

Mock mode:

```bash
export BT_MOCK_MODE=true
export BT_MOCK_CONFIG=/path/to/mock_config.json  # optional
```

Shared imports:

```python
from behavior_tree.core.config import is_node_mocked
from behavior_tree.nodes.Navigation import BtNode_GotoAction
from behavior_tree.nodes.Audio import BtNode_Announce
from behavior_tree.interfaces.vision import ObjectDetection
```

Tools:

```bash
ros2 run behavior_tree draw restaurant
ros2 run behavior_tree fetch-points
ros2 run behavior_tree verify-task-endpoints --task all
```

Tests:

```bash
ROS_LOG_DIR=/tmp/behavior_tree_test_logs \
PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
../../.venv_decision/bin/python -m pytest -q test
```
