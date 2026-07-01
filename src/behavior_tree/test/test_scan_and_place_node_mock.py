import os

os.environ["BT_MOCK_MODE"] = "true"  # noqa: E402 — force mock before config loads

import py_trees  # noqa: E402
import pytest  # noqa: E402

from behavior_tree.TemplateNodes.Manipulation import BtNode_ScanAndPlace  # noqa: E402
from behavior_tree.PickAndPlace.config import (  # noqa: E402
    SCAN_AND_PLACE_ACTION_NAME, KEY_OBJECT_LABEL, KEY_ACTIVE_TARGET_POINT,
)


@pytest.fixture(autouse=True)
def _clear_blackboard():
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()
    yield
    if callable(clear_fn):
        clear_fn()


def test_constructs_in_mock_mode():
    node = BtNode_ScanAndPlace(name="ScanPlace")
    assert node.mock_mode is True
    assert node.action_name == SCAN_AND_PLACE_ACTION_NAME


def test_default_bb_keys_match_contract():
    node = BtNode_ScanAndPlace(name="ScanPlace")
    # The contract defaults for the shared keys:
    assert node._reads["item_description"] == KEY_OBJECT_LABEL
    assert node._reads["fixed_target"] == KEY_ACTIVE_TARGET_POINT
    assert node._reads["placement_mode"] == "pp_active_placement_mode"


def test_ticks_to_success_in_mock_no_server():
    node = BtNode_ScanAndPlace(name="ScanPlace")
    node.setup(node=None)  # mock setup skips client creation
    status = py_trees.common.Status.RUNNING
    for _ in range(6):
        node.tick_once()
        status = node.status
        if status == py_trees.common.Status.SUCCESS:
            break
    assert status == py_trees.common.Status.SUCCESS
