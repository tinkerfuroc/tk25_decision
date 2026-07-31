"""Contracts for the canonical Pick-and-Place rulebook and its plain nodes."""

import os
from types import SimpleNamespace

os.environ.setdefault("BT_MOCK_MODE", "true")

import py_trees  # noqa: E402
import pytest  # noqa: E402

from behavior_tree.PickAndPlace import config  # noqa: E402
from behavior_tree.PickAndPlace import custom_nodes  # noqa: E402
from behavior_tree.PickAndPlace import pick_and_place_rulebook  # noqa: E402


@pytest.fixture(autouse=True)
def _clean_blackboard():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


def _set_bb(key, value):
    client = py_trees.blackboard.Client(name=f"test_writer_{key}")
    client.register_key(
        key="value",
        access=py_trees.common.Access.WRITE,
        remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key),
    )
    client.value = value


def _get_bb(key):
    client = py_trees.blackboard.Client(name=f"test_reader_{key}")
    client.register_key(
        key="value",
        access=py_trees.common.Access.READ,
        remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key),
    )
    return client.value


def test_rulebook_root_wraps_the_current_mission_in_a_global_deadline():
    root = pick_and_place_rulebook.pickAndPlaceRulebook(place_policy="vlm")

    assert isinstance(root, py_trees.composites.Sequence)
    assert root.memory is True
    assert len(root.children) == 3
    assert isinstance(root.children[0], py_trees.composites.Sequence)
    assert isinstance(root.children[1], py_trees.decorators.FailureIsSuccess)

    guarded_mission = root.children[2]
    assert isinstance(guarded_mission, py_trees.composites.Parallel)
    assert isinstance(
        guarded_mission.policy, py_trees.common.ParallelPolicy.SuccessOnOne
    )
    deadline, mission = guarded_mission.children
    assert isinstance(deadline, custom_nodes.BtNode_DeadlineGuard)
    assert mission.name == "mission phases"
    assert [child.name for child in mission.children] == [
        "phase: kitchen door",
        "phase: table scan",
        "phase: pull diswasher",
        "phase: grasp",
        "phase: pushing dish washer door",
    ]


def test_build_inventory_accepts_current_scan_shapes_and_routes_items():
    _set_bb(
        config.KEY_SCAN_RESULTS_TABLE,
        SimpleNamespace(
            objects=[
                SimpleNamespace(class_name="paper cup", segment="trash-segment"),
                SimpleNamespace(cls="bowl", segment="bowl-segment"),
                SimpleNamespace(cls="pringles", segment="snack-segment"),
            ]
        ),
    )
    node = custom_nodes.BtNode_BuildInventory(name="build inventory")

    assert node.update() == py_trees.common.Status.SUCCESS

    inventory = _get_bb(config.KEY_INVENTORY_TABLE)
    queue = _get_bb(config.KEY_WORK_QUEUE)
    assert [(item["label"], item["destination"]) for item in inventory] == [
        ("paper cup", "trash"),
        ("bowl", "wash_staging"),
        ("pringles", "cabinet"),
    ]
    assert [item["label"] for item in queue] == [
        "paper cup",
        "bowl",
        "pringles",
    ]


def test_pop_work_item_stamps_active_contract_and_empty_queue_terminates_loop():
    _set_bb(
        config.KEY_WORK_QUEUE,
        [
            {
                "label": "paper cup",
                "destination": "trash",
                "reference_label": "",
                "source_pose_key": config.KEY_POSE_TABLE,
            }
        ],
    )
    node = custom_nodes.BtNode_PopWorkItem(
        name="pop work item", place_policy="vlm"
    )

    assert node.update() == py_trees.common.Status.SUCCESS
    assert _get_bb(config.KEY_OBJECT_LABEL) == "paper cup"
    assert _get_bb(config.KEY_ACTIVE_OBJECT_CLASS) == "trash"
    assert _get_bb("pp_active_placement_mode") == config.PLACEMENT_MODE_NONE
    assert _get_bb("pp_active_skip_scan") is True
    assert _get_bb(config.KEY_WORK_QUEUE) == []
    assert node.update() == py_trees.common.Status.FAILURE


def test_deadline_guard_runs_until_its_injected_clock_reaches_budget():
    now = iter([10.0, 14.9, 15.0])
    node = custom_nodes.BtNode_DeadlineGuard(
        name="deadline", budget_sec=5.0, clock=lambda: next(now)
    )

    node.initialise()
    assert node.update() == py_trees.common.Status.RUNNING
    assert node.update() == py_trees.common.Status.SUCCESS
