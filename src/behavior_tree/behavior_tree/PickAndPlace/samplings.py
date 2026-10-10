"""PickAndPlace samplings — standalone `ros2 run` dev trees (mock-runnable).

Each main_<x> seeds the blackboard inside a factory and hands it to run_tree::

    BT_MOCK_MODE=true ros2 run behavior_tree pp-test-scan-place
    BT_MOCK_MODE=true ros2 run behavior_tree pp-test-categorize
    BT_MOCK_MODE=true ros2 run behavior_tree pp-test-cleanup-loop
    BT_MOCK_MODE=true ros2 run behavior_tree pp-test-breakfast
"""

import os

import py_trees

from behavior_tree.runtime import run_tree
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard

from .config import (
    KEY_ACTIVE_TARGET_POINT,
    KEY_INVENTORY_TABLE,
    KEY_OBJECT_LABEL,
    KEY_POSE_TABLE,
    KEY_SCAN_RESULTS_TABLE,
    KEY_WORK_QUEUE,
    PLACEMENT_MODE_FREE_SPACE,
    POINT_WASH_STAGING,
)
from .custom_nodes import BtNode_BuildInventory, BtNode_PopWorkItem
from behavior_tree.TemplateNodes.Manipulation import BtNode_ScanAndPlace
from .pick_and_place_rulebook import (
    createConstantWriter,
    handleOneItem,
    phaseServeBreakfast,
    phaseTableCleanup,
)

# 'pp_active_placement_mode' is BtNode_ScanAndPlace's default placement-mode key.
_KEY_ACTIVE_PLACEMENT_MODE = "pp_active_placement_mode"


def _write(name, key, value):
    return BtNode_WriteToBlackboard(
        name=name, bb_namespace="", bb_source=None, bb_key=key, object=value
    )


def _idle():
    return py_trees.behaviours.Running("idle")


# --------------------------------------------------------------------------- #
# pp-test-scan-place — seed a held object + FREE_SPACE mode, tick ScanAndPlace #
# --------------------------------------------------------------------------- #


def _scan_place_factory():
    seq = py_trees.composites.Sequence("pp-test-scan-place", memory=True)
    seq.add_child(createConstantWriter("vlm"))
    seq.add_child(_write("seed label", KEY_OBJECT_LABEL, "cup"))
    seq.add_child(_write("seed mode FREE_SPACE", _KEY_ACTIVE_PLACEMENT_MODE, PLACEMENT_MODE_FREE_SPACE))
    seq.add_child(_write("seed fixed target", KEY_ACTIVE_TARGET_POINT, POINT_WASH_STAGING))
    seq.add_child(BtNode_ScanAndPlace(name="scan and place (sampling)"))
    seq.add_child(_idle())
    return seq


def main_scan_place():
    os.environ.setdefault("BT_MOCK_MODE", "true")
    run_tree(_scan_place_factory, period_ms=400.0, title="pp scan-place")


# --------------------------------------------------------------------------- #
# pp-test-categorize — BuildInventory then drain PopWorkItem over a canned set #
# --------------------------------------------------------------------------- #


def _categorize_factory():
    seq = py_trees.composites.Sequence("pp-test-categorize", memory=True)
    seq.add_child(createConstantWriter("vlm"))
    seq.add_child(
        BtNode_BuildInventory(
            name="build inventory (canned)",
            in_key=KEY_SCAN_RESULTS_TABLE,
            out_inventory=KEY_INVENTORY_TABLE,
            out_queue=KEY_WORK_QUEUE,
            source_pose_key=KEY_POSE_TABLE,
            # mock_seed is a list of plain label STRINGS — BtNode_BuildInventory
            # does `[(str(s), None) for s in seed]`, so dicts would break it.
            # Routing: fork->wash_staging, paper cup->trash, snack->cabinet.
            mock_seed=["fork", "paper cup", "snack"],
        )
    )
    # Drain the queue: PopWorkItem FAILS on empty -> Repeat FAILS -> FailureIsSuccess.
    drain = py_trees.decorators.Repeat(
        name="drain (num_success=-1)",
        child=BtNode_PopWorkItem(name="pop (sampling)", place_policy="vlm"),
        num_success=-1,
    )
    seq.add_child(
        py_trees.decorators.FailureIsSuccess(name="drained", child=drain)
    )
    seq.add_child(_idle())
    return seq


def main_categorize():
    os.environ.setdefault("BT_MOCK_MODE", "true")
    run_tree(_categorize_factory, period_ms=400.0, title="pp categorize")


# --------------------------------------------------------------------------- #
# pp-test-cleanup-loop — full table-cleanup phase (BuildInventory mock-seeds)  #
# --------------------------------------------------------------------------- #


def _cleanup_loop_factory():
    seq = py_trees.composites.Sequence("pp-test-cleanup-loop", memory=True)
    seq.add_child(createConstantWriter("vlm"))
    seq.add_child(phaseTableCleanup("vlm"))
    seq.add_child(_idle())
    return seq


def main_cleanup_loop():
    os.environ.setdefault("BT_MOCK_MODE", "true")
    run_tree(_cleanup_loop_factory, period_ms=300.0, title="pp cleanup-loop")


# --------------------------------------------------------------------------- #
# pp-test-breakfast — serve-breakfast phase alone                             #
# --------------------------------------------------------------------------- #


def _breakfast_factory():
    seq = py_trees.composites.Sequence("pp-test-breakfast", memory=True)
    seq.add_child(createConstantWriter("vlm"))
    seq.add_child(phaseServeBreakfast("vlm"))
    seq.add_child(_idle())
    return seq


def main_breakfast():
    os.environ.setdefault("BT_MOCK_MODE", "true")
    run_tree(_breakfast_factory, period_ms=300.0, title="pp breakfast")
