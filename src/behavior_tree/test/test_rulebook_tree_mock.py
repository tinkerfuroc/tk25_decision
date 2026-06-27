# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

"""Layer-B whole-tree mock integration test for pickAndPlaceRulebook().

Runs the real rulebook tree end-to-end under BT_MOCK_MODE=true with every
subsystem mocked and keyboard control off (KEYPRESS -> IMMEDIATE). The plain
Behaviours (BuildInventory/PopWorkItem/DeadlineGuard/guards/markPhase) run their
real logic; BuildInventory mock-seeds a non-empty queue so the cleanup loop body
actually runs. An explicit tick cap asserts FAIL (not hang) if the tree never
terminates.

CRITICAL: this test does NOT trust the checked-out ``mock_config.json``. A
concurrent session flips vision/nav to un-mocked and announcement to real TTS,
which would make ``tree.setup()`` block on ``wait_for_service``/``wait_for_server``
forever. ``_force_full_mock`` therefore (1) writes a private all-mock config to a
temp file, (2) points ``BT_MOCK_CONFIG`` at it so any mtime-triggered reload
reads OUR file, (3) force-loads it into the config singleton now, and (4) mutates
the in-memory config in place as belt-and-suspenders. setup() then contacts NO
real service.
"""

import json
import os
import tempfile

os.environ["BT_MOCK_MODE"] = "true"  # noqa: E402 must precede config import

import py_trees  # noqa: E402
import py_trees_ros  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402

import behavior_tree.config as btcfg  # noqa: E402

TICK_CAP = 4000  # leaf-count x ~3 mock ticks x phases, with margin

# Stable private all-mock config path, written once and reused across tests.
_FORCED_CONFIG_PATH = None
_SAVED_BT_MOCK_CONFIG = None

_ALL_SUBSYSTEMS = (
    "vision",
    "manipulation",
    "navigation",
    "audio_input",
    "announcement",
    "mock_controls",
)


def _all_mock_config():
    """Every subsystem mocked (incl. announcement => no real TTS); keyboard off."""
    return {
        "force_mock_nodes": {},
        "mock_mode": {
            "enabled": True,
            "auto_detect": True,
            "subsystems": {s: {"enabled": True, "nodes": {}} for s in _ALL_SUBSYSTEMS},
        },
        "keyboard_control": {"enabled": False},
        "mock_keyboard": {"subsystem_start_keys": {}, "success_key": "ENTER"},
        "teleop": {"params": {}},
        "logging": {"print_mock_operations": False, "use_emoji": False},
    }


def _force_full_mock():
    """Force every subsystem mocked, robust against a concurrent edit of the
    checked-out mock_config.json; keyboard off => KEYPRESS degrades to IMMEDIATE.

    Pointing BT_MOCK_CONFIG at our own stable temp file means even an
    mtime-triggered reload (config._maybe_reload_config) reads OUR all-mock file,
    so a concurrent un-mock of the workspace config can never leak in mid-run.
    """
    global _FORCED_CONFIG_PATH, _SAVED_BT_MOCK_CONFIG
    if _FORCED_CONFIG_PATH is None:
        fd, path = tempfile.mkstemp(prefix="pp_all_mock_", suffix=".json")
        with os.fdopen(fd, "w") as f:
            json.dump(_all_mock_config(), f)
        _FORCED_CONFIG_PATH = path

    _SAVED_BT_MOCK_CONFIG = os.environ.get("BT_MOCK_CONFIG")
    os.environ["BT_MOCK_MODE"] = "true"
    os.environ["BT_MOCK_CONFIG"] = _FORCED_CONFIG_PATH

    cfg = btcfg._config
    cfg._load_mock_config(force=True)  # resolves to our temp file via BT_MOCK_CONFIG

    # Belt-and-suspenders: mutate the freshly loaded dict in place too.
    mc = cfg._mock_config
    mc.setdefault("mock_mode", {})["enabled"] = True
    for sub in mc["mock_mode"].setdefault("subsystems", {}).values():
        sub["enabled"] = True
    mc.setdefault("keyboard_control", {})["enabled"] = False


def _restore_config():
    """Undo BT_MOCK_CONFIG override + reload from disk so later test modules in a
    full-suite run see the on-disk config again."""
    if _SAVED_BT_MOCK_CONFIG is None:
        os.environ.pop("BT_MOCK_CONFIG", None)
    else:
        os.environ["BT_MOCK_CONFIG"] = _SAVED_BT_MOCK_CONFIG
    try:
        btcfg._config._load_mock_config(force=True)
    except Exception:
        pass


def _get_bb(key):
    client = py_trees.blackboard.Client(name="pp_mock_reader")
    client.register_key(
        key="v",
        access=py_trees.common.Access.READ,
        remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key),
    )
    return client.v


def _run_to_completion(root, node_name):
    """setup() the tree against a real rclpy node, tick to terminal under a cap."""
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name=node_name, timeout=15)
    ticks = 0
    try:
        while ticks < TICK_CAP:
            tree.tick()
            ticks += 1
            if root.status != py_trees.common.Status.RUNNING:
                break
    finally:
        try:
            tree.shutdown()
        except Exception:
            pass
    # Distinct from the in-tree wall-clock DeadlineGuard: a runaway tree FAILS the
    # test here rather than hanging the suite.
    assert ticks < TICK_CAP, f"tree did not terminate within {TICK_CAP} ticks"
    return root.status, ticks


@pytest.fixture(autouse=True)
def _ros_and_blackboard():
    _force_full_mock()
    py_trees.blackboard.Blackboard.clear()
    if not rclpy.ok():
        rclpy.init()
    yield
    py_trees.blackboard.Blackboard.clear()
    _restore_config()


def test_rulebook_ticks_to_success_visiting_all_phases_with_a_place():
    from behavior_tree.PickAndPlace.pick_and_place_rulebook import pickAndPlaceRulebook

    root = pickAndPlaceRulebook(place_policy="vlm")
    status, _ = _run_to_completion(root, "pp_mock_happy")

    assert status == py_trees.common.Status.SUCCESS

    trace = _get_bb("pp_score_trace")
    assert set(trace["visited_phases"]) >= {"table", "breakfast", "extra"}, trace[
        "visited_phases"
    ]
    place_events = [e for e in trace["events"] if e["action"] == "place"]
    assert len(place_events) >= 1, trace["events"]
    assert trace["place_policy"] == "vlm"


def test_forced_grasp_failure_skips_item_and_mission_still_completes(monkeypatch):
    import behavior_tree.PickAndPlace.pick_and_place_rulebook as R

    class _FailGrasp(py_trees.behaviour.Behaviour):
        def __init__(self, name, *args, **kwargs):
            super().__init__(name=name)

        def update(self):
            return py_trees.common.Status.FAILURE

    # Patch the symbol the rulebook factories reference at build time.
    monkeypatch.setattr(R, "BtNode_Grasp", _FailGrasp)

    root = R.pickAndPlaceRulebook(place_policy="vlm")
    status, _ = _run_to_completion(root, "pp_mock_failgrasp")

    # Mission still completes: grasp failures fall through to maybeHelpOrSkip
    # (skip), the cleanup loop's FailureIsSuccess keeps draining, breakfast's
    # per-item FailureIsSuccess keeps serving.
    assert status == py_trees.common.Status.SUCCESS

    trace = _get_bb("pp_score_trace")
    assert set(trace["visited_phases"]) >= {"table", "breakfast", "extra"}
    skip_events = [e for e in trace["events"] if e["action"] == "skip"]
    assert len(skip_events) >= 1, trace["events"]
