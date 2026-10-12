"""Full GPSR battery 2026-10-10, issue report 2026-10-11 Bug A / Bug B (decision side).

Bug B: the grasp drive-up sent GoToApproach a head-camera centroid at
z=0.46 for an object on the 0.734 m kitchen_table (cause still open). The planner's
barrier rule (min_height = target_z + 0.10) then counted the chairs around
the table as a barrier and aborted. The same table with z=0.70 passed. The
real bowl on side_table_02 came back 0.13 m low the same way, so a low z is
clamped up to the known surface top, not rejected; only a centroid far below
the top (more than SURFACE_Z_CLAMP_BAND_M) skips the drive-up.

Bug A: the fixed furniture poses must leave room for the sim footprint
(circumscribed radius 0.525 m) to turn in place.

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import py_trees
import pytest
from geometry_msgs.msg import Point, PointStamped
from py_trees.common import Access, Status
from std_msgs.msg import Header

from behavior_tree.GPSR import small_trees
from behavior_tree.GPSR.small_trees import bb_keys

CONSTANTS = Path(small_trees.__file__).parent / "constants.rcw2026.json"


def _walk(node):
    yield node
    for child in getattr(node, "children", []) or []:
        yield from _walk(child)
    decorated = getattr(node, "decorated", None)
    if decorated is not None and decorated not in (getattr(node, "children", []) or []):
        yield from _walk(decorated)


@pytest.fixture(autouse=True)
def _clean_blackboard():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


@pytest.fixture()
def surfaces(monkeypatch):
    from behavior_tree.GPSR import orchestrator as orch
    monkeypatch.setattr(orch, "SURFACE_TOP_Z", {"kitchen_table": 0.734, "side_table_02": 0.612})


def _writer(*keys):
    client = py_trees.blackboard.Client(name="seed")
    for key in keys:
        client.register_key(key, access=Access.WRITE)
        client.register_key(key, access=Access.READ)
    return client


def _clamp(location, z):
    bb = _writer(bb_keys.GRASP_HEAD_POINT, bb_keys.TARGET_LOCATION)
    bb.set(bb_keys.TARGET_LOCATION, location, overwrite=True)
    bb.set(bb_keys.GRASP_HEAD_POINT,
           PointStamped(header=Header(frame_id="map"), point=Point(x=2.83, y=-3.11, z=z)),
           overwrite=True)
    node = small_trees.BtNode_ClampTargetToSurface("clamp", bb_keys.GRASP_HEAD_POINT)
    node.setup()
    status = node.update()
    return status, bb.get(bb_keys.GRASP_HEAD_POINT), node


# ---------------------------------------------------------------------------
# Bug B: surface z clamp before the grasp drive-up
# ---------------------------------------------------------------------------

def test_low_centroid_on_a_known_table_is_clamped_to_the_surface_top(surfaces):
    # s70718-004: mug at z=0.46 on the 0.734 kitchen_table.
    status, point, node = _clamp("kitchen_table", 0.46)
    assert status is Status.SUCCESS
    assert point.point.z == pytest.approx(0.734)
    assert (point.point.x, point.point.y) == (2.83, -3.11)
    assert "clamped" in node.feedback_message and "0.46" in node.feedback_message


def test_bowl_low_by_the_same_bias_is_kept_and_clamped(surfaces):
    # s20261002-002: the real bowl came back at z=0.48 on the 0.612 side table.
    status, point, _ = _clamp("side_table_02", 0.48)
    assert status is Status.SUCCESS and point.point.z == pytest.approx(0.612)


def test_centroid_above_the_surface_is_untouched(surfaces):
    status, point, node = _clamp("kitchen_table", 0.80)
    assert status is Status.SUCCESS and point.point.z == pytest.approx(0.80)
    assert "clamped" not in node.feedback_message


def test_centroid_far_below_the_surface_skips_the_drive_up(surfaces):
    status, point, node = _clamp("kitchen_table", 0.734 - small_trees.SURFACE_Z_CLAMP_BAND_M - 0.01)
    assert status is Status.FAILURE
    assert point.point.z == pytest.approx(0.734 - small_trees.SURFACE_Z_CLAMP_BAND_M - 0.01)
    assert "below" in node.feedback_message


def test_unknown_surface_leaves_the_point_alone(surfaces):
    # shelves have several levels and rooms have no single surface
    for location in ("shelf", "kitchen", None):
        status, point, _ = _clamp(location, 0.30)
        assert status is Status.SUCCESS and point.point.z == pytest.approx(0.30)


def test_drive_up_clamps_the_target_before_the_approach():
    from behavior_tree.TemplateNodes.Navigation import BtNode_Approach

    tree = small_trees._grasp_table_detection()
    head_view = next(n for n in _walk(tree) if n.name == "table view 2: head camera re-aim")
    drive = next(n for n in head_view.children if n.name == "view 2: drive up (best effort)")
    steps = list(drive.decorated.children)
    kinds = [type(s) for s in steps]
    clamp = kinds.index(small_trees.BtNode_ClampTargetToSurface)
    assert kinds.index(small_trees.BtNode_ExtractDetection) < clamp < kinds.index(BtNode_Approach)
    assert steps[clamp]._point_key == bb_keys.GRASP_HEAD_POINT


def test_rcw2026_constants_carry_single_level_surface_tops():
    from behavior_tree.GPSR import orchestrator as orch

    orch.load_knowledge_from_constants(str(CONSTANTS))
    assert orch.SURFACE_TOP_Z["kitchen_table"] == pytest.approx(0.734)
    assert orch.SURFACE_TOP_Z["laundry_desk"] == pytest.approx(0.734)
    assert orch.SURFACE_TOP_Z["side_table"] == pytest.approx(0.612)
    assert orch.SURFACE_TOP_Z["side_table_02"] == pytest.approx(0.612)
    assert "shelf" not in orch.SURFACE_TOP_Z and "shelf_02" not in orch.SURFACE_TOP_Z


def test_constants_without_surface_heights_load_an_empty_table():
    from behavior_tree.GPSR import orchestrator as orch

    try:
        orch.load_knowledge_from_constants(str(CONSTANTS.parent / "constants.json"))
        assert orch.SURFACE_TOP_Z == {}
    finally:
        orch.load_knowledge_from_constants(str(CONSTANTS))


# ---------------------------------------------------------------------------
# Bug B: hold the head still briefly after a pan/tilt settle before scanning
# ---------------------------------------------------------------------------

def test_head_views_before_a_scan_hold_after_settling():
    from behavior_tree.TemplateNodes.Vision import BtNode_TurnPanTilt

    assert small_trees.HEAD_SETTLE_HOLD_S > 0.0
    tree = small_trees._grasp_table_detection()
    tilt = next(n for n in _walk(tree) if isinstance(n, BtNode_TurnPanTilt)
                and n.name.startswith("view 2: head tilt"))
    assert tilt.post_settle_sec == small_trees.HEAD_SETTLE_HOLD_S
    sweep = small_trees._pantilt_sweep("t", [20.0], lambda: py_trees.behaviours.Success("d"))
    turns = [n for n in _walk(sweep) if isinstance(n, BtNode_TurnPanTilt)]
    assert turns and all(t.post_settle_sec == small_trees.HEAD_SETTLE_HOLD_S for t in turns)


def test_turn_pan_tilt_holds_running_for_the_post_settle_time(monkeypatch):
    from types import SimpleNamespace
    from behavior_tree.TemplateNodes import Vision

    clock = {"t": 100.0}
    monkeypatch.setattr(Vision.time, "monotonic", lambda: clock["t"])
    node = Vision.BtNode_TurnPanTilt.__new__(Vision.BtNode_TurnPanTilt)
    node.mock_mode = False
    node.require_settled = False
    node.post_settle_sec = 0.5
    node.publisher = SimpleNamespace(publish=lambda msg: None)
    node._build_command_msg = lambda pan, tilt: None
    node.x, node.y = 0.0, 45.0
    node._arm_settle(0.0, 45.0)
    node._head_moves = True
    node._last_publish_time = clock["t"]
    node._latest_state = SimpleNamespace(feedback_ok=True, pan_rad=0.0, tilt_rad=math.radians(45.0))

    assert node.update() is Status.RUNNING          # settled now, hold starts
    clock["t"] += 0.4
    assert node.update() is Status.RUNNING
    clock["t"] += 0.2
    assert node.update() is Status.SUCCESS


def test_turn_pan_tilt_skips_the_hold_when_the_head_was_already_there(monkeypatch):
    from types import SimpleNamespace
    from behavior_tree.TemplateNodes import Vision

    node = Vision.BtNode_TurnPanTilt.__new__(Vision.BtNode_TurnPanTilt)
    node.mock_mode = False
    node.require_settled = False
    node.post_settle_sec = 0.5
    node.x_key = node.y_key = node.speed_key = None
    node.x, node.y, node.speed = 0.0, 45.0, 0.0
    node.publisher = SimpleNamespace(publish=lambda msg: None)
    node._build_command_msg = lambda pan, tilt: None
    node.logger = SimpleNamespace(info=lambda msg: None)
    node._latest_state = SimpleNamespace(feedback_ok=True, pan_rad=0.0, tilt_rad=math.radians(45.0))
    node.initialise()
    assert node._head_moves is False
    assert node.update() is Status.SUCCESS


def test_turn_pan_tilt_default_has_no_hold():
    from behavior_tree.TemplateNodes.Vision import BtNode_TurnPanTilt

    node =BtNode_TurnPanTilt("t", x=0.0, y=20.0)
    assert node.post_settle_sec == 0.0


# ---------------------------------------------------------------------------
# Bug A: fixed furniture poses leave turning room for the sim footprint
# ---------------------------------------------------------------------------

def _yaw_deg(o):
    return math.degrees(2.0 * math.atan2(o["z"], o["w"]))


@pytest.mark.parametrize("name, xy, yaw", [
    # static-map costmap_check (SIM footprint): 0/72 lethal headings,
    # >= 0.60 m to the nearest map obstacle; the old poses had 10, 31 and 22.
    ("kitchen_table", (3.56, -2.14), -135.0),
    ("side_table", (-0.62, -4.18), -72.0),
    ("shelf_02", (1.05, -0.70), 180.0),
    ("side_table_02", (1.172, 1.755), 180.0),   # already 0/72, unchanged
])
def test_furniture_nav_poses_are_turnable(name, xy, yaw):
    poses = json.loads(CONSTANTS.read_text())["possible_poses"]
    p = poses[name]
    assert (p["point"]["x"], p["point"]["y"]) == pytest.approx(xy)
    assert (_yaw_deg(p["orientation"]) - yaw + 180.0) % 360.0 - 180.0 == pytest.approx(0.0, abs=0.5)
