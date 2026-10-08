"""Real placement + 360 deg room survey (sim battery 2026-09-26, run
s70718-003, ``t2-rand-70718-fix3/runs/s70718-003-findObjInRoom``).

Deliverable 1 -- ``create_place`` used to open the gripper from the TUCKED
pose after the goto: the banana dropped on the floor and the bench still
recorded placed(). It now runs a real placement (arm to the RealSense scan
pose -> wrist point cloud -> free spot on the detected plane ->
``place_action``) and only on failure falls back to the old release.

Deliverable 2 -- a room-wide search only looked +-45 deg from each search
spot. ``create_find_object`` now ends with a gated 360 deg survey (four
90 deg base turns, full pan/tilt sweep at each heading).

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 (ROS pytest plugins break
collection).
"""
from __future__ import annotations

import math

import numpy as np
import py_trees
import pytest
from geometry_msgs.msg import PoseStamped, PointStamped, Pose
from py_trees.common import Access, Status
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

from behavior_tree.GPSR import small_trees
from behavior_tree.GPSR.small_trees import (
    BtNode_CheckRoomSurveyDue,
    BtNode_RotateInPlace,
    BtNode_SurfacePointFromCloud,
    BtNode_YawOffsetPose,
    PLACE_ACTION_NAME,
    PLACE_TCP_CLEARANCE_M,
    bb_keys,
    create_survey_room,
)
from behavior_tree.TemplateNodes.Manipulation import BtNode_Place
from behavior_tree.TemplateNodes.Navigation import BtNode_CaptureCurrentPose, BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import BtNode_GetPointCloud


def _walk(node):
    yield node
    for child in getattr(node, "children", ()):
        yield from _walk(child)


def _names(tree):
    return [n.name for n in _walk(tree)]


def _first(tree, cls):
    for n in _walk(tree):
        if isinstance(n, cls):
            return n
    raise AssertionError(f"no {cls.__name__} in {tree.name}")


def _seed(**pairs):
    py_trees.blackboard.Blackboard.clear()
    writer = py_trees.blackboard.Client(name="seed")
    for key, value in pairs.items():
        writer.register_key(key, access=Access.WRITE)
        writer.set(key, value, overwrite=True)
    return writer


def _read(key):
    reader = py_trees.blackboard.Client(name="reader")
    reader.register_key(key, access=Access.READ)
    try:
        return reader.get(key)
    except KeyError:
        return None


# ---------------------------------------------------------------------------
# Deliverable 1: create_place structure
# ---------------------------------------------------------------------------

def test_place_tree_order_nav_then_surface_selector_then_arm_back():
    tree = small_trees.create_place()
    assert tree.name == "small/place"
    top = [c.name for c in tree.children]
    assert top == [
        "announce placing",
        "retry tuck arm before place goto",
        "retry place goto",
        "place/on_surface",
        "arm back after place (best effort)",
    ]
    selector = tree.children[3]
    assert isinstance(selector, py_trees.composites.Selector)
    assert [c.name for c in selector.children] == ["place/primary", "place/release_fallback"]


def test_place_primary_scans_surface_with_wrist_camera_then_calls_place_action():
    tree = small_trees.create_place()
    primary = tree.children[3].children[0]
    names = _names(primary)
    # arm to the RealSense scan pose (with octomap) BEFORE the surface scan,
    # the scan BEFORE the place, the method claim LAST.
    order = [
        "arm to scan", "settle before surface scan", "realsense surface cloud",
        "find free surface point", "place on surface", "open gripper after place",
        "close gripper after place", "mark place autonomous",
    ]
    idx = [names.index(n) for n in order]
    assert idx == sorted(idx), names
    assert primary.children[-1].name == "mark place autonomous"

    arm = next(n for n in _walk(primary) if n.name == "arm to scan")
    assert arm.arm_pose_bb_key == bb_keys.ARM_SCAN
    assert arm.add_octomap is True

    cloud = _first(primary, BtNode_GetPointCloud)
    assert cloud.camera == "realsense"
    assert cloud.key == bb_keys.PLACE_ENV_POINTS

    surf = _first(primary, BtNode_SurfacePointFromCloud)
    assert surf.bb_cloud_key == bb_keys.PLACE_ENV_POINTS
    assert surf.bb_point_key == bb_keys.PLACE_TARGET_POINT
    assert surf.bb_orientation_key == bb_keys.PLACE_ORIENTATION

    place = _first(primary, BtNode_Place)
    assert place.action_name == PLACE_ACTION_NAME == "place_action"
    assert place.bb_key_point == bb_keys.PLACE_TARGET_POINT
    assert place.bb_key_pose == bb_keys.PLACE_ORIENTATION
    # the scan is retried; the place is retried once
    retry_scan = next(n for n in _walk(primary) if n.name == "retry scan surface")
    assert retry_scan.num_failures == 3
    retry_place = next(n for n in _walk(primary) if n.name == "retry place on surface")
    assert retry_place.num_failures == 2


def test_place_fallback_announces_then_releases_in_place():
    tree = small_trees.create_place()
    fallback = tree.children[3].children[1]
    names = [c.name for c in fallback.children]
    assert names == [
        "mark place release fallback", "announce release fallback",
        "open gripper", "wait", "close gripper",
    ]
    announce = fallback.children[1]
    assert "could not place it safely" in announce.given_msg
    assert "put it down here" in announce.given_msg
    from behavior_tree.TemplateNodes.Manipulation import BtNode_GripperAction
    assert isinstance(fallback.children[2], BtNode_GripperAction)
    assert isinstance(fallback.children[4], BtNode_GripperAction)


def _build_place_selector_with_stubs(primary_status: Status):
    """The real place/on_surface Selector with primary's heavy tail and the
    fallback's ROS tail replaced by stubs, so it ticks without ROS."""
    tree = small_trees.create_place()
    selector = tree.children[3]
    primary, fallback = selector.children
    marker = primary.children[-1]
    assert marker.name == "mark place autonomous"
    for child in list(primary.children[:-1]):
        primary.remove_child(child)
    stub = (
        py_trees.behaviours.Success(name="primary stub")
        if primary_status is Status.SUCCESS
        else py_trees.behaviours.Failure(name="primary stub")
    )
    primary.insert_child(stub, 0)
    fb_marker = fallback.children[0]
    assert fb_marker.name == "mark place release fallback"
    for child in list(fallback.children[1:]):
        fallback.remove_child(child)
    fallback.add_child(py_trees.behaviours.Success(name="fallback tail stub"))
    tree.remove_child(selector)
    py_trees.trees.BehaviourTree(selector).setup()
    return selector


def test_place_primary_success_claims_autonomous():
    _seed()
    selector = _build_place_selector_with_stubs(Status.SUCCESS)
    selector.tick_once()
    assert selector.status is Status.SUCCESS
    assert _read(bb_keys.STEP_METHOD) == "autonomous"


def test_place_primary_failure_falls_back_to_release_and_still_succeeds():
    _seed()
    selector = _build_place_selector_with_stubs(Status.FAILURE)
    selector.tick_once()
    assert selector.status is Status.SUCCESS  # the executor never hangs
    assert _read(bb_keys.STEP_METHOD) == "release_fallback"


# ---------------------------------------------------------------------------
# Deliverable 1: surface point selection
# ---------------------------------------------------------------------------

def _table_cloud(plane_z=0.75, obstacles=(), noise=0.002, step=0.01):
    """Synthetic base_link cloud: a table plane in front of the robot plus
    box obstacles ``(cx, cy, half, height)`` standing on it."""
    xs = np.arange(0.20, 1.00, step)
    ys = np.arange(-0.60, 0.60, step)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    rng = np.random.default_rng(0)
    pts = [np.column_stack([gx.ravel(), gy.ravel(),
                            plane_z + rng.normal(0, noise, gx.size)])]
    for cx, cy, half, height in obstacles:
        bx = np.arange(cx - half, cx + half, step)
        by = np.arange(cy - half, cy + half, step)
        bz = np.arange(plane_z, plane_z + height, step)
        ox, oy, oz = np.meshgrid(bx, by, bz, indexing="ij")
        pts.append(np.column_stack([ox.ravel(), oy.ravel(), oz.ravel()]))
    return np.vstack(pts)


def test_pick_surface_point_finds_plane_and_prefers_nominal_spot():
    xyz = _table_cloud(plane_z=0.75)
    x, y, plane_z = BtNode_SurfacePointFromCloud.pick_surface_point(xyz)
    assert abs(plane_z - 0.75) < 0.01
    assert abs(x - 0.50) < 0.03 and abs(y) < 0.03


def test_pick_surface_point_avoids_obstacle_on_the_preferred_spot():
    xyz = _table_cloud(plane_z=0.72, obstacles=[(0.50, 0.00, 0.04, 0.12)])
    x, y, plane_z = BtNode_SurfacePointFromCloud.pick_surface_point(xyz, free_radius=0.08)
    assert abs(plane_z - 0.72) < 0.01
    # clear of the obstacle's footprint by the free radius
    assert math.hypot(x - 0.50, y) >= 0.08 + 0.04 - 0.02
    # ... but still the nearest clear cell to the nominal spot, within reach
    assert 0.30 <= x <= 0.75 and abs(y) <= 0.40
    assert math.hypot(x - 0.50, y) < 0.30


def test_pick_surface_point_rejects_floor_and_empty_scenes():
    floor = _table_cloud(plane_z=0.0)
    assert BtNode_SurfacePointFromCloud.pick_surface_point(floor) is None
    assert BtNode_SurfacePointFromCloud.pick_surface_point(np.zeros((0, 3))) is None
    # a plane fully covered by clutter has no free cell
    cluttered = _table_cloud(plane_z=0.8, obstacles=[(0.55, 0.0, 0.45, 0.10)])
    assert BtNode_SurfacePointFromCloud.pick_surface_point(cluttered) is None


def _cloud_msg(xyz, frame_id="base_link"):
    return pc2.create_cloud_xyz32(Header(frame_id=frame_id), [tuple(map(float, p)) for p in xyz])


def test_surface_node_writes_tcp_target_and_empty_orientation():
    _seed()
    node = BtNode_SurfacePointFromCloud(
        "find free surface point", bb_cloud_key="t/cloud", bb_point_key="t/point",
        bb_orientation_key="t/orient",
    )
    writer = py_trees.blackboard.Client(name="w")
    writer.register_key("t/cloud", access=Access.WRITE)
    writer.set("t/cloud", _cloud_msg(_table_cloud(plane_z=0.70, step=0.02)), overwrite=True)
    node.setup()
    node.tick_once()
    assert node.status is Status.SUCCESS, node.feedback_message
    point = _read("t/point")
    assert isinstance(point, PointStamped)
    assert point.header.frame_id == "base_link"
    assert abs(point.point.z - (0.70 + PLACE_TCP_CLEARANCE_M)) < 0.01
    orient = _read("t/orient")
    assert isinstance(orient, Pose)
    q = orient.orientation
    assert (q.x, q.y, q.z, q.w) == (0.0, 0.0, 0.0, 0.0)  # "not supplied" for place_action


def test_surface_node_fails_on_foreign_frame_without_tf_and_on_empty_cloud(monkeypatch):
    _seed()
    node = BtNode_SurfacePointFromCloud(
        "find free surface point", bb_cloud_key="t/cloud", bb_point_key="t/point",
    )
    writer = py_trees.blackboard.Client(name="w")
    writer.register_key("t/cloud", access=Access.WRITE)
    writer.set("t/cloud", _cloud_msg(_table_cloud(step=0.02), frame_id="camera_optical"),
               overwrite=True)
    node.setup()
    node.tick_once()
    assert node.status is Status.FAILURE
    assert "no tf2 buffer" in node.feedback_message

    from behavior_tree import config as bt_config
    monkeypatch.setattr(bt_config, "is_node_mocked", lambda name: False)
    writer.set("t/cloud", _cloud_msg(np.zeros((0, 3))), overwrite=True)
    node.tick_once()
    assert node.status is Status.FAILURE
    assert "empty" in node.feedback_message


def test_surface_node_synthesises_point_when_cloud_provider_is_mocked(monkeypatch):
    _seed()
    from behavior_tree import config as bt_config
    monkeypatch.setattr(bt_config, "is_node_mocked",
                        lambda name: name == "BtNode_GetPointCloud")
    node = BtNode_SurfacePointFromCloud(
        "find free surface point", bb_cloud_key="t/cloud", bb_point_key="t/point",
    )
    writer = py_trees.blackboard.Client(name="w")
    writer.register_key("t/cloud", access=Access.WRITE)
    writer.set("t/cloud", _cloud_msg(np.zeros((0, 3))), overwrite=True)
    node.setup()
    node.tick_once()
    assert node.status is Status.SUCCESS
    assert isinstance(_read("t/point"), PointStamped)


# ---------------------------------------------------------------------------
# Deliverable 2: rotate node yaw math
# ---------------------------------------------------------------------------

def _pose(x, y, yaw_deg, frame="map"):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x, ps.pose.position.y = x, y
    ps.pose.orientation = BtNode_YawOffsetPose.quaternion_from_yaw(yaw_deg)
    return ps


@pytest.mark.parametrize("yaw, delta, expected", [
    (0.0, 90.0, 90.0),
    (90.0, 90.0, 180.0),
    (170.0, 90.0, -100.0),     # wraps past +180
    (-135.0, -90.0, 135.0),    # wraps past -180
    (-90.0, 90.0, 0.0),
    (45.0, 360.0, 45.0),
])
def test_rotate_pose_adds_yaw_and_wraps(yaw, delta, expected):
    out = BtNode_YawOffsetPose.rotate_pose(_pose(1.5, -2.0, yaw), delta)
    got = BtNode_YawOffsetPose.yaw_from_quaternion(out.pose.orientation)
    assert abs(BtNode_YawOffsetPose.wrap_deg(got - expected)) < 1e-6
    assert (out.pose.position.x, out.pose.position.y) == (1.5, -2.0)
    assert out.header.frame_id == "map"
    q = out.pose.orientation
    assert abs(math.hypot(math.hypot(q.x, q.y), math.hypot(q.z, q.w)) - 1.0) < 1e-9


def test_yaw_offset_node_reads_capture_and_writes_goal():
    _seed(**{bb_keys.SURVEY_CAPTURE_POSE: _pose(0.3, 0.4, 100.0)})
    node = BtNode_YawOffsetPose("yaw", bb_keys.SURVEY_CAPTURE_POSE, bb_keys.SURVEY_GOAL_POSE, 90.0)
    node.tick_once()
    assert node.status is Status.SUCCESS
    goal = _read(bb_keys.SURVEY_GOAL_POSE)
    assert abs(BtNode_YawOffsetPose.yaw_from_quaternion(goal.pose.orientation) - (-170.0)) < 1e-6


def test_yaw_offset_node_fails_without_a_capture():
    _seed()
    node = BtNode_YawOffsetPose("yaw", bb_keys.SURVEY_CAPTURE_POSE, bb_keys.SURVEY_GOAL_POSE, 90.0)
    node.tick_once()
    assert node.status is Status.FAILURE


def test_rotate_in_place_is_capture_then_yaw_then_goto():
    node = BtNode_RotateInPlace("rotate", delta_yaw_deg=90.0)
    assert node.delta_yaw_deg == 90.0
    capture, yaw, retry = node.children
    assert isinstance(capture, BtNode_CaptureCurrentPose)
    assert capture.bb_key == bb_keys.SURVEY_CAPTURE_POSE
    assert isinstance(yaw, BtNode_YawOffsetPose)
    assert (yaw.src_key, yaw.dst_key, yaw.delta_yaw_deg) == (
        bb_keys.SURVEY_CAPTURE_POSE, bb_keys.SURVEY_GOAL_POSE, 90.0)
    assert isinstance(retry, py_trees.decorators.Retry)
    goto = retry.decorated
    assert isinstance(goto, BtNode_GotoAction)
    assert goto._key == bb_keys.SURVEY_GOAL_POSE


# ---------------------------------------------------------------------------
# Deliverable 2: survey tree + find_object wiring
# ---------------------------------------------------------------------------

def test_survey_room_has_four_rotates_and_four_sweeps_in_order():
    calls = []

    def make_detect():
        leaf = py_trees.behaviours.Success(name=f"detect {len(calls)}")
        calls.append(leaf)
        return leaf

    survey = create_survey_room(make_detect, [0.0, 20.0], label="t")
    assert isinstance(survey, py_trees.composites.Selector)
    assert len(survey.children) == 4
    rotates = [n for n in _walk(survey) if isinstance(n, BtNode_RotateInPlace)]
    sweeps = [n for n in _walk(survey) if n.name.endswith("survey sweep")]
    assert len(rotates) == 4 and len(sweeps) == 4
    assert all(r.delta_yaw_deg == 90.0 for r in rotates)
    # the survey looks are NOT extra "pantilt sweep" (find_object_sweep) targets
    assert not any("pantilt sweep" in n.name for n in _walk(survey))
    for branch in survey.children:
        names = _names(branch)
        assert names.index(next(n for n in names if "rotate" in n)) < names.index(
            next(n for n in names if n.endswith("survey sweep")))
    # every (pan, tilt) look got its own fresh detector: 4 x 3 pans x 2 tilts
    assert len(calls) == 4 * len(small_trees.PAN_SWEEP_DEG) * 2


def test_find_object_survey_is_the_last_branch_after_sweep_and_reduction():
    tree = small_trees.create_find_object()
    selector = tree.children[1]
    assert selector.name == "find_object sweep with query reduction"
    names = [c.name for c in selector.children]
    assert names[0] == "find_object pantilt sweep"
    assert names[1] == "find_object reduced-query retry"
    assert names[-1] == "find_object room survey"
    survey = selector.children[-1]
    assert isinstance(survey.children[0], BtNode_CheckRoomSurveyDue)
    assert survey.children[-1].name == "find_object survey 360 survey"
    assert len([n for n in _walk(survey) if isinstance(n, BtNode_RotateInPlace)]) == 4
    # the existing steep-tilt look is still there and still first
    assert "find_object pan=+0 tilt=+45" in _names(selector.children[0])


def test_find_object_pan_tilt_override_reaches_the_survey_sweeps():
    tree = small_trees.create_find_object(pan_deg=[0.0, 90.0], tilt_deg=[10.0])
    survey = tree.children[1].children[-1]
    names = _names(survey)
    assert "find_object survey heading 1 pan=+90 tilt=+10" in names
    assert not any("tilt=+45" in n for n in names)


# ---------------------------------------------------------------------------
# Deliverable 2: the survey gate
# ---------------------------------------------------------------------------

@pytest.fixture()
def rooms(monkeypatch):
    from behavior_tree.GPSR import orchestrator as orch
    monkeypatch.setattr(orch, "ROOM_SEARCH_SPOTS", {"bedroom": ["side_table_02", "bed"]})


def test_survey_gate_requires_a_room(rooms):
    gate = BtNode_CheckRoomSurveyDue()
    _seed(**{bb_keys.TARGET_LOCATION: "side_table_02"})
    gate.setup()
    gate.tick_once()
    assert gate.status is Status.FAILURE
    _seed(**{bb_keys.TARGET_LOCATION: "Bedroom"})
    gate.tick_once()
    assert gate.status is Status.SUCCESS
    _seed()
    gate.tick_once()
    assert gate.status is Status.FAILURE


def test_survey_gate_waits_for_the_last_filled_search_spot(rooms):
    sweep = small_trees.create_search_object(capacity=4)
    gates = {}
    for branch in sweep.children:
        idx = int(branch.name.rsplit(" ", 1)[-1])
        gates[idx] = next(n for n in _walk(branch) if isinstance(n, BtNode_CheckRoomSurveyDue))
    # spots 0 and 2 filled, 1 and 3 empty
    _seed(**{
        bb_keys.TARGET_LOCATION: "bedroom",
        "gpsr/search_pose_0": "pose", "gpsr/search_pose_1": None,
        "gpsr/search_pose_2": "pose", "gpsr/search_pose_3": None,
    })
    for gate in gates.values():
        gate.setup()
    gates[0].tick_once()
    assert gates[0].status is Status.FAILURE
    assert "search spot 2 still pending" in gates[0].feedback_message
    gates[2].tick_once()
    assert gates[2].status is Status.SUCCESS
    # not a room -> never, even at the last spot
    _seed(**{bb_keys.TARGET_LOCATION: "side_table_02", "gpsr/search_pose_2": "pose"})
    gates[2].tick_once()
    assert gates[2].status is Status.FAILURE


def _small_table_cloud(x0, x1, y0, y1, plane_z=0.53, step=0.01):
    xs = np.arange(x0, x1, step)
    ys = np.arange(y0, y1, step)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    return np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, plane_z)])


def test_pick_surface_point_keeps_off_the_near_edge_of_a_small_table():
    # fix8/fix9 run s70718-003: side_table_02 (0.38 m square) seen from 0.46 m
    # away; the nominal 0.50 m spot was 4 cm inside the near edge and the
    # banana tipped off. Now the spot has plane support 10 cm out all round.
    from behavior_tree.GPSR.small_trees import PLACE_EDGE_MARGIN_M
    xyz = _small_table_cloud(0.46, 0.84, -0.19, 0.19)
    x, y, _ = BtNode_SurfacePointFromCloud.pick_surface_point(xyz)
    # >= 10 cm of table on every side (probe offset minus its 2 cm radius)
    assert x - 0.46 >= PLACE_EDGE_MARGIN_M - 0.02
    assert 0.84 - x >= PLACE_EDGE_MARGIN_M - 0.02
    assert abs(y) <= 0.19 - PLACE_EDGE_MARGIN_M + 0.02


def test_pick_surface_point_falls_back_when_no_spot_has_the_margin():
    # a 12 cm wide shelf board: nothing has 10 cm all round, still place
    xyz = _small_table_cloud(0.40, 0.70, -0.06, 0.06)
    assert BtNode_SurfacePointFromCloud.pick_surface_point(xyz) is not None
