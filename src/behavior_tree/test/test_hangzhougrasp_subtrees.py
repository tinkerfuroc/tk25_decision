from types import SimpleNamespace

import py_trees
import pytest

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.hangzhougrasp import hangzhougrasp as hg
from behavior_tree.hangzhougrasp.config import KEY_HOLDING, KEY_SCAN_RESULT
from behavior_tree.hangzhougrasp.pose_index import PoseRecord
from behavior_tree.hangzhougrasp.tk27_nodes import BtNode_Tk27GripperOpen

S = py_trees.common.Status

TABLE_HEAD = "Head to table scan tilt"
ORBBEC_SCAN = "Scan objects with YOLO26 SAM"
ANNOUNCE = "Announce detected objects in Chinese"
TABLE_POSE = "Arm to table pose"
REALSENSE_SCAN = "Scan table with RealSense"
SELECT = "Select nearest label"
OPEN = "Open gripper before pick"
PICK = "Pick selected object"
RETRACT = "Retract arm with object"
CLEANUP = "Retract arm on failure"
SAW = "Scan saw objects?"


class Leaf(py_trees.behaviour.Behaviour):
    def __init__(self, name, status, log):
        super().__init__(name)
        self.result, self.log = status, log

    def update(self):
        self.log.append(self.name)
        return self.result


def _rec(n):
    return PoseRecord("a", "table", 1, n, f"a_table_1_{n}", None)


def _cfg(**over):
    values = dict(
        nav_retry_limit=1, navigation_action="nav", destination_name="dest",
        prompt="coke", label_map={"coke": "可乐"}, announce_service="ann",
        announce_retry_limit=1, detection_service="det", scan_retry_limit=1,
        arm_retry_limit=1, grasp_retry_limit=1,
        saved_pose_action="/manipulation/move_to_saved_pose",
        arm_pose_table="pick45", arm_pose_navigating="navigation",
        pick_action="/manipulation/pick", pick_plan_effort="high",
        gripper_action="/xarm_gripper/gripper_action",
        gripper_open_position=0.0, gripper_open_tolerance=0.025,
        gripper_max_effort=50.0, head_pan_deg=0.0, table_scan_tilt_deg=20.0,
        ground_scan_tilt_deg=0.0, head_retry_limit=1)
    values.update(over)
    return SimpleNamespace(**values)


@pytest.fixture(autouse=True)
def clear_bb():
    py_trees.blackboard.Blackboard.storage.clear()
    yield
    py_trees.blackboard.Blackboard.storage.clear()


def _stub(monkeypatch, log, goto=S.SUCCESS, grasp=S.SUCCESS, overrides=None,
          real_scan_check=False):
    """Stub ROS primitives with leaves; `overrides` maps node name -> Status."""
    status = {PICK: grasp}
    status.update(overrides or {})

    def leaf(name):
        return Leaf(name, status.get(name, S.SUCCESS), log)

    monkeypatch.setattr(
        hg, "_gotoRetry", lambda name, key, config: Leaf(name, goto, log))
    for cls in ("BtNode_Tk27SavedPose", "BtNode_Tk27Pick", "BtNode_Tk27GripperOpen",
                "BtNode_SelectNearestLabel", "BtNode_AnnounceDetections",
                "BtNode_HeadTilt"):
        monkeypatch.setattr(hg, cls, lambda **kw: leaf(kw["name"]))
    if not real_scan_check:
        monkeypatch.setattr(hg, "BtNode_ScanSawObjects", lambda **kw: leaf(kw["name"]))
    monkeypatch.setattr(
        hg, "_scanYolo26Retry", lambda name, config, **kw: leaf(name))


def _tick(root):
    """Tick to completion: Retry reports RUNNING between attempts."""
    for _ in range(50):
        root.tick_once()
        if root.status != S.RUNNING:
            break
    return root.status


def _holding():
    return py_trees.blackboard.Blackboard.storage.get("/" + KEY_HOLDING)


def _arm_or_grasp(log):
    return [n for n in log if n.startswith(("Arm", "Retract", "Pick", "Open"))]


# ---- visit-table behaviour (ported) ----
def test_visit_table_stops_after_first_grasp(monkeypatch):
    log = []
    _stub(monkeypatch, log)
    root = hg.createVisitTable(_cfg(), [_rec(0), _rec(1)])
    assert _tick(root) == S.SUCCESS
    assert log.count("Go to a_table_1_0") == 1
    assert "Go to a_table_1_1" not in log
    assert _holding() is True


def test_visit_table_skips_on_miss(monkeypatch):
    log = []
    _stub(monkeypatch, log, grasp=S.FAILURE)
    root = hg.createVisitTable(_cfg(), [_rec(0), _rec(1)])
    assert _tick(root) == S.SUCCESS
    assert log.count(PICK) == 2
    assert log.count(CLEANUP) == 2
    assert not _holding()


def test_visit_table_navigation_failure_fails(monkeypatch):
    log = []
    _stub(monkeypatch, log, goto=S.FAILURE)
    root = hg.createVisitTable(_cfg(), [_rec(0), _rec(1)])
    assert _tick(root) == S.FAILURE
    assert PICK not in log


def test_cleanup_stow_failure_still_fails_run(monkeypatch):
    log = []
    _stub(monkeypatch, log, grasp=S.FAILURE, overrides={CLEANUP: S.FAILURE})
    root = hg.createVisitTable(_cfg(), [_rec(0), _rec(1)])
    assert _tick(root) == S.FAILURE
    assert "Go to a_table_1_1" not in log
    assert not _holding()


def test_post_grasp_retract_failure_keeps_holding_and_skips_rest(monkeypatch):
    log = []
    _stub(monkeypatch, log, overrides={RETRACT: S.FAILURE})
    root = hg.createVisitTable(_cfg(), [_rec(0), _rec(1)])
    assert _tick(root) == S.SUCCESS
    assert _holding() is True
    assert CLEANUP in log
    assert "Go to a_table_1_1" not in log


def test_post_grasp_retract_and_cleanup_failure_fails_with_holding(monkeypatch):
    log = []
    _stub(monkeypatch, log, overrides={RETRACT: S.FAILURE, CLEANUP: S.FAILURE})
    root = hg.createVisitTable(_cfg(), [_rec(0)])
    assert _tick(root) == S.FAILURE
    assert _holding() is True


def test_scan_grasp_part_miss_still_fails_after_recovery(monkeypatch):
    log = []
    _stub(monkeypatch, log, grasp=S.FAILURE)
    root = hg.createScanGraspWithCleanup(_cfg())
    assert _tick(root) == S.FAILURE
    assert CLEANUP in log


# ---- TK27 viewpoint flow ----
def test_viewpoint_order(monkeypatch):
    log = []
    _stub(monkeypatch, log)
    root = hg.createViewpointScanGrasp(_cfg(), miss_is_success=True)
    assert _tick(root) == S.SUCCESS
    # SetHolding is a real (unlogged) node; test_holding_set_before_retract
    # pins that it runs between Pick and the retract.
    assert log == [TABLE_HEAD, ORBBEC_SCAN, ANNOUNCE, SAW, SELECT, TABLE_POSE,
                   OPEN, PICK, RETRACT]
    assert _holding() is True


def test_head_failure_is_logged_and_scan_still_runs(monkeypatch):
    log = []
    _stub(monkeypatch, log, overrides={TABLE_HEAD: S.FAILURE})
    root = hg.createViewpointScanGrasp(_cfg(), miss_is_success=True)
    assert _tick(root) == S.SUCCESS
    assert log[:2] == [TABLE_HEAD, ORBBEC_SCAN]
    logged = [n for n in root.iterate() if n.name == "Log head failure"]
    assert len(logged) == 1 and logged[0].status == S.SUCCESS


def test_ground_and_table_scans_use_their_own_tilt():
    table = hg.createScanAnnounce(_cfg(), surface="table")
    ground = hg.createScanAnnounce(_cfg(), surface="ground")
    tilt = lambda root: [n.y for n in root.iterate() if isinstance(n, hg.BtNode_HeadTilt)]
    pan = lambda root: [n.x for n in root.iterate() if isinstance(n, hg.BtNode_HeadTilt)]
    assert tilt(table) == [20.0] and tilt(ground) == [0.0]
    assert pan(table) == [0.0] and pan(ground) == [0.0]
    with pytest.raises(ValueError, match="surface"):
        hg.createScanAnnounce(_cfg(), surface="shelf")


def test_orbbec_miss_moves_no_arm(monkeypatch):
    log = []
    _stub(monkeypatch, log, overrides={SAW: S.FAILURE})
    root = hg.createVisitTable(_cfg(), [_rec(0)])
    assert _tick(root) == S.SUCCESS
    assert _arm_or_grasp(log) == []
    assert REALSENSE_SCAN not in log


def test_orbbec_scan_failure_is_nothing_seen(monkeypatch):
    log = []
    _stub(monkeypatch, log, overrides={ORBBEC_SCAN: S.FAILURE}, real_scan_check=True)
    root = hg.createVisitTable(_cfg(), [_rec(0)])
    assert _tick(root) == S.SUCCESS
    assert _arm_or_grasp(log) == []


def test_stale_orbbec_result_is_cleared(monkeypatch):
    log = []
    _stub(monkeypatch, log, overrides={ORBBEC_SCAN: S.FAILURE}, real_scan_check=True)
    stale = SimpleNamespace(status=0, objects=[SimpleNamespace(cls="coke")])
    py_trees.blackboard.Blackboard.set("/" + KEY_SCAN_RESULT, stale)
    root = hg.createVisitTable(_cfg(), [_rec(0)])
    assert _tick(root) == S.SUCCESS
    assert _arm_or_grasp(log) == []


def test_no_label_means_no_arm_move_and_no_pick(monkeypatch):
    log = []
    _stub(monkeypatch, log, overrides={SELECT: S.FAILURE})
    root = hg.createVisitTable(_cfg(), [_rec(0)])
    assert _tick(root) == S.SUCCESS
    assert TABLE_POSE not in log and PICK not in log and OPEN not in log
    assert CLEANUP in log and not _holding()


def test_no_realsense_scan_in_grasp():
    names = [n.name for n in hg.createGrasp(_cfg()).iterate()]
    assert REALSENSE_SCAN not in names
    assert names.index(SELECT) < names.index(TABLE_POSE)


def test_whole_grasp_attempt_retried_up_to_limit(monkeypatch):
    log = []
    _stub(monkeypatch, log, overrides={OPEN: S.FAILURE})
    root = hg.createViewpointScanGrasp(_cfg(grasp_retry_limit=3), miss_is_success=True)
    assert _tick(root) == S.SUCCESS
    assert log.count(TABLE_POSE) == 3 and log.count(OPEN) == 3
    assert log.count(SELECT) == 1 and log.count(CLEANUP) == 1 and PICK not in log


def test_grasp_succeeds_on_a_later_attempt(monkeypatch):
    log = []
    picks = iter([S.FAILURE, S.FAILURE, S.SUCCESS])

    class Pick(py_trees.behaviour.Behaviour):
        def update(self):
            log.append(PICK)
            return next(picks)
    _stub(monkeypatch, log)
    monkeypatch.setattr(hg, "BtNode_Tk27Pick", lambda **kw: Pick(kw["name"]))
    root = hg.createViewpointScanGrasp(_cfg(grasp_retry_limit=3), miss_is_success=True)
    assert _tick(root) == S.SUCCESS
    assert log.count(TABLE_POSE) == 3 and log.count(OPEN) == 3 and log.count(PICK) == 3
    assert CLEANUP not in log and _holding() is True


def test_failed_open_never_ticks_pick(monkeypatch):
    log = []
    _stub(monkeypatch, log, overrides={OPEN: S.FAILURE})
    root = hg.createVisitTable(_cfg(), [_rec(0)])
    assert _tick(root) == S.SUCCESS
    assert PICK not in log
    assert CLEANUP in log
    assert not _holding()


def test_each_retry_round_opens_then_picks(monkeypatch):
    log = []
    _stub(monkeypatch, log, grasp=S.FAILURE)
    root = hg.createViewpointScanGrasp(_cfg(grasp_retry_limit=2), miss_is_success=True)
    assert _tick(root) == S.SUCCESS
    assert [n for n in log if n in (OPEN, PICK)] == [OPEN, PICK, OPEN, PICK]


def test_holding_only_after_pick_success(monkeypatch):
    log = []
    _stub(monkeypatch, log, grasp=S.FAILURE)
    root = hg.createVisitTable(_cfg(grasp_retry_limit=2), [_rec(0), _rec(1)])
    assert _tick(root) == S.SUCCESS
    assert log.count(PICK) == 4
    assert not _holding()


def test_holding_set_before_retract(monkeypatch):
    seen = []
    log = []
    _stub(monkeypatch, log)

    class Probe(Leaf):
        def update(self):
            seen.append(_holding())
            return S.SUCCESS
    monkeypatch.setattr(hg, "BtNode_Tk27SavedPose",
                        lambda **kw: Probe(kw["name"], S.SUCCESS, log))
    assert _tick(hg.createViewpointScanGrasp(_cfg(), miss_is_success=True)) == S.SUCCESS
    # table-pose move sees no holding; the retract after Pick sees holding
    assert seen == [None, True]


# ---- delivery and drop ----
def _nodes(root, cls):
    return [n for n in root.iterate() if isinstance(n, cls)]


def _goto_spy(monkeypatch, out, field):
    monkeypatch.setattr(
        hg, "_gotoRetry",
        lambda name, key, config: out.append(key if field == "key" else name)
        or py_trees.behaviours.Success(name))


def test_deliver_opens_gripper_and_clears_holding(monkeypatch):
    keys = []
    _goto_spy(monkeypatch, keys, "key")
    root = hg.createDeliver(_cfg())
    assert keys == ["hg_pose_dest"]
    grippers = _nodes(root, BtNode_Tk27GripperOpen)
    assert len(grippers) == 1
    assert grippers[0].open_position == 0.0
    assert grippers[0].action_name == "/xarm_gripper/gripper_action"
    assert isinstance(root.children[-1], hg.BtNode_SetHolding)
    assert root.children[-1].value is False


def test_drop_elsewhere_announces_and_opens_gripper(monkeypatch):
    keys = []
    _goto_spy(monkeypatch, keys, "key")
    root = hg.createDropElsewhere(_cfg(), _rec(0))
    assert keys == ["hg_pose_a_table_1_0"]
    assert len(_nodes(root, BtNode_Announce)) == 1
    order = list(root.iterate())
    announce = next(i for i, n in enumerate(order) if isinstance(n, BtNode_Announce))
    gripper = next(i for i, n in enumerate(order) if isinstance(n, BtNode_Tk27GripperOpen))
    assert announce < gripper
    assert root.children[-1].value is False


def test_drop_elsewhere_without_next_pose_goes_to_destination(monkeypatch):
    keys = []
    _goto_spy(monkeypatch, keys, "key")
    root = hg.createDropElsewhere(_cfg(), None)
    assert keys == ["hg_pose_dest"]
    assert not _nodes(root, BtNode_Announce)
    assert len(_nodes(root, BtNode_Tk27GripperOpen)) == 1


def test_drop_and_deliver_goto_named_after_target(monkeypatch):
    names = []
    _goto_spy(monkeypatch, names, "name")
    hg.createDeliver(_cfg())
    hg.createDropElsewhere(_cfg(), _rec(0))
    assert names == ["Go to destination dest", "Go to drop pose a_table_1_0"]


def test_holding_and_gripper_nodes_have_descriptive_names():
    deliver = hg.createDeliver(_cfg())
    drop = hg.createDropElsewhere(_cfg(), _rec(0))
    names = [n.name for r in (deliver, drop) for n in r.iterate()
             if isinstance(n, (hg.BtNode_SetHolding, BtNode_Tk27GripperOpen,
                               hg.BtNode_LogError))]
    assert len(names) == 6 and len(set(names)) == 6
    assert "Open gripper" not in names and "Clear holding" not in names


def test_release_failure_logs_continues_and_clears_holding(monkeypatch):
    log = []
    _stub(monkeypatch, log, overrides={"Open gripper at dest": S.FAILURE})
    py_trees.blackboard.Blackboard.set("/" + KEY_HOLDING, True)
    root = hg.createDeliver(_cfg(arm_retry_limit=2))
    assert _tick(root) == S.SUCCESS
    assert log.count("Open gripper at dest") == 2
    assert _holding() is False
    errors = [n for n in root.iterate() if isinstance(n, hg.BtNode_LogError)]
    assert len(errors) == 1 and errors[0].status == S.SUCCESS
    assert errors[0].name == "Log release failure at dest"
