from types import SimpleNamespace

import py_trees
import pytest

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Manipulation import BtNode_GripperAction
from behavior_tree.hangzhougrasp import hangzhougrasp as hg
from behavior_tree.hangzhougrasp.config import KEY_HOLDING
from behavior_tree.hangzhougrasp.pose_index import PoseRecord


class Leaf(py_trees.behaviour.Behaviour):
    def __init__(self, name, status, log):
        super().__init__(name)
        self.result, self.log = status, log

    def update(self):
        self.log.append(self.name)
        return self.result


def _rec(n):
    return PoseRecord("a", "table", 1, n, f"a_table_1_{n}", None)


def _cfg():
    return SimpleNamespace(
        arm_action="arm", arm_retry_limit=1, nav_retry_limit=1,
        navigation_action="nav", destination_name="dest")


@pytest.fixture(autouse=True)
def clear_bb():
    py_trees.blackboard.Blackboard.storage.clear()
    yield
    py_trees.blackboard.Blackboard.storage.clear()


def _stub(monkeypatch, log, goto=py_trees.common.Status.SUCCESS,
          grasp=py_trees.common.Status.SUCCESS):
    monkeypatch.setattr(
        hg, "_gotoRetry", lambda name, key, config: Leaf(name, goto, log))
    monkeypatch.setattr(
        hg, "createScanGraspWithCleanup",
        lambda config: Leaf("scan-grasp", grasp, log))


def _tick(root):
    root.tick_once()
    return root.status


def _holding():
    return py_trees.blackboard.Blackboard.storage.get("/" + KEY_HOLDING)


def test_visit_table_stops_after_first_grasp(monkeypatch):
    log = []
    _stub(monkeypatch, log)
    root = hg.createVisitTable(_cfg(), [_rec(0), _rec(1)])
    assert _tick(root) == py_trees.common.Status.SUCCESS
    assert log == ["Go to a_table_1_0", "scan-grasp"]
    assert _holding() is True


def test_visit_table_skips_on_miss(monkeypatch):
    log = []
    _stub(monkeypatch, log, grasp=py_trees.common.Status.FAILURE)
    root = hg.createVisitTable(_cfg(), [_rec(0), _rec(1)])
    assert _tick(root) == py_trees.common.Status.SUCCESS
    assert log.count("scan-grasp") == 2
    assert not _holding()


def test_visit_table_navigation_failure_fails(monkeypatch):
    log = []
    _stub(monkeypatch, log, goto=py_trees.common.Status.FAILURE)
    root = hg.createVisitTable(_cfg(), [_rec(0), _rec(1)])
    assert _tick(root) == py_trees.common.Status.FAILURE
    assert "scan-grasp" not in log


def _nodes(root, cls):
    return [n for n in root.iterate() if isinstance(n, cls)]


def test_deliver_opens_gripper_and_clears_holding(monkeypatch):
    keys = []
    monkeypatch.setattr(
        hg, "_gotoRetry",
        lambda name, key, config: keys.append(key) or py_trees.behaviours.Success(name))
    root = hg.createDeliver(_cfg())
    assert keys == ["hg_pose_dest"]
    grippers = _nodes(root, BtNode_GripperAction)
    assert len(grippers) == 1
    assert grippers[0].goal == 0.10
    assert isinstance(root.children[-1], hg.BtNode_SetHolding)
    assert root.children[-1].value is False


def test_drop_elsewhere_announces_and_opens_gripper(monkeypatch):
    keys = []
    monkeypatch.setattr(
        hg, "_gotoRetry",
        lambda name, key, config: keys.append(key) or py_trees.behaviours.Success(name))
    root = hg.createDropElsewhere(_cfg(), _rec(0))
    assert keys == ["hg_pose_a_table_1_0"]
    assert len(_nodes(root, BtNode_Announce)) == 1
    order = [type(c) for c in root.children]
    assert order.index(BtNode_Announce) < order.index(BtNode_GripperAction)
    assert root.children[-1].value is False


def test_drop_elsewhere_without_next_pose_goes_to_destination(monkeypatch):
    keys = []
    monkeypatch.setattr(
        hg, "_gotoRetry",
        lambda name, key, config: keys.append(key) or py_trees.behaviours.Success(name))
    root = hg.createDropElsewhere(_cfg(), None)
    assert keys == ["hg_pose_dest"]
    assert not _nodes(root, BtNode_Announce)
    assert len(_nodes(root, BtNode_GripperAction)) == 1
