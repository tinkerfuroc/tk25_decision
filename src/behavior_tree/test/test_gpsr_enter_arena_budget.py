"""Door watch budget (sim battery 2026-09-27 fix4 run 004): the previous run
left the robot facing a side table, the door detector had no door in view and
the mission never started. ``FailureIsRunningUntil`` keeps re-sensing like
``FailureIsRunning`` but gives up (SUCCESS) after ``max_wait_s``.

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1.
"""
import py_trees
import pytest
from py_trees.common import Status

from behavior_tree.GPSR import small_trees


class _AlwaysFail(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("always fail")
        self.feedback_message = "door closed"

    def update(self):
        return Status.FAILURE


class _AlwaysRunning(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("always running")
        self.feedback_message = "door service pending"

    def update(self):
        return Status.RUNNING


def _tick(node):
    node.tick_once()
    return node.status


def test_failure_reads_as_running_within_budget(monkeypatch):
    t = [100.0]
    monkeypatch.setattr(small_trees.time, "monotonic", lambda: t[0])
    node = small_trees.FailureIsRunningUntil("watch", _AlwaysFail(), max_wait_s=30.0)
    node.setup_with_descendants()
    assert _tick(node) == Status.RUNNING
    t[0] = 129.0
    assert _tick(node) == Status.RUNNING
    assert node.feedback_message == "door closed"


def test_failure_reads_as_success_after_budget(monkeypatch):
    t = [100.0]
    monkeypatch.setattr(small_trees.time, "monotonic", lambda: t[0])
    node = small_trees.FailureIsRunningUntil("watch", _AlwaysFail(), max_wait_s=30.0)
    node.setup_with_descendants()
    assert _tick(node) == Status.RUNNING
    t[0] = 131.0
    assert _tick(node) == Status.SUCCESS
    assert "proceeding anyway" in node.feedback_message


def test_pending_service_reads_as_success_after_budget(monkeypatch):
    t = [100.0]
    monkeypatch.setattr(small_trees.time, "monotonic", lambda: t[0])
    node = small_trees.FailureIsRunningUntil("watch", _AlwaysRunning(), max_wait_s=30.0)
    node.setup_with_descendants()
    assert _tick(node) == Status.RUNNING
    t[0] = 131.0
    assert _tick(node) == Status.SUCCESS
    assert "proceeding anyway" in node.feedback_message


def test_non_positive_budget_waits_forever(monkeypatch):
    t = [0.0]
    monkeypatch.setattr(small_trees.time, "monotonic", lambda: t[0])
    node = small_trees.FailureIsRunningUntil("watch", _AlwaysFail(), max_wait_s=0.0)
    node.setup_with_descendants()
    _tick(node)
    t[0] = 1e6
    assert _tick(node) == Status.RUNNING


def test_enter_arena_uses_the_budgeted_watch():
    tree = small_trees.create_enter_arena()
    from behavior_tree.TemplateNodes.Vision import BtNode_TurnPanTilt

    entry = next(n for n in tree.iterate() if n.name == "detect door + enter")
    head = entry.children[0]
    assert isinstance(head, BtNode_TurnPanTilt)
    assert head.name == "point head at entrance"
    assert head.x == pytest.approx(0.0)
    assert head.y == pytest.approx(45.0)
    assert head.require_settled is False
    assert entry.children[1].name == "announce ready for gpsr"
    watches = [n for n in tree.iterate() if n.name == "keep sensing until door opens"]
    assert len(watches) == 1
    assert isinstance(watches[0], small_trees.FailureIsRunningUntil)
    assert small_trees.DOOR_WAIT_MAX_S == pytest.approx(120.0)


def test_enter_arena_selector_keeps_running_door_watch():
    """A pending door service must not replay the greeting each tick.

    The selector's non-memory mode used to invalidate its running detection
    branch after the guard failed, which restarted the greeting and door
    request on every tree tick.  The running branch must be resumed in place.
    """
    tree = small_trees.create_enter_arena()
    selector = next(n for n in tree.iterate() if n.name == "enter arena (once)")
    assert isinstance(selector, py_trees.composites.Selector)
    assert selector.memory is True
