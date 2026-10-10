"""The production batch boundary must not re-intake after a terminal failure."""
from __future__ import annotations

from pathlib import Path

import py_trees
from py_trees.common import Status

import behavior_tree.GPSR.gpsr_orchestrator as gpsr


class _CountingFailure(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "batch implementation"):
        super().__init__(name)
        self.tick_count = 0

    def update(self):
        self.tick_count += 1
        return Status.FAILURE


def test_failed_batch_is_latched_and_not_re_intaken(monkeypatch, tmp_path: Path):
    """A failed batch completes the outer root's mission boundary once.

    Without the wrapper, the outer memory Sequence fails and restarts from its
    first child on each tick, causing command intake/planning to run again.
    """
    monkeypatch.setattr(gpsr, "load_knowledge_from_constants", lambda _path: None)
    monkeypatch.setattr(
        gpsr,
        "_arm_constants_to_bb",
        lambda root: root.add_child(py_trees.behaviours.Success("seed arm")),
    )
    monkeypatch.setattr(
        gpsr, "create_enter_arena", lambda: py_trees.behaviours.Success("enter arena")
    )
    monkeypatch.setattr(gpsr, "has_command_point", lambda: False)
    monkeypatch.setattr(
        gpsr,
        "create_orchestrator_init",
        lambda: py_trees.behaviours.Success("orchestrator init"),
    )
    # Avoid configuring a live supervisor in this pure py_trees regression.
    monkeypatch.setattr(gpsr, "get_default_supervisor", lambda: object())

    failed_batch = _CountingFailure()

    def make_batch(_planner, **_kwargs):
        return failed_batch

    monkeypatch.setattr(gpsr, "create_batch_command_flow_new", make_batch)

    root = gpsr.createGPSROrchestrator(
        commands=["bring water"],
        plan_dir=tmp_path,
    )
    batch_boundary = root.children[-2]
    assert isinstance(batch_boundary, py_trees.decorators.FailureIsSuccess)
    assert batch_boundary.name == "batch command flow"

    tree = py_trees.trees.BehaviourTree(root)
    # The real blackboard seed node requests a ROS ``node`` during setup; this
    # test replaces all ROS-backed work, so a sentinel is sufficient.
    tree.setup(node=object())
    for _ in range(6):
        tree.tick()

    assert failed_batch.tick_count == 1
    assert batch_boundary.status is Status.SUCCESS
    assert root.status is Status.RUNNING
