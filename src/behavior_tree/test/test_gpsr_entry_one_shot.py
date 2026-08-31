"""K1 (task-K, live-manipulation sim findings): entry arena must never replay
after it has already succeeded once.

F1's failure mode: the orchestrator root is a memory ``Sequence``
(``gpsr_orchestrator.py``) ticked forever at 500 ms. A memory ``Sequence``
resumes only from a RUNNING child -- on FAILURE every child is invalidated and
the next tick restarts from child 0. When a LATER sibling (goto/mission) keeps
failing (e.g. the arm action server rejecting every goal), the root keeps
failing too, and ``create_enter_arena()`` -- the FIRST child -- gets re-ticked
in full on every one of those restarts: re-announcing "Hi, I am Tinker..." and
re-running the door watch, thousands of times over a long run.

The fix wraps whatever is added at the entry-arena slot in
``py_trees.decorators.OneShot(policy=ON_SUCCESSFUL_COMPLETION)`` via a small
``_one_shot(child)`` helper (factored out so this test exercises the REAL
wrapping code, not a copy of it) -- once the wrapped subtree SUCCEEDS, later
ticks bounce back the memorised SUCCESS without ever ticking the child again.

Pure py_trees replica (no ROS): a minimal memory Sequence root of
``[_one_shot(enter-like stub), failing stub]`` mirrors the shape of the real
root's first two significant children closely enough to pin the behaviour.
"""
from __future__ import annotations

import py_trees
from py_trees.common import Status

from behavior_tree.GPSR.small_trees import _one_shot


class _CountingStub(py_trees.behaviour.Behaviour):
    """SUCCESS every tick; counts how many times it was actually ticked."""

    def __init__(self, name: str):
        super().__init__(name)
        self.tick_count = 0

    def update(self):
        self.tick_count += 1
        return Status.SUCCESS


class _AlwaysFail(py_trees.behaviour.Behaviour):
    def update(self):
        return Status.FAILURE


def test_one_shot_entry_never_replays_after_success_across_root_restarts():
    enter_stub = _CountingStub("enter-like stub")
    root = py_trees.composites.Sequence("root", memory=True)
    root.add_child(_one_shot(enter_stub))
    root.add_child(_AlwaysFail("failing stub"))

    for _ in range(6):
        root.tick_once()
        assert root.status is Status.FAILURE

    # The entry stub SUCCEEDED once, on the very first restart; every one of
    # the other 5 root restarts must NOT have re-ticked it (no re-announce,
    # no re-detect).
    assert enter_stub.tick_count == 1


def test_one_shot_keeps_the_wrapped_childs_name():
    # The brief requires the child's name to stay stable at the point it is
    # added to the root so name-based lookups (tests, tree_serialization)
    # keep working after the OneShot wrap.
    enter_stub = _CountingStub("enter arena")
    wrapped = _one_shot(enter_stub)
    assert wrapped.name == "enter arena"


def test_one_shot_bounces_failure_too_under_on_successful_completion_policy():
    # ON_SUCCESSFUL_COMPLETION only latches on SUCCESS; a child that fails
    # must still be ticked again on the next root restart (retries keep
    # working -- only a SUCCESSFUL entry is one-shot).
    class _FailNTimesThenSucceed(py_trees.behaviour.Behaviour):
        def __init__(self, name, n):
            super().__init__(name)
            self._n = n
            self.tick_count = 0

        def update(self):
            self.tick_count += 1
            return Status.SUCCESS if self.tick_count > self._n else Status.FAILURE

    entry = _FailNTimesThenSucceed("entry", n=2)
    root = py_trees.composites.Sequence("root", memory=True)
    root.add_child(_one_shot(entry))
    root.add_child(_AlwaysFail("failing stub"))

    for _ in range(5):
        root.tick_once()

    # Ticked 3 times to succeed (fails twice, then succeeds), then never again.
    assert entry.tick_count == 3
