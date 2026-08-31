"""K2 (task-K, live-manipulation sim findings): PersistentFailureCap.

F1's other half: even with entry-arena made OneShot (K1), the goto subtree's
tuck-arm node (``py_trees.decorators.Retry(num_failures=3)`` wrapping
``BtNode_MoveArmSingle``) still gets torn down to INVALID and rebuilt from
scratch on every one of the root's ~500ms restarts while the arm controller
is broken -- ``initialise()`` re-zeroes the Retry's own counter every time, so
it never actually gives up. A real run produced 3000+ rejected arm goals.

``PersistentFailureCap`` fixes this generically: it counts CONSECUTIVE child
FAILUREs on an instance attribute that survives re-entry (NOT reset in
``initialise()`` -- that is the entire point), resets to 0 on a child
SUCCESS, is fully transparent under the cap, and once the cap is reached
invokes ``on_exhausted`` exactly once and thereafter returns FAILURE
WITHOUT ever ticking the child again (no more goals / side effects).

Pure py_trees unit tests (no ROS) exercising the decorator directly, plus one
test that re-initialises it exactly the way an ancestor memory-Sequence
restart would (call ``initialise()`` directly, mirroring Decorator.stop(...)
-> terminate() semantics without dragging in a whole tree).
"""
from __future__ import annotations

import py_trees
from py_trees.common import Status

from behavior_tree.GPSR.small_trees import PersistentFailureCap


class _AlwaysFail(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "always fail"):
        super().__init__(name)
        self.tick_count = 0

    def update(self):
        self.tick_count += 1
        return Status.FAILURE


class _AlwaysSucceed(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "always succeed"):
        super().__init__(name)
        self.tick_count = 0

    def update(self):
        self.tick_count += 1
        return Status.SUCCESS


class _ScriptedStatus(py_trees.behaviour.Behaviour):
    """Returns each of ``statuses`` in order, then repeats the last one."""

    def __init__(self, name: str, statuses: list[Status]):
        super().__init__(name)
        self._statuses = statuses
        self.tick_count = 0

    def update(self):
        idx = min(self.tick_count, len(self._statuses) - 1)
        status = self._statuses[idx]
        self.tick_count += 1
        return status


def _reenter(cap: PersistentFailureCap) -> None:
    """Simulate an ancestor memory-Sequence restart: the cap node itself is
    invalidated (as every child is, on a root restart) and re-ticked fresh.
    """
    cap.stop(Status.INVALID)


def test_counter_survives_reentry_and_exhausts_after_cap_failures():
    child = _AlwaysFail("tuck arm (stub)")
    fired = []
    cap = PersistentFailureCap(
        "tuck arm cap", child, max_failures=5, on_exhausted=lambda: fired.append(1),
    )

    # 5 separate "root restart" cycles: each ticks the cap once to FAILURE,
    # then the ancestor Sequence invalidates it (re-entry) before the next
    # cycle -- mirroring F1's ~500ms restart loop.
    for _ in range(5):
        status = cap.tick_once() or cap.status
        assert cap.status is Status.FAILURE
        _reenter(cap)

    assert child.tick_count == 5
    assert fired == [1]
    assert cap.status in (Status.FAILURE, Status.INVALID)

    # Further re-entries: cap is exhausted, must return FAILURE immediately
    # WITHOUT ticking the child again, and on_exhausted must not fire again.
    for _ in range(3):
        cap.tick_once()
        assert cap.status is Status.FAILURE
        _reenter(cap)

    assert child.tick_count == 5  # unchanged -- no more ticks reached the child
    assert fired == [1]  # on_exhausted fired exactly once


def test_success_between_failures_resets_the_counter():
    statuses = [Status.FAILURE] * 4 + [Status.SUCCESS] + [Status.FAILURE] * 10
    child = _ScriptedStatus("tuck arm (stub)", statuses)
    fired = []
    cap = PersistentFailureCap(
        "tuck arm cap", child, max_failures=5, on_exhausted=lambda: fired.append(1),
    )

    for _ in range(15):
        cap.tick_once()
        _reenter(cap)

    # 4 failures (not yet at cap) + 1 success (resets to 0) + 10 more
    # failures -- cap (5) is only reached by the LAST 5 of those 10, so
    # on_exhausted fires once, after the 5th failure following the reset.
    assert fired == [1]
    assert child.tick_count == 4 + 1 + 5  # stops ticking once exhausted


def test_under_cap_statuses_pass_through_unchanged_including_running():
    child = _ScriptedStatus(
        "tuck arm (stub)", [Status.RUNNING, Status.RUNNING, Status.SUCCESS],
    )
    cap = PersistentFailureCap("tuck arm cap", child, max_failures=5, on_exhausted=None)

    cap.tick_once()
    assert cap.status is Status.RUNNING
    cap.tick_once()
    assert cap.status is Status.RUNNING
    cap.tick_once()
    assert cap.status is Status.SUCCESS


def test_on_exhausted_is_optional():
    child = _AlwaysFail()
    cap = PersistentFailureCap("cap", child, max_failures=2, on_exhausted=None)
    for _ in range(5):
        cap.tick_once()
        assert cap.status is Status.FAILURE  # never raises despite no callback
        _reenter(cap)


def test_v3_cap_is_a_pure_single_child_decorator():
    # V-3 (task-K review, MEDIUM): an earlier revision appended a second
    # ``announce_child`` to self.children, breaking py_trees's Decorator
    # single-child invariant (Decorator.stop()/tip() are hardcoded to
    # children[0] in the installed py_trees version -- an implementation
    # detail, not a documented contract). The one-time announce now lives as
    # a standard sibling Selector branch in create_goto() instead (see
    # test_gpsr_goto_exhaustion_quiet.py); this decorator itself must stay a
    # plain single-child wrapper with no children[1:] reliance anywhere.
    child = _AlwaysFail("tuck arm (stub)")
    cap = PersistentFailureCap("cap", child, max_failures=2, on_exhausted=None)

    assert len(cap.children) == 1
    assert cap.children[0] is child
    assert cap.decorated is child
    assert not hasattr(cap, "announce_child")

    # Still true post-exhaustion (nothing gets appended along the way).
    for _ in range(3):
        cap.tick_once()
        _reenter(cap)
    assert len(cap.children) == 1
