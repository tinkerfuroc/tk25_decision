"""G2 (task-G, round-2 review follow-up, pre-existing HIGH): `orchestrator.py`
`create_execute_one_step()`'s `dispatch_or_correct` livelock shape.

``dispatch_or_correct`` was a ``Selector(memory=False)`` over
``[monitor_then_log, correction]`` (OFF/SHADOW mode only -- ACTIVE supervision
never builds this Selector, see the ``supervisor.config.mode`` branch around
it). ``monitor_then_log`` wraps the router + dispatched action; ``correction``
is the multi-tick self-correction/replan sequence (log failure, bump counter,
announce, ``BtNode_PlanActions``).

After a dispatched action fails, ``execute_step`` (a memory ``Sequence``)
resumes directly at ``dispatch_or_correct`` on every following tick without
re-ticking ``pop`` -- so the same router still matches the same (failed)
action. A non-memory Selector re-enters child 0 on EVERY tick regardless of
which branch is actually in flight: the relaunched action goes RUNNING once
(its first tick after being torn down and rebuilt) then FAILURE again, and
the moment it goes RUNNING it becomes ``current_child`` -- the Selector's own
priority-interrupt handling (``previous != current_child``) then invalidates
the in-flight ``correction`` branch via ``stop(INVALID)``, abandoning the
replan before ``BtNode_PlanActions`` can ever resolve. The correction branch
is reinitialised from scratch next cycle, repeats. This is bounded only by
``max_corrections`` in production (not a true livelock), but the replan
itself never lands within that budget. ``memory=True`` fixes it: a memory
Selector resumes directly at whichever branch last went RUNNING, so a
resolved higher-priority branch is not repeatedly relaunched once a
lower-priority branch has taken over -- a fresh entry (this Selector's own
status != RUNNING, i.e. a new step) still resets to child 0 first, so normal
dispatch-first-then-correct semantics are unchanged.

This is a py_trees-only replica (no ROS, no orchestrator import) with stub
children reproducing the reviewer's own confirmed replica numbers: a
``dispatch``-stand-in that goes RUNNING once then FAILURE terminally each
cycle it is relaunched (``_RunOnceThenFail``, standing in for
``monitor_then_log``: router matches synchronously, the dispatched action
goes RUNNING once then fails), and a ``replan``-stand-in that needs 3 ticks
running before it succeeds (``_RunNTimesThenSucceed``, standing in for
``correction``'s ``BtNode_PlanActions``). Over an 8-tick budget under the OLD
``memory=False`` shape this reproduces the reviewer's own numbers exactly:
the dispatch stub is initialised 4 times, the replan stub is initialised 4
times and never gets past its first internal tick (never reaches SUCCESS).
"""
from __future__ import annotations

import py_trees
from py_trees.common import Status


class _RunOnceThenFail(py_trees.behaviour.Behaviour):
    """RUNNING on the first tick since the last ``initialise()``, FAILURE on
    every following tick until reinitialised -- stands in for
    ``monitor_then_log`` (router matches synchronously, the dispatched action
    goes RUNNING once then fails). Tracks how many times ``initialise()`` ran
    (i.e. how many times the dispatch was relaunched from scratch)."""

    def __init__(self, name: str):
        super().__init__(name)
        self._ticked = False
        self.initialise_count = 0

    def initialise(self):
        self.initialise_count += 1
        self._ticked = False

    def update(self):
        if not self._ticked:
            self._ticked = True
            return Status.RUNNING
        return Status.FAILURE


class _RunNTimesThenSucceed(py_trees.behaviour.Behaviour):
    """RUNNING for ``n`` ticks since the last ``initialise()``, then SUCCESS
    -- stands in for ``correction``'s multi-tick replan step
    (``BtNode_PlanActions``). Tracks how many times ``initialise()`` ran (how
    many times the replan was relaunched from scratch) as well as its own
    status, so a mid-flight teardown (``stop(INVALID)``) is visible."""

    def __init__(self, name: str, n: int = 3):
        super().__init__(name)
        self._n = n
        self._count = 0
        self.initialise_count = 0

    def initialise(self):
        self.initialise_count += 1
        self._count = 0

    def update(self):
        self._count += 1
        if self._count <= self._n:
            return Status.RUNNING
        return Status.SUCCESS


def _build(memory: bool):
    """A minimal replica of ``dispatch_or_correct``: a Selector over
    ``[dispatch_stub, replan_stub]`` with the memory flag under test."""
    dispatch_stub = _RunOnceThenFail("monitor_then_log (stub)")
    replan_stub = _RunNTimesThenSucceed("correction/replan (stub)", n=3)
    selector = py_trees.composites.Selector(
        "dispatch_or_correct", memory=memory,
        children=[dispatch_stub, replan_stub],
    )
    return selector, dispatch_stub, replan_stub


def _tick_to_completion_or_budget(selector, max_ticks: int):
    for _ in range(max_ticks):
        selector.tick_once()
        if selector.status != Status.RUNNING:
            break


def test_memory_true_resumes_the_replan_instead_of_relaunching_the_dispatch():
    """Pins the fix (task G2): with memory=True the in-flight replan/
    correction branch is resumed to completion, not torn down every cycle by
    a relaunched dispatch."""
    selector, dispatch_stub, replan_stub = _build(memory=True)

    _tick_to_completion_or_budget(selector, max_ticks=8)

    assert replan_stub.status is Status.SUCCESS
    assert replan_stub.initialise_count == 1
    assert dispatch_stub.initialise_count == 1
    assert selector.status is Status.SUCCESS


def test_memory_false_livelocks_the_replan_via_repeated_dispatch_relaunches():
    """Negative control (pins the pre-fix regression, round-2 review
    follow-up HIGH): with memory=False the dispatch branch is relaunched
    from scratch every cycle and, the moment it goes RUNNING again, tears
    down the in-flight replan via stop(INVALID) before it can accumulate its
    3 ticks -- matches the reviewer's own confirmed replica numbers (4 and 4
    over an 8-tick budget, replan never past its first internal tick)."""
    selector, dispatch_stub, replan_stub = _build(memory=False)

    _tick_to_completion_or_budget(selector, max_ticks=8)

    assert dispatch_stub.initialise_count == 4
    assert replan_stub.initialise_count == 4
    assert replan_stub.status is not Status.SUCCESS
    assert selector.status is Status.RUNNING
