"""F3 (round-2 review, fix round 2): sim run 005's real root cause -- livelock
at pan/tilt pose #1 for 116 cycles (~8.6 min).

``_person_scan_strategies`` built a ``Selector(memory=False)`` over memory
``Sequence`` branches whose tails are multi-tick async ServiceHandlers (the
waving specialist scan, an optional attribute-specialist scan, the generalist
scan). A non-memory Selector re-enters child 0 on EVERY tick regardless of
which branch is actually in flight. In sim run 005 (searching for "person
Liam") the attribute-specialist branch's guard always matched the descriptor,
so on every tick that branch was torn down and relaunched from scratch (a
FRESH ``initialise()`` on its scan leaf) the moment the Selector restarted at
index 0 -- and whenever that relaunch put the specialist back in front, the
selector's own priority-interrupt handling (`previous != current_child`)
invalidated the (already in-flight, lower-priority) generalist branch,
abandoning its call before it could ever complete. Fixing the Selector to
``memory=True`` (this file's real target) makes it resume directly at
whichever branch last went RUNNING, so a resolved higher-priority branch is
not repeatedly relaunched once a lower-priority branch has taken over.

This test builds the REAL ``small_trees._person_scan_strategies()`` (with an
``extra_specialist`` gate that always matches the seeded descriptor, so the
specialist branch is the fast-resolving higher-priority branch and the
generalist is the multi-tick lower-priority branch -- the exact run-005
shape) and stubs only the two scan leaves (ROS-backed, no server in a unit
test) with a plain leaf that tracks how many times its own ``initialise()``
ran.
"""
from __future__ import annotations

import py_trees
from py_trees.common import Access, Status

from behavior_tree.GPSR import small_trees
from behavior_tree.GPSR.small_trees import bb_keys


class _RunOnceThen(py_trees.behaviour.Behaviour):
    """RUNNING on the first tick since the last ``initialise()``, then
    ``final_status`` on every following tick -- tracks how many times
    ``initialise()`` actually ran (i.e. how many times this node was
    relaunched from scratch)."""

    def __init__(self, name: str, final_status: Status):
        super().__init__(name)
        self._final_status = final_status
        self._ticked = False
        self.initialise_count = 0

    def initialise(self):
        self.initialise_count += 1
        self._ticked = False

    def update(self):
        if not self._ticked:
            self._ticked = True
            return Status.RUNNING
        return self._final_status


def _build(memory: "bool | None" = None):
    """Real _person_scan_strategies() with an always-matching attribute
    specialist (so selector.children == [waving, specialist, generalist]),
    scan leaves replaced by _RunOnceThen stubs.

    `memory=None` (the default) leaves the constructor's own `memory` value
    untouched, so the positive test actually pins whatever
    `_person_scan_strategies()` builds in production. Passing an explicit
    bool overrides the flag post-construction (`Selector.tick()` reads
    `self.memory` live, so this is equivalent to constructing it either
    way) -- used only by the negative control to force the pre-fix shape."""
    py_trees.blackboard.Blackboard.clear()
    writer = py_trees.blackboard.Client(name="seed person scan strategies")
    writer.register_key(bb_keys.TARGET_PERSON_PROMPT, access=Access.WRITE)
    writer.set(bb_keys.TARGET_PERSON_PROMPT, "person liam", overwrite=True)

    selector = small_trees._person_scan_strategies(
        extra_specialist={"gate": "liam", "prompt": "person liam"},
    )
    if memory is not None:
        selector.memory = memory
    assert len(selector.children) == 3, "expected [waving, specialist, generalist]"
    waving_branch, specialist_branch, generalist_branch = selector.children

    # The waving specialist's guard never matches ("person liam" has no
    # "wav"), so this branch is never ticked past its guard -- but setup()
    # still walks it, and its real scan leaf (BtNode_ScanForWavingPersonNew)
    # needs a ROS node that isn't available in a unit test. Stub it too.
    for child in list(waving_branch.children[1:]):
        waving_branch.remove_child(child)
    waving_branch.add_child(py_trees.behaviours.Failure(name="waving scan (stub)"))

    specialist_stub = _RunOnceThen("specialist scan (stub)", Status.FAILURE)
    for child in list(specialist_branch.children[2:]):  # keep guard + pin-prompt
        specialist_branch.remove_child(child)
    specialist_branch.add_child(specialist_stub)

    generalist_stub = _RunOnceThen("generalist scan (stub)", Status.SUCCESS)
    for child in list(generalist_branch.children):
        generalist_branch.remove_child(child)
    generalist_branch.add_child(generalist_stub)

    py_trees.trees.BehaviourTree(selector).setup()
    return selector, specialist_stub, generalist_stub


def _tick_to_completion_or_budget(selector, max_ticks: int = 4):
    for _ in range(max_ticks):
        selector.tick_once()
        if selector.status != Status.RUNNING:
            break


def test_memory_true_resumes_the_generalist_instead_of_relaunching_the_specialist():
    """Pins the PRODUCTION value: builds the real selector without
    overriding `memory`, so this fails if `_person_scan_strategies()` is
    ever reverted to `memory=False` (round-2 review M1)."""
    selector, specialist_stub, generalist_stub = _build()

    assert selector.memory is True, (
        "production _person_scan_strategies() selector must be memory=True"
    )

    _tick_to_completion_or_budget(selector, max_ticks=4)

    assert specialist_stub.initialise_count == 1
    assert generalist_stub.status is Status.SUCCESS
    assert selector.status is Status.SUCCESS


def test_memory_false_livelocks_the_specialist_relaunches_and_abandons_the_generalist():
    """Negative control (pins the regression): with memory=False the
    specialist branch is torn down and relaunched from scratch every time the
    Selector restarts at index 0, and the generalist's in-flight call is
    abandoned before it can complete -- the selector never resolves within
    the same tick budget that resolves it under the fix."""
    selector, specialist_stub, generalist_stub = _build(memory=False)

    _tick_to_completion_or_budget(selector, max_ticks=4)

    assert specialist_stub.initialise_count >= 2
    assert generalist_stub.status is not Status.SUCCESS
    assert selector.status is Status.RUNNING
