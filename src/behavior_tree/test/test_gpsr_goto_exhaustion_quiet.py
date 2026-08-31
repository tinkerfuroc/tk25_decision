"""V-2/V-3 fix verification (task-K review): once the tuck-arm
``PersistentFailureCap`` inside ``create_goto()`` gives up, the ENTIRE goto
subtree goes quiet on later root restarts -- not just the tuck-arm retry.

Review finding V-2 (HIGH): K2's original wiring only wrapped the tuck-arm
node in the cap; ``BtNode_AnnounceFromBB("announce going", ...)`` was a
SIBLING outside the cap, so once the cap was exhausted, the surrounding
memory Sequence still returned terminal FAILURE every ~500ms and the root
restart cascade kept re-ticking "announce going" forever -- reproducing the
exact repeat-announce bug class (F1) one node earlier, with no test
coverage.

Fix: a new ``bb_keys.MISSION_UNRECOVERABLE`` flag, latched by the cap's
``on_exhausted`` callback, gates (a) a fail-fast guard as the FIRST child of
``create_goto()``'s Sequence (silences "announce going" and everything after
it on every LATER restart) and (b) a standard sibling Selector branch that
delivers the one-time "I cannot move my arm..." speech exactly once via
``_one_shot(..., ON_COMPLETION)``, then keeps the branch (and so the whole
goto) FAILED.

Builds the REAL ``small_trees.create_goto()`` tree (so the actual new guard/
Selector/flag wiring is exercised exactly as production uses them), with its
ROS-backed leaves swapped for counting stubs -- same convention as
test_gpsr_search_object_feedback.py's ``_build_tree`` and this task's own
K1 (test_gpsr_entry_one_shot.py) / K2 (test_gpsr_persistent_failure_cap.py)
tests. Root restarts are simulated with ``tree.stop(Status.INVALID)`` between
ticks, mirroring the real orchestrator root's memory-Sequence cascade (same
convention as the K2 unit tests' ``_reenter`` helper).
"""
from __future__ import annotations

import py_trees
from py_trees.common import Status

from behavior_tree.GPSR import small_trees


class _CountingSuccess(py_trees.behaviour.Behaviour):
    """SUCCESS every tick; counts how many times it was actually ticked."""

    def __init__(self, name: str):
        super().__init__(name)
        self.tick_count = 0

    def update(self):
        self.tick_count += 1
        return Status.SUCCESS


class _CountingFailure(py_trees.behaviour.Behaviour):
    """FAILURE every tick; counts how many times it was actually ticked."""

    def __init__(self, name: str):
        super().__init__(name)
        self.tick_count = 0

    def update(self):
        self.tick_count += 1
        return Status.FAILURE


def _restart(tree: py_trees.behaviour.Behaviour) -> None:
    """Simulate an ancestor memory-Sequence root restart: invalidate the
    whole subtree, exactly what the real orchestrator root does to every
    child on ANY sibling FAILURE. Blackboard state (e.g.
    MISSION_UNRECOVERABLE) is untouched by this -- only node status is.
    """
    tree.stop(Status.INVALID)


def _build_goto_under_test():
    """Real ``create_goto()``, with:

    - its Parallel drive+keepalive child dropped (never reached while the
      tuck-arm branch is failing -- avoids needing ROS setup() for it),
    - "announce going" swapped for a counting stub,
    - the tuck-arm cap's real (Retry-wrapped, ROS-backed) child swapped for
      a counting FAILURE stub (deterministic "arm rejects every goal"),
    - the exhaustion announce's real ``BtNode_Announce`` swapped for a
      counting stub, re-wrapped in the SAME ``_one_shot(..., ON_COMPLETION)``
      the production code uses (only the innermost leaf is a stub -- the
      one-shot wrapping itself is the real helper).

    Returns (tree, announce_going_stub, tuck_stub, cap, unrecoverable_announce_stub).
    """
    tree = small_trees.create_goto()
    guard, announce_going, selector, parallel = tree.children
    tree.remove_child(parallel)

    announce_going_stub = _CountingSuccess("announce going")
    tree.remove_child(announce_going)
    tree.insert_child(announce_going_stub, 1)

    cap, announce_unrecoverable_seq = selector.children
    tuck_stub = _CountingFailure("tuck stub")
    cap.children[0] = tuck_stub
    cap.decorated = tuck_stub
    tuck_stub.parent = cap

    check_node, oneshot_announce, failure_leaf = announce_unrecoverable_seq.children
    unrecoverable_announce_stub = _CountingFailure("unrecoverable announce stub")
    announce_unrecoverable_seq.remove_child(oneshot_announce)
    announce_unrecoverable_seq.insert_child(
        small_trees._one_shot(
            unrecoverable_announce_stub,
            policy=py_trees.common.OneShotPolicy.ON_COMPLETION,
        ),
        1,
    )

    py_trees.trees.BehaviourTree(tree).setup()
    return tree, announce_going_stub, tuck_stub, cap, unrecoverable_announce_stub


def test_goto_subtree_goes_fully_quiet_after_tuck_arm_exhaustion():
    py_trees.blackboard.Blackboard.clear()
    tree, announce_going, tuck, cap, unrec_announce = _build_goto_under_test()

    # Drive the cap to exhaustion (max_failures=5): 5 root-restart cycles,
    # each ticking the whole goto subtree to terminal FAILURE.
    for _ in range(5):
        tree.tick_once()
        assert tree.status is Status.FAILURE
        _restart(tree)

    assert cap.exhausted is True
    assert tuck.tick_count == 5
    # The exhaustion cycle's own restart already ran "announce going" (guard
    # passes before the flag is set mid-tick) and the unrecoverable announce
    # (flag set → branch reached) exactly once each -- capture the counts
    # right after exhaustion as the baseline for "must not grow further".
    announce_going_after_exhaustion = announce_going.tick_count
    assert unrec_announce.tick_count == 1

    # Further "root restarts": the fail-fast guard must now block EVERYTHING
    # downstream -- no more "announce going", no more tuck ticks (the cap
    # already stops ticking its own child too), no repeat of the
    # unrecoverable announce. This is the V-2 fix's core guarantee.
    for _ in range(6):
        tree.tick_once()
        assert tree.status is Status.FAILURE
        _restart(tree)

    assert tuck.tick_count == 5
    assert announce_going.tick_count == announce_going_after_exhaustion
    assert unrec_announce.tick_count == 1


def test_under_cap_failure_does_not_trigger_the_unrecoverable_announce():
    py_trees.blackboard.Blackboard.clear()
    tree, announce_going, tuck, cap, unrec_announce = _build_goto_under_test()

    # 3 restarts: under max_failures=5, the cap is not yet exhausted, so
    # MISSION_UNRECOVERABLE stays unset and the announce branch's own guard
    # (BtNode_CheckBBTrue) must keep failing it out before the announce stub
    # is ever ticked.
    for _ in range(3):
        tree.tick_once()
        assert tree.status is Status.FAILURE
        _restart(tree)

    assert cap.exhausted is False
    assert unrec_announce.tick_count == 0
    # "announce going" is unaffected pre-exhaustion (unchanged behaviour).
    assert announce_going.tick_count == 3


def test_unrecoverable_announce_fires_exactly_once_even_if_it_fails():
    # unrec_announce is a _CountingFailure -- simulates the TTS call itself
    # failing. ON_COMPLETION must still latch after exactly one attempt (not
    # keep retrying a failed announce forever).
    py_trees.blackboard.Blackboard.clear()
    tree, announce_going, tuck, cap, unrec_announce = _build_goto_under_test()

    for _ in range(8):
        tree.tick_once()
        _restart(tree)

    assert cap.exhausted is True
    assert unrec_announce.tick_count == 1
