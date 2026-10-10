"""K3 (task-K, live-manipulation sim findings): grasp records WHICH method
satisfied the step -- autonomous grasp vs. referee (ex-machina) fallback.

F2's failure mode: create_grasp() returns a Selector over
[guarded_primary, ex_machina] -- the referee fallback is a deterministic
SUCCESS (open gripper, ask, wait, close), so the Selector collapses both
branches into an indistinguishable SUCCESS and a referee-assisted grasp
hides behind an autonomous-looking PASS.

Builds the REAL small_trees.create_grasp() tree (so the actual setter nodes
this fix adds are exercised exactly as production uses them) but stubs out
each branch's heavy ROS-backed tail -- mirrors the established convention in
test_gpsr_search_object_feedback.py's _build_tree (replace everything after
the guard/marker nodes with a trivial Success/Failure leaf).
"""
from __future__ import annotations

import py_trees
from py_trees.common import Access, Status

from behavior_tree.GPSR import small_trees
from behavior_tree.GPSR.small_trees import bb_keys


def _seed_grasp_allowed(allowed: bool = True) -> None:
    py_trees.blackboard.Blackboard.clear()
    writer = py_trees.blackboard.Client(name="seed")
    writer.register_key(bb_keys.GRASP_ASK_REFEREE, access=Access.WRITE)
    writer.set(bb_keys.GRASP_ASK_REFEREE, not allowed, overwrite=True)


def _read_step_method():
    reader = py_trees.blackboard.Client(name="reader")
    reader.register_key(bb_keys.STEP_METHOD, access=Access.READ)
    try:
        return reader.get(bb_keys.STEP_METHOD)
    except KeyError:
        return None


def _build_grasp_tree(*, primary_status: Status):
    """Real create_grasp(), with guarded_primary's heavy primary_with_retry
    subtree and ex_machina's heavy tail (everything after their respective
    K3 marker/setter nodes) replaced by trivial stub leaves so this can be
    ticked without any ROS backend.
    """
    tree = small_trees.create_grasp()
    assert isinstance(tree, py_trees.composites.Selector)
    guarded_primary, ex_machina = tree.children

    # guarded_primary: [CheckGraspAllowed, primary_with_retry, marker setter].
    # Swap out the heavy primary_with_retry (index 1) for a stub, keep the
    # guard and the K3 setter either side of it untouched.
    check_node, primary_node, marker_node = guarded_primary.children
    assert marker_node.name == "mark grasp autonomous"
    guarded_primary.remove_child(primary_node)
    stub = (
        py_trees.behaviours.Success(name="primary stub")
        if primary_status is Status.SUCCESS
        else py_trees.behaviours.Failure(name="primary stub")
    )
    guarded_primary.insert_child(stub, 1)

    # ex_machina: [marker setter, <everything else>]. Replace everything
    # after the K3 setter with a single stub SUCCESS leaf (deterministic
    # SUCCESS, same as production's real ex_machina tail).
    marker = ex_machina.children[0]
    assert marker.name == "mark grasp referee fallback"
    for child in list(ex_machina.children[1:]):
        ex_machina.remove_child(child)
    ex_machina.add_child(py_trees.behaviours.Success(name="ex_machina tail stub"))

    py_trees.trees.BehaviourTree(tree).setup()
    return tree


def test_primary_success_claims_autonomous():
    _seed_grasp_allowed(allowed=True)
    tree = _build_grasp_tree(primary_status=Status.SUCCESS)

    tree.tick_once()

    assert tree.status is Status.SUCCESS
    assert _read_step_method() == "autonomous"


def test_primary_failure_falls_back_to_ex_machina_and_claims_referee_fallback():
    _seed_grasp_allowed(allowed=True)
    tree = _build_grasp_tree(primary_status=Status.FAILURE)

    tree.tick_once()

    assert tree.status is Status.SUCCESS  # ex_machina is deterministic SUCCESS
    assert _read_step_method() == "referee_fallback"
