"""F2 (round-2 review): search_object's FAILURE feedback_message must summarise
how many search spots were actually swept, not surface an unfilled slot's
internal ``gpsr/search_pose_N is None`` guard message.

Builds the REAL ``small_trees.create_search_object()`` tree (so the guard
layer, capacity handling and ``SearchObjectSelector`` are exercised exactly
as production uses them) but stubs out each branch's tail (tuck arm / goto /
find_object -- ROS-backed, no server available in a unit test) with a plain
``py_trees.behaviours.Success``/``Failure`` leaf standing in for "the rest of
this branch either found the object or didn't".
"""
from __future__ import annotations

import py_trees
from py_trees.common import Access, Status

from behavior_tree.GPSR import small_trees
from behavior_tree.GPSR.small_trees import bb_keys


def _seed(capacity: int, filled_index: int, location: str = "bedroom") -> None:
    py_trees.blackboard.Blackboard.clear()
    writer = py_trees.blackboard.Client(name="seed")
    writer.register_key(bb_keys.TARGET_LOCATION, access=Access.WRITE)
    writer.set(bb_keys.TARGET_LOCATION, location, overwrite=True)
    for i in range(capacity):
        key = f"gpsr/search_pose_{i}"
        writer.register_key(key, access=Access.WRITE)
        writer.set(key, "pose" if i == filled_index else None, overwrite=True)


def _build_tree(capacity: int, filled_index: int, filled_outcome: Status, call_setup: bool = True):
    """Real create_search_object(), with each branch's tail (everything after
    the BtNode_CheckBBKeySet guard) replaced by a trivial stub leaf."""
    tree = small_trees.create_search_object(capacity=capacity)
    for branch in tree.children:
        for child in list(branch.children[1:]):
            branch.remove_child(child)
        index = int(branch.name.rsplit(" ", 1)[-1])
        stub_status = filled_outcome if index == filled_index else Status.FAILURE
        stub = (
            py_trees.behaviours.Success(name=f"stub {index}")
            if stub_status is Status.SUCCESS
            else py_trees.behaviours.Failure(name=f"stub {index}")
        )
        branch.add_child(stub)
    if call_setup:
        py_trees.trees.BehaviourTree(tree).setup()
    return tree


class _RunOnceThenFail(py_trees.behaviour.Behaviour):
    """Stub leaf: RUNNING on its first tick, FAILURE on every tick after."""

    def __init__(self, name: str):
        super().__init__(name)
        self._ticked = False

    def update(self):
        if not self._ticked:
            self._ticked = True
            return Status.RUNNING
        return Status.FAILURE


def test_all_fail_reports_spots_swept_and_location():
    capacity = small_trees.MAX_SEARCH_SPOTS
    _seed(capacity, filled_index=0, location="bedroom")
    tree = _build_tree(capacity, filled_index=0, filled_outcome=Status.FAILURE)

    tree.tick_once()

    assert tree.status is Status.FAILURE
    assert tree.feedback_message == (
        "search_object: swept 1 of 6 spots at bedroom, nothing found"
    )
    # tip() must surface the composite's OWN summary, not the last (unfilled)
    # branch's "gpsr/search_pose_5 is None" guard message.
    tip = tree.tip()
    assert tip is tree
    assert "is None" not in tip.feedback_message
    assert tip.feedback_message == tree.feedback_message


def test_first_slot_succeeds_short_circuits_without_touching_feedback():
    capacity = small_trees.MAX_SEARCH_SPOTS
    _seed(capacity, filled_index=0, location="bedroom")
    tree = _build_tree(capacity, filled_index=0, filled_outcome=Status.SUCCESS)

    tree.tick_once()

    assert tree.status is Status.SUCCESS
    # No FAILURE summary was ever computed/assigned.
    assert tree.feedback_message == ""


def test_summary_survives_a_memory_sequence_parent_abandoning_the_generator():
    """H1 (round-2 review, fix round 2): the production dispatcher places
    search_object as the second child of a memory Sequence
    (create_dispatcher's ``branch:{action_name}``, orchestrator.py ~2777).
    ``Sequence.tick()`` closes the child's generator (GeneratorExit at its
    last yield) the instant the child yields itself with FAILURE -- any
    "after the loop" tick() logic never runs on this path. The summary must
    already be set by the time the parent observes the FAILURE yield.
    """
    capacity = small_trees.MAX_SEARCH_SPOTS
    _seed(capacity, filled_index=0, location="bedroom")
    search_tree = _build_tree(
        capacity, filled_index=0, filled_outcome=Status.FAILURE, call_setup=False,
    )

    parent = py_trees.composites.Sequence("branch:search_object", memory=True)
    parent.add_child(py_trees.behaviours.Success(name="router"))
    parent.add_child(search_tree)
    py_trees.trees.BehaviourTree(parent).setup()

    parent.tick_once()

    assert parent.status is Status.FAILURE
    tip = parent.tip()
    assert tip.feedback_message == (
        "search_object: swept 1 of 6 spots at bedroom, nothing found"
    )


def test_swept_count_survives_a_multi_tick_earlier_branch():
    """H2 (round-2 review, fix round 2): a memory Selector invalidates every
    child before current_child at the START of each tick, so once a later
    branch goes RUNNING, an earlier branch's already-succeeded guard flips
    to INVALID before the sweep concludes. Branch 0's stub goes RUNNING for
    one tick then FAILURE; branch 1 is also filled and fails outright. Both
    guards must still be counted: "swept 2 of 6".
    """
    capacity = small_trees.MAX_SEARCH_SPOTS
    _seed(capacity, filled_index=0, location="bedroom")
    writer = py_trees.blackboard.Client(name="seed extra spot")
    writer.register_key("gpsr/search_pose_1", access=Access.WRITE)
    writer.set("gpsr/search_pose_1", "pose", overwrite=True)

    tree = small_trees.create_search_object(capacity=capacity)
    for branch in tree.children:
        for child in list(branch.children[1:]):
            branch.remove_child(child)
        index = int(branch.name.rsplit(" ", 1)[-1])
        stub = (
            _RunOnceThenFail(name=f"stub {index}")
            if index == 0
            else py_trees.behaviours.Failure(name=f"stub {index}")
        )
        branch.add_child(stub)
    py_trees.trees.BehaviourTree(tree).setup()

    tree.tick_once()
    assert tree.status is Status.RUNNING

    tree.tick_once()
    assert tree.status is Status.FAILURE
    assert tree.feedback_message == (
        "search_object: swept 2 of 6 spots at bedroom, nothing found"
    )


def test_summary_falls_back_to_static_blackboard_when_setup_never_ran():
    """L2 (round-2 review): compute_small_tree_roles/serialize_tree build the
    tree without ever calling setup(), so _client stays None. The location
    must still come from the shared blackboard, not read as "at None"."""
    py_trees.blackboard.Blackboard.clear()
    writer = py_trees.blackboard.Client(name="seed no-setup")
    writer.register_key(bb_keys.TARGET_LOCATION, access=Access.WRITE)
    writer.set(bb_keys.TARGET_LOCATION, "kitchen", overwrite=True)
    for i in range(small_trees.MAX_SEARCH_SPOTS):
        key = f"gpsr/search_pose_{i}"
        writer.register_key(key, access=Access.WRITE)
        writer.set(key, "pose" if i == 0 else None, overwrite=True)

    tree = small_trees.create_search_object()
    for branch in tree.children:
        for child in list(branch.children[1:]):
            branch.remove_child(child)
        index = int(branch.name.rsplit(" ", 1)[-1])
        branch.add_child(py_trees.behaviours.Failure(name=f"stub {index}"))
    # No setup() call: _client stays None (mirrors compute_small_tree_roles'
    # audit path, which never runs setup on the trees it inspects).

    tree.tick_once()

    assert tree.status is Status.FAILURE
    assert "at kitchen" in tree.feedback_message
    assert "at None" not in tree.feedback_message


def test_search_object_root_node_id_and_role_are_unchanged():
    from behavior_tree.GPSR.small_trees import compute_small_tree_roles
    from behavior_tree.GPSR.tree_serialization import serialize_tree

    tree = small_trees.create_search_object()
    document = serialize_tree(tree, kind="small/search_object")
    root = document["nodes"][0]
    assert root["node_id"] == "small/search_object/root"
    assert root["semantics"]["kind"] == "selector"
    assert root["semantics"]["category"] == "composite"

    small_trees._ROLES_CACHE = None  # force a fresh audit against our change
    roles = compute_small_tree_roles()
    assert "small/search_object/root" in roles.get("search_object_sweep", ())
