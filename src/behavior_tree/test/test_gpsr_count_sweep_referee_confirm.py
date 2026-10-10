"""Sim battery 2026-09-26 fixes (t2-rand-70718-fix2 runs 000 / 001 / 004).

- s70718-001 ("how many kitchen items on the side_table"): create_count did
  ONE look at tilt 10 (the table was 0.6 m below the camera -- out of frame),
  asked the detector only for the category members' venue-enriched prompts,
  counted 0, the VLM said "Zero", and the counted() gate accepted it while
  bleach, mustard and a bowl sat on the table.
- s70718-000 / -004: the grasp ex_machina branch asked the referee without
  ever looking at the furniture, and the "Dear referee ..." announce looked
  silent (BtNode_AnnounceFromBB never mirrored its inner node's feedback, so
  the bench's announcements.txt extractor could not see it).
- s70718-004: "Found a person." from a single YOLO 'person' hit on empty
  floor -- no confirmation before announcing / approaching.

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 (ROS pytest plugins break
collection).
"""
from __future__ import annotations

import py_trees
import pytest
from py_trees.common import Access, Status

from behavior_tree.GPSR import small_trees, validators
from behavior_tree.GPSR.small_trees import (
    COUNT_TILT_DEG,
    HUMAN_TILT_DEG,
    PAN_SWEEP_DEG,
    BtNode_AnnounceFromBB,
    BtNode_ComposeRefereeRequest,
    BtNode_CountDetections,
    BtNode_CountKeepMax,
    bb_keys,
)


def _walk(node):
    yield node
    for child in getattr(node, "children", []) or []:
        yield from _walk(child)


def _names(tree):
    return [getattr(n, "name", "") for n in _walk(tree)]


class _Obj:
    def __init__(self, cls):
        self.cls = cls


class _Detection:
    def __init__(self, labels, rgb_image=None):
        self.objects = [_Obj(c) for c in labels]
        self.rgb_image = rgb_image
        self.status = 0


RCW_PROMPTS = {
    "soup": "red tomato soup can", "mug": "red ceramic mug", "banana": "yellow banana",
    "mustard": "yellow mustard bottle", "sugar_box": "yellow sugar box",
    "cheez_it": "red cracker box", "bowl": "red bowl",
    "bleach": "white bleach cleanser bottle",
}
RCW_CATEGORIES = {
    "kitchen_item": ["bowl", "mug", "mustard", "bleach"],
    "food": ["banana", "sugar_box", "cheez_it", "soup"],
    "drink": [],
}


@pytest.fixture()
def rcw_knowledge(monkeypatch):
    from behavior_tree.GPSR import orchestrator as orch
    monkeypatch.setattr(orch, "KNOWN_OBJECT_PROMPTS", dict(RCW_PROMPTS))
    monkeypatch.setattr(orch, "KNOWN_OBJECT_NAMES", set(RCW_PROMPTS))
    monkeypatch.setattr(orch, "KNOWN_CATEGORIES", {k: list(v) for k, v in RCW_CATEGORIES.items()})


@pytest.fixture()
def real_vision(monkeypatch):
    """BtNode_CountDetections short-circuits to a synthetic 1 when the vision
    subsystem is mocked -- force the real counting path for these units."""
    from behavior_tree import config as bt_config
    monkeypatch.setattr(bt_config, "is_subsystem_mocked", lambda subsystem: False)


@pytest.fixture(autouse=True)
def _clean_blackboard():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


def _writer(*keys, name="seed"):
    client = py_trees.blackboard.Client(name=name)
    for key in keys:
        client.register_key(key, access=Access.WRITE)
        client.register_key(key, access=Access.READ)
    return client


# ---------------------------------------------------------------------------
# 1. Counting looks for everything, then filters (prompt materialisation)
# ---------------------------------------------------------------------------

def _materialise_count(obj):
    from behavior_tree.GPSR.orchestrator import materialise_params
    bb = _writer(
        bb_keys.TARGET_OBJECT_NAME, bb_keys.TARGET_OBJECT_PROMPT,
        bb_keys.COUNT_MEMBER_PROMPTS, bb_keys.COUNT_VLM_SUBJECT, bb_keys.COUNT_RECOUNTED,
        bb_keys.COUNT_ATTRIBUTE_QUESTION,
        bb_keys.TARGET_LOCATION, bb_keys.TARGET_POSE,
    )
    materialise_params(bb, "count", {"object": obj})
    return bb


@pytest.mark.parametrize("word", ["kitchen items", "kitchen item", "Kitchen_Items"])
def test_count_category_asks_for_every_known_object_and_filters_members(rcw_knowledge, word):
    bb = _materialise_count(word)
    prompt = bb.get(bb_keys.TARGET_OBJECT_PROMPT)
    for value in RCW_PROMPTS.values():
        assert value in prompt.split(" . ")
    assert bb.get(bb_keys.COUNT_MEMBER_PROMPTS) == [
        "red bowl", "red ceramic mug", "yellow mustard bottle", "white bleach cleanser bottle",
    ]
    subject = bb.get(bb_keys.COUNT_VLM_SUBJECT)
    assert "red bowl" in subject and "white bleach cleanser bottle" in subject
    assert bb.get(bb_keys.COUNT_RECOUNTED) is False


def test_count_specific_object_keeps_its_own_prompt_without_filter(rcw_knowledge):
    bb = _materialise_count("mug")
    assert bb.get(bb_keys.TARGET_OBJECT_PROMPT) == "red ceramic mug"
    assert bb.get(bb_keys.COUNT_MEMBER_PROMPTS) is None
    assert bb.get(bb_keys.COUNT_VLM_SUBJECT) == "red ceramic mug"


def test_count_person_descriptor_detects_plain_person_and_asks_the_vlm(rcw_knowledge):
    # Sim battery 2026-09-30 fix5: YOLO-World on the full phrase boxed a
    # pillow, a toy whale and the robot's arm -- detect COCO "person" instead.
    bb = _materialise_count("persons pointing to the right")
    assert bb.get(bb_keys.TARGET_OBJECT_PROMPT) == "person"
    assert bb.get(bb_keys.COUNT_MEMBER_PROMPTS) is None
    assert bb.get(bb_keys.COUNT_VLM_SUBJECT) == "persons pointing to the right"
    assert bb.get(bb_keys.COUNT_ATTRIBUTE_QUESTION) == "pointing to the right"


def test_count_prompts_for_empty_category_falls_back_to_literal(rcw_knowledge):
    from behavior_tree.GPSR.orchestrator import count_prompts_for
    assert count_prompts_for("drink") == ("drink", None)


def test_non_count_actions_keep_the_member_expansion_prompt(rcw_knowledge):
    # find_object for a category still asks only for the members (unchanged).
    from behavior_tree.GPSR.orchestrator import materialise_params
    bb = _writer(bb_keys.TARGET_OBJECT_NAME, bb_keys.TARGET_OBJECT_PROMPT,
                 bb_keys.TARGET_LOCATION, bb_keys.TARGET_POSE)
    materialise_params(bb, "find_object", {"object": "kitchen items"})
    assert bb.get(bb_keys.TARGET_OBJECT_PROMPT) == (
        "red bowl . red ceramic mug . yellow mustard bottle . white bleach cleanser bottle"
    )


# ---------------------------------------------------------------------------
# Member-label matching (validators.count_member_label_matches)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    ("label", "member", "expected"),
    [
        ("red bowl", "red bowl", True),
        ("bowl", "red bowl", True),                      # colour dropped
        ("white bowl", "red bowl", True),                # different colour
        ("bowls", "red bowl", True),                     # plural
        ("kitchen item.white bowl", "red bowl", True),   # class-prefixed
        ("kitchen_item.bowl", "red bowl", True),
        ("red cracker box", "red bowl", False),          # shared colour only
        ("red ceramic mug", "red bowl", False),
        ("yellow banana", "red bowl", False),
        ("sugar box", "yellow sugar box", True),         # multi-token key
        ("food.yellow sugar box", "yellow sugar box", True),
        ("box", "yellow sugar box", False),              # key fragment only
        ("bleach", "white bleach cleanser bottle", True),
        ("white bottle", "white bleach cleanser bottle", False),
        ("", "red bowl", False),
        ("bowl", "", False),
    ],
)
def test_count_member_label_matches(rcw_knowledge, label, member, expected):
    assert validators.count_member_label_matches(label, member) is expected


def test_count_member_label_matches_without_loaded_knowledge():
    # No orchestrator knowledge: the prompt itself is the key.
    from behavior_tree.GPSR import orchestrator as orch
    saved = dict(orch.KNOWN_OBJECT_PROMPTS)
    orch.KNOWN_OBJECT_PROMPTS.clear()
    try:
        assert validators.count_member_label_matches("mug", "mug")
        assert validators.count_member_label_matches("kitchen item.red mug", "red mug")
        assert not validators.count_member_label_matches("bowl", "mug")
    finally:
        orch.KNOWN_OBJECT_PROMPTS.update(saved)


# ---------------------------------------------------------------------------
# BtNode_CountDetections member filtering
# ---------------------------------------------------------------------------

def _count_node(members, labels, fail_if_zero=False):
    bb = _writer(bb_keys.TARGET_OBJECT_DETECTION, bb_keys.COUNT_MEMBER_PROMPTS, bb_keys.COUNT_VALUE)
    bb.set(bb_keys.TARGET_OBJECT_DETECTION, _Detection(labels), overwrite=True)
    bb.set(bb_keys.COUNT_MEMBER_PROMPTS, members, overwrite=True)
    node = BtNode_CountDetections(
        "count", bb_detection_src=bb_keys.TARGET_OBJECT_DETECTION,
        bb_count_dst=bb_keys.COUNT_VALUE, fail_if_zero=fail_if_zero,
        bb_member_prompts_key=bb_keys.COUNT_MEMBER_PROMPTS,
    )
    node.setup()
    return node, bb


def test_count_detections_keeps_only_member_matches(rcw_knowledge, real_vision):
    node, bb = _count_node(
        ["red bowl", "red ceramic mug"],
        ["white bowl", "kitchen item.bowl", "red ceramic mug", "yellow banana", "red cracker box"],
    )
    assert node.update() is Status.SUCCESS
    assert bb.get(bb_keys.COUNT_VALUE) == 3
    assert "3 of 5 detections" in node.feedback_message


def test_count_detections_counts_everything_without_members(rcw_knowledge, real_vision):
    for members in (None, []):
        node, bb = _count_node(members, ["yellow banana", "person", "red bowl"])
        assert node.update() is Status.SUCCESS
        assert bb.get(bb_keys.COUNT_VALUE) == 3


def test_count_detections_member_filter_zero_fails_when_asked(rcw_knowledge, real_vision):
    node, bb = _count_node(["red bowl"], ["yellow banana"], fail_if_zero=True)
    assert node.update() is Status.FAILURE
    assert bb.get(bb_keys.COUNT_VALUE) == 0


def test_count_detections_without_member_key_is_unchanged(real_vision):
    bb = _writer(bb_keys.TARGET_OBJECT_DETECTION, bb_keys.COUNT_VALUE)
    bb.set(bb_keys.TARGET_OBJECT_DETECTION, _Detection(["a", "b"]), overwrite=True)
    node = BtNode_CountDetections(
        "count", bb_detection_src=bb_keys.TARGET_OBJECT_DETECTION,
        bb_count_dst=bb_keys.COUNT_VALUE,
    )
    node.setup()
    assert node.update() is Status.SUCCESS
    assert bb.get(bb_keys.COUNT_VALUE) == 2


# ---------------------------------------------------------------------------
# 2. Count sweeps and lowers its gaze (tree structure)
# ---------------------------------------------------------------------------

def test_count_tilts_look_progressively_down_within_the_pan_tilt_limits():
    from behavior_tree.GPSR.modifiable_nodes import PAN_RANGE_DEG, TILT_RANGE_DEG
    assert COUNT_TILT_DEG == [20.0, 45.0, 60.0]
    assert all(TILT_RANGE_DEG[0] <= t <= TILT_RANGE_DEG[1] for t in COUNT_TILT_DEG)
    assert all(PAN_RANGE_DEG[0] <= p <= PAN_RANGE_DEG[1] for p in PAN_SWEEP_DEG)


def test_count_tree_shape():
    tree = small_trees.create_count()
    assert tree.name == "small/count"
    assert [c.name for c in tree.children] == [
        "arm to orbbec look (best effort)", "announce scanning", "mark not recounted",
        "count strategies", "announce count", "buffer count",
    ]
    strategies = tree.children[3]
    assert isinstance(strategies, py_trees.composites.Selector) and strategies.memory
    assert [c.name for c in strategies.children] == [
        "count/by_detector", "count/vlm_fallback", "count/recount",
    ]
    names = _names(tree)
    # The single tilt-10 look is gone; every pan x COUNT_TILT_DEG view exists
    # in BOTH the first sweep and the recount sweep.
    assert "turn pantilt" not in names
    for label in ("count objects", "recount objects"):
        for tilt in COUNT_TILT_DEG:
            for pan in PAN_SWEEP_DEG:
                assert f"{label} pan={pan:+.0f} tilt={tilt:+.0f}" in names
    # Person counts sweep the human tilts instead.
    for tilt in HUMAN_TILT_DEG:
        assert f"count persons pan=+0 tilt={tilt:+.0f}" in names
    assert names.count("counting persons?") == 2


def test_count_sweeps_visit_every_view_not_first_hit():
    tree = small_trees.create_count()
    sweeps = [n for n in _walk(tree) if n.name.endswith("pantilt sweep")]
    assert len(sweeps) == 4  # (persons, objects) x (count, recount)
    for sweep in sweeps:
        assert isinstance(sweep, py_trees.composites.Sequence), sweep.name
        assert sweep.memory
        for view in sweep.children:
            assert view.children[0].name == "sweep saturated?"


def test_count_vlm_fallback_asks_about_the_members_on_the_best_frame():
    tree = small_trees.create_count()
    vlms = [n for n in _walk(tree) if type(n).__name__ == "BtNode_VLMQuery"
            and n.name == "vlm count"]
    assert len(vlms) == 1  # the vlm-template modification targets exactly one
    vlm = vlms[0]
    assert "{value}" in vlm._template
    assert vlm._fill_key == bb_keys.COUNT_VLM_SUBJECT
    assert vlm._src == bb_keys.COUNT_BEST_DETECTION
    vlm_branch = tree.children[3].children[1]
    assert [c.name for c in vlm_branch.children] == [
        "announce vlm fallback", "vlm count", "parse vlm count", "vlm counted any?",
    ]


def test_count_recount_branch_announces_marks_and_resweeps():
    tree = small_trees.create_count()
    recount = tree.children[3].children[2]
    assert [c.name for c in recount.children] == [
        "announce recount", "mark recounted", "recount sweep",
    ]
    assert recount.children[0].given_msg == "I did not see any, let me look again."
    marker = recount.children[1]
    assert marker._key == bb_keys.COUNT_RECOUNTED and marker._value is True
    reset = tree.children[2]
    assert reset._key == bb_keys.COUNT_RECOUNTED and reset._value is False


def test_count_view_verifies_members_and_keeps_max():
    tree = small_trees.create_count()
    view = next(n for n in _walk(tree) if n.name == "count objects view pan=+0 tilt=+60")
    look = view.children[1].decorated
    assert [c.name for c in look.children] == [
        "pan +0 tilt +60", "scan to count (no-match keeps frame)",
        "count view detections", "keep best count", "keep unclassified frame",
    ]
    counter = look.children[2]
    assert counter._member_key == bb_keys.COUNT_MEMBER_PROMPTS
    assert counter._dst == bb_keys.COUNT_VIEW_VALUE
    keep = look.children[3]
    assert keep._count_dst == bb_keys.COUNT_VALUE
    assert keep._best_dst == bb_keys.COUNT_BEST_DETECTION
    scan = look.children[1].decorated
    assert scan.return_rgb_image is True
    assert scan.return_segments is True  # masks -> pixel boxes for the filters
    assert counter._unclassified_dst == bb_keys.COUNT_VIEW_UNCLASSIFIED


# ---------------------------------------------------------------------------
# Count sweep behaviour (ticked, scan/turn leaves stubbed)
# ---------------------------------------------------------------------------

class _ScriptedScan(py_trees.behaviour.Behaviour):
    """Stands in for turn+scan: writes the next scripted detection."""

    def __init__(self, name, script, calls):
        super().__init__(name)
        self._script = script
        self._calls = calls
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(bb_keys.TARGET_OBJECT_DETECTION, access=Access.WRITE)

    def update(self):
        labels = self._script.pop(0) if self._script else []
        self._calls.append(list(labels))
        self._client.set(
            bb_keys.TARGET_OBJECT_DETECTION, _Detection(labels, rgb_image=len(labels)),
            overwrite=True,
        )
        return Status.SUCCESS


def _skip_whole_body_relook(tree):
    """Stub the person-attribute whole-body re-look (real pan-tilt + scan)
    as unavailable; its own tests cover it."""
    for node in list(_walk(tree)):
        if node.name.endswith(" whole-body re-look") and getattr(node, "children", None):
            for child in list(node.children):
                node.remove_child(child)
            node.add_child(py_trees.behaviours.Failure("re-look stubbed"))


def _stubbed_count_sweep(script, prompt="red bowl . red ceramic mug", members=None):
    sweep = small_trees._count_sweep("count")
    calls = []
    for node in list(_walk(sweep)):
        if node.name.startswith("count objects pan=") or node.name.startswith("count persons pan="):
            turn, scan = node.children[0], node.children[1]
            node.remove_child(turn)
            node.remove_child(scan)
            node.insert_child(_ScriptedScan(f"scripted {node.name}", script, calls), 0)
    _skip_whole_body_relook(sweep)
    seed = _writer(bb_keys.TARGET_OBJECT_PROMPT, bb_keys.COUNT_MEMBER_PROMPTS,
                   bb_keys.COUNT_VALUE, bb_keys.COUNT_BEST_DETECTION,
                   bb_keys.COUNT_SWEEP_DONE, bb_keys.COUNT_VIEW_COUNTS)
    seed.set(bb_keys.TARGET_OBJECT_PROMPT, prompt, overwrite=True)
    seed.set(bb_keys.COUNT_MEMBER_PROMPTS, members, overwrite=True)
    py_trees.trees.BehaviourTree(sweep).setup()
    return sweep, calls, seed


def _tick_done(root, budget=200):
    for _ in range(budget):
        root.tick_once()
        if root.status != Status.RUNNING:
            return root.status
    raise AssertionError("tree did not finish within the tick budget")


def test_count_sweep_keeps_maximum_and_stops_once_a_view_adds_nothing(real_vision):
    script = [[], ["a"], ["a", "b", "c"], ["a", "b"], ["x"], [], [], [], []]
    sweep, calls, seed = _stubbed_count_sweep(list(script))
    assert _tick_done(sweep) is Status.SUCCESS
    # view 1: 0 -> max 0; view 2: 1 -> max 1; view 3: 3 -> max 3; view 4:
    # 2 <= 3 with max >= 1 -> saturated, the remaining five views are skipped.
    assert calls == [[], ["a"], ["a", "b", "c"], ["a", "b"]]
    assert seed.get(bb_keys.COUNT_VALUE) == 3
    assert seed.get(bb_keys.COUNT_SWEEP_DONE) is True
    assert seed.get(bb_keys.COUNT_BEST_DETECTION).rgb_image == 3


def test_count_sweep_zero_everywhere_visits_all_views_and_keeps_busiest_frame(real_vision, rcw_knowledge):
    # Member filter: nothing on the table is a kitchen item, but one view saw
    # two (non-member) objects -- that frame is the VLM fallback's best shot.
    script = [[], ["yellow banana"], [], ["yellow banana", "red cracker box"], [], [], [], [], []]
    sweep, calls, seed = _stubbed_count_sweep(list(script), members=["red bowl", "red ceramic mug"])
    assert _tick_done(sweep) is Status.SUCCESS
    assert len(calls) == len(PAN_SWEEP_DEG) * len(COUNT_TILT_DEG)
    assert seed.get(bb_keys.COUNT_VALUE) == 0
    assert seed.get(bb_keys.COUNT_SWEEP_DONE) is False
    assert seed.get(bb_keys.COUNT_BEST_DETECTION).rgb_image == 2


def test_count_sweep_for_persons_uses_human_tilts(real_vision):
    script = [[] for _ in range(20)]
    sweep, calls, seed = _stubbed_count_sweep(list(script), prompt="persons pointing to the right")
    assert _tick_done(sweep) is Status.SUCCESS
    # people stand off to the side of a search spot: persons also look +-90
    from behavior_tree.GPSR.small_trees import HUMAN_PAN_SWEEP_DEG
    assert len(calls) == len(HUMAN_PAN_SWEEP_DEG) * len(HUMAN_TILT_DEG)
    assert {90.0, -90.0} <= set(HUMAN_PAN_SWEEP_DEG)


def test_count_sweep_resets_a_previous_count_before_sweeping(real_vision):
    script = [[] for _ in range(20)]
    sweep, calls, seed = _stubbed_count_sweep(list(script))
    seed.set(bb_keys.COUNT_VALUE, 7, overwrite=True)
    seed.set(bb_keys.COUNT_SWEEP_DONE, True, overwrite=True)
    seed.set(bb_keys.COUNT_VIEW_COUNTS, [9], overwrite=True)
    assert _tick_done(sweep) is Status.SUCCESS
    assert seed.get(bb_keys.COUNT_VALUE) == 0
    assert 9 not in seed.get(bb_keys.COUNT_VIEW_COUNTS)
    assert len(calls) == len(PAN_SWEEP_DEG) * len(COUNT_TILT_DEG)


def test_count_keep_max_unit():
    bb = _writer(bb_keys.COUNT_VIEW_VALUE, bb_keys.TARGET_OBJECT_DETECTION, bb_keys.COUNT_VALUE,
                 bb_keys.COUNT_BEST_DETECTION, bb_keys.COUNT_SWEEP_DONE)
    node = BtNode_CountKeepMax(
        "keep", bb_view_key=bb_keys.COUNT_VIEW_VALUE, bb_detection_key=bb_keys.TARGET_OBJECT_DETECTION,
        bb_count_dst=bb_keys.COUNT_VALUE, bb_best_dst=bb_keys.COUNT_BEST_DETECTION,
        bb_done_key=bb_keys.COUNT_SWEEP_DONE,
    )
    node.setup()

    def view(count, raw):
        bb.set(bb_keys.COUNT_VIEW_VALUE, count, overwrite=True)
        bb.set(bb_keys.TARGET_OBJECT_DETECTION, _Detection(["o"] * raw, rgb_image=raw), overwrite=True)
        assert node.update() is Status.SUCCESS
        return bb.get(bb_keys.COUNT_VALUE), bb.get(bb_keys.COUNT_SWEEP_DONE), bb.get(bb_keys.COUNT_BEST_DETECTION).rgb_image

    assert view(0, 1) == (0, False, 1)
    assert view(0, 4) == (0, False, 4)      # zero views: busiest frame wins
    assert view(2, 2) == (2, False, 2)      # first hit never stops the sweep
    assert view(3, 3) == (3, False, 3)      # a later view can add
    assert view(3, 9) == (3, True, 9)       # adds nothing -> saturated
    assert view(1, 1) == (3, True, 9)       # the max is kept


def test_count_keep_max_waits_for_three_views_before_saturating():
    bb = _writer("view", "det", "count", "best", "done", "history")
    bb.set("count", 0, overwrite=True)
    bb.set("best", None, overwrite=True)
    bb.set("history", [], overwrite=True)
    node = BtNode_CountKeepMax(
        "keep", "view", "det", "count", "best", "done",
        bb_history_dst="history",
    )
    node.setup()

    for count in (4, 3):
        bb.set("view", count, overwrite=True)
        bb.set("det", _Detection(["o"] * count), overwrite=True)
        assert node.update() is Status.SUCCESS
        assert bb.get("done") is False

    bb.set("view", 3, overwrite=True)
    bb.set("det", _Detection(["o"] * 3), overwrite=True)
    assert node.update() is Status.SUCCESS
    assert bb.get("history") == [4, 3, 3]
    assert bb.get("done") is True


# ---------------------------------------------------------------------------
# 3. Gate: zero is suspicious (evidence plumbing; verdicts in
#    test_gpsr_fact_validators.py)
# ---------------------------------------------------------------------------

def test_gate_evidence_carries_the_recount_marker():
    from behavior_tree.GPSR.orchestrator import _target_gate_evidence
    bb = _writer(bb_keys.COUNT_VALUE, bb_keys.COUNT_RECOUNTED)
    bb.set(bb_keys.COUNT_VALUE, 0, overwrite=True)
    assert _target_gate_evidence(bb) == {"count_value": 0}
    bb.set(bb_keys.COUNT_RECOUNTED, True, overwrite=True)
    assert _target_gate_evidence(bb) == {"count_value": 0, "count_recounted": True}


def test_postcondition_gate_rejects_zero_without_recount_and_accepts_with(monkeypatch):
    from behavior_tree.GPSR.orchestrator import BtNode_TargetPostconditionCheck
    bb = _writer(bb_keys.FACTS, bb_keys.COUNT_VALUE, bb_keys.COUNT_RECOUNTED,
                 bb_keys.TARGET_OBJECT_NAME, bb_keys.DEFERRED_PRECONDITIONS,
                 bb_keys.GATE_COMPLETED_STEPS)
    bb.set(bb_keys.FACTS, [], overwrite=True)
    bb.set(bb_keys.DEFERRED_PRECONDITIONS, [], overwrite=True)
    bb.set(bb_keys.TARGET_OBJECT_NAME, "kitchen items", overwrite=True)
    bb.set(bb_keys.COUNT_VALUE, 0, overwrite=True)
    plan = [{"action": "count", "params": {"object": "kitchen items"}}]

    def gate():
        node = BtNode_TargetPostconditionCheck(
            "post", ["counted(kitchen items)"], 0, action_plan=plan,
            target_object="kitchen items",
        )
        node.setup()
        return node

    node = gate()
    assert node.update() is Status.FAILURE
    assert node.feedback_message == "postcondition unmet: counted(kitchen items) (INVALID)"
    # The failed count is NOT committed, so the replan re-runs it.
    assert bb.get(bb_keys.GATE_COMPLETED_STEPS) == []
    assert bb.get(bb_keys.FACTS) == []

    bb.set(bb_keys.COUNT_RECOUNTED, True, overwrite=True)
    node = gate()
    assert node.update() is Status.SUCCESS
    assert "counted(kitchen_items)" in bb.get(bb_keys.FACTS)


def test_replan_alternatives_name_count_for_a_zero_count_gate_failure():
    from behavior_tree.GPSR.planner import _alternatives_for_reason
    text = _alternatives_for_reason("postcondition unmet: counted(kitchen items) (INVALID)")
    assert "count" in text


# ---------------------------------------------------------------------------
# 4. Referee asks with eyes open
# ---------------------------------------------------------------------------

def test_ex_machina_looks_at_the_furniture_before_opening_the_gripper():
    tree = small_trees.create_grasp()
    _, ex_machina = tree.children
    names = [c.name for c in ex_machina.children]
    look = names.index("look for object at furniture (best effort)")
    compose = names.index("compose referee request")
    open_idx = names.index("open gripper")
    ask = names.index("ask referee")
    assert names.index("open appliance if needed (best effort)") < look < compose < open_idx < ask
    assert names[ask + 1:] == ["announce put in gripper", "wait", "sim referee hand-object", "close gripper"]
    sweep = ex_machina.children[look].decorated
    sweep_names = _names(sweep)
    assert sweep.name == "grasp/referee look pantilt sweep"
    for tilt in small_trees.OBJECT_TILT_DEG:
        assert f"grasp/referee look pan=+0 tilt={tilt:+.0f}" in sweep_names
    verify = [n for n in _walk(sweep) if n.name == "verify object seen at furniture"]
    assert verify and all(v._dst == bb_keys.REFEREE_SEEN_COUNT for v in verify)
    scans = [n for n in _walk(sweep) if n.name == "referee generalist scan"]
    assert scans and all(s.bb_source == bb_keys.TARGET_OBJECT_PROMPT for s in scans)


def test_ex_machina_ask_referee_speaks_the_composed_request():
    tree = small_trees.create_grasp()
    _, ex_machina = tree.children
    ask = next(c for c in ex_machina.children if c.name == "ask referee")
    assert isinstance(ask, BtNode_AnnounceFromBB)
    assert ask._bb_text_key == bb_keys.REFEREE_REQUEST_TEXT
    assert ask._prefix == ""
    thanks = next(c for c in ex_machina.children if c.name == "announce put in gripper")
    assert thanks.given_msg == "Thank you."


@pytest.mark.parametrize(
    ("obj", "furniture", "label", "expected"),
    [
        ("soup", "shelf", "red tomato soup can",
         "Dear referee, I can see the red tomato soup can on the shelf. Please put the soup in my gripper."),
        ("soup", "shelf", "",
         "Dear referee, I cannot see the soup on the shelf. Please hand me the soup and put it in my gripper."),
        ("sugar_box", "coat_rack", "kitchen_item.sugar box",
         "Dear referee, I can see the kitchen item.sugar box on the coat rack. Please put the sugar box in my gripper."),
        ("", "", "",
         "Dear referee, I cannot see the object on the furniture. Please hand me the object and put it in my gripper."),
    ],
)
def test_compose_referee_request_text(obj, furniture, label, expected):
    assert BtNode_ComposeRefereeRequest.compose(obj, furniture, label) == expected


def test_compose_referee_request_reads_detection_and_furniture_from_blackboard(monkeypatch):
    # rooms are "in", furniture is "on" (fix4 run 3 said "on the bedroom")
    from behavior_tree.GPSR import orchestrator as orch
    monkeypatch.setattr(orch, "ROOM_SEARCH_SPOTS", {"kitchen": ["kitchen_table"]})
    bb = _writer(bb_keys.REFEREE_REQUEST_TEXT, bb_keys.TARGET_OBJECT_DETECTION,
                 bb_keys.TARGET_OBJECT_NAME, bb_keys.GRASP_REFEREE_LOCATION, bb_keys.TARGET_LOCATION)
    bb.set(bb_keys.TARGET_OBJECT_NAME, "soup", overwrite=True)
    bb.set(bb_keys.GRASP_REFEREE_LOCATION, "shelf", overwrite=True)
    bb.set(bb_keys.TARGET_LOCATION, "kitchen", overwrite=True)
    node = BtNode_ComposeRefereeRequest("compose", bb_dst=bb_keys.REFEREE_REQUEST_TEXT)
    node.setup()

    bb.set(bb_keys.TARGET_OBJECT_DETECTION, _Detection(["red tomato soup can", "red bowl"]), overwrite=True)
    assert node.update() is Status.SUCCESS
    assert bb.get(bb_keys.REFEREE_REQUEST_TEXT) == (
        "Dear referee, I can see the red tomato soup can on the shelf. Please put the soup in my gripper."
    )
    assert node.feedback_message == bb.get(bb_keys.REFEREE_REQUEST_TEXT)

    bb.set(bb_keys.TARGET_OBJECT_DETECTION, _Detection([]), overwrite=True)
    assert node.update() is Status.SUCCESS
    assert bb.get(bb_keys.REFEREE_REQUEST_TEXT) == (
        "Dear referee, I cannot see the soup on the shelf. Please hand me the soup and put it in my gripper."
    )

    # No referee furniture resolved -> the step's own location, then "furniture".
    bb.set(bb_keys.GRASP_REFEREE_LOCATION, "", overwrite=True)
    bb.set(bb_keys.TARGET_OBJECT_DETECTION, None, overwrite=True)
    assert node.update() is Status.SUCCESS
    assert "in the kitchen." in bb.get(bb_keys.REFEREE_REQUEST_TEXT)
    bb.set(bb_keys.TARGET_LOCATION, None, overwrite=True)
    assert node.update() is Status.SUCCESS
    assert "on the furniture." in bb.get(bb_keys.REFEREE_REQUEST_TEXT)


class _FakeAnnounce:
    def __init__(self, status, feedback):
        self._status = status
        self.feedback_message = feedback

    def update(self):
        return self._status

    def terminate(self, new_status):
        self.feedback_message = f"terminated {new_status.name}: " + self.feedback_message


def test_announce_from_bb_mirrors_inner_feedback_so_the_bench_can_see_it():
    # Root cause of the "silent" Dear-referee ask in runs s70718-000/-004:
    # the wrapper returned the inner status with an EMPTY feedback, so the
    # tree dump (and the bench's announcements.txt, extracted from
    # "Finished announcing ..." lines) never showed what was spoken.
    node = BtNode_AnnounceFromBB("ask referee", bb_keys.REFEREE_REQUEST_TEXT)
    node._inner = _FakeAnnounce(Status.RUNNING, "Still announcing...")
    assert node.update() is Status.RUNNING
    assert node.feedback_message == "Still announcing..."
    node._inner = _FakeAnnounce(Status.SUCCESS, "Finished announcing Dear referee, I can see the soup on the shelf.")
    assert node.update() is Status.SUCCESS
    assert node.feedback_message.startswith("Finished announcing Dear referee")
    node.terminate(Status.SUCCESS)
    assert node.feedback_message.startswith("terminated SUCCESS: Finished announcing Dear referee")


# ---------------------------------------------------------------------------
# 5. Person confirmation
# ---------------------------------------------------------------------------

def test_find_person_sweep_branch_scans_then_confirms():
    tree = small_trees.create_find_person()
    assert tree.children[3].name == "find_person pantilt sweep"  # index pinned by modifiable_nodes tests
    for branch in tree.children[3].children:
        assert [c.name for c in branch.children][1] == "person scan + confirm"
        scan_confirm = branch.children[1]
        assert isinstance(scan_confirm, py_trees.composites.Sequence) and scan_confirm.memory
        first, second = scan_confirm.children
        assert first.name == "person scan strategies"
        assert second.name == "person confirm strategies"
        assert first is not second
        assert isinstance(second, py_trees.composites.Selector) and second.memory
        assert [c.name for c in first.children] == [c.name for c in second.children]
    assert tree.children[4].given_msg == "Found a person."
    assert tree.children[5].name == "record resolved person query"


def test_find_person_confirm_carries_the_attribute_specialist():
    tree = small_trees.create_find_person(extra_specialist={"gate": "red jacket"})
    scan_confirm = tree.children[3].children[0].children[1]
    for strategies in scan_confirm.children:
        assert "red jacket person branch" in [c.name for c in strategies.children]


def test_deliver_uses_the_confirming_find_person():
    tree = small_trees.create_deliver()
    goto = tree.children[2]
    assert isinstance(goto, py_trees.composites.Selector)
    assert goto.name == "retry deliver goto"
    assert goto.children[0].name == "reuse resolved delivery navigation"
    assert goto.children[1].name == "reuse successful target navigation"
    assert isinstance(goto.children[2], py_trees.decorators.FailureIsSuccess)
    assert goto.children[2].name == "deliver room navigation best effort"
    find_person = next(c for c in tree.children if c.name == "small/find_person")
    assert "person confirm strategies" in _names(find_person)


def test_deliver_reuses_only_successful_matching_return_navigation():
    """An explicit goto(start_position) must not be repeated by deliver.

    The navigation evidence is success-gated by the orchestrator.  A matching
    ``LAST_NAV_LOCATION`` therefore permits the delivery tree to reuse the
    already reached pose, while a stale/mismatched location falls through to
    its normal room-navigation retry.
    """
    bb = _writer(bb_keys.LAST_NAV_LOCATION, bb_keys.TARGET_LOCATION)
    bb.set(bb_keys.LAST_NAV_LOCATION, "start_position", overwrite=True)
    bb.set(bb_keys.TARGET_LOCATION, "start position", overwrite=True)

    matched = small_trees.BtNode_CheckLastNavMatchesTarget()
    matched.setup()
    matched.tick_once()
    assert matched.status is Status.SUCCESS

    bb.set(bb_keys.LAST_NAV_LOCATION, "shelf", overwrite=True)
    mismatched = small_trees.BtNode_CheckLastNavMatchesTarget()
    mismatched.setup()
    mismatched.tick_once()
    assert mismatched.status is Status.FAILURE


def test_delivery_reuses_resolved_person_when_deliver_has_no_location():
    """A split plan must not navigate back to its stale room anchor.

    The preceding find_person/approach_person steps leave both person poses on
    the blackboard. A location-less deliver step should accept that evidence;
    an explicit recipient_location must still force the normal fresh path.
    """
    bb = _writer(
        bb_keys.CURRENT_PARAMS,
        bb_keys.TARGET_PERSON_POSE,
        bb_keys.PERSON_NAV_POSE,
        bb_keys.RESOLVED_PERSON_QUERY,
    )
    bb.set(bb_keys.CURRENT_PARAMS, {
        "object": "bowl", "recipient": "person pointing to the left",
    }, overwrite=True)
    bb.set(bb_keys.TARGET_PERSON_POSE, object(), overwrite=True)
    bb.set(bb_keys.PERSON_NAV_POSE, object(), overwrite=True)
    bb.set(bb_keys.RESOLVED_PERSON_QUERY, {
        "descriptor": "person pointing to the left", "location": "living_room",
    }, overwrite=True)

    guard = small_trees.BtNode_CheckResolvedDeliveryTarget()
    guard.setup()
    guard.tick_once()
    assert guard.status is Status.SUCCESS

    # A plan may emit find_person but omit a separate approach_person. The
    # delivery tree must still skip the stale room goto and approach this pose
    # itself.
    bb.set(bb_keys.PERSON_NAV_POSE, None, overwrite=True)
    goto_guard = small_trees.BtNode_CheckResolvedDeliveryTarget(require_nav=False)
    goto_guard.setup()
    goto_guard.tick_once()
    assert goto_guard.status is Status.SUCCESS

    bb.set(bb_keys.CURRENT_PARAMS, {
        "object": "bowl", "recipient": "person pointing to the left",
        "recipient_location": "living_room",
    }, overwrite=True)
    bb.set(bb_keys.PERSON_NAV_POSE, object(), overwrite=True)
    explicit_match = small_trees.BtNode_CheckResolvedDeliveryTarget()
    explicit_match.setup()
    explicit_match.tick_once()
    assert explicit_match.status is Status.SUCCESS

    bb.set(bb_keys.CURRENT_PARAMS, {
        "object": "bowl", "recipient": "person pointing to the left",
        "recipient_location": "kitchen",
    }, overwrite=True)
    explicit_mismatch = small_trees.BtNode_CheckResolvedDeliveryTarget()
    explicit_mismatch.setup()
    explicit_mismatch.tick_once()
    assert explicit_mismatch.status is Status.FAILURE

    bb.set(bb_keys.CURRENT_PARAMS, {
        "object": "bowl", "recipient": "person waving",
    }, overwrite=True)
    recipient_mismatch = small_trees.BtNode_CheckResolvedDeliveryTarget()
    recipient_mismatch.setup()
    recipient_mismatch.tick_once()
    assert recipient_mismatch.status is Status.FAILURE

    bb.set(bb_keys.CURRENT_PARAMS, {
        "object": "bowl", "recipient": "person pointing to the left",
    }, overwrite=True)
    bb.set(bb_keys.RESOLVED_PERSON_QUERY, None, overwrite=True)
    no_context = small_trees.BtNode_CheckResolvedDeliveryTarget()
    no_context.setup()
    no_context.tick_once()
    assert no_context.status is Status.FAILURE

    tree = small_trees.create_deliver()
    find_person = next(c for c in tree.children if c.name == "small/find_person")
    approach = next(c for c in tree.children if c.name == "small/approach_person")
    assert find_person.children[0].name == "operator handoff: skip person scan"
    assert find_person.children[1].name == "reuse resolved delivery person"
    assert approach.children[0].name == "operator handoff: skip approach"
    assert approach.children[1].name == "reuse resolved delivery person"


def test_find_person_records_confirmed_query_context():
    bb = _writer(
        bb_keys.TARGET_PERSON_PROMPT,
        bb_keys.TARGET_PERSON_POSE,
        bb_keys.TARGET_LOCATION,
        bb_keys.RESOLVED_PERSON_QUERY,
    )
    bb.set(bb_keys.TARGET_PERSON_PROMPT, "Person pointing to the left", overwrite=True)
    bb.set(bb_keys.TARGET_LOCATION, "living_room", overwrite=True)
    bb.set(bb_keys.TARGET_PERSON_POSE, object(), overwrite=True)

    record = small_trees.BtNode_RecordResolvedPersonQuery()
    record.setup()
    record.tick_once()
    assert record.status is Status.SUCCESS
    assert bb.get(bb_keys.RESOLVED_PERSON_QUERY) == {
        "descriptor": "Person pointing to the left", "location": "living_room",
    }

    bb.set(bb_keys.TARGET_PERSON_POSE, None, overwrite=True)
    record.tick_once()
    assert bb.get(bb_keys.RESOLVED_PERSON_QUERY) is None


def test_delivery_leaves_gripper_open_after_handoff():
    """The handoff must not issue a close command after opening the gripper.

    The simulator latches successful gripper goals.  A close immediately after
    the release wait can catch the delivered object before it has cleared the
    fingers, leaving it attached at the recipient.
    """
    tree = small_trees.create_deliver()
    assert [child.name for child in tree.children[-4:]] == [
        "announce take it", "wait for take", "open gripper", "wait",
    ]
    assert all(child.name != "close gripper" for child in tree.children)


def test_delivery_approach_uses_wider_ring_for_stale_person_centroid():
    from behavior_tree.TemplateNodes.Navigation import BtNode_Approach

    tree = small_trees.create_deliver()
    reach = next(node for node in _walk(tree) if node.name == "reach person")
    approach = reach.children[0].decorated
    assert isinstance(approach, BtNode_Approach)
    assert approach.max_distance == small_trees.DELIVERY_PERSON_MAX_DISTANCE_M
    assert isinstance(reach.children[2], py_trees.composites.Sequence)
    assert reach.children[2].children[1].name == "recipient approach not confirmed"


def test_delivery_does_not_succeed_when_approach_and_goto_fail():
    tree = small_trees.create_deliver()
    reach = next(node for node in _walk(tree) if node.name == "reach person")

    # Force both navigation strategies to fail, and stand in for the spoken
    # request completing. The delivery-specific fallback must still fail, so
    # the parent delivery Sequence cannot proceed to "open gripper".
    for index, old_child in enumerate(list(reach.children[:2])):
        reach.remove_child(old_child)
        reach.insert_child(py_trees.behaviours.Failure(f"navigation {index} failed"), index)
    ask_then_fail = reach.children[2]
    announcement = ask_then_fail.children[0]
    ask_then_fail.remove_child(announcement)
    ask_then_fail.insert_child(py_trees.behaviours.Success("announcement completed"), 0)

    reach.tick_once()

    assert reach.status is Status.FAILURE


def test_failed_delivery_approach_clears_unconfirmed_nav_pose():
    tree = small_trees.create_deliver()
    approach = next(node for node in _walk(tree) if node.name == "small/approach_person")
    delivery_seq = approach.children[2]
    confirmation = next(
        node for node in _walk(delivery_seq)
        if node.name == "delivery approach confirmation"
    )
    old_reach = confirmation.children[0]
    confirmation.remove_child(old_reach)
    confirmation.insert_child(py_trees.behaviours.Failure("navigation failed"), 0)

    bb = _writer(bb_keys.PERSON_NAV_POSE)
    bb.set(bb_keys.PERSON_NAV_POSE, object(), overwrite=True)
    py_trees.trees.BehaviourTree(confirmation).setup()
    confirmation.tick_once()

    assert confirmation.status is Status.FAILURE
    assert bb.get(bb_keys.PERSON_NAV_POSE) is None


@pytest.mark.parametrize("location", [
    "start_position", "instruction point", "operator", "me", "the_operator",
])
def test_operator_delivery_guard_accepts_start_pose_aliases(location):
    # Operator handoff uses the pose captured at command intake.  The
    # delivery-specific find/approach trees keep their normal scan branches,
    # but select a no-op branch for these aliases so a false person detection
    # cannot pull the robot away from the handoff pose.
    bb = _writer(bb_keys.TARGET_LOCATION)
    bb.set(bb_keys.TARGET_LOCATION, location, overwrite=True)
    guard = small_trees.BtNode_CheckOperatorDelivery()
    guard.setup()
    guard.tick_once()
    assert guard.status is Status.SUCCESS

    tree = small_trees.create_deliver()
    find_person = next(c for c in tree.children if c.name == "small/find_person")
    approach = next(c for c in tree.children if c.name == "small/approach_person")
    assert find_person.children[0].name == "operator handoff: skip person scan"
    assert approach.children[0].name == "operator handoff: skip approach"


class _ScriptedDetect(py_trees.behaviour.Behaviour):
    def __init__(self, name, script, calls):
        super().__init__(name)
        self._script = script
        self._calls = calls

    def update(self):
        status = self._script.pop(0)
        self._calls.append((self.name, status))
        return status


def test_person_sweep_moves_on_when_confirmation_fails_and_stops_when_it_holds():
    # Pose 1: first scan sees a person, the confirm does not -> the branch
    # fails and the sweep continues. Pose 2: both see it -> SUCCESS there.
    script = [Status.SUCCESS, Status.FAILURE, Status.SUCCESS, Status.SUCCESS]
    calls = []
    counter = {"n": 0}

    def make_detect():
        seq = small_trees._person_scan_and_confirm()
        for i, child in enumerate(list(seq.children)):
            seq.remove_child(child)
        counter["n"] += 1
        seq.add_child(_ScriptedDetect(f"scan@{counter['n']}", script, calls))
        seq.add_child(_ScriptedDetect(f"confirm@{counter['n']}", script, calls))
        return seq

    sweep = small_trees._pantilt_sweep("find_person", [45.0], make_detect, pan_deg=[0.0, 45.0, -45.0])
    for branch in sweep.children:
        turn = branch.children[0]
        branch.remove_child(turn)
        branch.insert_child(py_trees.behaviours.Success(name=turn.name), 0)
    py_trees.trees.BehaviourTree(sweep).setup()
    assert _tick_done(sweep) is Status.SUCCESS
    assert calls == [
        ("scan@1", Status.SUCCESS), ("confirm@1", Status.FAILURE),
        ("scan@2", Status.SUCCESS), ("confirm@2", Status.SUCCESS),
    ]
    assert sweep.current_child.name == "find_person pan=+45 tilt=+45"


def test_person_sweep_fails_when_no_pose_confirms():
    script = [Status.SUCCESS, Status.FAILURE] * 3
    calls = []

    def make_detect():
        seq = small_trees._person_scan_and_confirm()
        for child in list(seq.children):
            seq.remove_child(child)
        seq.add_child(_ScriptedDetect("scan", script, calls))
        seq.add_child(_ScriptedDetect("confirm", script, calls))
        return seq

    sweep = small_trees._pantilt_sweep("find_person", [45.0], make_detect)
    for branch in sweep.children:
        turn = branch.children[0]
        branch.remove_child(turn)
        branch.insert_child(py_trees.behaviours.Success(name=turn.name), 0)
    py_trees.trees.BehaviourTree(sweep).setup()
    assert _tick_done(sweep) is Status.FAILURE
    assert len(calls) == 6


# ---------------------------------------------------------------------------
# Regression (sim battery 2026-09-27 fix4 runs 001/002): materialising a count
# step through the REAL executor node on a REAL blackboard must not raise --
# the orchestrator process died with "client 'materialise:0:0:1' does not
# have write access to '/gpsr/count_member_prompts'" because the new count
# keys were written by materialise_params without being registered on the
# materialising nodes. Mock-client tests cannot catch this.
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("node_cls", ["BtNode_MaterialiseStep", "BtNode_PopNextAction"])
def test_materialising_a_count_step_on_a_real_blackboard_registers_every_key(rcw_knowledge, node_cls):
    from behavior_tree.GPSR import orchestrator as orch
    plan_key = "gpsr/saved_target_plan_0_0"
    seed = _writer(plan_key, bb_keys.PLAN, bb_keys.PLAN_INDEX)
    plan = [{"action": "count", "params": {"object": "kitchen items", "location": "side_table"}}]
    seed.set(plan_key, plan)
    seed.set(bb_keys.PLAN, plan)
    seed.set(bb_keys.PLAN_INDEX, 0)
    if node_cls == "BtNode_MaterialiseStep":
        node = orch.BtNode_MaterialiseStep("materialise:0:0:1", plan_key, 0)
    else:
        node = orch.BtNode_PopNextAction("pop next action")
    node.setup()
    node.tick_once()  # raises AttributeError before the fix
    reader = _writer(bb_keys.COUNT_MEMBER_PROMPTS, bb_keys.COUNT_VLM_SUBJECT, bb_keys.COUNT_RECOUNTED,
                     bb_keys.TARGET_OBJECT_PROMPT, name="reader")
    assert reader.get(bb_keys.COUNT_RECOUNTED) is False
    assert reader.get(bb_keys.COUNT_MEMBER_PROMPTS)
    assert " . " in reader.get(bb_keys.TARGET_OBJECT_PROMPT)


# ---------------------------------------------------------------------------
# fix4 run 3 follow-ups: the sim referee hand-off must name a concrete known
# object (the sim nacked the category "food"), and a room is "in", not "on".
# ---------------------------------------------------------------------------
def test_object_key_for_label_resolves_prompts_names_and_categories(rcw_knowledge):
    from behavior_tree.GPSR.orchestrator import object_key_for_label, KNOWN_OBJECT_PROMPTS
    soup_prompt = KNOWN_OBJECT_PROMPTS["soup"]
    assert object_key_for_label(soup_prompt) == "soup"
    assert object_key_for_label("soup") == "soup"
    assert object_key_for_label("Sugar Box") == "sugar_box"
    assert object_key_for_label("yellow sugar boxes") == "sugar_box"
    assert object_key_for_label("food") is None
    assert object_key_for_label("") is None


def test_compose_uses_in_for_rooms_and_on_for_furniture(monkeypatch):
    from behavior_tree.GPSR import orchestrator as orch
    from behavior_tree.GPSR.small_trees import BtNode_ComposeRefereeRequest as C
    monkeypatch.setattr(orch, "ROOM_SEARCH_SPOTS", {"bedroom": ["side_table_02"]})
    assert C.compose("food", "bedroom", "red tomato soup can") == (
        "Dear referee, I can see the red tomato soup can in the bedroom. Please put the food in my gripper.")
    assert C.compose("soup", "shelf", "") == (
        "Dear referee, I cannot see the soup on the shelf. Please hand me the soup and put it in my gripper.")


def test_compose_node_writes_handoff_entity_from_seen_label(rcw_knowledge):
    from behavior_tree.GPSR.small_trees import BtNode_ComposeRefereeRequest
    from behavior_tree.GPSR.orchestrator import KNOWN_OBJECT_PROMPTS

    class _Obj:
        cls = KNOWN_OBJECT_PROMPTS["soup"]

    class _Det:
        objects = [_Obj()]

    seed = _writer(bb_keys.TARGET_OBJECT_DETECTION, bb_keys.TARGET_OBJECT_NAME, bb_keys.TARGET_LOCATION)
    seed.set(bb_keys.TARGET_OBJECT_DETECTION, _Det())
    seed.set(bb_keys.TARGET_OBJECT_NAME, "food")
    seed.set(bb_keys.TARGET_LOCATION, "side_table_02")
    node = BtNode_ComposeRefereeRequest("compose referee request", bb_dst=bb_keys.REFEREE_REQUEST_TEXT)
    node.setup()
    assert node.tick_once() is None or True
    reader = _writer(bb_keys.REFEREE_HANDOFF_ENTITY, bb_keys.REFEREE_REQUEST_TEXT, name="reader")
    assert reader.get(bb_keys.REFEREE_HANDOFF_ENTITY) == "soup"
    assert "I can see the" in reader.get(bb_keys.REFEREE_REQUEST_TEXT)


def test_compose_node_sends_category_members_when_the_label_is_unusable(rcw_knowledge):
    # fix4b run 004: the detector echoed the whole multi-class prompt as the
    # label; guessing one member was nacked -> send every member as candidates.
    from behavior_tree.GPSR.small_trees import BtNode_ComposeRefereeRequest
    from behavior_tree.GPSR.orchestrator import category_members

    class _Obj:
        cls = "yellow banana . yellow sugar box . red tomato soup can"

    class _Det:
        objects = [_Obj()]

    seed = _writer(bb_keys.TARGET_OBJECT_DETECTION, bb_keys.TARGET_OBJECT_NAME, bb_keys.TARGET_LOCATION)
    seed.set(bb_keys.TARGET_OBJECT_DETECTION, _Det())
    seed.set(bb_keys.TARGET_OBJECT_NAME, "food")
    seed.set(bb_keys.TARGET_LOCATION, "kitchen_table")
    node = BtNode_ComposeRefereeRequest("compose referee request", bb_dst=bb_keys.REFEREE_REQUEST_TEXT)
    node.setup()
    node.tick_once()
    reader = _writer(bb_keys.REFEREE_HANDOFF_ENTITY, name="reader")
    assert reader.get(bb_keys.REFEREE_HANDOFF_ENTITY) == "|".join(category_members("food"))


# ---------------------------------------------------------------------------
# fix4b run 002: one spawned person, two overlapping 'persons pointing to the
# right' boxes -> "I counted 2". Duplicates of one thing count once.
# ---------------------------------------------------------------------------
class _Pt:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _Box:
    def __init__(self, cls, box, centroid=(0.0, 0.0, 0.0)):
        self.cls = cls
        self.xmin, self.ymin, self.xmax, self.ymax = box
        self.centroid = _Pt(*centroid)


def test_count_merges_overlapping_boxes_of_one_person():
    from behavior_tree.GPSR.small_trees import BtNode_CountDetections as C
    objs = [_Box("persons pointing to the right", (100, 50, 300, 500)),
            _Box("persons pointing to the right", (110, 60, 310, 505))]
    assert C.count_members(objs, []) == 1


def test_count_merges_by_3d_centroid_and_keeps_distinct_ones():
    from behavior_tree.GPSR.small_trees import BtNode_CountDetections as C
    objs = [_Box("red bowl", (0, 0, 50, 50), (1.0, 0.2, 0.6)),
            _Box("red bowl", (400, 0, 450, 50), (1.05, 0.25, 0.6)),   # same bowl, other box
            _Box("red bowl", (200, 0, 250, 50), (1.6, -0.3, 0.6))]    # a second bowl
    assert C.count_members(objs, []) == 2


def test_count_keeps_non_overlapping_boxes_without_centroids():
    from behavior_tree.GPSR.small_trees import BtNode_CountDetections as C
    objs = [_Box("mug", (0, 0, 50, 50)), _Box("mug", (100, 100, 150, 150)), _Box("mug", (300, 0, 350, 50))]
    assert C.count_members(objs, []) == 3
