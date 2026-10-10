"""Sim battery 2026-09-30 fix5 (t2-rand-70718-fix5) count / grasp fixes.

- countPrsInRoom: YOLO-World prompted with "persons pointing to the right"
  boxed a pillow, a toy whale and the robot's own arm (conf 0.10/0.06/0.05)
  -> "I counted 3". Now: confidence floor, own-arm self-occlusion filter, and
  a person count detects COCO "person" and asks the VLM about the descriptor
  (final = min(VLM, detector)).
- countObjOnPlcmt: detections labelled with the whole ' . '-joined prompt
  must not silently vanish from a category count.
- goToLoc/grasp: "retry detect on table" re-sent the identical request from
  the identical pose 5x; now each attempt is a different view.

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1.
"""
from __future__ import annotations

from types import SimpleNamespace

import py_trees
import pytest
from geometry_msgs.msg import PoseStamped
from py_trees.common import Access, Status

from behavior_tree.GPSR import small_trees
from behavior_tree.GPSR.small_trees import (
    COUNT_MIN_CONF,
    BtNode_CountCombine,
    BtNode_CountDetections,
    BtNode_CheckDetectionUsable,
    BtNode_YawOffsetPose,
    BtNode_YawTowardDetection,
    bb_keys,
)


def _walk(node):
    yield node
    for child in getattr(node, "children", []) or []:
        yield from _walk(child)
    decorated = getattr(node, "decorated", None)
    if decorated is not None and decorated not in (getattr(node, "children", []) or []):
        yield from _walk(decorated)


def _names(tree):
    return [getattr(n, "name", "") for n in _walk(tree)]


@pytest.fixture(autouse=True)
def _clean_blackboard():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


@pytest.fixture()
def real_vision(monkeypatch):
    from behavior_tree import config as bt_config
    monkeypatch.setattr(bt_config, "is_subsystem_mocked", lambda subsystem: False)


RCW_PROMPTS = {
    "soup": "red tomato soup can", "mug": "red ceramic mug", "banana": "yellow banana",
    "bowl": "red bowl", "bleach": "white bleach cleanser bottle",
}
RCW_CATEGORIES = {"kitchen_item": ["bowl", "mug", "bleach"], "food": ["banana", "soup"]}


@pytest.fixture()
def rcw_knowledge(monkeypatch):
    from behavior_tree.GPSR import orchestrator as orch
    monkeypatch.setattr(orch, "KNOWN_OBJECT_PROMPTS", dict(RCW_PROMPTS))
    monkeypatch.setattr(orch, "KNOWN_OBJECT_NAMES", set(RCW_PROMPTS))
    monkeypatch.setattr(orch, "KNOWN_CATEGORIES", {k: list(v) for k, v in RCW_CATEGORIES.items()})


def _writer(*keys, name="seed"):
    client = py_trees.blackboard.Client(name=name)
    for key in keys:
        client.register_key(key, access=Access.WRITE)
        client.register_key(key, access=Access.READ)
    return client


class _Obj:
    def __init__(self, cls, conf=None, box=None):
        self.cls = cls
        if conf is not None:
            self.conf = conf
        if box is not None:
            self.xmin, self.ymin, self.xmax, self.ymax = box


def _count(objects, members=None, width=640, height=480, segments=None):
    bb = _writer(bb_keys.TARGET_OBJECT_DETECTION, bb_keys.COUNT_MEMBER_PROMPTS,
                 bb_keys.COUNT_VALUE, bb_keys.COUNT_VIEW_UNCLASSIFIED)
    result = SimpleNamespace(
        objects=objects, segments=segments or [],
        rgb_image=SimpleNamespace(width=width, height=height), status=0,
    )
    bb.set(bb_keys.TARGET_OBJECT_DETECTION, result, overwrite=True)
    bb.set(bb_keys.COUNT_MEMBER_PROMPTS, members, overwrite=True)
    node = BtNode_CountDetections(
        "count", bb_detection_src=bb_keys.TARGET_OBJECT_DETECTION,
        bb_count_dst=bb_keys.COUNT_VALUE,
        bb_member_prompts_key=bb_keys.COUNT_MEMBER_PROMPTS,
        bb_unclassified_dst=bb_keys.COUNT_VIEW_UNCLASSIFIED,
    )
    node.setup()
    status = node.update()
    return status, bb.get(bb_keys.COUNT_VALUE), bb.get(bb_keys.COUNT_VIEW_UNCLASSIFIED), node


# ---------------------------------------------------------------------------
# (a) confidence floor
# ---------------------------------------------------------------------------

def test_count_min_conf_is_a_quarter():
    assert COUNT_MIN_CONF == 0.25


def test_count_drops_low_confidence_yolo_world_boxes(real_vision):
    # fix5 overlay 095858: pillow 0.10, toy whale 0.06, own arm 0.05.
    objs = [_Obj("person", 0.10, (0, 0, 50, 50)), _Obj("person", 0.06, (100, 0, 150, 50)),
            _Obj("person", 0.05, (200, 0, 250, 50)), _Obj("person", 0.88, (300, 0, 350, 50))]
    status, count, _, node = _count(objs)
    assert status is Status.SUCCESS and count == 1
    assert "dropped 3 below conf" in node.feedback_message


def test_count_keeps_vlm_and_unscored_detections(real_vision):
    objs = [_Obj("red bowl", 1.0, (0, 0, 50, 50)),          # VLM path reports 1.0
            _Obj("red bowl", None, (100, 0, 150, 50)),      # no conf field at all
            _Obj("red bowl", 0.0, (200, 0, 250, 50)),       # unset msg default
            _Obj("red bowl", 0.25, (300, 0, 350, 50))]      # exactly at the floor
    assert _count(objs)[1] == 4


# ---------------------------------------------------------------------------
# (b) self-occlusion (own arm / gripper in the head camera)
# ---------------------------------------------------------------------------

def test_count_drops_wide_person_box_at_the_bottom_of_the_frame(real_vision):
    objs = [_Obj("person", 0.9, (100, 380, 500, 479)),   # 400/640 wide, bottom -> own arm
            _Obj("person", 0.9, (10, 50, 110, 300))]     # a real person
    status, count, _, node = _count(objs)
    assert count == 1
    assert "self-occlusion" in node.feedback_message


@pytest.mark.parametrize("obj", [
    _Obj("person", 0.9, (100, 380, 300, 479)),   # bottom but narrow (200/640)
    _Obj("person", 0.9, (0, 0, 640, 300)),       # wide but not in the bottom quarter
    _Obj("red bowl", 0.9, (100, 380, 500, 479)), # not a person box
])
def test_self_occlusion_needs_person_bottom_and_wide(real_vision, obj):
    assert _count([obj])[1] == 1


def test_self_occlusion_box_comes_from_the_segment_mask(real_vision):
    width, height = 40, 20
    mask = bytearray(width * height)
    for y in range(16, 20):          # bottom rows ...
        for x in range(4, 36):       # ... 32/40 of the width
            mask[y * width + x] = 255
    seg = SimpleNamespace(width=width, height=height, step=width, data=bytes(mask))
    obj = _Obj("person", 0.9)
    assert BtNode_CountDetections._bbox(obj, seg) == (4, 16, 35, 19)
    assert _count([obj], width=width, height=height, segments=[seg])[1] == 0


# ---------------------------------------------------------------------------
# (c) unclassifiable multi-class labels
# ---------------------------------------------------------------------------

MULTI = "red tomato soup can . red ceramic mug . yellow banana . red bowl"


def test_multiclass_label_counts_without_a_member_filter(real_vision):
    objs = [_Obj(MULTI, 0.6, (0, 0, 50, 50)), _Obj(MULTI, 0.6, (100, 0, 150, 50))]
    status, count, unclassified, _ = _count(objs, members=None)
    assert count == 2 and unclassified == 0


def test_multiclass_label_is_unclassified_under_a_member_filter(rcw_knowledge, real_vision):
    objs = [_Obj(MULTI, 0.6, (0, 0, 50, 50)), _Obj(MULTI, 0.6, (100, 0, 150, 50)),
            _Obj("red bowl", 0.6, (200, 0, 250, 50))]
    status, count, unclassified, node = _count(objs, members=["red bowl", "red ceramic mug"])
    assert count == 1 and unclassified == 2
    assert "2 unclassified" in node.feedback_message


def test_classify_members_unit(rcw_knowledge):
    C = BtNode_CountDetections
    assert C.classify_members([_Obj(MULTI)], ["red bowl"]) == (0, 1)
    assert C.classify_members([_Obj(MULTI)], []) == (1, 0)
    assert C.count_members([_Obj(MULTI)], ["red bowl"]) == 0
    assert C.is_unclassified_label(MULTI) and not C.is_unclassified_label("kitchen item.bowl")


# ---------------------------------------------------------------------------
# (b) person phrases: prompt materialisation
# ---------------------------------------------------------------------------

def _materialise_count(obj):
    from behavior_tree.GPSR.orchestrator import materialise_params
    bb = _writer(
        bb_keys.TARGET_OBJECT_NAME, bb_keys.TARGET_OBJECT_PROMPT,
        bb_keys.COUNT_MEMBER_PROMPTS, bb_keys.COUNT_VLM_SUBJECT, bb_keys.COUNT_RECOUNTED,
        bb_keys.COUNT_ATTRIBUTE_QUESTION, bb_keys.TARGET_LOCATION, bb_keys.TARGET_POSE,
    )
    materialise_params(bb, "count", {"object": obj})
    return bb


@pytest.mark.parametrize("phrase,descriptor", [
    ("lying persons", "lying horizontally on the floor, with the body resting on the floor"),
    ("persons pointing to the right", "pointing to the right"),
    ("people waving", "waving"),
    ("sitting persons", "sitting"),
    ("person", None),
    ("people", None),
    ("persons wearing a red shirt", "wearing a red shirt"),
    ("women who are standing", "standing"),
])
def test_person_count_detects_person_and_carries_the_descriptor(rcw_knowledge, phrase, descriptor):
    bb = _materialise_count(phrase)
    assert bb.get(bb_keys.TARGET_OBJECT_PROMPT) == "person"
    assert bb.get(bb_keys.COUNT_MEMBER_PROMPTS) is None
    assert bb.get(bb_keys.COUNT_ATTRIBUTE_QUESTION) == descriptor
    # The VLM fallback still gets the whole phrase.
    assert bb.get(bb_keys.COUNT_VLM_SUBJECT) == phrase


def test_lying_person_count_asks_about_lying_only(rcw_knowledge):
    bb = _materialise_count("lying persons")
    assert bb.get(bb_keys.TARGET_OBJECT_PROMPT) == "person"
    descriptor = bb.get(bb_keys.COUNT_ATTRIBUTE_QUESTION)
    assert descriptor == "lying horizontally on the floor, with the body resting on the floor"

    refine = small_trees._count_refinements("count")
    attr = next(n for n in _walk(refine) if n.name == "vlm attribute count")
    assert attr._template.format(value=descriptor) == (
        "How many people in this image match this description: "
        "lying horizontally on the floor, with the body resting on the floor? "
        "Count only people whose visible pose or appearance matches it; "
        "exclude people who do not match. Answer with a number first."
    )


@pytest.mark.parametrize("first,second,expected", [
    (0, 0, 0),       # two independent views agree there is no match
    (0, None, 3),    # one zero could be a crop or VLM miss
    (1, 0, 1),       # one positive view is enough
    (4, 2, 3),       # use the larger view, capped by detector total
    (None, 2, 2),    # a valid confirmation can stand when the first query fails
])
def test_confirmed_attribute_count_uses_two_views_for_zero(first, second, expected):
    from behavior_tree.GPSR.small_trees import BtNode_CountCombine
    assert BtNode_CountCombine.combine(
        3, first, "attribute_confirmed", confirm=second,
    ) == expected


@pytest.mark.parametrize("obj", ["mug", "kitchen items", "food"])
def test_object_counts_have_no_attribute_question(rcw_knowledge, obj):
    bb = _materialise_count(obj)
    assert bb.get(bb_keys.TARGET_OBJECT_PROMPT) != "person"
    assert bb.get(bb_keys.COUNT_ATTRIBUTE_QUESTION) is None


def test_count_person_descriptor_unit(rcw_knowledge):
    from behavior_tree.GPSR.orchestrator import count_person_descriptor
    assert count_person_descriptor("persons pointing to the right") == (True, "pointing to the right")
    assert count_person_descriptor("Person") == (True, None)
    assert count_person_descriptor("red bowl") is None
    assert count_person_descriptor("kitchen items") is None
    assert count_person_descriptor("") is None


# ---------------------------------------------------------------------------
# (b)/(c) count refinements: tree + min/max logic
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("count,vlm,mode,expected", [
    (3, 1, "min", 1), (3, 5, "min", 3), (1, 0, "min", 0),
    (1, 3, "max", 3), (4, 2, "max", 4), (0, 0, "max", 0),
    (1, 0, "attribute", 1), (1, 1, "attribute", 1),
    (2, 3, "attribute", 2),
])
def test_count_combine_logic(count, vlm, mode, expected):
    assert BtNode_CountCombine.combine(count, vlm, mode) == expected


@pytest.mark.parametrize("count,vlm,views,expected", [
    (4, 3, [3, 4, 3], 3),
    (4, 2, [0, 4, 0], 4),
    (5, 6, [3, 5, 5], 6),
    (4, 3, [3, 4, 3, 4], 3),
    # A one-view detector spike must not beat an independent VLM answer
    # that agrees with another view.  This is the live countObjOnPlcmt
    # failure shape: views [3, 4, 2], VLM=3, scene truth=3.
    (4, 3, [3, 4, 2], 3),
])
def test_category_count_uses_repeated_view_consensus(count, vlm, views, expected):
    assert BtNode_CountCombine.combine(
        count, vlm, "consensus", view_counts=views,
    ) == expected


def test_count_combine_rejects_unknown_mode():
    with pytest.raises(ValueError):
        BtNode_CountCombine("x", bb_keys.COUNT_VALUE, bb_keys.COUNT_VLM_VALUE, mode="avg")


def test_category_refinement_fuses_vlm_with_repeated_view_consensus():
    refine = small_trees._count_refinements("count")
    combine = next(n for n in _walk(refine)
                   if n.name == "count = consensus(members, category vlm)")
    assert combine._mode == "consensus"
    assert combine._views_key == bb_keys.COUNT_VIEW_COUNTS


def test_count_deduplicates_overlapping_mask_boxes_without_centroids():
    def mask(box):
        width = height = 20
        x0, y0, x1, y1 = box
        data = bytearray(width * height)
        for y in range(y0, y1):
            for x in range(x0, x1):
                data[y * width + x] = 255
        return SimpleNamespace(width=width, height=height, step=width, data=bytes(data))

    objects = [_Obj("red ceramic mug"), _Obj("red ceramic mug")]
    segments = [mask((2, 2, 12, 12)), mask((3, 3, 13, 13))]
    assert _count(objects, members=[], segments=segments)[1] == 1

    separate = [mask((2, 2, 8, 8)), mask((12, 12, 18, 18))]
    assert _count(objects, members=[], segments=separate)[1] == 2


def test_count_mask_dedupe_keeps_segments_aligned_after_filtering(real_vision):
    def mask(box):
        width = height = 20
        x0, y0, x1, y1 = box
        data = bytearray(width * height)
        for y in range(y0, y1):
            for x in range(x0, x1):
                data[y * width + x] = 255
        return SimpleNamespace(width=width, height=height, step=width, data=bytes(data))

    objects = [
        _Obj("red mug", 0.05),  # removed before dedupe
        _Obj("red mug", 0.8),
        _Obj("red mug", 0.8),
    ]
    # The filtered pair has separate masks. The rejected object's mask overlaps
    # the first retained mask, so using unfiltered segment indices would merge
    # two distinct retained detections.
    segments = [mask((2, 2, 12, 12)), mask((3, 3, 13, 13)), mask((14, 14, 19, 19))]
    assert _count(objects, members=[], segments=segments)[1] == 2

    count_scan = next(
        node for node in _walk(small_trees._count_view("count", 0.0, 20.0))
        if type(node).__name__ == "BtNode_ScanForGeneralist"
    )
    assert count_scan.return_segments is True


def test_count_refinements_follow_both_sweeps():
    tree = small_trees.create_count()
    names = _names(tree)
    for label in ("count", "recount"):
        assert f"{label} refinements" in names
        assert f"{label} unclassified vlm (best effort)" in names
        assert f"{label} attribute vlm (best effort)" in names
    sweep = next(n for n in _walk(tree) if n.name == "count sweep")
    assert sweep.children[-1].name == "count refinements"
    # vlm-template still targets only the fallback query.
    roles = small_trees.compute_small_tree_roles()
    count_vlm = [i for i in roles["count_vlm_branch"] if i.startswith("small/count/")]
    assert len(count_vlm) == 1


def test_attribute_refinement_asks_the_descriptor_on_the_best_frame():
    refine = small_trees._count_refinements("count")
    attr = next(n for n in _walk(refine) if n.name == "vlm attribute count")
    assert attr._src == bb_keys.COUNT_BEST_DETECTION
    assert attr._fill_key == bb_keys.COUNT_ATTRIBUTE_QUESTION
    assert attr._template == (
        "How many people in this image match this description: {value}? "
        "Count only people whose visible pose or appearance matches it; "
        "exclude people who do not match. Answer with a number first."
    )
    combine = next(n for n in _walk(refine) if n.name == "count = confirmed attribute(vlm, detector)")
    assert combine._mode == "attribute_confirmed"
    assert combine._confirm_key == bb_keys.COUNT_ATTR_VLM_VALUE
    unc = next(n for n in _walk(refine) if n.name == "vlm unclassified count")
    assert unc._src == bb_keys.COUNT_UNCLASSIFIED_DETECTION
    assert unc._fill_key == bb_keys.COUNT_VLM_SUBJECT


@pytest.mark.parametrize("response,expected", [
    (None, False),
    (SimpleNamespace(status=1, objects=[_Obj("person")]), False),
    (SimpleNamespace(status=0, objects=[]), False),
    (SimpleNamespace(status=0, objects=[_Obj("person")]), True),
])
def test_detection_attribute_evidence_requires_a_successful_nonempty_scan(response, expected):
    assert BtNode_CheckDetectionUsable.is_usable(response) is expected


def test_whole_body_attribute_relook_guards_failed_scan_before_vlm_query():
    refine = small_trees._count_refinements("count")
    whole = next(n for n in _walk(refine) if n.name == "count whole-body re-look")
    names = [n.name for n in whole.children]
    assert names.index("whole-body frame usable?") < names.index(
        "vlm whole-body attribute count"
    )


class _ScriptedAnswer(py_trees.behaviour.Behaviour):
    def __init__(self, name, answer, calls):
        super().__init__(name)
        self._answer, self._calls = answer, calls

    def setup(self, **kwargs):
        self._c = self.attach_blackboard_client(name=self.name)
        self._c.register_key(bb_keys.VLM_ANSWER, access=Access.WRITE)

    def update(self):
        self._calls.append(self.name)
        if self._answer is None:
            return Status.FAILURE
        self._c.set(bb_keys.VLM_ANSWER, self._answer, overwrite=True)
        return Status.SUCCESS


def _skip_whole_body_relook(tree):
    """Stub the person-attribute whole-body re-look (real pan-tilt + scan)
    as unavailable; its own tests cover it."""
    for node in list(_walk(tree)):
        if node.name.endswith(" whole-body re-look") and getattr(node, "children", None):
            for child in list(node.children):
                node.remove_child(child)
            node.add_child(py_trees.behaviours.Failure("re-look stubbed"))


def _run_refinements(answer, *, count, attribute=None, unclassified=0):
    refine = small_trees._count_refinements("count")
    _skip_whole_body_relook(refine)
    calls = []
    for parent in list(_walk(refine)):
        for child in list(getattr(parent, "children", []) or []):
            if type(child).__name__ == "BtNode_VLMQuery":
                idx = parent.children.index(child)
                parent.remove_child(child)
                parent.insert_child(_ScriptedAnswer(child.name, answer, calls), idx)
    seed = _writer(bb_keys.COUNT_VALUE, bb_keys.COUNT_ATTRIBUTE_QUESTION, bb_keys.COUNT_UNCLASSIFIED,
                   bb_keys.COUNT_VLM_VALUE)
    seed.set(bb_keys.COUNT_VALUE, count)
    seed.set(bb_keys.COUNT_ATTRIBUTE_QUESTION, attribute)
    seed.set(bb_keys.COUNT_UNCLASSIFIED, unclassified)
    tree = py_trees.trees.BehaviourTree(refine)
    tree.setup()
    for _ in range(20):
        refine.tick_once()
        if refine.status != Status.RUNNING:
            break
    assert refine.status is Status.SUCCESS
    return seed.get(bb_keys.COUNT_VALUE), calls


def test_attribute_vlm_lowers_the_count():
    value, calls = _run_refinements("One person is pointing to the right.", count=3,
                                    attribute="pointing to the right")
    assert value == 1 and calls == ["vlm attribute count"]


def test_attribute_vlm_never_exceeds_the_detector():
    value, _ = _run_refinements("5", count=2, attribute="waving")
    assert value == 2


def test_attribute_vlm_skipped_without_descriptor_or_persons():
    assert _run_refinements("1", count=3, attribute=None) == (3, [])
    assert _run_refinements("1", count=0, attribute="waving") == (0, [])


def test_attribute_vlm_failure_keeps_the_detector_count():
    assert _run_refinements(None, count=3, attribute="waving")[0] == 3
    assert _run_refinements("I cannot tell", count=3, attribute="waving")[0] == 3


def test_unclassified_detections_are_counted_by_the_vlm():
    value, calls = _run_refinements("3 kitchen items", count=0, unclassified=2)
    assert value == 3 and calls == ["vlm unclassified count"]
    value, _ = _run_refinements("1", count=2, unclassified=1)
    assert value == 2  # max: never below the classified members


def test_count_keep_unclassified_keeps_the_busiest_frame():
    node = small_trees.BtNode_CountKeepUnclassified(
        "keep", bb_keys.COUNT_VIEW_UNCLASSIFIED, bb_keys.TARGET_OBJECT_DETECTION,
        bb_keys.COUNT_UNCLASSIFIED, bb_keys.COUNT_UNCLASSIFIED_DETECTION)
    node.setup()
    seed = _writer(bb_keys.COUNT_VIEW_UNCLASSIFIED, bb_keys.TARGET_OBJECT_DETECTION,
                   bb_keys.COUNT_UNCLASSIFIED, bb_keys.COUNT_UNCLASSIFIED_DETECTION)
    seed.set(bb_keys.COUNT_UNCLASSIFIED, 0)
    for view, frame in ((1, "a"), (3, "b"), (2, "c"), (0, "d")):
        seed.set(bb_keys.COUNT_VIEW_UNCLASSIFIED, view)
        seed.set(bb_keys.TARGET_OBJECT_DETECTION, frame)
        assert node.update() is Status.SUCCESS
    assert seed.get(bb_keys.COUNT_UNCLASSIFIED) == 3
    assert seed.get(bb_keys.COUNT_UNCLASSIFIED_DETECTION) == "b"


def test_count_sweep_resets_unclassified_state():
    sweep = small_trees._count_sweep("count")
    resets = {c.name: c for c in sweep.children if type(c).__name__ == "BtNode_BlackboardSet"}
    assert resets["reset unclassified"]._key == bb_keys.COUNT_UNCLASSIFIED
    assert resets["reset unclassified frame"]._key == bb_keys.COUNT_UNCLASSIFIED_DETECTION


# ---------------------------------------------------------------------------
# (d) grasp detection re-look
# ---------------------------------------------------------------------------

def _grasp_sweep():
    tree = small_trees.create_grasp()
    return next(n for n in _walk(tree) if n.name == "detect on table (view sweep)"), tree


def test_grasp_no_longer_retries_the_identical_view():
    sweep, tree = _grasp_sweep()
    names = _names(tree)
    assert "retry detect on table" not in names
    views = next(n for n in _walk(sweep) if n.name == "table views")
    assert isinstance(views, py_trees.composites.Selector) and views.memory
    assert [c.name for c in views.children] == [
        "table view 1: wrist at arm scan",
        "table view 2: head camera re-aim",
        "table view 3: base yaw +15",
        "table view 4: base yaw -15",
    ]


def test_grasp_sweep_makes_at_most_five_detection_attempts():
    sweep, _ = _grasp_sweep()
    scans = [n for n in _walk(sweep) if type(n).__name__ == "BtNode_ScanForGeneralist"]
    assert len(scans) <= 5
    wrist = [s for s in scans if not s.use_orbbec]
    head = [s for s in scans if s.use_orbbec]
    assert len(wrist) == 4 and len(head) == 1
    for s in wrist:  # the grasp server reads these from TARGET_OBJECT
        assert s.bb_key == bb_keys.TARGET_OBJECT
        assert s.return_depth_image and s.return_segments and s.return_rgb_image
        assert s.transform_to_map is False
    assert head[0].transform_to_map is True
    assert head[0].bb_key == bb_keys.GRASP_HEAD_DETECTION
    # No Retry decorator repeats a scan in place.
    for n in _walk(sweep):
        if isinstance(n, py_trees.decorators.Retry):
            assert type(n.decorated).__name__ != "BtNode_ScanForGeneralist"


def test_grasp_base_turns_only_with_the_arm_stowed():
    sweep, _ = _grasp_sweep()
    views = next(n for n in _walk(sweep) if n.name == "table views")
    for view in views.children[1:]:
        names = [c.name for c in view.children]
        stow = next(i for i, n in enumerate(names) if "arm to orbbec look" in n)
        turn = next(i for i, n in enumerate(names) if "turn" in n and "best effort" in n)
        scan = next(i for i, n in enumerate(names) if "arm to scan" in n)
        assert stow < turn < scan, names
        assert names[-1].endswith("realsense detect")
    head_view = views.children[1]
    assert any(type(c).__name__ == "BtNode_YawTowardDetection" for c in head_view.children)
    tilt = next(c for c in head_view.children if c.name.startswith("view 2: head tilt"))
    assert tilt.y == small_trees.GRASP_HEAD_TILT_DEG == 45.0


def test_grasp_yaw_views_are_relative_to_the_start_heading_and_restored():
    sweep, _ = _grasp_sweep()
    offsets = [n for n in _walk(sweep) if type(n).__name__ == "BtNode_YawOffsetPose"]
    assert [(o.src_key, o.delta_yaw_deg) for o in offsets] == [
        (bb_keys.GRASP_START_POSE, 15.0), (bb_keys.GRASP_START_POSE, -15.0),
    ]
    names = _names(sweep)
    assert names[:3] == ["detect on table (view sweep)", "reset start heading",
                         "capture start heading (best effort)"]
    assert "restore heading (best effort)" in names
    give_up = next(n for n in _walk(sweep) if n.name == "no view detected the object")
    assert type(give_up.children[-1]).__name__ == "Failure"


def test_grasp_roles_unchanged():
    roles = small_trees.compute_small_tree_roles()
    assert "small/grasp/root/0" in roles["grasp_primary"]
    assert "small/grasp/root/1" in roles["grasp_ex_machina"]


def _pose(x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation = BtNode_YawOffsetPose.quaternion_from_yaw(yaw)
    return p


def test_yaw_toward_detection_math():
    f = BtNode_YawTowardDetection.delta_to_point
    assert f(_pose(0, 0, 0), 1.0, 1.0, 45.0) == pytest.approx(45.0)
    assert f(_pose(0, 0, 30), 1.0, 0.0, 45.0) == pytest.approx(-30.0)
    assert f(_pose(0, 0, 0), -1.0, 0.01, 45.0) == pytest.approx(45.0)   # clamped
    assert f(_pose(0, 0, 0), 1.0, -0.2, 45.0) == pytest.approx(-11.31, abs=0.01)


def test_yaw_toward_detection_node():
    seed = _writer(bb_keys.SURVEY_CAPTURE_POSE, bb_keys.GRASP_HEAD_DETECTION, bb_keys.GRASP_VIEW_GOAL)
    node = BtNode_YawTowardDetection("face", bb_keys.SURVEY_CAPTURE_POSE,
                                     bb_keys.GRASP_HEAD_DETECTION, bb_keys.GRASP_VIEW_GOAL)
    node.setup()
    seed.set(bb_keys.SURVEY_CAPTURE_POSE, _pose(1.0, 2.0, 90.0))
    seed.set(bb_keys.GRASP_HEAD_DETECTION, SimpleNamespace(objects=[]))
    assert node.update() is Status.FAILURE
    seed.set(bb_keys.GRASP_HEAD_DETECTION, SimpleNamespace(
        objects=[SimpleNamespace(centroid=SimpleNamespace(x=1.2, y=3.0, z=0.8))]))
    assert node.update() is Status.SUCCESS
    goal = seed.get(bb_keys.GRASP_VIEW_GOAL)
    assert (goal.pose.position.x, goal.pose.position.y) == (1.0, 2.0)
    yaw = BtNode_YawOffsetPose.yaw_from_quaternion(goal.pose.orientation)
    assert yaw == pytest.approx(78.69, abs=0.05)


# ---------------------------------------------------------------------------
# Sim battery 2026-10-01 fix8 run s70718-001: bowl / bleach / mustard on
# side_table (0.17-0.26 m apart), mustard seen only at 0.11-0.14 -> "I
# counted 1" of 3. Objects merge only when ~co-located and keep >= 0.10.
# ---------------------------------------------------------------------------

def _at(obj, x, y, z=0.612):
    obj.centroid = SimpleNamespace(x=x, y=y, z=z)
    return obj


def test_fix8_side_table_kitchen_items_count_three(real_vision):
    objs = [
        _at(_Obj("red bowl", 0.81, (395, 598, 480, 665)), -0.42, -5.16),
        _at(_Obj("white bleach cleanser bottle", 0.66, (436, 495, 485, 599)), -0.24, -5.16),
        _at(_Obj("red bowl", 0.60, (395, 598, 480, 664)), -0.42, -5.16),          # same bowl
        _at(_Obj("yellow mustard bottle", 0.136, (358, 514, 403, 595)), -0.42, -4.98),
        _at(_Obj("yellow mustard bottle", 0.112, (357, 514, 403, 595)), -0.42, -4.98),  # same
    ]
    members = ["red bowl", "red ceramic mug", "yellow mustard bottle", "white bleach cleanser bottle"]
    status, count, _, _ = _count(objs, members=members, width=1280, height=720)
    assert status is Status.SUCCESS and count == 3


def test_fix8_objects_below_object_floor_still_dropped(real_vision):
    objs = [_Obj("red bowl", 0.095, (0, 0, 50, 50)), _Obj("red bowl", 0.056, (100, 0, 150, 50))]
    assert _count(objs)[1] == 0


def test_fix8_person_boxes_keep_wide_merge_radius_and_quarter_floor(real_vision):
    objs = [_at(_Obj("person", 0.9, (0, 0, 100, 300)), 1.0, 1.0, 0.9),
            _at(_Obj("person", 0.8, (150, 0, 250, 300)), 1.2, 1.0, 0.9),   # same person, 0.2 m
            _Obj("person", 0.12, (400, 0, 450, 50))]                        # pillow-grade box
    assert _count(objs)[1] == 1


# ---------------------------------------------------------------------------
# Sim battery 2026-10-01 fix8 run s70718-000: "goto person" targeted the
# person's own position (inside their footprint) and aborted 3/3. The approach
# now uses go_to_approach, falls back to the raw goto, then asks the person to
# come over instead of failing the hand-over.
# ---------------------------------------------------------------------------

def test_approach_person_uses_go_to_approach_then_falls_back_to_asking():
    from behavior_tree.TemplateNodes.Audio import BtNode_Announce
    from behavior_tree.TemplateNodes.Navigation import BtNode_Approach, BtNode_GotoAction

    tree = small_trees.create_approach_person()
    reach = next(n for n in _walk(tree) if n.name == "reach person")
    assert isinstance(reach, py_trees.composites.Selector)
    first, second, last = reach.children
    assert isinstance(first.decorated, BtNode_Approach)
    assert first.decorated.max_distance == 0.0
    assert isinstance(second, BtNode_GotoAction)
    assert isinstance(last, BtNode_Announce) and "come over" in last.given_msg
    # a missing person pose still fails before any of that
    assert tree.children[0].name == "person point to nav pose"


def test_grasp_head_view_drives_up_to_the_object_before_the_wrist_scan():
    # fix8 run s70718-004: food ~1 m away from the kitchen_table pose, out of
    # the wrist view and reach; the head-camera view now drives up first.
    from behavior_tree.TemplateNodes.Navigation import BtNode_Approach

    tree = small_trees._grasp_table_detection()
    head_view = next(n for n in _walk(tree) if n.name == "table view 2: head camera re-aim")
    names = [c.name for c in head_view.children]
    drive = names.index("view 2: drive up (best effort)")
    assert names.index("orbbec generalist detect") < drive < names.index("view 2: realsense detect")
    approach = next(n for n in _walk(head_view.children[drive]) if isinstance(n, BtNode_Approach))
    assert approach.desired_distance == small_trees.GRASP_APPROACH_DISTANCE_M
    # a failed drive-up never aborts the view
    assert isinstance(head_view.children[drive], py_trees.decorators.FailureIsSuccess)


def test_extract_detection_skips_the_robots_own_arm_person_box(real_vision):
    # fix9 run s70718-000 (yolo 044328): operator box (650,1)-(1014,557) and
    # the robot's own arm (377,555)-(1033,716) labelled "person"; sort_closest
    # put the arm first and the approach drove to the robot's own position.
    from behavior_tree.GPSR.small_trees import BtNode_ExtractDetection

    arm = _Obj("person", 0.83, (377, 555, 1033, 716))
    from geometry_msgs.msg import Point
    arm.centroid = Point(x=-2.16, y=-1.76, z=0.9)
    operator = _Obj("person", 0.75, (650, 1, 1014, 557))
    operator.centroid = Point(x=-1.0, y=-2.0, z=0.9)
    bb = _writer("det", "obj", "pt")
    bb.set("det", SimpleNamespace(objects=[arm, operator], segments=[], header=None,
                                  rgb_image=SimpleNamespace(width=1280, height=720)), overwrite=True)
    node = BtNode_ExtractDetection("extract", bb_detection_src="det", bb_object_dst="obj", bb_point_dst="pt")
    node.setup()
    assert node.update() is Status.SUCCESS
    assert bb.get("obj") is operator and bb.get("pt").point.x == -1.0

    bb.set("det", SimpleNamespace(objects=[arm], segments=[], header=None,
                                  rgb_image=SimpleNamespace(width=1280, height=720)), overwrite=True)
    assert node.update() is Status.FAILURE and "own arm" in node.feedback_message


def test_person_attribute_count_relooks_with_the_head_up_at_the_best_pan():
    # A zero from one cropped view is inconclusive; the second view is kept
    # separately so the reducer can require agreement before reporting zero.
    from behavior_tree.TemplateNodes.Vision import BtNode_TurnPanTilt

    tree = small_trees._count_refinements("persons")
    names = [n.name for n in _walk(tree)]
    assert "persons whole-body re-look (best effort)" in names
    turn = next(n for n in _walk(tree) if n.name == "head up at best pan")
    assert isinstance(turn, BtNode_TurnPanTilt)
    assert turn.x_key == bb_keys.COUNT_BEST_PAN_RAD
    assert names.index("parse attribute count") < names.index("parse whole-body attribute count")
    assert names.index("parse whole-body attribute count") < names.index(
        "count = confirmed attribute(vlm, detector)")


def test_required_pan_tilt_retries_until_feedback_and_fails_if_unsettled(real_vision, monkeypatch):
    from behavior_tree.TemplateNodes import Vision
    from behavior_tree.TemplateNodes.Vision import BtNode_TurnPanTilt

    now = [10.0]
    monkeypatch.setattr(Vision.time, "monotonic", lambda: now[0])

    class _Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    publisher = _Publisher()
    from builtin_interfaces.msg import Time

    clock = SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    ros_node = SimpleNamespace(
        create_publisher=lambda *_args: publisher,
        create_subscription=lambda *_args: object(),
        get_clock=lambda: clock,
    )
    turn = BtNode_TurnPanTilt("required view", y=45.0, require_settled=True)
    turn.setup(node=ros_node)
    turn.initialise()
    assert len(publisher.messages) == 1
    assert turn.update() is Status.RUNNING

    now[0] += turn.RETRY_INTERVAL_SEC + 0.01
    assert turn.update() is Status.RUNNING
    assert len(publisher.messages) == 2

    now[0] = turn._settle_deadline + 0.01
    assert turn.update() is Status.FAILURE
    assert turn.feedback_message == "Pan/tilt feedback unavailable"


def test_required_pan_tilt_succeeds_only_after_target_feedback(real_vision, monkeypatch):
    from behavior_tree.TemplateNodes import Vision
    from behavior_tree.TemplateNodes.Vision import BtNode_TurnPanTilt

    now = [10.0]
    monkeypatch.setattr(Vision.time, "monotonic", lambda: now[0])
    turn = BtNode_TurnPanTilt("required view", y=45.0, require_settled=True)
    turn.mock_mode = False
    turn._skip_settle = False
    turn._arm_settle(0.0, 45.0)
    turn._last_publish_time = now[0]
    turn._state_cb(SimpleNamespace(feedback_ok=True, pan_rad=0.0, tilt_rad=0.7853981633974483))
    assert turn.update() is Status.SUCCESS


def test_keep_max_records_the_pan_of_the_best_view():
    import math as _m
    from behavior_tree.GPSR.small_trees import BtNode_CountKeepMax

    bb = _writer("view", "det", "count", "best", "done", "pan")
    bb.set("count", 0, overwrite=True)
    bb.set("best", None, overwrite=True)
    for pan, n in ((0.0, 0), (90.0, 1), (-90.0, 1)):
        bb.set("view", n, overwrite=True)
        bb.set("det", SimpleNamespace(objects=[object()] * n), overwrite=True)
        node = BtNode_CountKeepMax("keep", "view", "det", "count", "best", "done",
                                   view_pan_deg=pan, bb_best_pan_dst="pan")
        node.setup()
        node.update()
    assert bb.get("count") == 1 and bb.get("pan") == pytest.approx(_m.radians(90.0))
