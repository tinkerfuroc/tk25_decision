"""N2 (round-5 rerun fix, sim run 019) -- generic-person scan branch.

Every existing scan in ``_person_scan_strategies`` queries the descriptor
verbatim; the relaxed_branch re-parse only has something to relax when the
STRICT scan actually returned objects (just none descriptor-matching). Run
019: "Scan returned 0 objects" x5, relaxed_generic never appeared -- a
zero-match scan leaves nothing for the re-parse to work with.

Fix: a fifth branch in the selector, gated on GPSR_SIM_IDENTITY_RELAXED and
placed AFTER the cheap re-parse (relaxed_branch) so a person the strict
query already found costs no extra VLM call -- issues its own fresh,
bare-"person" scan (via a NEW bb key, PERSON_VISION_PROMPT_GENERIC, so it
never clobbers PERSON_VISION_PROMPT) and relax-extracts THAT response.

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 (ROS pytest plugins break
collection).
"""
from __future__ import annotations

from unittest import mock

import py_trees
from py_trees.common import Access, Status
import pytest

from behavior_tree.GPSR import small_trees
from behavior_tree.GPSR.small_trees import bb_keys


class _FakeItem:
    def __init__(self, cls, x=1.0, y=2.0, z=0.0):
        from geometry_msgs.msg import Point

        self.cls = cls
        self.centroid = Point(x=x, y=y, z=z)


class _FakeResult:
    def __init__(self, objects):
        self.objects = objects
        self.header = None


class _ScanStub(py_trees.behaviour.Behaviour):
    """Stand-in for a real ``BtNode_ScanForGeneralist`` (needs a ROS
    service client for setup()): ticks to ``status``, and -- when a
    detection is given -- writes it to ``bb_key`` first, exactly like the
    real node does before it can itself return FAILURE (see
    ``_person_scan_strategies``'s own comment on this)."""

    def __init__(self, name: str, status: Status, bb_key: str, detection=None):
        super().__init__(name)
        self._status = status
        self._bb_key = bb_key
        self._detection = detection
        self._client = None
        self.ticks = 0

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._bb_key, access=Access.WRITE)

    def update(self):
        self.ticks += 1
        self._client.set(self._bb_key, self._detection, overwrite=True)
        return self._status


@pytest.fixture(autouse=True)
def _real_vision(monkeypatch):
    # BtNode_ExtractDetection.update() checks is_subsystem_mocked("vision")
    # first -- force the REAL branch so the relaxed logic under test runs.
    import behavior_tree.config as config

    monkeypatch.setattr(config, "is_subsystem_mocked", lambda name: False)


def _build(*, strict_objects, generic_objects):
    """Real _person_scan_strategies() (no extra_specialist -> selector.children
    == [waving, generalist, relaxed, generic_scan]), scan leaves replaced by
    controllable stubs. ``strict_objects``/``generic_objects`` are the
    detection lists the strict/generic scans "find" (empty list == the scan
    ran but matched nothing, same shape a real zero-match response has)."""
    py_trees.blackboard.Blackboard.clear()
    writer = py_trees.blackboard.Client(name="seed person scan strategies")
    writer.register_key(bb_keys.TARGET_PERSON_PROMPT, access=Access.WRITE)
    writer.set(bb_keys.TARGET_PERSON_PROMPT, "person", overwrite=True)

    selector = small_trees._person_scan_strategies()
    assert len(selector.children) == 4, (
        "expected [waving, generalist, relaxed_generalist, generic_scan]"
    )
    waving_branch, generalist_branch, _relaxed_branch, generic_scan_branch = (
        selector.children
    )

    # The waving guard never matches a bare "person" descriptor -- but
    # setup() still walks the branch, and its real scan leaf needs a ROS
    # server. Stub it out (never actually ticked by any scenario below).
    for child in list(waving_branch.children[1:]):
        waving_branch.remove_child(child)
    waving_branch.add_child(py_trees.behaviours.Failure(name="waving scan (stub)"))

    strict_status = Status.SUCCESS if strict_objects else Status.FAILURE
    strict_stub = _ScanStub(
        "generalist scan (stub)",
        strict_status,
        bb_keys.TARGET_PERSON_DETECTION,
        _FakeResult(list(strict_objects)),
    )
    for child in list(generalist_branch.children):
        generalist_branch.remove_child(child)
    generalist_branch.add_child(strict_stub)

    # Stub ONLY the scan leaf (index 2) -- keep the REAL, un-stubbed
    # BtNode_ExtractDetection(relaxed=True) (index 3) so the test actually
    # exercises the extraction this fix depends on (pose/object/provenance
    # written from whatever the stub leaves on TARGET_PERSON_DETECTION),
    # same as the untouched relaxed_branch is left un-stubbed for the same
    # reason.
    generic_status = Status.SUCCESS if generic_objects else Status.FAILURE
    generic_stub = _ScanStub(
        "generic scan (stub)",
        generic_status,
        bb_keys.TARGET_PERSON_DETECTION,
        _FakeResult(list(generic_objects)),
    )
    scan_leaf = generic_scan_branch.children[2]
    generic_scan_branch.remove_child(scan_leaf)
    generic_scan_branch.insert_child(generic_stub, 2)

    py_trees.trees.BehaviourTree(selector).setup()
    return selector, strict_stub, generic_stub


def _tick_to_completion(selector, max_ticks: int = 6):
    for _ in range(max_ticks):
        selector.tick_once()
        if selector.status != Status.RUNNING:
            break


def test_zero_match_strict_scan_falls_through_to_generic_scan(monkeypatch):
    """flag on + strict scan returns 0 objects + generic scan finds a
    person -> pose written, provenance relaxed_generic, strategy succeeds."""
    monkeypatch.setenv("GPSR_SIM_IDENTITY_RELAXED", "1")
    picked = _FakeItem("person")
    selector, strict_stub, generic_stub = _build(
        strict_objects=(), generic_objects=(picked,)
    )

    _tick_to_completion(selector)

    assert selector.status is Status.SUCCESS
    assert strict_stub.ticks == 1
    assert generic_stub.ticks == 1
    assert py_trees.blackboard.Blackboard.get(bb_keys.TARGET_OBJECT) is picked
    assert py_trees.blackboard.Blackboard.exists(bb_keys.TARGET_PERSON_POSE)
    assert (
        py_trees.blackboard.Blackboard.get(bb_keys.PERSON_PROVENANCE)
        == "relaxed_generic"
    )
    # The generic scan used its OWN prompt key, never touching the
    # descriptor-derived PERSON_VISION_PROMPT other branches re-read.
    assert (
        py_trees.blackboard.Blackboard.get(bb_keys.PERSON_VISION_PROMPT_GENERIC)
        == "person"
    )
    assert not py_trees.blackboard.Blackboard.exists(bb_keys.PERSON_VISION_PROMPT)


def test_flag_off_generic_branch_is_never_ticked(monkeypatch):
    monkeypatch.delenv("GPSR_SIM_IDENTITY_RELAXED", raising=False)
    selector, strict_stub, generic_stub = _build(
        strict_objects=(), generic_objects=(_FakeItem("person"),)
    )

    _tick_to_completion(selector)

    assert selector.status is Status.FAILURE
    assert strict_stub.ticks == 1
    assert generic_stub.ticks == 0, (
        "the generic scan branch must be dead (no VLM call) when the flag "
        "is off"
    )


def test_strict_scan_success_never_reaches_the_generic_scan(monkeypatch):
    """A person already found by the strict query costs no extra VLM
    call -- the generic scan branch must not even be ticked."""
    monkeypatch.setenv("GPSR_SIM_IDENTITY_RELAXED", "1")
    picked = _FakeItem("person")
    selector, strict_stub, generic_stub = _build(
        strict_objects=(picked,), generic_objects=(_FakeItem("person"),)
    )

    _tick_to_completion(selector)

    assert selector.status is Status.SUCCESS
    assert strict_stub.ticks == 1
    assert generic_stub.ticks == 0, (
        "generalist_branch already succeeded -- the generic scan is the "
        "last resort, not tried when unnecessary"
    )
