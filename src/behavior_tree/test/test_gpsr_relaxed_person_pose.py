"""L2 — relaxed person acceptance must materialize the pose it promises.

Sim battery runs 008/011: ``approach_person`` failed "Unsupported type for
gpsr/target_person_pose: NoneType" after the relaxed identity gate accepted
person_found() from a leftover BtNode_ScanForGeneralist detection response
that was never actually extracted into a pose (the strict
``BtNode_ExtractDetection`` never even ran -- its Sequence sibling,
``BtNode_ScanForGeneralist``, already returned FAILURE, which short-circuits
the Sequence before the extract step).

Fix at the scan layer:
- ``BtNode_ExtractDetection`` gains a ``relaxed=True`` mode: instead of
  blindly taking ``objects[0]``, it picks the first detection whose label
  token-matches the generic person-class labels (shared with
  ``validators._SIM_PERSON_CLASS_LABELS``), and (only in relaxed mode)
  writes an optional provenance key. Dead (FAILURE) whenever
  ``GPSR_SIM_IDENTITY_RELAXED`` is not "1".
- ``BtNode_PointToPoseStamped`` gives a clearer FAILURE feedback message
  when the point key is explicitly None, instead of the confusing
  "Unsupported type ...: NoneType".

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 (ROS pytest plugins break
collection).
"""
from __future__ import annotations

from unittest import mock

import pytest


# ---------------------------------------------------------------------------
# BtNode_PointToPoseStamped — clearer None-point diagnostics (L2c)
# ---------------------------------------------------------------------------

def _make_point_to_pose_node():
    from behavior_tree.GPSR.small_trees import BtNode_PointToPoseStamped
    node = BtNode_PointToPoseStamped.__new__(BtNode_PointToPoseStamped)
    node.feedback_message = ""
    node._point_key = "gpsr/target_person_pose"
    node._pose_key = "gpsr/person_nav_pose"
    node._client = mock.Mock()
    return node


def test_none_point_gives_clear_feedback_not_type_error():
    import py_trees as pytree

    node = _make_point_to_pose_node()
    node._client.get.return_value = None

    status = node.update()

    assert status == pytree.common.Status.FAILURE
    assert node.feedback_message == (
        "no gpsr/target_person_pose recorded — the person scan must "
        "succeed first"
    )
    assert "Unsupported type" not in node.feedback_message


def test_missing_key_still_fails_with_original_message():
    import py_trees as pytree

    node = _make_point_to_pose_node()
    node._client.get.side_effect = KeyError("gpsr/target_person_pose")

    status = node.update()

    assert status == pytree.common.Status.FAILURE
    assert "Missing point key" in node.feedback_message


def test_unrelated_bad_type_still_reports_unsupported_type():
    import py_trees as pytree

    node = _make_point_to_pose_node()
    node._client.get.return_value = 42  # neither PoseStamped nor PointStamped nor None

    status = node.update()

    assert status == pytree.common.Status.FAILURE
    assert "Unsupported type" in node.feedback_message


# ---------------------------------------------------------------------------
# BtNode_ExtractDetection relaxed mode (L2a)
# ---------------------------------------------------------------------------

class _FakeItem:
    def __init__(self, cls, x=1.0, y=2.0, z=0.0):
        from geometry_msgs.msg import Point

        self.cls = cls
        self.centroid = Point(x=x, y=y, z=z)


class _FakeResult:
    def __init__(self, objects, header=None):
        self.objects = objects
        self.header = header


def _make_extract_node(relaxed, provenance_dst=None):
    from behavior_tree.GPSR.small_trees import BtNode_ExtractDetection
    node = BtNode_ExtractDetection.__new__(BtNode_ExtractDetection)
    node.feedback_message = ""
    node._src = "gpsr/target_person_detection"
    node._object_dst = "gpsr/target_object"
    node._point_dst = "gpsr/target_person_pose"
    node._target_frame = "map"
    node._relaxed = relaxed
    node._provenance_dst = provenance_dst
    node._client = mock.Mock()
    return node


@pytest.fixture(autouse=True)
def _real_vision(monkeypatch):
    # BtNode_ExtractDetection.update() checks is_subsystem_mocked("vision")
    # first -- force the REAL branch so the relaxed logic under test runs.
    import behavior_tree.config as config

    monkeypatch.setattr(config, "is_subsystem_mocked", lambda name: False)


def test_relaxed_mode_off_by_default_env(monkeypatch):
    import py_trees as pytree

    monkeypatch.delenv("GPSR_SIM_IDENTITY_RELAXED", raising=False)
    node = _make_extract_node(relaxed=True, provenance_dst="gpsr/person_provenance")
    node._client.get.return_value = _FakeResult([_FakeItem("person")])

    status = node.update()

    assert status == pytree.common.Status.FAILURE
    node._client.set.assert_not_called()


def test_relaxed_mode_picks_generic_person_detection(monkeypatch):
    import py_trees as pytree

    monkeypatch.setenv("GPSR_SIM_IDENTITY_RELAXED", "1")
    node = _make_extract_node(relaxed=True, provenance_dst="gpsr/person_provenance")
    picked = _FakeItem("person")
    node._client.get.return_value = _FakeResult([picked])

    status = node.update()

    assert status == pytree.common.Status.SUCCESS
    calls = {c.args[0]: c.args[1] for c in node._client.set.call_args_list}
    assert calls["gpsr/target_object"] is picked
    assert "gpsr/target_person_pose" in calls
    assert calls["gpsr/person_provenance"] == "relaxed_generic"


def test_relaxed_mode_skips_non_person_detections(monkeypatch):
    import py_trees as pytree

    monkeypatch.setenv("GPSR_SIM_IDENTITY_RELAXED", "1")
    node = _make_extract_node(relaxed=True, provenance_dst="gpsr/person_provenance")
    node._client.get.return_value = _FakeResult([_FakeItem("chair"), _FakeItem("table")])

    status = node.update()

    assert status == pytree.common.Status.FAILURE
    node._client.set.assert_not_called()


def test_strict_mode_unaffected_by_relaxed_flag(monkeypatch):
    """relaxed=False (the pre-existing behaviour) must not consult the env
    flag at all -- objects[0] is always picked, matching today's contract."""
    import py_trees as pytree

    monkeypatch.setenv("GPSR_SIM_IDENTITY_RELAXED", "1")
    node = _make_extract_node(relaxed=False)
    picked = _FakeItem("chair")
    node._client.get.return_value = _FakeResult([picked])

    status = node.update()

    assert status == pytree.common.Status.SUCCESS
    calls = {c.args[0]: c.args[1] for c in node._client.set.call_args_list}
    assert calls["gpsr/target_object"] is picked
    assert "gpsr/person_provenance" not in calls
