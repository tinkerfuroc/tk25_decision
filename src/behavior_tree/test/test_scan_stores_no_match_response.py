"""A no-match generalist scan must still store its response.

The vision service attaches ``rgb_image`` to the response regardless of
status when ``return_rgb_image=True`` — but BtNode_ScanForGeneralist used to
discard the response on status != 0. The count subtree's VLM fallback then
died with "no usable rgb_image at gpsr/target_object_detection" and the whole
count action failed (battery run s2026-002, 2026-08-28). The scan node now
stores the response before returning FAILURE so image-consuming fallbacks can
look at the frame the detector saw.

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 (ROS pytest plugins break
collection).
"""

from __future__ import annotations

import sys
import types
from pathlib import Path
from unittest import mock

import pytest

SRC = Path(__file__).resolve().parents[1]
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


class _FakeResult:
    """Shape of ObjectDetectionGeneralist.Response the node touches."""

    def __init__(self, status, error_msg=""):
        self.status = status
        self.objects = []
        self.error_msg = error_msg
        self.detection_source = "vlm_sam"
        self.rgb_image = object()  # frame present even on no-match


class _DoneFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


def _make_scan_node(result):
    from behavior_tree.TemplateNodes.Vision import BtNode_ScanForGeneralist

    node = BtNode_ScanForGeneralist.__new__(BtNode_ScanForGeneralist)
    node.mock_mode = False
    node.response = _DoneFuture(result)
    node.use_orbbec = True
    node.object = "persons pointing to the left"
    node.bb_key = "gpsr/target_object_detection"
    node.bb_write_client = mock.Mock()
    node.feedback_message = ""
    return node


def test_no_match_failure_still_stores_response():
    import py_trees as pytree

    result = _FakeResult(status=1, error_msg='no matches for "persons" via vlm_sam')
    node = _make_scan_node(result)

    status = node.update()

    assert status == pytree.common.Status.FAILURE
    node.bb_write_client.set.assert_called_once_with(
        "gpsr/target_object_detection", result, overwrite=True
    )


def test_success_stores_response_and_succeeds():
    import py_trees as pytree

    result = _FakeResult(status=0)
    node = _make_scan_node(result)

    status = node.update()

    assert status == pytree.common.Status.SUCCESS
    node.bb_write_client.set.assert_called_once_with(
        "gpsr/target_object_detection", result, overwrite=True
    )


class _FakeObject:
    def __init__(self, cls):
        self.cls = cls


def test_success_feedback_reports_count_and_classes():
    """Success feedback must carry the count + class names for the run's
    contact sheets, not just the bb key it stored to (battery run
    s2026-002, 2026-08-28 needed this to tell what a scan actually saw from
    the log alone)."""
    result = _FakeResult(status=0)
    result.objects = [_FakeObject("person"), _FakeObject("chair"), _FakeObject("cup")]
    node = _make_scan_node(result)

    node.update()

    assert "found 3" in node.feedback_message
    assert "person" in node.feedback_message
    assert "chair" in node.feedback_message
    assert "cup" in node.feedback_message
    assert "source=vlm_sam" in node.feedback_message


def test_success_feedback_caps_at_five_classes():
    result = _FakeResult(status=0)
    result.objects = [_FakeObject(f"obj{i}") for i in range(7)]
    node = _make_scan_node(result)

    node.update()

    assert "found 7" in node.feedback_message
    assert "obj0" in node.feedback_message
    assert "obj4" in node.feedback_message
    assert "obj5" not in node.feedback_message
    assert "obj6" not in node.feedback_message
