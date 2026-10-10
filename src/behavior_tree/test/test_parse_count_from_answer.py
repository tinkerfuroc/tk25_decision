"""The VLM count fallback must produce the count artifact the gate verifies.

The target postcondition gate validates ``counted(X)`` against the
``count_value`` evidence key — which only the detector path wrote. A battery
run (s2026-002, 2026-08-28) whose count came from the VLM fallback answered
"0 persons" correctly, then failed its postcondition with "counted(...)
(UNKNOWN)" and replanned the whole target — bedroom leg included — in a loop
until the 900 s timeout. BtNode_ParseCountFromAnswer parses the number out of
the VLM answer and stores it at COUNT_VALUE.

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 (ROS pytest plugins break
collection).
"""

from __future__ import annotations

import sys
from pathlib import Path
from unittest import mock

SRC = Path(__file__).resolve().parents[1]
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from behavior_tree.GPSR.custom_nodes import BtNode_ParseCountFromAnswer  # noqa: E402

parse = BtNode_ParseCountFromAnswer.parse_count


def test_parse_leading_digit():
    assert parse("0 persons.") == 0
    assert parse("3 persons are pointing to the left.") == 3
    assert parse("There are 12 apples visible.") == 12


def test_parse_number_words():
    assert parse("Zero persons are pointing left.") == 0
    assert parse("There are two apples on the table.") == 2
    assert parse("No persons are visible.") == 0
    assert parse("None of them are pointing left.") == 0


def test_parse_unparseable_returns_none():
    assert parse("I cannot tell from this image.") is None
    assert parse("") is None
    assert parse(None) is None


def _make_node():
    node = BtNode_ParseCountFromAnswer.__new__(BtNode_ParseCountFromAnswer)
    node._src = "gpsr/vlm_answer"
    node._dst = "gpsr/count_value"
    node._bb = mock.Mock()
    node.feedback_message = ""
    return node


def test_update_stores_parsed_count():
    import py_trees

    node = _make_node()
    node._bb.get.return_value = "0 persons are pointing to the left."

    status = node.update()

    assert status == py_trees.common.Status.SUCCESS
    node._bb.set.assert_called_once_with("gpsr/count_value", 0, overwrite=True)


def test_update_fails_without_a_number():
    import py_trees

    node = _make_node()
    node._bb.get.return_value = "I cannot tell from this image."

    status = node.update()

    assert status == py_trees.common.Status.FAILURE
    node._bb.set.assert_not_called()
