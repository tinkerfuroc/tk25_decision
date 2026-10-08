"""A TTS call that never returns must not stall the mission (GPSR sim battery
2026-09-30 fix6 run 0: stuck 25 min on "announce plan 1 (speak)").

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1.
"""
import py_trees
import pytest
from py_trees.common import Status

from behavior_tree.TemplateNodes import Audio
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, announce_timeout_s


class _NeverDone:
    def done(self):
        return False


def _node(monkeypatch, t):
    node = BtNode_Announce.__new__(BtNode_Announce)
    py_trees.behaviour.Behaviour.__init__(node, "announce")
    node.mock_mode = False
    node.announce_msg = "My plan: go to the shelf."
    node.response = _NeverDone()
    node.logger = py_trees.logging.Logger("announce")
    monkeypatch.setattr(Audio.time, "monotonic", lambda: t[0])
    node._announce_started = t[0]
    node._announce_timeout_s = announce_timeout_s(node.announce_msg)
    return node


def test_running_within_budget_then_success_after_it(monkeypatch):
    t = [1000.0]
    node = _node(monkeypatch, t)
    assert node.update() == Status.RUNNING
    t[0] += node._announce_timeout_s + 0.1
    assert node.update() == Status.SUCCESS
    assert "timed out" in node.feedback_message


def test_budget_scales_with_text_and_env_override(monkeypatch):
    monkeypatch.delenv("GPSR_ANNOUNCE_TIMEOUT_S", raising=False)
    assert announce_timeout_s("hi") == pytest.approx(35.0)
    assert announce_timeout_s("x" * 500) == pytest.approx(75.0)
    monkeypatch.setenv("GPSR_ANNOUNCE_TIMEOUT_S", "0")
    assert announce_timeout_s("hi") == float("inf")
