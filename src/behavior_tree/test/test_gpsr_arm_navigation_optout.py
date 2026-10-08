"""The live navigation benchmark must not block on an unavailable arm."""

from __future__ import annotations

import py_trees

from behavior_tree.GPSR.small_trees import _arm_to_orbbec_look


def test_orbbec_look_skips_arm_action_when_navigation_optout_is_enabled(monkeypatch):
    monkeypatch.setenv("GPSR_ALLOW_NAV_WITHOUT_ARM", "1")

    node = _arm_to_orbbec_look()

    assert isinstance(node, py_trees.behaviours.Success)
    assert node.name == "arm to orbbec look (skipped: GPSR_ALLOW_NAV_WITHOUT_ARM=1)"


def test_orbbec_look_keeps_real_arm_action_when_optout_is_disabled(monkeypatch):
    monkeypatch.delenv("GPSR_ALLOW_NAV_WITHOUT_ARM", raising=False)

    node = _arm_to_orbbec_look()

    assert isinstance(node, py_trees.decorators.FailureIsSuccess)
    assert isinstance(node.decorated, py_trees.decorators.Retry)
    assert node.decorated.decorated.name == "arm to orbbec look"
