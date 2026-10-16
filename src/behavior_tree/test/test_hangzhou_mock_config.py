"""Shared mock switches must advance the task without sending robot goals."""
import json
from types import SimpleNamespace
from unittest.mock import Mock

import py_trees
import pytest

from behavior_tree.HomeServiceHangzhou.control import MotionBarrier
from behavior_tree.HomeServiceHangzhou.ros_nodes import (
    HangzhouAnnounce, HangzhouGoto, HangzhouListen, SettledPanTilt, Tk27SavedPose,
)
from behavior_tree.HomeServiceHangzhou.state import PREFIX


@pytest.fixture
def mock_settings(tmp_path, monkeypatch):
    path = tmp_path / "mock.json"
    path.write_text(json.dumps({
        "mock_mode": {"enabled": True, "auto_detect": False, "subsystems": {
            "navigation": {"enabled": True, "nodes": {"BtNode_GotoAction": "KEYPRESS"}},
            "manipulation": {"enabled": True, "nodes": {"BtNode_MoveArmSingle": "KEYPRESS"}},
            "audio_input": {"enabled": False, "nodes": {"BtNode_ListenAction": "KEYPRESS"}},
            "announcement": {"enabled": False, "nodes": {"BtNode_Announce": "KEYPRESS"}},
            "vision": {"enabled": False, "nodes": {"BtNode_TurnPanTilt": "IMMEDIATE"}},
        }},
        "keyboard_control": {"enabled": False},
    }))
    monkeypatch.setenv("BT_MOCK_CONFIG", str(path))
    monkeypatch.delenv("BT_MOCK_MODE", raising=False)
    return path


def test_navigation_mock_does_not_mock_audio_or_head(mock_settings):
    nav = HangzhouGoto("navigate", PREFIX + "nav_goal")
    assert nav.mock_mode and nav.mock_interaction_mode == "immediate"
    assert nav.mock_subsystem == "navigation"
    assert not HangzhouAnnounce(name="say", bb_source=None, message="test").mock_mode
    assert not HangzhouListen(name="listen", bb_dest_key=PREFIX + "heard").mock_mode
    assert not SettledPanTilt(name="head", x=0.0, y=0.0).mock_mode


def test_mock_stow_navigation_and_next_step_finish(mock_settings):
    arm = Tk27SavedPose("arm", PREFIX + "arm_pose_name")
    nav = HangzhouGoto("navigate", PREFIX + "nav_goal")
    for leaf in (arm, nav):
        leaf.node = Mock()
        leaf.send_goal_request = Mock(side_effect=AssertionError("Real goal in mock mode"))
    context = SimpleNamespace(motion_blocked=False, motion_actions=[arm, nav], config={},
                              event=Mock())
    root = py_trees.composites.Sequence("route", memory=True, children=[
        arm, MotionBarrier(context), nav, MotionBarrier(context),
        py_trees.behaviours.Success("Next task step"),
    ])
    for _ in range(12):
        root.tick_once()
        if root.status != py_trees.common.Status.RUNNING:
            break
    assert root.status == py_trees.common.Status.SUCCESS
    for leaf in (arm, nav):
        leaf.send_goal_request.assert_not_called()


def test_explicit_live_override_is_respected(mock_settings, monkeypatch):
    monkeypatch.setenv("BT_MOCK_MODE", "false")
    assert not HangzhouGoto("live", PREFIX + "nav_goal").mock_mode


def test_cli_preserves_mock_environment(mock_settings, monkeypatch):
    import os
    import sys
    from behavior_tree.HomeServiceHangzhou import cli

    monkeypatch.setenv("BT_MOCK_MODE", "true")
    monkeypatch.setattr(sys, "argv", ["hangzhou-home"])
    monkeypatch.setattr(cli, "wait_for_start", lambda: False)
    cli.main()
    assert os.environ["BT_MOCK_MODE"] == "true"
    assert os.environ["BT_MOCK_CONFIG"] == str(mock_settings)
