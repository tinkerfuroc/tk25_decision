import os

os.environ["BT_MOCK_MODE"] = "true"  # noqa: E402 — force mock before config loads

import importlib  # noqa: E402


def test_messages_exposes_scan_and_place():
    messages = importlib.import_module("behavior_tree.interfaces.messages")
    assert hasattr(messages, "ScanAndPlace")
    goal = messages.ScanAndPlace.Goal()
    for field in (
        "item_description", "margin_m", "orientation", "max_candidates",
        "placement_mode", "reference_label", "fixed_target",
        "scan_pose_deg", "skip_scan_move", "dry_run",
    ):
        assert hasattr(goal, field), field
    result = messages.ScanAndPlace.Result()
    for field in ("status", "placed_at", "error_msg", "placement_mode_used"):
        assert hasattr(result, field), field
    fb = messages.ScanAndPlace.Feedback()
    assert hasattr(fb, "stage")


def test_mock_messages_stub_present():
    mock = importlib.import_module("behavior_tree.interfaces.mock_messages")
    assert hasattr(mock, "ScanAndPlace")
    g = mock.ScanAndPlace.Goal()
    assert g.placement_mode == 0
    assert g.scan_pose_deg == []
    assert g.skip_scan_move is False and g.dry_run is False
