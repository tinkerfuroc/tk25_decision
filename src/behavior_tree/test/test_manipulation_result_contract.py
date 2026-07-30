from pathlib import Path

from behavior_tree import mock_messages

ROOT = Path(__file__).resolve().parents[1] / "behavior_tree/TemplateNodes"


def assert_status_stage_error(result):
    assert result.status == 0
    assert result.stage == 0
    assert result.error_msg == ""


def test_place_mock_matches_real_result_contract():
    result = mock_messages.Place.Result()
    assert_status_stage_error(result)
    assert not hasattr(result, "success")


def test_cartesian_and_joint_mocks_match_real_result_contract():
    for action in (mock_messages.CartesianMove, mock_messages.JointMove):
        result = action.Result()
        assert result.success is True
        assert_status_stage_error(result)


def test_fold_mock_matches_real_result_contract():
    result = mock_messages.Fold.Result()
    assert result.success is True
    assert_status_stage_error(result)


def test_changed_consumers_have_no_stale_success_only_contract():
    source = "\n".join(
        (ROOT / name).read_text(encoding="utf-8")
        for name in ("Manipulation.py", "FoldClothingAction.py")
    )
    assert "Result has ONLY `success`" not in source
    assert "getattr(result, \"status\"" in source
