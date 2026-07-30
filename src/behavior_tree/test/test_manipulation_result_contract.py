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


# -- behavioural result-status gating tests --

def _simulate_consumer_check(result):
    """Mirror the defensive ``process_result`` logic used by every
    changed-action consumer (Manipulation.py, FoldClothingAction.py).

    Returns (succeeded, status, stage, error_msg) so tests can verify
    both the binary outcome and the diagnostic fields.
    """
    legacy_success = bool(getattr(result, "success", True))
    status = int(getattr(result, "status", 0 if legacy_success else 9))
    stage = int(getattr(result, "stage", 0))
    error_msg = str(getattr(result, "error_msg", "missing error detail"))
    succeeded = legacy_success and status == 0
    return succeeded, status, stage, error_msg


def test_place_nonzero_result_status_rejected():
    """Place has no ``success`` field, so its success signal is purely
    ``status == 0``.  A non-zero result status must be treated as failure
    even when action-level STATUS_SUCCEEDED.
    """
    r = mock_messages.Place.Result()
    r.status = 9
    r.stage = 2
    r.error_msg = "placement collision"
    succ, st, sg, err = _simulate_consumer_check(r)
    assert not succ, "Place with status=9 must not succeed"
    assert st == 9
    assert sg == 2
    assert "placement collision" in err


def test_cartesian_move_nonzero_result_status_rejected():
    """CartesianMove consumer must reject non-zero result status even
    when ``success=True`` (the legacy flag survives for back-compat but
    is insufficient alone).
    """
    r = mock_messages.CartesianMove.Result()
    r.success = True
    r.status = 4
    r.stage = 1
    r.error_msg = "IK failure"
    succ, st, sg, err = _simulate_consumer_check(r)
    assert not succ, "CartesianMove with status=4 must not succeed despite success=True"
    assert st == 4
    assert sg == 1
    assert "IK failure" in err


def test_fold_nonzero_result_status_rejected():
    """Fold consumer must gate on both ``success`` and ``status == 0``."""
    # Default (success=True, status=0) → succeeded
    r = mock_messages.Fold.Result()
    succ, _, _, _ = _simulate_consumer_check(r)
    assert succ

    # success=False with consistent non-zero status → rejected
    r2 = mock_messages.Fold.Result(success=False)
    succ, st, sg, err = _simulate_consumer_check(r2)
    assert not succ
    assert st != 0


def test_place_zero_status_succeeds():
    """Place.Result with default status=0 and no success field passes."""
    r = mock_messages.Place.Result()
    succ, _, _, _ = _simulate_consumer_check(r)
    assert succ


def test_motion_consumer_diagnostics_include_status_stage_error():
    """Failure diagnostics emitted by the defensive consumer pattern
    must include result-level status, stage, and error_msg.
    """
    for factory in (mock_messages.CartesianMove.Result,
                    mock_messages.JointMove.Result,
                    mock_messages.Fold.Result):
        r = factory()
        r.status = 7
        r.stage = 3
        r.error_msg = "joint limit exceeded"
        _, st, sg, err = _simulate_consumer_check(r)
        assert st == 7, f"{factory.__qualname__} status not picked up"
        assert sg == 3, f"{factory.__qualname__} stage not picked up"
        assert "joint limit exceeded" in err, f"{factory.__qualname__} error not picked up"
