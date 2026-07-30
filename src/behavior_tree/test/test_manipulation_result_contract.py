"""
Focused contract tests for manipulation action result handling.

Tests both mock shape (no-dependency static checks) and the real
``process_result()`` methods (via ``object.__new__`` + mocked heavy deps
in a scoped fixture with deterministic sys.modules teardown).

The static shape tests run at module level with no stubs.  The real-consumer
behavioural tests use the ``stub_env`` fixture which installs lightweight
sys.modules stubs for py_trees/rclpy/action_msgs/geometry_msgs/std_msgs/
nav_msgs/tf2 and reimports the consumer classes fresh.  After the last
consumer test the fixture tears down, restoring every sys.modules entry
to its pre-fixture state and deleting any synthetic entries.
"""
import sys
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import MagicMock

import pytest

# ---- Module-level snapshot (taken before any fixture runs) -----------------
_MODULE_SNAPSHOT = dict(sys.modules)

# ---- Module-level imports (no stubs needed) --------------------------------

from behavior_tree import mock_messages  # noqa: E402

ROOT = Path(__file__).resolve().parents[1] / "behavior_tree/TemplateNodes"


# ---- Helper utilities (no stubs needed) -------------------------------------

class FakeResultMessage:
    """Minimal stand-in for the action-framework result message wrapper."""
    def __init__(self, result):
        self.result = result


def _node_bypass(cls):
    """Build an instance via ``object.__new__``, bypassing the heavy
    ``__init__`` (py_trees.Behaviour + ROS2 client setup), and set only
    the attributes that ``process_result()`` reads or writes."""
    obj = object.__new__(cls)
    obj.result_status = 0
    obj.result_message = FakeResultMessage(None)
    obj.feedback_message = ""
    obj.logger = MagicMock()
    obj.angle = 0.0  # BtNode_PointTo's failure branch uses self.angle
    return obj


# ============================================================================
#  Contract tests: mock shape (no stubs needed)
# ============================================================================

def test_place_mock_matches_real_result_contract():
    result = mock_messages.Place.Result()
    assert result.status == 0
    assert result.stage == 0
    assert result.error_msg == ""
    assert not hasattr(result, "success")


def test_cartesian_and_joint_mocks_match_real_result_contract():
    for action in (mock_messages.CartesianMove, mock_messages.JointMove):
        result = action.Result()
        assert result.success is True
        assert result.status == 0
        assert result.stage == 0
        assert result.error_msg == ""


def test_fold_mock_matches_real_result_contract():
    result = mock_messages.Fold.Result()
    assert result.success is True
    assert result.status == 0
    assert result.stage == 0
    assert result.error_msg == ""


def test_changed_consumers_have_no_stale_success_only_contract():
    source = "\n".join(
        (ROOT / name).read_text(encoding="utf-8")
        for name in ("Manipulation.py", "FoldClothingAction.py")
    )
    assert "Result has ONLY `success`" not in source
    assert 'getattr(result, "status"' in source


# ============================================================================
#  Fixture: scoped sys.modules stubs with deterministic restore
# ============================================================================

@pytest.fixture
def stub_env():
    """Provide real consumer classes with cleanup around every test."""
    with _stubbed_consumer_env() as env:
        yield env


@contextmanager
def _stubbed_consumer_env():
    """Install consumer import stubs and always restore ``sys.modules``.

    The snapshot is restored if setup, import, test use, or teardown raises.
    Function-scoped fixture users and standalone regression cycles therefore
    share exactly one cleanup path and cannot leak synthetic modules.
    """
    snapshot = dict(sys.modules)
    try:
        # Modules cached from a neighbouring test may hold real dependencies
        # that conflict with this cycle's stubs.
        for key in list(sys.modules):
            if key.startswith("behavior_tree."):
                del sys.modules[key]

        sentinels = _install_stubs()

        # Short-circuit the Navigation -> tf2_ros import chain.
        sys.modules["behavior_tree.PickAndPlace"] = MagicMock()
        sys.modules["behavior_tree.PickAndPlace.config"] = MagicMock()

        from behavior_tree.TemplateNodes.Manipulation import (
            BtNode_CartesianMove,
            BtNode_JointMoveAction,
            BtNode_MoveArm,
            BtNode_MoveArmSingle,
            BtNode_Place,
            BtNode_PointTo,
        )
        from behavior_tree.TemplateNodes.FoldClothingAction import (
            BtNode_FoldClothingAction,
        )

        act_gs = sentinels["act_gs"]
        env = {
            "SUCCESS": sentinels["SUCCESS"],
            "FAILURE": sentinels["FAILURE"],
            "SUCCEEDED_VALUE": act_gs.STATUS_SUCCEEDED,
            "ABORTED_VALUE": act_gs.STATUS_ABORTED,
            "BtNode_Place": BtNode_Place,
            "BtNode_CartesianMove": BtNode_CartesianMove,
            "BtNode_JointMoveAction": BtNode_JointMoveAction,
            "BtNode_FoldClothingAction": BtNode_FoldClothingAction,
            "BtNode_MoveArm": BtNode_MoveArm,
            "BtNode_MoveArmSingle": BtNode_MoveArmSingle,
            "BtNode_PointTo": BtNode_PointTo,
        }
        yield env
    finally:
        _restore_sys_modules(snapshot)


def _install_stubs():
    """Insert sys.modules stubs for all heavyweight ROS2 / py_trees deps.

    Returns a dict of sentinel objects that consumer imports resolve to.
    Uses real ``types.ModuleType`` for packages (so ``import X.Y as Z``
    resolves sub-modules correctly) and pinned ``MagicMock`` instances for
    attribute-having singletons (``GoalStatus``, ``py_trees.common.Status``).

    Sentinels are plain ``object()`` instances so ``is`` comparisons work
    in tests.
    """
    import types as _types

    SUCCESS = object()
    FAILURE = object()

    # -- py_trees tree -------------------------------------------------------
    _pyt = MagicMock()
    _pyt_common = MagicMock()
    _pyt_common.Status = type("Status", (), {"SUCCESS": SUCCESS, "FAILURE": FAILURE})
    _pyt.common = _pyt_common
    _pyt_behaviour = MagicMock()
    _pyt_behaviour.Behaviour = object
    _pyt.behaviour = _pyt_behaviour
    _pyt_blackboard = MagicMock()
    _pyt.blackboard = _pyt_blackboard
    _pyt_blackboard.Blackboard.absolute_name = staticmethod(lambda s, k: k)

    sys.modules["py_trees"] = _pyt
    sys.modules["py_trees.common"] = _pyt_common
    sys.modules["py_trees.behaviour"] = _pyt_behaviour
    sys.modules["py_trees.blackboard"] = _pyt_blackboard

    # -- rclpy tree ----------------------------------------------------------
    _rclpy = MagicMock()
    _rclpy.__path__ = []
    sys.modules["rclpy"] = _rclpy
    sys.modules["rclpy.node"] = MagicMock()
    sys.modules["rclpy.action"] = MagicMock()
    sys.modules["rclpy.impl"] = MagicMock()
    sys.modules["rclpy.impl.implementation_singleton"] = MagicMock()

    # -- py_trees_ros --------------------------------------------------------
    _pyt_ros = MagicMock()
    sys.modules["py_trees_ros"] = _pyt_ros
    sys.modules["py_trees_ros.exceptions"] = MagicMock()

    # -- action_msgs ---------------------------------------------------------
    _act_gs = MagicMock()
    _act_gs.STATUS_SUCCEEDED = 4
    _act_gs.STATUS_ABORTED = 6
    _act_msg_pkg = _types.ModuleType("action_msgs")
    _act_msg_pkg.__path__ = []
    _act_msg_pkg.__package__ = "action_msgs"
    _act_sub = _types.ModuleType("action_msgs.msg")
    _act_sub.__package__ = "action_msgs.msg"
    _act_sub.GoalStatus = _act_gs
    sys.modules["action_msgs"] = _act_msg_pkg
    sys.modules["action_msgs.msg"] = _act_sub

    # -- geometry_msgs -------------------------------------------------------
    _geo_msg = MagicMock()
    _geo_msg.PointStamped = type("PointStamped", (), {})
    _geo_msg.Pose = type("Pose", (), {})
    _geo_msg.Point = type("Point", (), {})
    _geo_msg.PoseStamped = type("PoseStamped", (), {})
    _geo_msg.Quaternion = type("Quaternion", (), {})
    sys.modules["geometry_msgs"] = MagicMock()
    sys.modules["geometry_msgs.msg"] = _geo_msg

    # -- std_msgs ------------------------------------------------------------
    _std_msg = MagicMock()
    _std_msg.Header = type("Header", (), {})
    sys.modules["std_msgs"] = MagicMock()
    sys.modules["std_msgs.msg"] = _std_msg

    # -- nav_msgs ------------------------------------------------------------
    sys.modules["nav_msgs"] = MagicMock()
    _nav_msg = MagicMock()
    _nav_msg.Odometry = type("Odometry", (), {})
    sys.modules["nav_msgs.msg"] = _nav_msg

    # -- tf2 -----------------------------------------------------------------
    sys.modules["tf2_geometry_msgs"] = MagicMock()
    sys.modules["tf2_ros"] = MagicMock()
    sys.modules["tf2_ros.buffer"] = MagicMock()
    sys.modules["tf2_ros.transform_listener"] = MagicMock()

    return {"SUCCESS": SUCCESS, "FAILURE": FAILURE, "act_gs": _act_gs}


def _restore_sys_modules(snapshot):
    """Restore sys.modules to *snapshot*: delete every current key not in
    snapshot, then overlay snapshot entries.
    """
    for key in list(sys.modules):
        if key not in snapshot:
            del sys.modules[key]
    for key, mod in snapshot.items():
        sys.modules[key] = mod


# ============================================================================
#  Behavioural tests: real process_result() on real consumer classes
# ============================================================================

def _run_node_process_result(env, cls, *, result, action_status):
    """Construct a raw consumer instance, wire in *action_status* and
    *result*, and return (return_value, feedback_message)."""
    node = _node_bypass(cls)
    node.result_status = action_status
    node.result_message = FakeResultMessage(result)
    status = node.process_result()
    return status, node.feedback_message


# -- Place -------------------------------------------------------------------

def test_place_action_success_zero_status_returns_success(stub_env):
    """Place: action STATUS_SUCCEEDED + result.status == 0 → SUCCESS."""
    r = mock_messages.Place.Result()
    r.status = 0
    ret, _ = _run_node_process_result(
        stub_env, stub_env["BtNode_Place"],
        result=r, action_status=stub_env["SUCCEEDED_VALUE"],
    )
    assert ret is stub_env["SUCCESS"]


def test_place_action_success_nonzero_status_returns_failure(stub_env):
    """Place: action STATUS_SUCCEEDED + result.status != 0 → FAILURE."""
    r = mock_messages.Place.Result()
    r.status = 8
    r.stage = 2
    r.error_msg = "placement collision"
    ret, fb = _run_node_process_result(
        stub_env, stub_env["BtNode_Place"],
        result=r, action_status=stub_env["SUCCEEDED_VALUE"],
    )
    assert ret is stub_env["FAILURE"]
    assert "status=8" in fb
    assert "stage=2" in fb
    assert "placement collision" in fb


# -- CartesianMove -----------------------------------------------------------

def test_cartesian_action_success_zero_status_returns_success(stub_env):
    """CartesianMove: action STATUS_SUCCEEDED + result.status == 0 → SUCCESS."""
    r = mock_messages.CartesianMove.Result()
    r.status = 0
    ret, _ = _run_node_process_result(
        stub_env, stub_env["BtNode_CartesianMove"],
        result=r, action_status=stub_env["SUCCEEDED_VALUE"],
    )
    assert ret is stub_env["SUCCESS"]


def test_cartesian_action_success_nonzero_status_returns_failure(stub_env):
    """CartesianMove: action SUCCEEDED + result.status != 0 (even with
    legacy ``success=True``) → FAILURE, with diagnostics."""
    r = mock_messages.CartesianMove.Result()
    r.success = True
    r.status = 4
    r.stage = 1
    r.error_msg = "IK failure"
    ret, fb = _run_node_process_result(
        stub_env, stub_env["BtNode_CartesianMove"],
        result=r, action_status=stub_env["SUCCEEDED_VALUE"],
    )
    assert ret is stub_env["FAILURE"]
    assert "status=4" in fb
    assert "stage=1" in fb
    assert "IK failure" in fb


def test_cartesian_action_level_failure_diagnostics(stub_env):
    """CartesianMove: action-level failure branch includes result
    status/stage/error_msg in feedback_message."""
    r = mock_messages.CartesianMove.Result()
    r.status = 5
    r.stage = 2
    r.error_msg = "timeout"
    _, fb = _run_node_process_result(
        stub_env, stub_env["BtNode_CartesianMove"],
        result=r, action_status=stub_env["ABORTED_VALUE"],
    )
    assert "status=5" in fb
    assert "stage=2" in fb
    assert "timeout" in fb


# -- JointMoveAction ---------------------------------------------------------

def test_jointmove_action_success_zero_status_returns_success(stub_env):
    """BtNode_JointMoveAction: STATUS_SUCCEEDED + result.status == 0 → SUCCESS."""
    r = mock_messages.JointMove.Result()
    r.status = 0
    ret, _ = _run_node_process_result(
        stub_env, stub_env["BtNode_JointMoveAction"],
        result=r, action_status=stub_env["SUCCEEDED_VALUE"],
    )
    assert ret is stub_env["SUCCESS"]


def test_jointmove_action_success_nonzero_status_returns_failure(stub_env):
    """BtNode_JointMoveAction: SUCCEEDED + result.status != 0 → FAILURE."""
    r = mock_messages.JointMove.Result()
    r.success = True
    r.status = 7
    r.stage = 2
    r.error_msg = "joint limit"
    ret, fb = _run_node_process_result(
        stub_env, stub_env["BtNode_JointMoveAction"],
        result=r, action_status=stub_env["SUCCEEDED_VALUE"],
    )
    assert ret is stub_env["FAILURE"]
    assert "status=7" in fb
    assert "stage=2" in fb
    assert "joint limit" in fb


def test_jointmove_action_level_failure_diagnostics(stub_env):
    """BtNode_JointMoveAction: action-level failure branch includes
    result status/stage/error_msg in feedback_message."""
    r = mock_messages.JointMove.Result()
    r.status = 3
    r.stage = 1
    r.error_msg = "cancelled by server"
    _, fb = _run_node_process_result(
        stub_env, stub_env["BtNode_JointMoveAction"],
        result=r, action_status=stub_env["ABORTED_VALUE"],
    )
    assert "status=3" in fb
    assert "stage=1" in fb
    assert "cancelled by server" in fb


# -- FoldClothingAction ------------------------------------------------------

def test_fold_action_success_zero_status_returns_success(stub_env):
    """FoldClothingAction: STATUS_SUCCEEDED + result.status == 0 → SUCCESS."""
    r = mock_messages.Fold.Result()
    r.status = 0
    ret, _ = _run_node_process_result(
        stub_env, stub_env["BtNode_FoldClothingAction"],
        result=r, action_status=stub_env["SUCCEEDED_VALUE"],
    )
    assert ret is stub_env["SUCCESS"]


def test_fold_action_success_nonzero_status_returns_failure(stub_env):
    """FoldClothingAction: SUCCEEDED + result.status != 0 → FAILURE,
    with diagnostics."""
    r = mock_messages.Fold.Result()
    r.success = True
    r.status = 7
    r.stage = 3
    r.error_msg = "grasp lost"
    r.folds_completed = 1
    r.message = "partial"
    ret, fb = _run_node_process_result(
        stub_env, stub_env["BtNode_FoldClothingAction"],
        result=r, action_status=stub_env["SUCCEEDED_VALUE"],
    )
    assert ret is stub_env["FAILURE"]
    assert "status=7" in fb
    assert "stage=3" in fb
    assert "grasp lost" in fb
    assert "folds_completed=1" in fb


def test_fold_action_level_failure_diagnostics(stub_env):
    """FoldClothingAction: action-level failure branch includes result
    status/stage/error_msg in feedback_message."""
    r = mock_messages.Fold.Result()
    r.status = 2
    r.stage = 1
    r.error_msg = "server timeout"
    _, fb = _run_node_process_result(
        stub_env, stub_env["BtNode_FoldClothingAction"],
        result=r, action_status=stub_env["ABORTED_VALUE"],
    )
    assert "status=2" in fb
    assert "stage=1" in fb
    assert "server timeout" in fb


def test_fold_success_consistent_with_mock_default(stub_env):
    """Fold.Result() default (success=True, status=0) is accepted as
    success; Fold.Result(success=False) (status=9) is rejected."""
    node = _node_bypass(stub_env["BtNode_FoldClothingAction"])
    node.result_status = stub_env["SUCCEEDED_VALUE"]

    r1 = mock_messages.Fold.Result()
    node.result_message = FakeResultMessage(r1)
    assert node.process_result() is stub_env["SUCCESS"]

    r2 = mock_messages.Fold.Result(success=False)
    node.result_message = FakeResultMessage(r2)
    assert node.process_result() is stub_env["FAILURE"]


# -- MoveArm / MoveArmSingle / PointTo (all use JointMove.Result) ------------

@pytest.mark.parametrize("cls_key", ["BtNode_MoveArm", "BtNode_MoveArmSingle",
                                      "BtNode_PointTo"])
def test_jointmove_based_node_gates_on_status(stub_env, cls_key):
    """Every consumer that uses ``JointMove.Result`` must reject nonzero
    result status even when action-level STATUS_SUCCEEDED."""
    cls = stub_env[cls_key]
    r = mock_messages.JointMove.Result()
    r.success = True
    r.status = 6
    r.stage = 2
    r.error_msg = "motor stall"
    ret, fb = _run_node_process_result(
        stub_env, cls,
        result=r, action_status=stub_env["SUCCEEDED_VALUE"],
    )
    assert ret is stub_env["FAILURE"]
    assert "status=6" in fb


# ============================================================================
#  Regression: sys.modules roundtrip proves zero contamination
# ============================================================================

def _assert_sys_modules_restored(snapshot):
    """Assert exact identity restoration and no synthetic-module leaks."""
    mismatched = [
        (key, type(original).__name__, type(sys.modules.get(key)).__name__)
        for key, original in snapshot.items()
        if sys.modules.get(key) is not original
    ]
    assert not mismatched, (
        f"{len(mismatched)} sys.modules entries not restored to original identity: "
        + "; ".join(f"{key}: was {old} now {new}" for key, old, new in mismatched[:10])
    )
    leaked = [key for key in sys.modules if key not in snapshot]
    assert not leaked, (
        f"{len(leaked)} synthetic modules leaked: {sorted(leaked)[:20]}"
    )


def test_sys_modules_roundtrip_restoration():
    """Observe a clean state after each context has exited."""
    snapshot = dict(sys.modules)

    with _stubbed_consumer_env() as env:
        # Verify stubs are active while the context owns them.
        import action_msgs
        import py_trees
        assert sys.modules["py_trees"] is py_trees
        assert sys.modules["action_msgs"] is action_msgs
        assert "action_msgs.msg" in sys.modules
        assert "rclpy.node" in sys.modules
        assert "behavior_tree.PickAndPlace" in sys.modules

        # Exercise a real consumer process_result() under the stubs.
        result = mock_messages.Place.Result()
        result.status = 8
        result.error_msg = "roundtrip check"
        node = _node_bypass(env["BtNode_Place"])
        node.result_status = env["SUCCEEDED_VALUE"]
        node.result_message = FakeResultMessage(result)
        assert node.process_result() is env["FAILURE"]
        assert "roundtrip check" in node.feedback_message

    # This assertion is intentionally outside the context manager.
    _assert_sys_modules_restored(snapshot)

    # A second independent cycle proves the setup is repeatable after cleanup.
    second_snapshot = dict(sys.modules)
    with _stubbed_consumer_env() as env:
        result = mock_messages.Place.Result()
        result.status = 8
        result.error_msg = "order-independence check"
        node = _node_bypass(env["BtNode_Place"])
        node.result_status = env["SUCCEEDED_VALUE"]
        node.result_message = FakeResultMessage(result)
        assert node.process_result() is env["FAILURE"]
        assert "order-independence check" in node.feedback_message
    _assert_sys_modules_restored(second_snapshot)


def test_stub_context_restores_modules_when_use_raises():
    """A failure while using the scoped stubs must still restore sys.modules."""
    snapshot = dict(sys.modules)

    with pytest.raises(RuntimeError, match="intentional consumer failure"):
        with _stubbed_consumer_env():
            raise RuntimeError("intentional consumer failure")

    mismatched = [
        key for key, original in snapshot.items()
        if sys.modules.get(key) is not original
    ]
    leaked = [key for key in sys.modules if key not in snapshot]
    assert not mismatched
    assert not leaked


def test_stub_context_restores_modules_when_setup_raises(monkeypatch):
    """Setup failures after mutation must also restore sys.modules."""
    snapshot = dict(sys.modules)
    original_install = _install_stubs

    def install_then_raise():
        original_install()
        raise RuntimeError("intentional setup failure")

    monkeypatch.setattr(
        sys.modules[__name__], "_install_stubs", install_then_raise,
    )
    with pytest.raises(RuntimeError, match="intentional setup failure"):
        with _stubbed_consumer_env():
            pass

    mismatched = [
        key for key, original in snapshot.items()
        if sys.modules.get(key) is not original
    ]
    leaked = [key for key in sys.modules if key not in snapshot]
    assert not mismatched
    assert not leaked
