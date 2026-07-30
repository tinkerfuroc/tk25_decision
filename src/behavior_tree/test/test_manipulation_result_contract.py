"""
Focused contract tests for manipulation action result handling.

Tests both mock shape (no-dependency static checks) and the real
``process_result()`` methods (via ``object.__new__`` + mocked heavy deps).

Must be at the top to install sys.modules mocks before any
behavior_tree module triggers real ROS2/py_trees imports.
"""
import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

# ---- module-level mock setup -----------------------------------------------
# These mocks are installed in sys.modules BEFORE importing Manipulation.py
# and FoldClothingAction.py so that the module-level py_trees / rclpy /
# action_msgs / geometry_msgs / std_msgs imports resolve to lightweight
# stubs rather than hitting the real (unavailable-in-PYTHONPATH) packages.

_PYT_SUCCESS = object()
_PYT_FAILURE = object()

_pyt = MagicMock()
_pyt_common = MagicMock()
_pyt_common.Status = type("Status", (), {"SUCCESS": _PYT_SUCCESS, "FAILURE": _PYT_FAILURE})
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

# rclpy tree
_rclpy = MagicMock()
_rclpy.__path__ = []
sys.modules["rclpy"] = _rclpy
sys.modules["rclpy.node"] = MagicMock()
sys.modules["rclpy.action"] = MagicMock()
sys.modules["rclpy.impl"] = MagicMock()
sys.modules["rclpy.impl.implementation_singleton"] = MagicMock()

# py_trees_ros
_pyt_ros = MagicMock()
sys.modules["py_trees_ros"] = _pyt_ros
sys.modules["py_trees_ros.exceptions"] = MagicMock()

# action_msgs — must have GoalStatus.STATUS_SUCCEEDED = 4 for the
# action-level status comparison in every process_result().  Use real
# ModuleType objects (not MagicMock) so that ``import X.Y as ...``
# resolves the sub-module as a proper package.
import types as _types

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

# geometry_msgs.msg
_geo_msg = MagicMock()
_geo_msg.PointStamped = type("PointStamped", (), {})
_geo_msg.Pose = type("Pose", (), {})
_geo_msg.Point = type("Point", (), {})
_geo_msg.PoseStamped = type("PoseStamped", (), {})
_geo_msg.Quaternion = type("Quaternion", (), {})
sys.modules["geometry_msgs"] = MagicMock()
sys.modules["geometry_msgs.msg"] = _geo_msg

# std_msgs.msg
_std_msg = MagicMock()
_std_msg.Header = type("Header", (), {})
sys.modules["std_msgs"] = MagicMock()
sys.modules["std_msgs.msg"] = _std_msg

# nav_msgs.msg
sys.modules["nav_msgs"] = MagicMock()
_nav_msg = MagicMock()
_nav_msg.Odometry = type("Odometry", (), {})
sys.modules["nav_msgs.msg"] = _nav_msg

# tf2_geometry_msgs
sys.modules["tf2_geometry_msgs"] = MagicMock()

# tf2_ros
sys.modules["tf2_ros"] = MagicMock()
sys.modules["tf2_ros.buffer"] = MagicMock()
sys.modules["tf2_ros.transform_listener"] = MagicMock()

# Mock PickAndPlace to prevent Navigation import chain at
# Manipulation.py line 1070, which triggers Navigation -> tf2 etc.
sys.modules["behavior_tree.PickAndPlace"] = MagicMock()
sys.modules["behavior_tree.PickAndPlace.config"] = MagicMock()


# ---- real imports (safe after mocks are installed) -------------------------

from behavior_tree import mock_messages
from behavior_tree.TemplateNodes.Manipulation import (  # noqa: E402
    BtNode_CartesianMove,
    BtNode_JointMoveAction,
    BtNode_MoveArm,
    BtNode_MoveArmSingle,
    BtNode_Place,
    BtNode_PointTo,
)
from behavior_tree.TemplateNodes.FoldClothingAction import (  # noqa: E402
    BtNode_FoldClothingAction,
)

ROOT = Path(__file__).resolve().parents[1] / "behavior_tree/TemplateNodes"


# ---- helper ----------------------------------------------------------------

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
#  Contract tests: mock shape
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
    assert "getattr(result, \"status\"" in source


# ============================================================================
#  Behavioural tests: real process_result() on real consumer classes
# ============================================================================

def _SUCCEEDED():
    """Return the action-level success value (4)."""
    return _act_gs.STATUS_SUCCEEDED


# -- Place -------------------------------------------------------------------

def test_place_action_success_zero_status_returns_success():
    """Place: action STATUS_SUCCEEDED + result.status == 0 → SUCCESS."""
    node = _node_bypass(BtNode_Place)
    node.result_status = _SUCCEEDED()
    r = mock_messages.Place.Result()
    r.status = 0
    node.result_message = FakeResultMessage(r)
    assert node.process_result() is _PYT_SUCCESS


def test_place_action_success_nonzero_status_returns_failure():
    """Place: action STATUS_SUCCEEDED + result.status != 0 → FAILURE."""
    node = _node_bypass(BtNode_Place)
    node.result_status = _SUCCEEDED()
    r = mock_messages.Place.Result()
    r.status = 8
    r.stage = 2
    r.error_msg = "placement collision"
    node.result_message = FakeResultMessage(r)
    assert node.process_result() is _PYT_FAILURE
    assert "status=8" in node.feedback_message
    assert "stage=2" in node.feedback_message
    assert "placement collision" in node.feedback_message


# -- CartesianMove -----------------------------------------------------------

def test_cartesian_action_success_zero_status_returns_success():
    """CartesianMove: action STATUS_SUCCEEDED + result.status == 0 → SUCCESS."""
    node = _node_bypass(BtNode_CartesianMove)
    node.result_status = _SUCCEEDED()
    r = mock_messages.CartesianMove.Result()
    r.status = 0
    node.result_message = FakeResultMessage(r)
    assert node.process_result() is _PYT_SUCCESS


def test_cartesian_action_success_nonzero_status_returns_failure():
    """CartesianMove: action SUCCEEDED + result.status != 0 (even with
    legacy ``success=True``) → FAILURE, with diagnostics."""
    node = _node_bypass(BtNode_CartesianMove)
    node.result_status = _SUCCEEDED()
    r = mock_messages.CartesianMove.Result()
    r.success = True
    r.status = 4
    r.stage = 1
    r.error_msg = "IK failure"
    node.result_message = FakeResultMessage(r)
    assert node.process_result() is _PYT_FAILURE
    assert "status=4" in node.feedback_message
    assert "stage=1" in node.feedback_message
    assert "IK failure" in node.feedback_message


def test_cartesian_action_level_failure_diagnostics():
    """CartesianMove: action-level failure branch includes result
    status/stage/error_msg in feedback_message."""
    node = _node_bypass(BtNode_CartesianMove)
    node.result_status = _act_gs.STATUS_ABORTED  # 6
    r = mock_messages.CartesianMove.Result()
    r.status = 5
    r.stage = 2
    r.error_msg = "timeout"
    node.result_message = FakeResultMessage(r)
    node.process_result()
    assert "status=5" in node.feedback_message
    assert "stage=2" in node.feedback_message
    assert "timeout" in node.feedback_message


# -- JointMoveAction ---------------------------------------------------------

def test_jointmove_action_success_zero_status_returns_success():
    """BtNode_JointMoveAction: STATUS_SUCCEEDED + result.status == 0 → SUCCESS."""
    node = _node_bypass(BtNode_JointMoveAction)
    node.result_status = _SUCCEEDED()
    r = mock_messages.JointMove.Result()
    r.status = 0
    node.result_message = FakeResultMessage(r)
    assert node.process_result() is _PYT_SUCCESS


def test_jointmove_action_success_nonzero_status_returns_failure():
    """BtNode_JointMoveAction: SUCCEEDED + result.status != 0 → FAILURE."""
    node = _node_bypass(BtNode_JointMoveAction)
    node.result_status = _SUCCEEDED()
    r = mock_messages.JointMove.Result()
    r.success = True
    r.status = 7
    r.stage = 2
    r.error_msg = "joint limit"
    node.result_message = FakeResultMessage(r)
    assert node.process_result() is _PYT_FAILURE
    assert "status=7" in node.feedback_message
    assert "stage=2" in node.feedback_message
    assert "joint limit" in node.feedback_message


def test_jointmove_action_level_failure_diagnostics():
    """BtNode_JointMoveAction: action-level failure branch includes
    result status/stage/error_msg in feedback_message."""
    node = _node_bypass(BtNode_JointMoveAction)
    node.result_status = _act_gs.STATUS_ABORTED
    r = mock_messages.JointMove.Result()
    r.status = 3
    r.stage = 1
    r.error_msg = "cancelled by server"
    node.result_message = FakeResultMessage(r)
    node.process_result()
    assert "status=3" in node.feedback_message
    assert "stage=1" in node.feedback_message
    assert "cancelled by server" in node.feedback_message


# -- FoldClothingAction ------------------------------------------------------

def test_fold_action_success_zero_status_returns_success():
    """FoldClothingAction: STATUS_SUCCEEDED + result.status == 0 → SUCCESS."""
    node = _node_bypass(BtNode_FoldClothingAction)
    node.result_status = _SUCCEEDED()
    r = mock_messages.Fold.Result()
    r.status = 0
    node.result_message = FakeResultMessage(r)
    assert node.process_result() is _PYT_SUCCESS


def test_fold_action_success_nonzero_status_returns_failure():
    """FoldClothingAction: SUCCEEDED + result.status != 0 → FAILURE,
    with diagnostics."""
    node = _node_bypass(BtNode_FoldClothingAction)
    node.result_status = _SUCCEEDED()
    r = mock_messages.Fold.Result()
    r.success = True
    r.status = 7
    r.stage = 3
    r.error_msg = "grasp lost"
    r.folds_completed = 1
    r.message = "partial"
    node.result_message = FakeResultMessage(r)
    assert node.process_result() is _PYT_FAILURE
    assert "status=7" in node.feedback_message
    assert "stage=3" in node.feedback_message
    assert "grasp lost" in node.feedback_message
    assert "folds_completed=1" in node.feedback_message


def test_fold_action_level_failure_diagnostics():
    """FoldClothingAction: action-level failure branch includes result
    status/stage/error_msg in feedback_message."""
    node = _node_bypass(BtNode_FoldClothingAction)
    node.result_status = _act_gs.STATUS_ABORTED
    r = mock_messages.Fold.Result()
    r.status = 2
    r.stage = 1
    r.error_msg = "server timeout"
    node.result_message = FakeResultMessage(r)
    node.process_result()
    assert "status=2" in node.feedback_message
    assert "stage=1" in node.feedback_message
    assert "server timeout" in node.feedback_message


def test_fold_success_consistent_with_mock_default():
    """Fold.Result() default (success=True, status=0) is accepted as
    success; Fold.Result(success=False) (status=9) is rejected.  This
    proves the mock's invariant is consistent with the consumer gate."""
    node = _node_bypass(BtNode_FoldClothingAction)
    node.result_status = _SUCCEEDED()

    # default → success
    r1 = mock_messages.Fold.Result()
    node.result_message = FakeResultMessage(r1)
    assert node.process_result() is _PYT_SUCCESS

    # explicit fail → failure
    r2 = mock_messages.Fold.Result(success=False)
    node.result_message = FakeResultMessage(r2)
    assert node.process_result() is _PYT_FAILURE


# -- MoveArm / MoveArmSingle / PointTo (all use JointMove.Result) ------------

@pytest.mark.parametrize("cls", [BtNode_MoveArm, BtNode_MoveArmSingle, BtNode_PointTo])
def test_jointmove_based_node_gates_on_status(cls):
    """Every consumer that uses ``JointMove.Result`` must reject nonzero
    result status even when action-level STATUS_SUCCEEDED."""
    node = _node_bypass(cls)
    node.result_status = _SUCCEEDED()
    r = mock_messages.JointMove.Result()
    r.success = True
    r.status = 6
    r.stage = 2
    r.error_msg = "motor stall"
    node.result_message = FakeResultMessage(r)
    assert node.process_result() is _PYT_FAILURE
    assert "status=6" in node.feedback_message
