"""TK27 wire contracts through the real tree adapters; no robot or GPU."""

from types import SimpleNamespace
import time

import py_trees
import pytest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import PointCloud2
from tk27_manipulation_msgs.action import Pick, Place

from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_Place
from behavior_tree.StoringGroceries.customNodes import BtNode_GraspWithPose


@pytest.fixture(autouse=True)
def clean_blackboard():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


def seed(**values):
    writer = py_trees.blackboard.Client(name="tk27_test_writer")
    for key, value in values.items():
        writer.register_key(key, access=py_trees.common.Access.WRITE)
        writer.set(key, value)
    return writer


def capture(node):
    sent = []
    node.mock_mode = False
    node.send_goal_request = sent.append
    return sent


def test_pick_uses_prompt_and_effort_only():
    seed(prompt="  red bottle  ")
    node = BtNode_Grasp("pick", bb_source="prompt", plan_effort="low")
    sent = capture(node)
    node.send_goal()
    assert node.action_type is Pick
    assert node.action_name == "/manipulation/pick"
    assert sent[0].prompt == "red bottle"
    assert sent[0].plan_effort == "low"
    assert set(sent[0].get_fields_and_field_types()) == {"prompt", "plan_effort"}


def test_pick_uses_label_without_requiring_capture_images():
    seed(label="bowl")
    node = BtNode_Grasp("pick", bb_key_object_label="label")
    sent = capture(node)
    node.send_goal()
    assert sent[0].prompt == "bowl"


def test_existing_discovery_result_supplies_only_text():
    seed(detection=SimpleNamespace(objects=[SimpleNamespace(cls="can")]))
    node = BtNode_Grasp("pick", bb_key_vision_res="detection")
    sent = capture(node)
    node.send_goal()
    assert sent[0].prompt == "can"


@pytest.mark.parametrize("prompt", ["", " ", None, 42])
def test_invalid_prompt_sends_no_goal(prompt):
    seed(prompt=prompt)
    node = BtNode_Grasp("pick", bb_source="prompt")
    sent = capture(node)
    assert node.send_goal() == py_trees.common.Status.FAILURE
    assert sent == []


def place_node(orientation=None, **kwargs):
    point = PointStamped()
    point.header.frame_id = "base_link"
    point.header.stamp.sec = 12
    point.point.x, point.point.y, point.point.z = 0.4, 0.1, 0.6
    cloud = PointCloud2()
    cloud.header = point.header
    seed(point=point, orientation=orientation, cloud=cloud)
    node = BtNode_Place("place", "point", "orientation", "cloud", **kwargs)
    return node, capture(node), point, cloud


@pytest.mark.parametrize(
    "orientation", [None, Pose(orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0))]
)
def test_place_preserves_frame_and_explicitly_retains_cached_orientation(orientation):
    node, sent, point, cloud = place_node(orientation)
    node.send_goal()
    assert node.action_type is Place
    assert node.action_name == "/manipulation/place"
    goal = sent[0]
    assert goal.tcp_pose.header == point.header
    assert goal.tcp_pose.pose.position == point.point
    assert goal.env_points == cloud
    assert goal.retain_grasp_orientation
    assert goal.tcp_pose.pose.orientation.w == 1.0


def test_place_preserves_supplied_unit_orientation():
    pose = Pose()
    pose.orientation.w = 0.0
    pose.orientation.y = 1.0
    node, sent, _, _ = place_node(pose)
    node.send_goal()
    assert not sent[0].retain_grasp_orientation
    assert sent[0].tcp_pose.pose.orientation == pose.orientation


def test_place_rejects_mismatched_orientation_frame():
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.orientation.w = 1.0
    node, sent, _, _ = place_node(pose)
    assert node.send_goal() == py_trees.common.Status.FAILURE
    assert not sent


def test_place_rejects_missing_explicit_orientation():
    node, sent, _, _ = place_node(retain_grasp_orientation=False)
    assert node.send_goal() == py_trees.common.Status.FAILURE
    assert not sent


def test_place_rejects_unframed_target():
    node, sent, point, _ = place_node()
    point.header.frame_id = ""
    assert node.send_goal() == py_trees.common.Status.FAILURE
    assert not sent


@pytest.mark.parametrize("kind", [Pick, Place])
@pytest.mark.parametrize(
    "ros_status,success,status,expected",
    [
        (GoalStatus.STATUS_SUCCEEDED, True, 0, py_trees.common.Status.SUCCESS),
        (GoalStatus.STATUS_SUCCEEDED, False, 2, py_trees.common.Status.FAILURE),
        (GoalStatus.STATUS_SUCCEEDED, True, 2, py_trees.common.Status.FAILURE),
        (GoalStatus.STATUS_ABORTED, False, 3, py_trees.common.Status.FAILURE),
        (GoalStatus.STATUS_CANCELED, False, 4, py_trees.common.Status.FAILURE),
    ],
)
def test_minimal_results_require_ros_and_business_success(
    kind, ros_status, success, status, expected
):
    node = BtNode_Grasp("pick", prompt="bottle") if kind is Pick else place_node()[0]
    node.result_status = ros_status
    node.result_message = SimpleNamespace(
        result=kind.Result(success=success, status=status, error_msg="test outcome")
    )
    assert node.process_result() == expected
    assert "test outcome" in node.feedback_message


@pytest.mark.parametrize("kind", [Pick, Place])
def test_feedback_uses_task_progress_without_legacy_timeout_fields(kind):
    node = BtNode_Grasp("pick", prompt="bottle") if kind is Pick else place_node()[0]
    node.action_status = 0
    feedback = kind.Feedback()
    feedback.progress.stage = feedback.progress.PLANNING
    feedback.progress.detail = "Planning"
    feedback.progress.elapsed_s = 0.2
    before = time.time()
    node.feedback_callback(SimpleNamespace(feedback=feedback))
    assert node.last_feedback_time >= before
    assert node.action_stage == feedback.progress.PLANNING
    assert node.action_status == 0
    assert "Planning" in node.feedback_message


def test_grasp_with_pose_does_not_fabricate_pose_or_accept_failed_result():
    node = BtNode_GraspWithPose("pick", "detection", "orientation")
    node.result_status = GoalStatus.STATUS_SUCCEEDED
    node.result_message = SimpleNamespace(result=Pick.Result(success=False, status=2))
    assert node.process_result() == py_trees.common.Status.FAILURE
    node.result_message = SimpleNamespace(result=Pick.Result(success=True, status=0))
    assert node.process_result() == py_trees.common.Status.SUCCESS
    assert py_trees.blackboard.Blackboard.get("orientation") is None


@pytest.mark.parametrize("kind", [Pick, Place])
def test_task_timeout_keeps_inherited_cancellation(kind):
    node = BtNode_Grasp("pick", prompt="bottle") if kind is Pick else place_node()[0]
    node.mock_mode = False
    node.counter = 1
    node.action_timeout_ticks = 1
    canceled = []
    node.node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(debug=lambda _: None)
    )
    node.goal_handle = SimpleNamespace(
        cancel_goal_async=lambda: (
            canceled.append(True)
            or SimpleNamespace(add_done_callback=lambda callback: None)
        )
    )
    assert node.update() == py_trees.common.Status.FAILURE
    assert canceled == [True]


@pytest.mark.parametrize(
    "node_name", ["BtNode_MoveArmSingle", "BtNode_JointMoveAction"]
)
def test_joint_tree_adapter_uses_named_radian_targets(node_name):
    from behavior_tree.TemplateNodes import Manipulation
    from tk27_manipulation_msgs.action import JointMove

    seed(joints=[0.1] * 7)
    node = getattr(Manipulation, node_name)("joint", "joints")
    sent = capture(node)
    node.send_goal()
    assert node.action_type is JointMove
    assert node.action_name == "/manipulation/joint_move"
    assert list(sent[0].joint_names) == [f"joint{i}" for i in range(1, 8)]
    assert list(sent[0].positions) == [0.1] * 7
    assert set(sent[0].get_fields_and_field_types()) == {"joint_names", "positions"}
    node.result_status = GoalStatus.STATUS_SUCCEEDED
    node.result_message = SimpleNamespace(
        result=JointMove.Result(success=True, status=5)
    )
    assert node.process_result() == py_trees.common.Status.FAILURE
    feedback = JointMove.Feedback(stage=JointMove.Feedback.VERIFYING, detail="Settling")
    node.feedback_callback(SimpleNamespace(feedback=feedback))
    assert node.action_stage == JointMove.Feedback.VERIFYING


def test_named_joint_adapter_supports_another_arm_group():
    from behavior_tree.TemplateNodes.Manipulation import _joint_move_goal

    goal = _joint_move_goal({"axis_b": 0.4, "axis_a": -0.2})
    assert list(goal.joint_names) == ["axis_b", "axis_a"]
    assert list(goal.positions) == [0.4, -0.2]
