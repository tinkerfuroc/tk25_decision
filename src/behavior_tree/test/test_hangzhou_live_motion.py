"""Physical configuration and motion guards; no robot commands are sent."""
import math
from pathlib import Path
from types import SimpleNamespace

import py_trees
import pytest
from unittest.mock import Mock
from behavior_tree.HomeServiceHangzhou.config import load_config, validate
from behavior_tree.HomeServiceHangzhou.ros_nodes import Tk27MoveArm
from behavior_tree.HomeServiceHangzhou.state import put, PREFIX
from test_hangzhou_home_service import config, run_tree


def test_every_base_motion_requires_successful_stow(config):
    root, io = run_tree(config)
    assert root.status == py_trees.common.Status.SUCCESS
    previous = None
    for call in io.calls:
        if call[0] in ('goto', 'approach'):
            assert previous == ('arm', 'Retract arm', config['arm_stow'])
        if call[0] in ('arm', 'goto', 'approach'):
            previous = call


def test_tk27_goal_and_result_contract():
    leaf = Tk27MoveArm(name='TK27 contract', arm_pose_bb_key=PREFIX+'arm_goal',
                      action_name='/manipulation/joint_move')
    put('arm_goal', [0.1] * 7)
    goals = []
    leaf.send_goal_request = goals.append
    leaf.send_goal()
    assert goals[0].joint_names == ['joint'+str(i) for i in range(1, 8)]
    assert list(goals[0].positions) == [0.1] * 7
    leaf.result_status = 4
    leaf.result_message = SimpleNamespace(result=leaf.action_type.Result(
        success=False, status=9, error_msg='arrival verification failed'))
    assert leaf.process_result() == py_trees.common.Status.FAILURE
    leaf.result_message.result.success = True
    leaf.result_message.result.status = 0
    assert leaf.process_result() == py_trees.common.Status.SUCCESS


def test_live_tkmr_configuration():
    from behavior_tree.HomeServiceHangzhou import config as module
    c = load_config(Path(module.__file__).with_name('tkmr_live.json'))
    validate(c)
    assert list(c['waypoint_names'].values()) == ['sofa','pillar','round_table','corner']
    assert c['endpoints']['arm'] == '/manipulation/joint_move'
    assert c['endpoints']['approach'] == 'go_to_approach'
    assert math.isclose(c['arm_stow'][0], math.radians(-86))


def test_completed_arm_failure_preserves_original_error():
    leaf = Tk27MoveArm(name='Retract arm', arm_pose_bb_key=PREFIX+'arm_goal',
                      action_name='/manipulation/joint_move')
    leaf.mock_mode = False
    leaf.node = SimpleNamespace(get_logger=lambda: logger)
    logger = Mock()
    leaf.status = py_trees.common.Status.RUNNING
    leaf.send_goal_future = Mock()
    leaf.goal_handle = Mock(accepted=True)
    leaf.get_result_future = Mock()
    leaf.get_result_future.done.return_value = True
    leaf.result_status = 6
    leaf.result_status_string = 'STATUS_ABORTED'
    leaf.result_message = SimpleNamespace(result=leaf.action_type.Result(
        success=False, status=14,
        error_msg='Depth image is stale or ahead of the active clock'))
    leaf.send_cancel_request = Mock()
    assert leaf.update() == py_trees.common.Status.FAILURE
    leaf.stop(py_trees.common.Status.FAILURE)
    leaf.send_cancel_request.assert_not_called()
    assert 'Depth image is stale' in leaf.feedback_message
    assert 'Depth image is stale' in logger.error.call_args.args[0]


@pytest.mark.parametrize('status', list(py_trees.common.Status))
def test_terminal_mission_stops_timer_and_keeps_node(status):
    from behavior_tree.runtime import stop_terminal_ticks
    tree = Mock()
    tree.root.status = status
    stop_terminal_ticks(tree)
    terminal = status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE)
    assert tree.timer.cancel.called == terminal
    tree.shutdown.assert_not_called()
    tree.node.destroy_node.assert_not_called()
