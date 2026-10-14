"""Physical configuration and motion guards; no robot commands are sent."""
import math
from pathlib import Path
from types import SimpleNamespace

import py_trees
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
