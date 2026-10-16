"""Physical configuration and motion guards; no robot commands are sent."""
import json

from pathlib import Path
from types import SimpleNamespace

import py_trees
import pytest
from unittest.mock import Mock
from behavior_tree.HomeServiceHangzhou.config import load_config, validate
from behavior_tree.HomeServiceHangzhou.ros_nodes import Tk27SavedPose
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
    leaf = Tk27SavedPose(name='TK27 contract', pose_name_bb_key=PREFIX+'arm_pose_name',
                      action_name='/manipulation/move_to_saved_pose')
    put('arm_pose_name', 'navigation')
    leaf.mock_mode = False
    goals = []
    leaf.send_goal_request = goals.append
    leaf.send_goal()
    from tk27_manipulation_msgs.action import MoveToSavedPose
    assert leaf.action_type is MoveToSavedPose
    assert isinstance(goals[0], MoveToSavedPose.Goal)
    assert goals[0].pose_name == "navigation"
    assert goals[0].avoid_collisions is True
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
    from map_studio.pose_store import PoseStore
    path = Path(c["map_yaml"])
    assert path.stem == "robocup_home_2026_measured_5cm"
    points = PoseStore(path.parent).load(path.stem)
    for room, name in (("living_room", "livingroom_search"), ("bedroom", "bedroom_search"),
                       ("dining_room", "diningroom_search"), ("kitchen", "kitchen_search")):
        point = points[name]
        assert c["room_poses"][room] == [point.x, point.y, point.yaw]
    assert c['endpoints']['arm'] == '/manipulation/move_to_saved_pose'
    assert c['endpoints']['approach'] == 'go_to_approach'
    assert c["arm_stow"] == "navigation"
    assert c["arm_assist"] == "help"
    assert set(c["switches"]) == {"living_room", "bedroom"}
    validate(c)
    assert all(command["arm"] == "click" for room in c["switches"].values()
               for command in room.values())


def test_completed_arm_failure_preserves_original_error():
    leaf = Tk27SavedPose(name='Retract arm', pose_name_bb_key=PREFIX+'arm_pose_name',
                      action_name='/manipulation/move_to_saved_pose')
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


def test_saved_pose_configuration_needs_no_local_export_files(tmp_path, config):
    raw = config
    path = tmp_path / "site.json"
    path.write_text(json.dumps(raw))
    config = load_config(path)
    validate(config)
    assert config["arm_stow"] == "navigation"


@pytest.mark.parametrize("name", [None, "", [0.0] * 7])
def test_saved_pose_configuration_rejects_missing_names_or_angle_arrays(config, name):
    config["arm_stow"] = name
    with pytest.raises(ValueError, match="arm_stow"):
        validate(config)


def test_stow_failure_after_assistance_prevents_further_base_motion(config):
    from behavior_tree.HomeServiceHangzhou.tree import create_task
    from behavior_tree.HomeServiceHangzhou.state import S, get
    from test_hangzhou_home_service import FakeLeaves

    class FailedStow(FakeLeaves):
        blocked_at = None

        def arm(self, name):
            if name != "Retract arm":
                return super().arm(name)

            def stow():
                self.calls.append(("arm", name, get("arm_pose_name")))
                assisted = any(call[:2] == ("arm", "Extend assistance arm over person")
                               for call in self.calls)
                if assisted:
                    if self.blocked_at is None:
                        self.blocked_at = len(self.calls)
                    return False
            return self.node(name, stow)

    root = create_task(config, FailedStow)
    from test_hangzhou_home_service import skip_startup_delay
    skip_startup_delay(root)
    for _ in range(200):
        root.tick_once()
        if root.status in (S.SUCCESS, S.FAILURE):
            break
    io = root.context.fake_io
    assert io.blocked_at is not None
    assert root.status == S.FAILURE
    assert not any(call[0] in ("goto", "approach") for call in io.calls[io.blocked_at:])


def test_map_studio_poses_are_resolved_on_each_task_start(tmp_path, config):
    from map_studio.pose_store import PoseStore, make_pose
    name = "arena"
    store = PoseStore(tmp_path)
    store.write_new(name, {"register": make_pose(1, 2, 0.1),
                           "search": make_pose(3, 4, 0.3),
                           "switch": make_pose(5, 6, 0.4)})
    config["map_yaml"] = str(tmp_path / (name + ".yaml"))
    config["registration_pose"] = "register"
    config["room_poses"] = {room: "search" for room in config["rooms"]}
    for room in config["switches"].values():
        for goal in room.values():
            goal["navigation"] = "switch"
    path = tmp_path / "config.json"
    path.write_text(json.dumps(config))
    resolved = load_config(path)
    validate(resolved)
    assert resolved["registration_pose"] == pytest.approx([1., 2., 0.1])
    assert resolved["room_poses"]["kitchen"] == pytest.approx([3., 4., 0.3])
    assert resolved["switches"]["bedroom"]["on"]["navigation"] == pytest.approx([5., 6., 0.4])
    store.put(name, "search", make_pose(7, 8, 0.5))
    assert load_config(path)["room_poses"]["kitchen"] == pytest.approx([7., 8., 0.5])


def test_missing_map_studio_point_does_not_use_old_coordinates(tmp_path, config):
    config["map_yaml"] = str(tmp_path / "arena.yaml")
    config["registration_pose"] = "missing"
    path = tmp_path / "config.json"
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match="Map Studio pose 'missing'"):
        load_config(path)
