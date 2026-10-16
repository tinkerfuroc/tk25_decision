"""ROS-free regression tests: real tree/control/state, deterministic I/O boundaries.

Run with PYTHONPATH=src/behavior_tree python -m pytest -q this_file.
Only image persistence and geometry construction are replaced; no ROS or hardware.
"""
from collections import Counter
from concurrent.futures import Future
from types import SimpleNamespace as NS
import sys

import py_trees
import pytest
from behavior_tree.HomeServiceHangzhou import state, control
from behavior_tree.HomeServiceHangzhou.config import load_config, validate
from behavior_tree.HomeServiceHangzhou.tree import create_task
from behavior_tree.HomeServiceHangzhou.state import S, get, put, Context, StateNode
from behavior_tree.HomeServiceHangzhou.perception import DetectScene


@pytest.fixture
def cfg(tmp_path, monkeypatch):
    c = load_config()
    c.update(registration_pose=[0., 0., 0.],
             arm_stow="navigation", arm_assist="help", max_passes=1,
             evidence_directory=str(tmp_path), retry_delay_seconds=0., identity_facing_wait_seconds=0.)
    for i, room in enumerate(c['rooms']):
        c['room_poses'][room] = [float(i), 2., 0.]
        c['switches'].setdefault(room, {})
        for command in ('on', 'off'):
            c['switches'][room][command] = {'navigation': [1., 1., 0.], 'arm': 'click'}
    monkeypatch.setattr('behavior_tree.HomeServiceHangzhou.tree.pose', lambda x: list(x))
    monkeypatch.setattr(state, 'pose', lambda x: list(x))
    monkeypatch.setitem(sys.modules, 'cv_bridge', NS(CvBridge=lambda: NS(imgmsg_to_cv2=lambda *a, **k: b'image')))
    monkeypatch.setitem(sys.modules, 'cv2', NS(imwrite=lambda *a: True))
    py_trees.blackboard.Blackboard.clear()
    return c


def scene(owners=(), litter=False):
    labels = {1: 'sitting_on_sofa', 2: 'lying_on_floor_possible_fall', 3: 'standing_waving'}
    header = NS(frame_id='map', stamp=NS(sec=100, nanosec=0))
    observations = [NS(labels=[labels[o]], position=NS(header=header), position_valid=True,
                       bbox=NS(x_offset=0, y_offset=0, width=4, height=4))
                    for o in owners]
    if litter:
        observations = [NS(labels=['floor_litter'])]
    return NS(observations=observations, header=header, log_json='evidence.json',
              rgb_image=NS(width=4, height=4, header=header))


class IO:
    def __init__(self, context, fail=None, names=None, owners=(), identities=None):
        self.context, self.fail = context, fail or {}
        self.counts, self.calls = Counter(), []
        self.names = iter(names or ['杰克', '安东', '李华'])
        self.owners = owners
        self.identities = iter(identities) if identities is not None else None
        context.io = self

    def leaf(self, name, effect=lambda: None):
        def invoke():
            self.counts[name] += 1
            self.calls.append(name)
            if self.counts[name] <= self.fail.get(name, 0):
                return False
            return effect()
        return StateNode(name, invoke)

    def say(self, name, text=None):
        return self.leaf(name, lambda: self.calls.append(('speech', text or get('speech'))))

    def listen(self, name, initial_prompt=""):
        return self.leaf(name, lambda: put('heard', next(self.names) if name == 'Listen owner name' else '请给我一杯水'))

    def feature(self):
        def capture():
            put('features', '蓝衣服')
            put('reference_image', NS(width=4, height=4, header=NS(stamp=NS(sec=100, nanosec=0))))
        return self.leaf('feature', capture)

    def command(self):
        return self.leaf('command', lambda: put('command', 'on'))

    def arm(self, name):
        return self.leaf(name)

    def goto(self, name):
        def move():
            assert not self.context.arm_dirty
        return self.leaf(name, move)

    def approach(self):
        return self.leaf('approach')

    def head(self, pan, tilt):
        return self.leaf(f'head {pan} {tilt}', lambda: put('capture_after', 100.))

    def settle(self):
        return self.leaf('settle')

    def detect(self, task):
        def capture():
            put('scene' if task == 'behaviour' else 'litter_scene',
                scene(self.owners, litter=task == 'litter'))
            put('scene_received_at', 100.)
        return self.leaf('detect ' + task, capture)

    def identify(self):
        def match():
            role = next(e['role'] for e in reversed(self.context.events) if e['event'] == 'behaviour_selected')
            put('identities', next(self.identities) if self.identities is not None else [role])
        return self.leaf('identify', match)


def run(c, **kwargs):
    root = create_task(c, lambda ctx: IO(ctx, **kwargs))
    for node in root.iterate():
        if node.name == "Wait silently before registration":
            node.duration = 0.0
    for _ in range(2000):
        root.tick_once()
        if root.status in (S.SUCCESS, S.FAILURE):
            return root, root.context.io
    pytest.fail('tree did not terminate within 2000 deterministic ticks')


@pytest.mark.parametrize('failures', [0, 1, 4, 5])
def test_entry_navigation_has_five_bounded_attempts(cfg, failures):
    name = 'Navigate to registration point'
    root, io = run(cfg, fail={name: failures})
    assert io.counts[name] == min(failures + 1, 5)
    assert root.status == (S.FAILURE if failures == 5 else S.SUCCESS)
    if failures == 5:
        assert io.counts['Listen owner name'] == 0
    calls = len(io.calls)
    root.tick_once()
    assert len(io.calls) == calls


def test_unreachable_room_skipped_but_other_rooms_still_visited(cfg):
    root, io = run(cfg, fail={'Navigate to bedroom scan point': 100})
    assert root.status == S.SUCCESS
    assert io.counts['Navigate to bedroom scan point'] == 5
    assert root.context.visited == set(cfg['rooms']) - {'bedroom'}
    assert any(e['event'] == 'room_skipped' for e in root.context.events)
    assert io.counts['Leave through separate exit'] == 0


def test_retraction_exhaustion_blocks_navigation_and_records_recovery(cfg):
    root, io = run(cfg, fail={'Retract arm': 100})
    assert root.status == S.FAILURE
    assert io.counts['Retract arm'] == 6  # three normal + three recovery attempts
    assert not any('Navigate to registration point' in str(c) for c in io.calls)
    kinds = [e['event'] for e in root.context.events]
    assert 'mission_failed' in kinds and 'retraction_failed' in kinds
    assert ('speech', '任务已停止，机械臂已收回。') not in io.calls


def test_duplicate_name_reasks_without_duplicate_memory(cfg):
    root, io = run(cfg, names=['杰克', '杰克', '安东', '李华'])
    assert root.status == S.SUCCESS
    assert [o.name for o in root.context.owners] == ['杰克', '安东', '李华']
    assert io.counts['Call owner 2'] == 2
    assert len([e for e in root.context.events if e['event'] == 'owner_registered']) == 3


@pytest.mark.parametrize('text,expected', [('杰克', '杰克'), ('我叫安东。', '安东'),
    ('你好，我的名字叫李华。', '李华'), ('杰克，杰克', '杰克')])
def test_candidate_name_matching(text, expected):
    assert state.extract_owner_name(text) == expected


@pytest.mark.parametrize('text', [None, '', '(empty)', '王五', 'Jack', '杰克和李华', '安东或者杰克'])
def test_unknown_or_ambiguous_name_rejected(text):
    with pytest.raises(ValueError):
        state.extract_owner_name(text)


def test_unknown_name_retries_and_enrollment_order_is_not_fixed(cfg):
    root, io = run(cfg, names=['王五', '我叫李华', '我是杰克', '安东'])
    assert root.status == S.SUCCESS
    assert [o.name for o in root.context.owners] == ['李华', '杰克', '安东']
    assert io.counts['Call owner 1'] == 2


@pytest.mark.parametrize('text,expected', [('我叫李花。', '李华'), ('梨华', '李华'),
    ('你好，我是安冬', '安东'), ('岸东', '安东'), ('杰可', '杰克'),
    ('我的名字是洁克。', '杰克'), ('李华，李花', '李华')])
def test_name_homophones_ignore_tones(text, expected):
    assert state.extract_owner_name(text) == expected


@pytest.mark.parametrize('text', ['杰克和梨花', '安冬还是李花', '王五',
    '李，花', '李 花', '李abc花', '力发', 'jieke'])
def test_homophones_do_not_force_ambiguous_or_near_matches(text):
    with pytest.raises(ValueError):
        state.extract_owner_name(text)


def test_homophones_are_normalized_in_real_enrollment_tree(cfg):
    root, io = run(cfg, names=['梨花', '我叫杰可', '安冬'])
    assert root.status == S.SUCCESS
    assert [o.name for o in root.context.owners] == ['李华', '杰克', '安东']
    assert any(c[0] == 'speech' and c[1].startswith('李华，')
               for c in io.calls if isinstance(c, tuple))


def test_homophone_of_registered_name_is_still_duplicate(cfg):
    root, io = run(cfg, names=['李华', '梨花', '杰可', '安冬'])
    assert root.status == S.SUCCESS
    assert [o.name for o in root.context.owners] == ['李华', '杰克', '安东']
    assert io.counts['Call owner 2'] == 2


def test_feature_failure_retries_without_reasking_name(cfg):
    root, io = run(cfg, fail={'feature': 2})
    assert root.status == S.SUCCESS
    assert io.counts['feature'] == 5
    assert io.counts['Listen owner name'] == 3


def test_nonessential_tts_failure_does_not_stop_task(cfg):
    root, io = run(cfg, fail={'Enrollment completed': 100, 'Speak task summary': 100})
    assert root.status == S.SUCCESS
    assert io.counts['Leave through separate exit'] == 0


def test_litter_is_not_credited_until_announcement_succeeds(cfg):
    root, _ = run(cfg, fail={'Announce floor litter': 100})
    assert not root.context.litter_rooms
    assert not any(e['event'] == 'litter_announced' for e in root.context.events)


def test_dialogue_retries_only_dialogue_not_approach_or_arm(cfg):
    root, io = run(cfg, owners=(2,), fail={'Listen fallen owner': 2})
    assert root.context.completed == {2}
    assert io.counts['Listen fallen owner'] == 3
    assert io.counts['approach'] == 1
    assert io.counts['Extend assistance arm over person'] == 1


def test_required_repeat_failure_never_marks_owner_complete(cfg):
    root, io = run(cfg, owners=(3,), fail={'Speak original help request': 100})
    assert not root.context.completed
    assert io.counts['Leave through separate exit'] == 0


def test_move_discards_other_people_from_old_frame(cfg):
    root, io = run(cfg, owners=(1, 2, 3))
    first_move = io.calls.index('Return to room scan point')
    next_detection = io.calls.index('detect behaviour', first_move)
    assert 'approach' not in io.calls[first_move:next_detection]
    assert root.context.completed == {1, 2, 3}


@pytest.mark.parametrize('text', [None, '', '(empty)', 123])
def test_empty_speech_never_becomes_a_name_or_request(cfg, text):
    with pytest.raises(ValueError):
        state.extract_name(text)
    put('heard', text)
    with pytest.raises(ValueError):
        Context(cfg).repeat_request()


def test_motion_waits_for_terminal_result_not_cancel_ack(cfg, monkeypatch):
    context = Context(cfg)
    future = Future()
    handle = NS(accepted=True)
    future.set_result(handle)
    action = NS(name='previous goal', send_goal_future=future, goal_handle=handle,
                result_status=None, _cancel_requested=True)
    context.motion_actions.append(action)
    gate = control.MotionBarrier(context)
    gate.tick_once()
    assert gate.status == S.RUNNING
    action.result_status = 5
    gate.tick_once()
    assert gate.status == S.SUCCESS


def test_unconfirmed_stop_latches_motion_block(cfg, monkeypatch):
    context = Context(cfg)
    context.motion_actions.append(NS(name='pending', send_goal_future=Future()))
    gate = control.MotionBarrier(context)
    clock = [0.]
    monkeypatch.setattr(control.time, 'monotonic', lambda: clock[0])
    gate.tick_once()
    clock[0] = 11.
    gate.tick_once()
    assert gate.status == S.FAILURE and context.motion_blocked
    next_gate = control.MotionBarrier(context)
    next_gate.tick_once()
    assert next_gate.status == S.FAILURE


def test_retry_pause_and_limit(monkeypatch):
    clock = [0.]
    monkeypatch.setattr(control.time, 'monotonic', lambda: clock[0])
    calls = []
    child = StateNode('fail', lambda: calls.append(1) or False)
    root = control.PacedRetry('retry', child, 2, 1.)
    root.tick_once()
    root.tick_once()
    assert len(calls) == 1 and root.status == S.RUNNING
    clock[0] = 1.
    root.tick_once()
    assert len(calls) == 2 and root.status == S.FAILURE


def test_detector_transport_exception_returns_failure(cfg):
    node = DetectScene('detector', Context(cfg), 'behaviour')
    put('capture_after', 90.)
    node.initialise()
    def broken(_):
        raise RuntimeError('transport unavailable')
    node.message_type = NS(Request=lambda: NS())
    node.client = NS(service_is_ready=lambda: True, call_async=broken)
    assert node.update() == S.FAILURE
    assert 'transport unavailable' in node.feedback_message


def test_audio_timeout_does_not_overlap_next_recording(cfg, monkeypatch):
    context = Context(cfg)
    response = Future()
    context.audio_actions.append(NS(name='TTS still playing', response=response))
    gate = control.MotionBarrier(context, 'audio')
    gate.tick_once()
    assert gate.status == S.RUNNING
    response.set_result(NS(status=0))
    gate.tick_once()
    assert gate.status == S.SUCCESS
    assert not context.motion_blocked


def test_rejected_goal_is_idle_but_pending_acceptance_is_not():
    response = Future()
    action = NS(send_goal_future=response, goal_handle=None, result_status=None)
    assert not control.action_idle(action)
    response.set_result(NS(accepted=False))
    assert control.action_idle(action)


def test_timeout_cancels_child_before_retry(cfg, monkeypatch):
    clock = [0.]
    monkeypatch.setattr(control.time, 'monotonic', lambda: clock[0])
    canceled = []
    class Running(py_trees.behaviour.Behaviour):
        def update(self):
            return S.RUNNING
        def terminate(self, status):
            canceled.append(status)
    child = Running('running goal')
    timeout = py_trees.decorators.Timeout(name='deadline', child=child, duration=1.)
    root = control.PacedRetry('retry', timeout, 2, 1.)
    root.tick_once()
    clock[0] = 2.
    root.tick_once()
    assert child.status == S.INVALID and canceled == [S.INVALID]
    assert root.status == S.RUNNING


def test_perception_error_never_turns_into_negative_detection(cfg):
    root, io = run(cfg, fail={'detect behaviour': 1000})
    assert root.status == S.SUCCESS
    assert not root.context.completed
    assert io.counts['detect behaviour'] == 32
    assert io.counts['identify'] == 0


@pytest.mark.parametrize("role", [1, 2, 3])
def test_rooms_without_switches_keep_other_services(cfg, role):
    cfg["switches"] = {}
    validate(cfg)
    root, io = run(cfg, owners=(role,))
    assert root.status == S.SUCCESS
    assert root.context.visited == set(cfg["rooms"])
    assert io.counts["Announce location, owner and behaviour"] > 0
    assert io.counts["detect litter"] == 4
    assert io.counts["command"] == 0
    assert io.counts["Navigate to switch mark"] == 0
    if role == 1:
        assert io.counts["approach"] == 0
        assert not root.context.serviced_roles
        assert any(e["event"] == "switch_service_not_applicable" for e in root.context.events)
    else:
        assert role in root.context.serviced_roles
        assert io.counts["approach"] > 0


def test_mission_completes_without_exit_navigation(cfg):
    root, io = run(cfg)
    assert root.status == S.SUCCESS
    assert io.counts['Leave through separate exit'] == 0
    assert any(e['event'] == 'mission_completed' for e in root.context.events)



def test_placeholder_config_never_calls_io(tmp_path):
    c = load_config()
    c['retry_delay_seconds'] = 0
    c['evidence_directory'] = str(tmp_path)
    root, io = run(c)
    assert root.status == S.FAILURE and not io.calls


@pytest.fixture
def adapters(monkeypatch):
    """Load real adapter methods with ROS-only parent imports replaced locally."""
    import importlib.util
    from pathlib import Path
    class Parent(py_trees.behaviour.Behaviour):
        def process_result(self):
            return S.SUCCESS if self.result_message.result.status == 0 else S.FAILURE
    modules = {
        'Audio': ['BtNode_Announce', 'BtNode_ListenAction', 'BtNode_PhraseExtractionAction'],
        'Navigation': ['BtNode_GotoAction', 'BtNode_Approach'],
        'ActionBase': ['ActionHandler'],
        'Vision': ['BtNode_FeatureExtraction', 'BtNode_TurnPanTilt'],
    }
    for module, names in modules.items():
        monkeypatch.setitem(sys.modules, 'behavior_tree.TemplateNodes.' + module,
                            NS(**{n: type(n, (Parent,), {}) for n in names}))
    path = Path(state.__file__).with_name('ros_nodes.py')
    spec = importlib.util.spec_from_file_location('behavior_tree.HomeServiceHangzhou._test_adapters', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('status,code,expected', [(4, 0, S.SUCCESS), (5, 0, S.FAILURE),
    (6, 0, S.FAILURE), (0, 0, S.FAILURE), (4, 1, S.FAILURE)])
def test_actual_navigation_adapter_status_mapping(adapters, status, code, expected):
    node = adapters.HangzhouGoto('navigation')
    node.result_status = status
    node.result_message = NS(result=NS(error_code=code, error_msg='detail'))
    assert node.process_result() == expected


@pytest.mark.parametrize('status,semantic,expected', [(4, 0, S.SUCCESS),
    (4, 2, S.FAILURE), (5, 0, S.FAILURE), (6, 0, S.FAILURE)])
def test_actual_approach_adapter_requires_transport_and_semantic_success(adapters, status, semantic, expected):
    node = adapters.HangzhouApproach('approach')
    node.result_status = status
    node.result_message = NS(result=NS(status=semantic))
    assert node.process_result() == expected


@pytest.mark.parametrize("stamp", [0, 1, 100, 10**9])
def test_target_position_does_not_require_a_clock_or_fresh_timestamp(adapters, stamp):
    put("target", NS(header=NS(frame_id="map", stamp=NS(sec=stamp, nanosec=0)),
                     point=NS(x=1., y=2., z=0.)))
    put("target_valid", True)
    assert adapters.ValidTarget().update() == S.SUCCESS


@pytest.mark.parametrize("frame,valid,x", [("camera", True, 1.), ("map", False, 1.),
                                          ("map", True, float("nan"))])
def test_invalid_target_position_still_blocks_approach(adapters, frame, valid, x):
    put("target", NS(header=NS(frame_id=frame), point=NS(x=x, y=2., z=0.)))
    put("target_valid", valid)
    assert adapters.ValidTarget().update() == S.FAILURE


def test_unknown_transport_result_is_not_proof_motion_stopped():
    response = Future()
    response.set_exception(RuntimeError('connection lost'))
    assert not control.action_idle(NS(send_goal_future=response, goal_handle=None))


def test_service_request_error_returns_failure_instead_of_crashing(adapters):
    node = adapters.HangzhouAnnounce('speech')
    errors = []
    node.node = NS(get_logger=lambda: NS(error=errors.append))
    node._request_error = 'transport unavailable'
    assert node.update() == S.FAILURE
    assert errors and 'transport unavailable' in node.feedback_message


def test_action_transport_error_returns_failure_with_diagnostics(adapters):
    node = adapters.HangzhouGoto('navigation')
    errors = []
    node.node = NS(get_logger=lambda: NS(error=errors.append))
    node.action_name = 'navigate_to_pose'
    node._transport_error = 'result transport failed'
    assert node.update() == S.FAILURE
    assert errors and 'result transport failed' in node.feedback_message


@pytest.mark.parametrize('role,dialogue_end', [(1, 'Repeat electrical request'),
    (2, 'Acknowledge fallen owner'), (3, 'Speak original help request')])
def test_location_identity_and_behaviour_are_announced_before_approach(cfg, role, dialogue_end):
    root, io = run(cfg, owners=(role,))
    calls = io.calls
    assert root.context.completed == {role}
    announcement = calls.index("Announce location, owner and behaviour")
    assert calls.index("detect behaviour") < calls.index("identify") < announcement
    assert announcement < calls.index("approach") < calls.index(dialogue_end)
    report = calls[announcement + 1][1]
    assert report == (f"\u5728{cfg['room_names'][cfg['current_room']]}\u53d1\u73b0"
                      f"{root.context.owners[role - 1].name}\uff0c\u6b63\u5728"
                      f"{state.BEHAVIOUR_ZH[role]}\u3002")
    assert io.counts["identify"] == 1
    assert io.counts["Ask person to face top camera"] == 0


def test_unknown_behaviour_image_falls_back_to_facing_image(cfg):
    root, io = run(cfg, owners=(2,), identities=[[None], [2]])
    assert root.context.completed == {2}
    assert io.counts['identify'] == 2 and io.counts['approach'] == 1
    records = [e for e in root.context.events if e['event'] == 'owner_identified']
    assert records[0]['source'] == 'facing_image'
    selected = get('behaviour_identity_scene')
    assert len(selected.observations) == 1
    assert selected.observations[0].labels == ['lying_on_floor_possible_fall']


def test_behaviour_identity_success_skips_facing_capture(cfg):
    root, io = run(cfg, owners=(2,), identities=[[2]])
    assert root.context.completed == {2}
    assert io.counts['identify'] == 1
    assert io.counts['feature'] == 3  # Enrollment only.
    assert any(e['event'] == 'owner_identified' and e['source'] == 'behaviour_image'
               for e in root.context.events)


def test_unidentified_person_still_receives_service_without_false_identity(cfg):
    # Original crop, facing image, then two re-prompted facing images.
    root, io = run(cfg, owners=(2,), identities=[[None]] * 4)
    assert root.status == S.SUCCESS
    assert root.context.completed == set()
    assert root.context.serviced_roles == {2}
    assert io.counts['identify'] == 4
    assert io.counts['Ask person to face top camera again'] == 2
    assert io.counts['Extend assistance arm over person'] == 1
    assert io.counts['approach'] == 1
    assert any(e['event'] == 'interaction_completed_unidentified' for e in root.context.events)
    assert len([e for e in root.context.events if e['event'] == 'identity_attempt_unresolved']) == 4


def test_reprompted_facing_image_identifies_after_both_first_sources_unknown(cfg):
    root, io = run(cfg, owners=(3,), identities=[[None], [None], [3]])
    assert root.context.completed == {3}
    assert io.counts['identify'] == 3 and io.counts['approach'] == 1
    assert io.counts['Ask person to face top camera again'] == 1
    calls = io.calls
    assert calls.index('Ask person to face top camera') < calls.index('Ask person to face top camera again')
    records = [e for e in root.context.events if e['event'] == 'owner_identified']
    assert [r['source'] for r in records] == ['facing_image']


def test_identity_retry_attempts_are_configurable(cfg):
    cfg['identity_retry_attempts'] = 4
    root, io = run(cfg, owners=(1,), identities=[[None]] * 6)
    assert io.counts['identify'] == 6
    assert io.counts['Ask person to face top camera again'] == 4
    assert root.context.serviced_roles == {1} and root.context.completed == set()


def behaviour_scene(*label_sets):
    header = NS(frame_id='map', stamp=NS(sec=100, nanosec=0))
    return NS(header=header, log_json='evidence.json', observations=[
        NS(labels=list(labels), position=NS(header=header), position_valid=True,
           bbox=NS(x_offset=0, y_offset=0, width=4, height=4)) for labels in label_sets])


@pytest.mark.parametrize('labels,role', [
    (('sitting_on_chair', 'sitting_waving'), 3),
    (('sitting_on_sofa', 'standing_waving'), 3),
    (('lying_on_floor_possible_fall', 'lying_resting_or_sleeping'), 2),
    (('sitting_on_bed',), 1)])
def test_multi_label_person_gets_highest_priority_role(cfg, labels, role):
    context = Context(cfg)
    put('scene', behaviour_scene(labels))
    context.observe_behaviours()
    assert list(context.active) == [role]


def test_served_waver_still_sitting_is_not_reassigned_as_resting(cfg):
    context = Context(cfg)
    context.serviced_roles.add(3)
    put('scene', behaviour_scene(('sitting_on_chair', 'sitting_waving'), ('sitting_on_sofa',)))
    context.observe_behaviours()
    assert context.active == {1: (1, get('scene').observations[1])}


def test_identity_is_not_inferred_from_behaviour_role(cfg):
    root, _ = run(cfg, owners=(2,), identities=[[3]])
    assert root.context.completed == {3}
    assert root.context.serviced_roles == {2}
    assert any(e['event'] == 'interaction_completed' and e['owner'] == 3 and e['role'] == 2
               for e in root.context.events)


def test_facing_image_does_not_require_post_prompt_timestamp(cfg):
    context = Context(cfg)
    put("capture_after", 101.)
    image = NS(width=4, height=4, header=NS(stamp=NS(sec=0, nanosec=0)))
    put("reference_image", image)
    context.prepare_facing_identity()
    assert get("scene").rgb_image is image


@pytest.mark.parametrize('key,value', [('navigation_attempts', 0), ('arm_attempts', True),
    ('retry_delay_seconds', -1), ('cancel_timeout_seconds', 0), ('arm_timeout_seconds', float('nan'))])
def test_invalid_recovery_settings_fail_preflight(cfg, key, value):
    cfg[key] = value
    with pytest.raises(ValueError, match=key):
        validate(cfg)


def test_unknown_identity_is_explicit_in_pre_approach_report(cfg):
    root, io = run(cfg, owners=(3,), identities=[[None]] * 4)
    index = io.calls.index("Announce location, owner and behaviour")
    text = io.calls[index + 1][1]
    assert cfg["room_names"][cfg["current_room"]] in text
    assert "\u8eab\u4efd\u5c1a\u672a\u786e\u8ba4" in text
    assert state.BEHAVIOUR_ZH[3] in text
    assert all(owner.name not in text for owner in root.context.owners)
    assert index < io.calls.index("approach")


def test_announcement_failure_does_not_start_service(cfg):
    root, io = run(cfg, owners=(3,), fail={"Announce location, owner and behaviour": 100})
    assert io.counts["approach"] == 0
    assert not root.context.serviced_roles


@pytest.mark.parametrize("stamp", [0, 1, 100, 10**9])
def test_detector_accepts_capture_age_and_missing_diagnostic_log(cfg, stamp):
    value = scene()
    value.header.stamp.sec = stamp
    value.status, value.detected, value.error_msg, value.log_json = 0, False, "", ""
    context = Context(cfg)
    node = DetectScene("detector", context, "behaviour")
    put("capture_after", 100.)
    node.initialise()
    node.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000)))
    node.response = Future()
    node.response.set_result(value)
    assert node.update() == S.SUCCESS
    assert get("scene") is value


@pytest.mark.parametrize("width,expected", [(4, S.SUCCESS), (0, S.FAILURE)])
def test_feature_image_checks_content_not_timestamp(adapters, width, expected):
    node = adapters.HangzhouFeature("feature")
    node.result_message = NS(result=NS(status=0))
    node.capture_started = 100.
    node.blackboard = NS(comparison_image=NS(width=width, height=4,
                         header=NS(stamp=NS(sec=0, nanosec=0))))
    assert node.process_result() == expected
