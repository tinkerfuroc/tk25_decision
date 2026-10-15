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
    c.update(entry_pose=[0., 0., 0.], exit_pose=[9., 0., 0.],
             arm_stow=[0.] * 7, arm_assist=[1.] * 7, max_passes=1,
             evidence_directory=str(tmp_path), retry_delay_seconds=0., identity_facing_wait_seconds=0.)
    for i, room in enumerate(c['rooms']):
        c['room_poses'][room] = [float(i), 2., 0.]
        for command in ('on', 'off'):
            c['switches'][room][command] = {'navigation': [1., 1., 0.], 'arm': [2.] * 7}
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

    def listen(self, name):
        return self.leaf(name, lambda: put('heard', next(self.names) if name == 'Listen owner name' else '请给我一杯水'))

    def feature(self):
        def capture():
            put('features', '蓝衣服')
            put('reference_image', NS(width=4, height=4, header=NS(stamp=NS(sec=100, nanosec=0))))
        return self.leaf('feature', capture)

    def command(self):
        return self.leaf('command', lambda: put('command', '开'))

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
    for _ in range(2000):
        root.tick_once()
        if root.status in (S.SUCCESS, S.FAILURE):
            return root, root.context.io
    pytest.fail('tree did not terminate within 2000 deterministic ticks')


@pytest.mark.parametrize('failures', [0, 1, 4, 5])
def test_entry_navigation_has_five_bounded_attempts(cfg, failures):
    name = 'Enter arena to enrollment point'
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
    assert io.counts['Leave through separate exit'] == 1


def test_retraction_exhaustion_blocks_navigation_and_records_recovery(cfg):
    root, io = run(cfg, fail={'Retract arm': 100})
    assert root.status == S.FAILURE
    assert io.counts['Retract arm'] == 6  # three normal + three recovery attempts
    assert not any('Enter arena' in str(c) for c in io.calls)
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
    assert io.counts['Leave through separate exit'] == 1


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
    assert io.counts['Leave through separate exit'] == 1


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


def test_exit_navigation_exhaustion_retains_failure(cfg):
    root, io = run(cfg, fail={'Leave through separate exit': 100})
    assert root.status == S.FAILURE
    assert io.counts['Leave through separate exit'] == 5
    assert not any(e['event'] == 'exited' for e in root.context.events)


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
        'Manipulation': ['BtNode_MoveArmSingle'],
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


def test_actual_fresh_target_blocks_expired_position(cfg, adapters):
    put('target', NS(header=NS(frame_id='map', stamp=NS(sec=100, nanosec=0)),
                     point=NS(x=1., y=2., z=0.)))
    put('target_valid', True)
    node = adapters.FreshTarget(Context(cfg))
    now = [100.]
    node.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=now[0]*1e9)))
    assert node.update() == S.SUCCESS
    now[0] = 146.
    assert node.update() == S.FAILURE
    assert 'expired' in node.feedback_message


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
def test_identity_only_after_approach_and_dialogue(cfg, role, dialogue_end):
    root, io = run(cfg, owners=(role,))
    calls = io.calls
    assert root.context.completed == {role}
    assert calls.index('detect behaviour') < calls.index('approach') < calls.index(dialogue_end)
    assert calls.index(dialogue_end) < calls.index('Ask person to face top camera') < calls.index('identify')
    assert calls.index('identify') < calls.index('Return to room scan point')
    if role == 1:
        assert calls.index('identify') < calls.index('Navigate to switch mark')


def test_unknown_facing_image_falls_back_to_selected_behaviour_image(cfg):
    root, io = run(cfg, owners=(2,), identities=[[None], [2]])
    assert root.context.completed == {2}
    assert io.counts['identify'] == 2 and io.counts['approach'] == 1
    records = [e for e in root.context.events if e['event'] == 'post_interaction_identity']
    assert records[0]['source'] == 'behaviour_image'
    selected = get('behaviour_identity_scene')
    assert len(selected.observations) == 1
    assert selected.observations[0].labels == ['lying_on_floor_possible_fall']


def test_facing_success_does_not_require_original_behaviour_face(cfg):
    root, io = run(cfg, owners=(2,), identities=[[2]])
    assert root.context.completed == {2}
    assert io.counts['identify'] == 1
    assert any(e['event'] == 'post_interaction_identity' and e['source'] == 'facing_image'
               for e in root.context.events)


def test_unidentified_person_still_receives_service_without_false_identity(cfg):
    root, io = run(cfg, owners=(2,), identities=[[None], [None]])
    assert root.status == S.SUCCESS
    assert root.context.completed == set()
    assert root.context.serviced_roles == {2}
    assert io.counts['Extend assistance arm over person'] == 1
    assert io.counts['approach'] == 1
    assert any(e['event'] == 'interaction_completed_unidentified' for e in root.context.events)


def test_identity_is_not_inferred_from_behaviour_role(cfg):
    root, _ = run(cfg, owners=(2,), identities=[[3]])
    assert root.context.completed == {3}
    assert root.context.serviced_roles == {2}
    assert any(e['event'] == 'interaction_completed' and e['owner'] == 3 and e['role'] == 2
               for e in root.context.events)


def test_facing_image_requires_post_prompt_head_timestamp(cfg):
    context = Context(cfg)
    put('capture_after', 101.)
    put('reference_image', NS(width=4, height=4, header=NS(stamp=NS(sec=100, nanosec=0))))
    with pytest.raises(ValueError, match='早于'):
        context.prepare_facing_identity()


@pytest.mark.parametrize('key,value', [('navigation_attempts', 0), ('arm_attempts', True),
    ('retry_delay_seconds', -1), ('cancel_timeout_seconds', 0), ('arm_timeout_seconds', float('nan'))])
def test_invalid_recovery_settings_fail_preflight(cfg, key, value):
    cfg[key] = value
    with pytest.raises(ValueError, match=key):
        validate(cfg)
