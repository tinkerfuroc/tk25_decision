"""Task P: wire the sim referee-handoff actuation into ex_machina's grasp
fallback (small, flag-gated).

SIM f0dff4c added referee actuation: publish the target object's SEMANTIC
name on ``/sim/referee/hand_object``; the sim resolves it to the spawned
entity, teleports it to the TCP, and acks on
``/sim/referee/hand_object_result`` with
``{"ok": true|false, "entity": ..., "xyz"|"error": ...}``. This makes the
ex_machina referee-fallback grasp physically real in sim so the
delivered/placed gates verify honestly (referee_assisted is a legitimate
PASS path).

``BtNode_RefereeHandObject`` is gated by ``GPSR_SIM_REFEREE_HANDOFF``
(idiom matches ``_sim_identity_relaxed_enabled()``): flag off (default) it
is completely dead -- SUCCESS immediately, no ROS entities created, so the
ex_machina branch is byte-identical to before this task. Flag on, it
publishes the target object name and waits (RUNNING) for the ack up to a
timeout, always resolving to SUCCESS (never FAILs the branch -- nack /
timeout / malformed ack all degrade gracefully so the flow proceeds to
close-on-air and fails honestly at the gates).
"""
from __future__ import annotations

import json

import py_trees
from py_trees.common import Access, Status

from behavior_tree.GPSR import small_trees
from behavior_tree.GPSR.small_trees import bb_keys


class _FakeMsg:
    def __init__(self, data: str):
        self.data = data


class _FakePublisher:
    def __init__(self):
        self.published: list[str] = []

    def publish(self, msg) -> None:
        self.published.append(msg.data)


class _FakeLogger:
    def info(self, *a, **k):
        pass

    def warning(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass

    def debug(self, *a, **k):
        pass


class _FakeNode:
    """Records publisher/subscription creation; hands the subscription
    callback back to the test so it can inject an ack."""

    def __init__(self):
        self.publishers: dict[str, _FakePublisher] = {}
        self.subscriptions: dict[str, object] = {}

    def create_publisher(self, msg_type, topic, qos=None):
        pub = _FakePublisher()
        self.publishers[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic, callback, qos=None):
        self.subscriptions[topic] = callback
        return object()

    def get_clock(self):
        raise AssertionError("get_clock() should not be needed by this node")

    def get_logger(self):
        return _FakeLogger()


class _RaisingNode:
    """A node stub that blows up if any ROS entity is created -- used to
    prove the flag-off path is completely dead."""

    def create_publisher(self, *a, **k):
        raise AssertionError("create_publisher() must not be called when the flag is off")

    def create_subscription(self, *a, **k):
        raise AssertionError("create_subscription() must not be called when the flag is off")


def _seed_target_object_name(name) -> None:
    py_trees.blackboard.Blackboard.clear()
    writer = py_trees.blackboard.Client(name="seed")
    writer.register_key(bb_keys.TARGET_OBJECT_NAME, access=Access.WRITE)
    writer.set(bb_keys.TARGET_OBJECT_NAME, name, overwrite=True)


def _build_and_setup(fake_node):
    node = small_trees.BtNode_RefereeHandObject("referee handoff")
    py_trees.trees.BehaviourTree(node).setup(node=fake_node)
    return node, node


# ---------------------------------------------------------------------------
# Flag off -- completely dead
# ---------------------------------------------------------------------------

def test_flag_off_succeeds_instantly_with_no_ros_entities(monkeypatch):
    monkeypatch.delenv("GPSR_SIM_REFEREE_HANDOFF", raising=False)
    _seed_target_object_name("coke")

    node, tree = _build_and_setup(_RaisingNode())
    assert node._publisher is None
    assert node._subscription is None

    tree.tick_once()

    assert node.status is Status.SUCCESS


# ---------------------------------------------------------------------------
# Flag on
# ---------------------------------------------------------------------------

def test_flag_on_ok_ack_succeeds_with_entity_in_feedback(monkeypatch):
    monkeypatch.setenv("GPSR_SIM_REFEREE_HANDOFF", "1")
    _seed_target_object_name("coke")

    fake_node = _FakeNode()
    node, tree = _build_and_setup(fake_node)

    tree.tick_once()
    assert node.status is Status.RUNNING
    topic = small_trees.BtNode_RefereeHandObject.HAND_OBJECT_TOPIC
    assert fake_node.publishers[topic].published == ["coke"]

    ack_cb = fake_node.subscriptions[small_trees.BtNode_RefereeHandObject.HAND_OBJECT_RESULT_TOPIC]
    ack_cb(_FakeMsg(json.dumps({"ok": True, "entity": "coke_1", "xyz": [0.1, 0.2, 0.3]})))

    tree.tick_once()
    assert node.status is Status.SUCCESS
    assert "coke_1" in node.feedback_message


def test_flag_on_nack_ack_succeeds_with_error_in_feedback(monkeypatch):
    monkeypatch.setenv("GPSR_SIM_REFEREE_HANDOFF", "1")
    _seed_target_object_name("fanta")

    fake_node = _FakeNode()
    node, tree = _build_and_setup(fake_node)

    tree.tick_once()
    assert node.status is Status.RUNNING

    ack_cb = fake_node.subscriptions[small_trees.BtNode_RefereeHandObject.HAND_OBJECT_RESULT_TOPIC]
    ack_cb(_FakeMsg(json.dumps({"ok": False, "error": "ambiguous"})))

    tree.tick_once()
    assert node.status is Status.SUCCESS
    assert "ambiguous" in node.feedback_message


def test_flag_on_no_ack_runs_until_timeout_then_succeeds(monkeypatch):
    monkeypatch.setenv("GPSR_SIM_REFEREE_HANDOFF", "1")
    monkeypatch.setenv("GPSR_SIM_REFEREE_HANDOFF_TIMEOUT_S", "3.0")
    _seed_target_object_name("coke")

    fake_node = _FakeNode()
    node, tree = _build_and_setup(fake_node)

    clock = {"t": 1000.0}
    monkeypatch.setattr(small_trees.time, "monotonic", lambda: clock["t"])

    tree.tick_once()
    assert node.status is Status.RUNNING

    clock["t"] += 1.0
    tree.tick_once()
    assert node.status is Status.RUNNING

    clock["t"] += 2.5  # now 3.5s elapsed -- past the 3.0s timeout
    tree.tick_once()
    assert node.status is Status.SUCCESS
    assert "no referee ack" in node.feedback_message


def test_flag_on_malformed_ack_json_treated_as_nack(monkeypatch):
    monkeypatch.setenv("GPSR_SIM_REFEREE_HANDOFF", "1")
    _seed_target_object_name("coke")

    fake_node = _FakeNode()
    node, tree = _build_and_setup(fake_node)

    tree.tick_once()
    assert node.status is Status.RUNNING

    ack_cb = fake_node.subscriptions[small_trees.BtNode_RefereeHandObject.HAND_OBJECT_RESULT_TOPIC]
    ack_cb(_FakeMsg("not-json{{{"))

    tree.tick_once()
    assert node.status is Status.SUCCESS


def test_flag_on_unset_target_object_name_succeeds_without_publish(monkeypatch):
    monkeypatch.setenv("GPSR_SIM_REFEREE_HANDOFF", "1")
    py_trees.blackboard.Blackboard.clear()
    writer = py_trees.blackboard.Client(name="seed")
    writer.register_key(bb_keys.TARGET_OBJECT_NAME, access=Access.WRITE)
    writer.set(bb_keys.TARGET_OBJECT_NAME, "", overwrite=True)

    fake_node = _FakeNode()
    node, tree = _build_and_setup(fake_node)

    tree.tick_once()

    assert node.status is Status.SUCCESS
    pub = fake_node.publishers.get(small_trees.BtNode_RefereeHandObject.HAND_OBJECT_TOPIC)
    assert pub is None or pub.published == []


# ---------------------------------------------------------------------------
# Tree structure
# ---------------------------------------------------------------------------

def test_ex_machina_contains_referee_hand_object_between_wait_and_close_gripper():
    tree = small_trees.create_grasp()
    assert isinstance(tree, py_trees.composites.Selector)
    _, ex_machina = tree.children

    names = [child.name for child in ex_machina.children]
    wait_idx = names.index("wait")
    close_idx = names.index("close gripper")
    assert close_idx == wait_idx + 2
    inserted = ex_machina.children[wait_idx + 1]
    assert isinstance(inserted, small_trees.BtNode_RefereeHandObject)
