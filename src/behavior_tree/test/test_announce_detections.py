"""Detection label translation and asynchronous announce contract."""

from concurrent.futures import Future
from types import SimpleNamespace

import py_trees
import pytest

from behavior_tree.TemplateNodes import BaseBehaviors
from behavior_tree.TemplateNodes.Audio import BtNode_AnnounceDetections
from behavior_tree.messages import TextToSpeech

Status = py_trees.common.Status


def detection(*labels, status=0, error_msg=""):
    return SimpleNamespace(
        status=status, error_msg=error_msg,
        objects=[SimpleNamespace(cls=label) for label in labels],
    )


@pytest.fixture(autouse=True)
def clear_blackboard():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


@pytest.fixture
def make_node(monkeypatch):
    monkeypatch.setattr(BaseBehaviors, "is_node_mocked", lambda name: False)

    def make(**kwargs):
        requests = []
        future = Future()
        client = SimpleNamespace(
            wait_for_service=lambda **kw: True,
            call_async=lambda request: (requests.append(request), future)[1],
        )
        connections = []
        ros_node = SimpleNamespace(create_client=lambda srv, endpoint: (
            connections.append((srv, endpoint)), client)[1])
        params = dict(
            name="AnnounceDetections", bb_source="detections",
            label_map={"cup": "杯子", "bottle": "瓶子", "dining table": "餐桌"},
            bb_text_key="spoken_text",
        )
        params.update(kwargs)
        node = BtNode_AnnounceDetections(**params)
        node.setup(node=ros_node)
        assert connections == [(TextToSpeech, node.service_name)]
        return node, requests, future
    return make


def test_mapping_order_dedup_and_tts_completion(make_node):
    node, requests, future = make_node()
    original = detection("bottle", "cup", "cup")
    py_trees.blackboard.Blackboard.set("detections", original)
    node.initialise()
    assert requests[-1].text == "我看到了瓶子、杯子。"
    assert py_trees.blackboard.Blackboard.get("spoken_text") == requests[-1].text
    assert py_trees.blackboard.Blackboard.get("detections") is original
    assert node.update() == Status.RUNNING
    future.set_result(SimpleNamespace(status=0))
    assert node.update() == Status.SUCCESS


def test_case_whitespace_multiword_and_unknown(make_node):
    node, requests, _ = make_node()
    py_trees.blackboard.Blackboard.set("detections", detection(" CUP ", "mug", "Dining Table"))
    node.initialise()
    assert requests[-1].text == "我看到了杯子、餐桌。"


def test_custom_sentence_and_duplicate_instances(make_node):
    node, requests, _ = make_node(
        prefix="检测到了", suffix="！", deduplicate=False, service_name="/test_announce")
    py_trees.blackboard.Blackboard.set("detections", detection("cup", "cup"))
    node.initialise()
    assert requests[-1].text == "检测到了杯子、杯子！"


@pytest.mark.parametrize("labels", [(), ("unknown",)])
def test_no_mapped_names_succeeds_silently(make_node, labels):
    node, requests, _ = make_node()
    py_trees.blackboard.Blackboard.set("detections", detection(*labels))
    node.initialise()
    assert node.update() == Status.SUCCESS
    assert requests == []
    assert py_trees.blackboard.Blackboard.get("spoken_text") == ""


def test_empty_message_option(make_node):
    node, requests, _ = make_node(empty_message="没有识别到已知物体。")
    py_trees.blackboard.Blackboard.set("detections", detection())
    node.initialise()
    assert requests[-1].text == "没有识别到已知物体。"


@pytest.mark.parametrize("result", [
    None, SimpleNamespace(), detection("cup", status=1, error_msg="camera unavailable"),
    SimpleNamespace(objects=[SimpleNamespace(cls=None)]),
])
def test_invalid_or_failed_detection_never_calls_tts(make_node, result):
    node, requests, _ = make_node()
    py_trees.blackboard.Blackboard.set("detections", result)
    node.initialise()
    assert node.update() == Status.FAILURE
    assert not requests
    assert py_trees.blackboard.Blackboard.get("spoken_text") == ""


def test_missing_blackboard_does_not_replay_previous_text(make_node):
    node, requests, _ = make_node()
    py_trees.blackboard.Blackboard.set("detections", detection("cup"))
    node.initialise()
    py_trees.blackboard.Blackboard.unset("detections")
    node.initialise()
    assert node.update() == Status.FAILURE
    assert len(requests) == 1
    assert node.response is None and node.announce_msg == ""
    assert py_trees.blackboard.Blackboard.get("spoken_text") == ""


def test_rereads_on_each_visit(make_node):
    node, requests, _ = make_node()
    for label in ["cup", "bottle"]:
        py_trees.blackboard.Blackboard.set("detections", detection(label))
        node.initialise()
    assert [r.text for r in requests] == ["我看到了杯子。", "我看到了瓶子。"]


@pytest.mark.parametrize("raises", [False, True])
def test_tts_failure_returns_failure(make_node, raises):
    node, _, future = make_node()
    py_trees.blackboard.Blackboard.set("detections", detection("cup"))
    node.initialise()
    if raises:
        future.set_exception(RuntimeError("service disconnected"))
    else:
        future.set_result(SimpleNamespace(status=1))
    assert node.update() == Status.FAILURE


def test_mock_prepares_same_chinese_without_service(make_node):
    node, requests, _ = make_node()
    node.mock_mode = True
    node.wait_for_keypress_in_mock = lambda: Status.SUCCESS
    py_trees.blackboard.Blackboard.set("detections", detection("cup"))
    node.initialise()
    assert node.announce_msg == "我看到了杯子。"
    assert node.update() == Status.SUCCESS
    assert not requests


@pytest.mark.parametrize("mapping", [
    {"cup": ""}, {"cup": None}, {1: "杯子"}, {"Cup": "杯子", "cup": "水杯"},
])
def test_invalid_dictionary_rejected(mapping):
    with pytest.raises(ValueError):
        BtNode_AnnounceDetections("BadMap", "detections", mapping)


def test_dictionary_is_copied_and_chinese_synonyms_deduped(make_node):
    mapping = {"cup": "杯子", "mug": "杯子"}
    node, requests, _ = make_node(label_map=mapping)
    mapping["cup"] = "错误"
    py_trees.blackboard.Blackboard.set("detections", detection("cup", "mug"))
    node.initialise()
    assert requests[-1].text == "我看到了杯子。"


def test_output_key_cannot_overwrite_detection_input():
    with pytest.raises(ValueError):
        BtNode_AnnounceDetections(
            "BadKey", "detections", {"cup": "杯子"}, bb_text_key="/detections")

