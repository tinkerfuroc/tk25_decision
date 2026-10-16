"""YOLO26 BT request and blackboard contract, without cameras or a GPU."""

import json
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

import py_trees
import pytest

from behavior_tree.TemplateNodes import BaseBehaviors
from behavior_tree.TemplateNodes.Vision import (
    BtNode_ScanForGeneralist, BtNode_ScanForYolo26,
)
from behavior_tree.messages import ObjectDetectionGeneralist


@pytest.fixture(autouse=True)
def clean_blackboard():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


@pytest.fixture
def make_node(monkeypatch):
    monkeypatch.setattr(BaseBehaviors, 'is_node_mocked', lambda name: False)

    def make(**kwargs):
        requests = []
        future = Future()
        client = SimpleNamespace(
            wait_for_service=lambda **kw: True,
            call_async=lambda request: (requests.append(request), future)[1],
        )
        calls = []
        ros_node = SimpleNamespace(create_client=lambda srv, name: (
            calls.append((srv, name)), client)[1])
        node = BtNode_ScanForYolo26(name='TestYolo26', **kwargs)
        node.setup(node=ros_node)
        assert calls == [(ObjectDetectionGeneralist, node.service_name)]
        return node, requests, future
    return make


def test_default_all_labels_and_flags(make_node):
    node, requests, _ = make_node()
    node.initialise()
    request = requests[-1]
    assert node.service_name == 'object_detection_yolo26'
    assert node.bb_key == 'yolo26_detection'
    assert node.read is False
    assert request.prompt == ''
    assert request.camera == 'orbbec'
    assert request.target_frame == ''
    assert request.sort_closest and not request.sort_highest
    assert request.return_segments
    assert not request.return_rgb_image and not request.return_depth_image
    assert not request.force_vlm_sam and not request.use_vlm_sam_fallback
    assert node.update() == py_trees.common.Status.RUNNING


def test_fixed_labels_and_all_request_options(make_node):
    node, requests, _ = make_node(
        object='cup . bottle', bb_key='selected', use_orbbec=False,
        transform_to_map=True, sort_closest=False, sort_highest=True,
        return_segments=False, return_rgb_image=True, return_depth_image=True,
        service_name='/custom_yolo26',
    )
    node.initialise()
    request = requests[-1]
    assert request.prompt == 'cup . bottle'
    assert request.camera == 'realsense'
    assert request.target_frame == 'map'
    assert request.sort_highest and not request.sort_closest
    assert request.return_rgb_image and request.return_depth_image
    assert not request.return_segments
    assert node.service_name == '/custom_yolo26'


def test_blackboard_prompt_is_reread_including_empty(make_node):
    node, requests, _ = make_node(bb_source='labels')
    for prompt in ['cup', '', 'person . bus']:
        py_trees.blackboard.Blackboard.set('labels', prompt)
        node.initialise()
        assert requests[-1].prompt == prompt
    assert node.read is True


@pytest.mark.parametrize('prompt', ['', 'bottle'])
def test_explicit_object_overrides_blackboard(make_node, prompt):
    node, requests, _ = make_node(object=prompt, bb_source='missing_key')
    node.initialise()
    assert requests[-1].prompt == prompt
    assert node.read is False


def test_complete_sorted_response_written_unchanged(make_node):
    node, _, future = make_node(bb_key='detections')
    node.initialise()
    objects = [SimpleNamespace(cls='bottle'), SimpleNamespace(cls='cup')]
    segments = [object(), object()]
    response = SimpleNamespace(
        status=0, objects=objects, segments=segments,
        rgb_image=object(), depth_image=object(), header=object(),
        detection_source='yolo', error_msg='',
    )
    future.set_result(response)
    assert node.update() == py_trees.common.Status.SUCCESS
    stored = py_trees.blackboard.Blackboard.get('detections')
    assert stored is response
    assert stored.objects is objects and stored.segments is segments


@pytest.mark.parametrize('error', ['no matches', 'Unknown checkpoint label: mug'])
def test_empty_and_unknown_results_fail(make_node, error):
    node, _, future = make_node(bb_key='detections')
    node.initialise()
    future.set_result(SimpleNamespace(
        status=1, objects=[], error_msg=error, detection_source='none'))
    assert node.update() == py_trees.common.Status.FAILURE
    assert error in node.feedback_message
    assert not py_trees.blackboard.Blackboard.exists('detections')


def test_missing_blackboard_prompt_fails_before_service(make_node):
    node, requests, _ = make_node(bb_source='missing_key')
    with pytest.raises(KeyError):
        node.initialise()
    assert not requests


def test_generalist_defaults_unchanged():
    node = BtNode_ScanForGeneralist(
        name='Generalist', bb_source=None, bb_key='generalist', object='cup')
    assert node.service_name == 'object_detection_generalist'
    assert node.use_vlm_sam_fallback is True


def test_mock_node_is_registered():
    config = Path(__file__).parents[1] / 'behavior_tree' / 'mock_config.json'
    nodes = json.loads(config.read_text())['mock_mode']['subsystems']['vision']['nodes']
    assert nodes['BtNode_ScanForYolo26'] == 'KEYPRESS'

