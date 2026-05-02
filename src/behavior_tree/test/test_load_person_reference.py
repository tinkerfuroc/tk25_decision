"""Unit tests for BtNode_LoadPersonReference (TemplateNodes/Vision.py)."""

import os

import cv2
import numpy as np
import py_trees
import pytest

from behavior_tree.TemplateNodes.Vision import BtNode_LoadPersonReference


def _write_fixture(tmp_path, *, image_w=4, image_h=4, description="A tall person in a red shirt."):
    img_path = tmp_path / "person.jpg"
    txt_path = tmp_path / "person.txt"

    bgr = np.zeros((image_h, image_w, 3), dtype=np.uint8)
    bgr[:, :, 0] = 255  # solid blue, just so encoded JPEG isn't degenerate
    assert cv2.imwrite(str(img_path), bgr)

    txt_path.write_text(description, encoding="utf-8")
    return str(img_path), str(txt_path)


@pytest.fixture(autouse=True)
def _clear_blackboard():
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()
    yield
    if callable(clear_fn):
        clear_fn()


def _make_node(image_path, description_path, suffix=""):
    return BtNode_LoadPersonReference(
        name=f"load{suffix}",
        image_path=image_path,
        description_path=description_path,
        bb_features_key=f"test_features{suffix}",
        bb_image_key=f"test_image{suffix}",
    )


def _read_blackboard(suffix=""):
    client = py_trees.blackboard.Client(name=f"reader{suffix}")
    client.register_key(key=f"test_features{suffix}", access=py_trees.common.Access.READ)
    client.register_key(key=f"test_image{suffix}", access=py_trees.common.Access.READ)
    features = getattr(client, f"test_features{suffix}")
    image = getattr(client, f"test_image{suffix}")
    return features, image


def test_loads_image_and_description_into_blackboard(tmp_path):
    img_path, txt_path = _write_fixture(tmp_path, image_w=8, image_h=6, description="hello world")
    node = _make_node(img_path, txt_path, suffix="_ok")

    status = node.tick_once()
    assert status is None or node.status == py_trees.common.Status.SUCCESS
    assert node.status == py_trees.common.Status.SUCCESS

    features, image = _read_blackboard(suffix="_ok")
    assert features == "hello world"
    assert image.width == 8
    assert image.height == 6
    assert image.encoding == "bgr8"


def test_missing_image_returns_failure(tmp_path):
    _, txt_path = _write_fixture(tmp_path)
    node = _make_node(str(tmp_path / "does_not_exist.jpg"), txt_path, suffix="_noimg")

    node.tick_once()
    assert node.status == py_trees.common.Status.FAILURE
    assert "image file not found" in node.feedback_message


def test_missing_description_returns_failure(tmp_path):
    img_path, _ = _write_fixture(tmp_path)
    node = _make_node(img_path, str(tmp_path / "does_not_exist.txt"), suffix="_notxt")

    node.tick_once()
    assert node.status == py_trees.common.Status.FAILURE
    assert "description file not found" in node.feedback_message


def test_empty_description_succeeds_with_empty_string(tmp_path):
    img_path, _ = _write_fixture(tmp_path)
    txt_path = tmp_path / "empty.txt"
    txt_path.write_text("", encoding="utf-8")
    node = _make_node(img_path, str(txt_path), suffix="_empty")

    node.tick_once()
    assert node.status == py_trees.common.Status.SUCCESS

    features, image = _read_blackboard(suffix="_empty")
    assert features == ""
    assert image.width > 0


def test_path_expanduser_is_applied(tmp_path, monkeypatch):
    img_path, txt_path = _write_fixture(tmp_path)
    monkeypatch.setenv("HOME", str(tmp_path))

    rel_img = "~/" + os.path.basename(img_path)
    rel_txt = "~/" + os.path.basename(txt_path)
    node = _make_node(rel_img, rel_txt, suffix="_tilde")

    node.tick_once()
    assert node.status == py_trees.common.Status.SUCCESS
