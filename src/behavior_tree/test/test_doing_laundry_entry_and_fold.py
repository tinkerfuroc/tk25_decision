import os

os.environ.setdefault("BT_MOCK_MODE", "true")

import py_trees  # noqa: E402
from behavior_tree.DoingLaundry.laundry import (  # noqa: E402
    createDoingLaundry,
    foldClothingOnce,
)


def _names(root):
    return [b.name for b in root.iterate()]


def test_door_gate_ordering():
    """Enter gate -> door detection -> door-open announce -> nav to table."""
    names = _names(createDoingLaundry())
    gate = names.index("Wait for operator to start")
    door = names.index("Door detection")
    announce_open = names.index("Announce door open")
    nav = names.index("Navigate to laundry area")
    assert gate < door < announce_open < nav


def test_door_detection_is_retried():
    root = createDoingLaundry()
    retry = next(b for b in root.iterate() if b.name == "Retry door detection")
    assert isinstance(retry, py_trees.decorators.Retry)
    assert retry.children[0].name == "Door detection"


def test_fold_announce_references_screen():
    fold = foldClothingOnce()
    announce = next(
        b for b in fold.iterate() if b.name == "Announce folding start"
    )
    assert "lay the shirt out in the manner as shown on my screen" in announce.given_msg


def test_fold_wait_extended_to_ten_seconds():
    fold = foldClothingOnce()
    timer = next(
        b for b in fold.iterate() if b.name == "Wait for clothing to be laid flat"
    )
    assert timer.duration == 10.0
