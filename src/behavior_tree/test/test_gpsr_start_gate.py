import os

os.environ.setdefault("BT_MOCK_MODE", "true")

import py_trees  # noqa: E402
from behavior_tree.GPSR.gpsr_new import createGPSR  # noqa: E402


def test_root_starts_with_operator_gate():
    root = createGPSR()
    assert isinstance(root, py_trees.composites.Sequence)
    assert len(root.children) == 3
    assert root.children[0].__class__.__name__ == "BtNode_PressEnterToSucceed"


def test_constant_writer_still_second():
    root = createGPSR()
    assert isinstance(root.children[1], py_trees.composites.Parallel)
