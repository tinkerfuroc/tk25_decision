import os

os.environ["BT_MOCK_MODE"] = "true"  # noqa: E402

import py_trees  # noqa: E402

from behavior_tree.PickAndPlace.custom_nodes import BtNode_DeadlineGuard  # noqa: E402

R = py_trees.common.Status.RUNNING
S = py_trees.common.Status.SUCCESS


def test_running_until_boundary_then_success():
    t = [0.0]
    g = BtNode_DeadlineGuard("g", budget_sec=10.0, clock=lambda: t[0])
    g.initialise()
    assert g.update() == R
    t[0] = 9.999
    assert g.update() == R
    t[0] = 10.0  # boundary fires
    assert g.update() == S
    t[0] = 11.0
    assert g.update() == S  # never FAILURE


def test_deadline_latched_in_initialise():
    t = [100.0]
    g = BtNode_DeadlineGuard("g", budget_sec=5.0, clock=lambda: t[0])
    g.initialise()  # latch deadline = 105 (not at construction)
    t[0] = 104.0
    assert g.update() == R
    t[0] = 105.0
    assert g.update() == S


def test_default_clock_is_monotonic_and_does_not_fire_early():
    g = BtNode_DeadlineGuard("g", budget_sec=1000.0)
    g.initialise()
    assert g.update() == R
