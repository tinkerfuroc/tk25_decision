import os

os.environ.setdefault("BT_MOCK_MODE", "true")

import py_trees  # noqa: E402

from behavior_tree.Restaurant.approach_narrated import (  # noqa: E402
    APPROACH_NARRATION_INTERVAL_SEC,
    APPROACH_NARRATION_LINES,
    createApproachCustomerNarrated,
)


def _iter(root):
    return list(root.iterate())


def _find(root, pred):
    return next((n for n in root.iterate() if pred(n)), None)


def test_narration_lines_are_three_short_lines():
    assert APPROACH_NARRATION_LINES == [
        "Approaching the customer.",
        "Planning my route.",
        "Adjusting my approach.",
    ]
    assert APPROACH_NARRATION_INTERVAL_SEC == 5.0
    assert all(len(line) <= 30 for line in APPROACH_NARRATION_LINES)


def test_root_is_selector_with_two_paths():
    root = createApproachCustomerNarrated()
    assert isinstance(root, py_trees.composites.Selector)
    assert len(root.children) == 2


def test_approach_targets_stable_server():
    root = createApproachCustomerNarrated()
    approach = _find(root, lambda n: n.__class__.__name__ == "BtNode_Approach")
    assert approach is not None
    assert approach.action_name == "go_to_approach_stable"


def test_parallel_success_gated_only_on_the_approach():
    root = createApproachCustomerNarrated()
    parallel = _find(root, lambda n: isinstance(n, py_trees.composites.Parallel))
    approach = _find(root, lambda n: n.__class__.__name__ == "BtNode_Approach")
    assert parallel is not None
    assert isinstance(
        parallel.policy, py_trees.common.ParallelPolicy.SuccessOnSelected)
    assert parallel.policy.children == [approach]


def test_narrator_loops_all_lines_and_cannot_fail():
    root = createApproachCustomerNarrated()
    msgs = [n.given_msg for n in _iter(root)
            if n.__class__.__name__ == "BtNode_Announce"]
    for line in APPROACH_NARRATION_LINES:
        assert line in msgs
    timers = [n for n in _iter(root) if isinstance(n, py_trees.timers.Timer)]
    assert len(timers) == len(APPROACH_NARRATION_LINES)
    assert _find(root, lambda n: isinstance(
        n, py_trees.decorators.SuccessIsRunning)) is not None
    fis = [n for n in _iter(root)
           if isinstance(n, py_trees.decorators.FailureIsSuccess)]
    assert len(fis) == len(APPROACH_NARRATION_LINES)


def test_collect_one_order_uses_narrated_stable_approach():
    from behavior_tree.Restaurant.order_intake_items import (
        createCollectOneOrderItems,
    )
    root = createCollectOneOrderItems()
    approach = _find(root, lambda n: n.__class__.__name__ == "BtNode_Approach")
    assert approach is not None
    assert approach.action_name == "go_to_approach_stable"
