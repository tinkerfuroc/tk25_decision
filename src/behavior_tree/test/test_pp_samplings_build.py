import os
os.environ.setdefault("BT_MOCK_MODE", "true")
import py_trees
from behavior_tree.PickAndPlace import samplings as S


def test_all_factory_builders_return_a_tree():
    for builder in (
        S._scan_place_factory,
        S._categorize_factory,
        S._cleanup_loop_factory,
        S._breakfast_factory,
    ):
        root = builder()
        assert isinstance(root, py_trees.behaviour.Behaviour)


def test_mains_are_callable_attrs():
    for name in ("main_scan_place", "main_categorize", "main_cleanup_loop", "main_breakfast"):
        assert callable(getattr(S, name))
