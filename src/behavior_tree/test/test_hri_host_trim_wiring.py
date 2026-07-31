"""Composition locks for HRI host-trim wiring (spec 2026-07-02).

Walks the factory-built subtrees and asserts the wiring, so regressions
are caught without ROS or hardware: the intro's matching leaf must trim
the host; createWriteHostInfo must no longer contain the disk-reference
loader (host features are unknown beforehand) but must still seed the
blackboard keys BtNode_CombinePerson hard-reads.
"""
import behavior_tree.HRI.hri as hri
from behavior_tree.nodes.Vision import (
    BtNode_FeatureMatching,
    BtNode_LoadPersonReference,
)


def _iter_tree(root):
    yield root
    for child in getattr(root, 'children', []):
        yield from _iter_tree(child)


def test_two_way_introduction_trims_host_from_matching():
    tree = hri.createTwoWayIntroduction()
    matchers = [n for n in _iter_tree(tree)
                if isinstance(n, BtNode_FeatureMatching)]
    assert matchers, 'intro tree lost its FeatureMatching leaf'
    for m in matchers:
        assert m.trim_first_person is True
        assert m.trim_last_person is False


def test_write_host_info_has_no_disk_reference_loader():
    tree = hri.createWriteHostInfo()
    loaders = [n for n in _iter_tree(tree)
               if isinstance(n, BtNode_LoadPersonReference)]
    assert loaders == [], 'host reference files must no longer be required'


def test_write_host_info_seeds_empty_host_media():
    # BtNode_CombinePerson hard-reads KEY_HOST_FEATURES/KEY_HOST_IMAGE
    # (KeyError if never written); the seeds replace the dropped loader.
    tree = hri.createWriteHostInfo()
    names = [n.name for n in _iter_tree(tree)]
    assert 'Write empty host features' in names
    assert 'Write empty host image' in names
