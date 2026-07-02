"""Unit tests for BtNode_FeatureMatching's trim_first_person seams.

Spec: docs/superpowers/specs/2026-07-02-hri-trim-host-from-matching-design.md.
The host occupies KEY_PERSONS[0] but must not enter the matching request;
the response centroids are re-padded with None at index 0 so
KEY_PERSON_CENTROIDS stays index-aligned with KEY_PERSONS (TurnTo/PointTo
target_id=1/2 need no re-indexing and never dereference index 0).
"""
from behavior_tree.TemplateNodes.Vision import BtNode_FeatureMatching


F = ['host-features', 'guest1-features', 'guest2-features']
I = ['host-img', 'guest1-img', 'guest2-img']


def test_apply_trims_first_only():
    f, i = BtNode_FeatureMatching._apply_trims(F, I, True, False)
    assert f == ['guest1-features', 'guest2-features']
    assert i == ['guest1-img', 'guest2-img']


def test_apply_trims_last_only_matches_receptionist_behavior():
    f, i = BtNode_FeatureMatching._apply_trims(F, I, False, True)
    assert f == ['host-features', 'guest1-features']
    assert i == ['host-img', 'guest1-img']


def test_apply_trims_first_and_last_compose():
    f, i = BtNode_FeatureMatching._apply_trims(F, I, True, True)
    assert f == ['guest1-features']
    assert i == ['guest1-img']


def test_apply_trims_default_noop():
    f, i = BtNode_FeatureMatching._apply_trims(F, I, False, False)
    assert f == F and i == I


def test_apply_trims_empty_lists_are_safe():
    assert BtNode_FeatureMatching._apply_trims([], [], True, True) == ([], [])


def test_pad_centroids_prepends_none_when_trimmed():
    out = BtNode_FeatureMatching._pad_centroids(['c1', 'c2'], True)
    assert out == [None, 'c1', 'c2']


def test_pad_centroids_noop_when_not_trimmed():
    assert BtNode_FeatureMatching._pad_centroids(['c0', 'c1'], False) == ['c0', 'c1']
