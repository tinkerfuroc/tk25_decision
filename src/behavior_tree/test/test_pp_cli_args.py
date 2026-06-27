import os
os.environ.setdefault("BT_MOCK_MODE", "true")
from behavior_tree.PickAndPlace.cli import _build_parser


def test_place_policy_default_is_vlm():
    args = _build_parser().parse_args([])
    assert args.place_policy == "vlm"


def test_place_policy_accepts_hardcoded():
    args = _build_parser().parse_args(["--place-policy", "hardcoded"])
    assert args.place_policy == "hardcoded"


def test_place_policy_rejects_unknown():
    import pytest
    with pytest.raises(SystemExit):
        _build_parser().parse_args(["--place-policy", "vision"])
