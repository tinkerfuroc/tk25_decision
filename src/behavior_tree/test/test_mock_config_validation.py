import json
from pathlib import Path

import pytest

from behavior_tree.config import MOCK_SUBSYSTEMS, get_config


_CONFIG_ROOT = Path(__file__).parents[1]


def _read_config(name):
    if name == "mock_config.json":
        path = _CONFIG_ROOT / "behavior_tree" / name
    else:
        path = _CONFIG_ROOT / "config" / name
    return json.loads(path.read_text())


def test_shipped_mock_configs_validate():
    config = get_config()
    for name in (
        "mock_config.json",
        "full_mock.json",
        "f4_mock_config.json",
        "vision_live_bench.json",
    ):
        assert config.validate_mock_config(_read_config(name))


def test_normalize_node_mode_handles_bool_and_numbers_before_string_lookup():
    config = get_config()
    assert config._normalize_node_mode(True) == "wait_keypress"
    assert config._normalize_node_mode(False) == "no_mock"
    assert config._normalize_node_mode(1) == "wait_keypress"
    assert config._normalize_node_mode(0) == "no_mock"


def test_unknown_mode_and_subsystem_are_rejected():
    config = get_config()
    bad_mode = {
        "mock_mode": {
            "subsystems": {
                "vision": {"enabled": True, "nodes": {"BtNode_X": "typo"}}
            }
        }
    }
    with pytest.raises(ValueError, match="Unsupported node mock mode"):
        config.validate_mock_config(bad_mode)

    bad_subsystem = {
        "mock_mode": {"subsystems": {"vison": {"enabled": True, "nodes": {}}}}
    }
    with pytest.raises(ValueError, match="unsupported mock subsystem"):
        config.validate_mock_config(bad_subsystem)


def test_full_mock_detection_distinguishes_bench_config(monkeypatch):
    config = get_config()
    for name, expected in (("full_mock.json", True), ("f4_mock_config.json", False)):
        path = _CONFIG_ROOT / "config" / name
        monkeypatch.setenv("BT_MOCK_MODE", "true")
        monkeypatch.setenv("BT_MOCK_CONFIG", str(path))
        config._load_mock_config(force=True)
        assert config.is_full_mock_mode() is expected

    monkeypatch.delenv("BT_MOCK_MODE")
    monkeypatch.delenv("BT_MOCK_CONFIG")
    config._load_mock_config(force=True)


def test_mock_subsystem_catalog_is_stable():
    assert MOCK_SUBSYSTEMS == (
        "vision",
        "manipulation",
        "navigation",
        "audio_input",
        "announcement",
        "mock_controls",
    )
