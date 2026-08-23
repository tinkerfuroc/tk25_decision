import json
from pathlib import Path

BENCH = Path(__file__).resolve().parents[1] / "behavior_tree" / "mock_config.bench.json"
SHIPPED = BENCH.with_name("mock_config.json")


def _nodes(cfg):
    for sub in cfg["mock_mode"]["subsystems"].values():
        yield from sub.get("nodes", {}).items()


def test_bench_config_mocks_every_subsystem_immediately():
    cfg = json.loads(BENCH.read_text())
    assert cfg["mock_mode"]["enabled"] is True
    assert all(sub["enabled"] is True for sub in cfg["mock_mode"]["subsystems"].values())
    assert all(mode == "IMMEDIATE" for _, mode in _nodes(cfg)), dict(_nodes(cfg))


def test_bench_config_covers_the_same_nodes_as_the_shipped_config():
    bench = json.loads(BENCH.read_text())
    shipped = json.loads(SHIPPED.read_text())
    assert set(bench["mock_mode"]["subsystems"]) == set(shipped["mock_mode"]["subsystems"])
    assert {n for n, _ in _nodes(bench)} == {n for n, _ in _nodes(shipped)}


def test_loader_reports_full_mock_for_bench_config(monkeypatch):
    monkeypatch.setenv("BT_MOCK_MODE", "true")
    monkeypatch.setenv("BT_MOCK_CONFIG", str(BENCH))
    import importlib
    import behavior_tree.config as config
    importlib.reload(config)
    assert config.is_full_mock_mode() is True
