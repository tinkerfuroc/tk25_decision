import json
from pathlib import Path

BENCH = Path(__file__).resolve().parents[1] / "behavior_tree" / "mock_config.bench.json"
SHIPPED = BENCH.with_name("mock_config.json")
SIM = BENCH.with_name("mock_config.sim.json")
SIM_HYBRID = BENCH.with_name("mock_config.sim-hybrid.json")


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
    # importlib.reload mutates the already-imported `behavior_tree.config` module object IN
    # PLACE, so without restoring it, every later test (and any module that already did
    # `from ..config import is_full_mock_mode`) would keep seeing this test's bench-config
    # state -- order-dependent pollution. Reload back to the pre-test env in a finally.
    import importlib
    import behavior_tree.config as config
    monkeypatch.setenv("BT_MOCK_MODE", "true")
    monkeypatch.setenv("BT_MOCK_CONFIG", str(BENCH))
    try:
        importlib.reload(config)
        assert config.is_full_mock_mode() is True
    finally:
        monkeypatch.undo()
        importlib.reload(config)


def test_sim_hybrid_config_parses():
    cfg = json.loads(SIM_HYBRID.read_text())
    assert cfg["mock_mode"]["enabled"] is True


def test_sim_hybrid_config_only_mocks_manipulation():
    cfg = json.loads(SIM_HYBRID.read_text())
    subsystems = cfg["mock_mode"]["subsystems"]
    enabled = {name for name, sub in subsystems.items() if sub["enabled"] is True}
    assert enabled == {"manipulation"}
    assert cfg["keyboard_control"]["enabled"] is False


def test_sim_hybrid_config_manipulation_nodes_are_immediate_and_match_bench():
    sim_hybrid = json.loads(SIM_HYBRID.read_text())
    bench = json.loads(BENCH.read_text())
    hybrid_nodes = sim_hybrid["mock_mode"]["subsystems"]["manipulation"]["nodes"]
    bench_nodes = bench["mock_mode"]["subsystems"]["manipulation"]["nodes"]
    assert set(hybrid_nodes) == set(bench_nodes)
    assert all(mode == "IMMEDIATE" for mode in hybrid_nodes.values())


def test_sim_hybrid_config_other_subsystems_match_sim_config():
    sim = json.loads(SIM.read_text())
    sim_hybrid = json.loads(SIM_HYBRID.read_text())
    for name in sim["mock_mode"]["subsystems"]:
        if name == "manipulation":
            continue
        assert sim_hybrid["mock_mode"]["subsystems"][name] == sim["mock_mode"]["subsystems"][name]


def test_sim_hybrid_config_is_not_full_mock_mode(monkeypatch):
    import importlib
    import behavior_tree.config as config
    monkeypatch.setenv("BT_MOCK_MODE", "true")
    monkeypatch.setenv("BT_MOCK_CONFIG", str(SIM_HYBRID))
    try:
        importlib.reload(config)
        assert config.is_full_mock_mode() is False
    finally:
        monkeypatch.undo()
        importlib.reload(config)
