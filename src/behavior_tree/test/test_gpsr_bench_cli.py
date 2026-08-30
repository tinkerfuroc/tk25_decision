import json
import os
os.environ.setdefault("BT_MOCK_MODE", "true")
from pathlib import Path

from behavior_tree.GPSR import gpsr_bench
from behavior_tree.GPSR.bench.report import BenchResult

CONSTANTS = Path(__file__).resolve().parents[1] / "behavior_tree" / "GPSR" / "constants.rcw2026.json"


def test_gen_writes_a_corpus_with_every_template(tmp_path):
    out = tmp_path / "corpus.jsonl"
    assert gpsr_bench.main(["gen", "--seed", "1", "--per-template", "1", "--constants", str(CONSTANTS),
                            "--edge", "--out", str(out)]) == 0
    lines = [json.loads(l) for l in out.read_text().splitlines()]
    assert len({l["template"] for l in lines if l["template"] != "edge"}) == 22
    assert any(l["template"] == "edge" for l in lines)


def test_tier0_uses_injected_runner_and_writes_report(tmp_path, monkeypatch):
    corpus = tmp_path / "corpus.jsonl"
    gpsr_bench.main(["gen", "--seed", "1", "--per-template", "1", "--constants", str(CONSTANTS), "--out", str(corpus)])

    def fake_run(entries, planner, **kwargs):
        return [BenchResult(e.id, e.template, e.feasibility, 0, "PASS") for e in entries]

    monkeypatch.setattr(gpsr_bench, "run_tier0", fake_run)
    monkeypatch.setattr(gpsr_bench, "_make_planner", lambda: object())
    out = tmp_path / "t0"
    assert gpsr_bench.main(["tier0", "--corpus", str(corpus), "--out", str(out), "--constants", str(CONSTANTS)]) == 0
    assert (out / "SUMMARY.md").exists() and (out / "report.json").exists()


def test_resolved_source_path_matches_the_imported_package():
    # M-3 (round-3 fix review): a stale colcon install/ can shadow this
    # source checkout on sys.path even though every import still appears to
    # work -- this must resolve the ACTUAL imported behavior_tree.__file__,
    # not e.g. a hardcoded checkout-relative guess.
    import behavior_tree
    from pathlib import Path
    assert gpsr_bench._resolved_source_path() == str(Path(behavior_tree.__file__).resolve())


def test_main_prints_source_path_banner_before_running(tmp_path, capsys, monkeypatch):
    corpus = tmp_path / "corpus.jsonl"
    gpsr_bench.main(["gen", "--seed", "1", "--per-template", "1", "--constants", str(CONSTANTS), "--out", str(corpus)])
    capsys.readouterr()  # discard the gen command's own output

    def fake_run(entries, planner, **kwargs):
        return [BenchResult(e.id, e.template, e.feasibility, 0, "PASS") for e in entries]

    monkeypatch.setattr(gpsr_bench, "run_tier0", fake_run)
    monkeypatch.setattr(gpsr_bench, "_make_planner", lambda: object())
    out = tmp_path / "t0"
    gpsr_bench.main(["tier0", "--corpus", str(corpus), "--out", str(out), "--constants", str(CONSTANTS)])
    stdout = capsys.readouterr().out
    assert "[gpsr-bench] behavior_tree resolved from:" in stdout
    assert gpsr_bench._resolved_source_path() in stdout


def test_tier0_report_meta_carries_source_path(tmp_path, monkeypatch):
    corpus = tmp_path / "corpus.jsonl"
    gpsr_bench.main(["gen", "--seed", "1", "--per-template", "1", "--constants", str(CONSTANTS), "--out", str(corpus)])

    def fake_run(entries, planner, **kwargs):
        return [BenchResult(e.id, e.template, e.feasibility, 0, "PASS") for e in entries]

    monkeypatch.setattr(gpsr_bench, "run_tier0", fake_run)
    monkeypatch.setattr(gpsr_bench, "_make_planner", lambda: object())
    out = tmp_path / "t0"
    gpsr_bench.main(["tier0", "--corpus", str(corpus), "--out", str(out), "--constants", str(CONSTANTS)])
    report = json.loads((out / "report.json").read_text())
    assert report["meta"]["source_path"] == gpsr_bench._resolved_source_path()


def test_knowledge_includes_start_location_aliases():
    known_actions, known_locations = gpsr_bench._knowledge(CONSTANTS)
    assert "start_position" in known_locations
    assert "goto" in known_actions


def test_knowledge_includes_room_search_spot_aliases_i7():
    # I7 (round-3 adversarial review): the live orchestrator's
    # load_knowledge_from_constants aliases every search_spots room without
    # its own pose to its first spot (bedroom -> side_table_02, laundry_room
    # -> laundry_desk, ...) so goto(bedroom)/deliver(recipient_location=
    # laundry_room) resolve. gpsr_bench._knowledge must call that SAME
    # function (not a possible_poses-only reimplementation) so tier0/tier1
    # see the identical known-location set the sim orchestrator does --
    # otherwise tier0 rejects a plan the real robot accepts (corpus 029).
    known_actions, known_locations = gpsr_bench._knowledge(CONSTANTS)
    assert "bedroom" in known_locations
    assert "laundry_room" in known_locations


def test_knowledge_includes_command_point_when_constants_leave_it_unfilled_h1l4(tmp_path):
    # H-1-L4 (round-3 fix2 review): a constants file whose command_point is
    # still the `{}` placeholder leaves it out of KNOWN_LOCATIONS entirely
    # (H-1's `_try_pose` skips it) -- gpsr_bench._knowledge must still
    # include it in the known-locations set it hands to validate_plan,
    # same as the live orchestrator (KNOWN_LOCATION_VALIDATION_EXTRAS), or
    # tier0/tier1 reject goto(command_point) as unknown where the sim
    # doesn't.
    constants = json.loads(CONSTANTS.read_text())
    constants["possible_poses"]["command_point"] = {}
    unfilled = tmp_path / "constants_unfilled_cp.json"
    unfilled.write_text(json.dumps(constants))

    known_actions, known_locations = gpsr_bench._knowledge(unfilled)
    assert "command_point" in known_locations


def test_knowledge_calls_the_shared_orchestrator_loader(monkeypatch):
    # I7: guard against a future regression back to a separate/parallel
    # possible-poses-only loader -- _knowledge must be backed by
    # orchestrator.load_knowledge_from_constants itself (byte-identical
    # KNOWN_LOCATIONS/DEFAULT_OBJECT_LOCATIONS/ROOM_SEARCH_SPOTS by
    # construction), not a bench-local reimplementation.
    from behavior_tree.GPSR import orchestrator

    calls = []
    original = orchestrator.load_knowledge_from_constants

    def _spy(path):
        calls.append(path)
        return original(path)

    monkeypatch.setattr(orchestrator, "load_knowledge_from_constants", _spy)
    gpsr_bench._knowledge(CONSTANTS)
    assert calls == [str(CONSTANTS)]


def test_only_class_filters_entries(tmp_path, monkeypatch):
    corpus = tmp_path / "corpus.jsonl"
    gpsr_bench.main(["gen", "--seed", "1", "--per-template", "1", "--constants", str(CONSTANTS), "--out", str(corpus)])
    seen = []

    def fake_run(entries, planner, **kwargs):
        seen.extend(entries)
        return [BenchResult(e.id, e.template, e.feasibility, 0, "FAIL", "x") for e in entries]

    monkeypatch.setattr(gpsr_bench, "run_tier0", fake_run)
    monkeypatch.setattr(gpsr_bench, "_make_planner", lambda: object())
    rc = gpsr_bench.main(["tier0", "--corpus", str(corpus), "--out", str(tmp_path / "o"),
                          "--constants", str(CONSTANTS), "--only-class", "A"])
    assert rc == 1
    assert seen and all(e.feasibility == "A" for e in seen)
