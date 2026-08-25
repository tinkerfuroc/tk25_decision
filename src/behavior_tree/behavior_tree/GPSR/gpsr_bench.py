"""GPSR command bench CLI: generate a corpus, run it through a tier, render the matrix.

    gpsr-bench gen   --seed 42 --per-template 3 --edge --out corpus-42.jsonl
    gpsr-bench tier0 --corpus corpus-42.jsonl --out gpsr_runs/bench/t0-42
    gpsr-bench tier1 --corpus corpus-42.jsonl --out gpsr_runs/bench/t1-42 --group-size 3
    gpsr-bench report --out gpsr_runs/bench/t1-42
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from behavior_tree.GPSR.bench.corpus import (
    EDGE_COMMANDS, generate_corpus, generate_sim_corpus, read_jsonl, write_jsonl,
)
from behavior_tree.GPSR.bench.report import BenchResult, write_report
from behavior_tree.GPSR.bench.tier0 import run_tier0
from behavior_tree.GPSR.bench.tier1 import run_tier1

HERE = Path(__file__).resolve().parent
DEFAULT_CONSTANTS = HERE / "constants.rcw2026.json"
DEFAULT_MOCK_CONFIG = HERE.parent / "mock_config.bench.json"


def _make_planner():
    from behavior_tree.GPSR.planner import GPSRPlanner
    return GPSRPlanner()


def _knowledge(constants: Path):
    from behavior_tree.GPSR import orchestrator, small_trees
    orchestrator.load_knowledge_from_constants(str(constants))
    return (set(small_trees.ACTION_FACTORIES),
            set(orchestrator.KNOWN_LOCATIONS) | set(orchestrator.START_LOCATION_ALIASES))


def _filter(entries, only_class: str | None):
    if not only_class:
        return list(entries)
    allowed = {c.strip().upper() for c in only_class.split(",")}
    return [e for e in entries if e.feasibility in allowed]


def _corpus_seed(entries):
    """The `gpsr-bench gen --seed` value isn't stored anywhere but each generated entry's
    own `seed` field (edge commands are hand-written and carry seed=-1); use the first
    non-edge entry's, or None for an edge-only / empty selection."""
    for e in entries:
        if e.template != "edge":
            return e.seed
    return None


def _finish(results, out: Path, corpus: Path, *, meta: dict | None = None) -> int:
    summary = write_report(results, out, corpus_path=corpus, meta=meta)
    print(summary.read_text())
    return 0 if results and all(r.verdict == "PASS" for r in results) else 1


def cmd_gen(args) -> int:
    if args.sim_feasible:
        if args.edge:
            raise SystemExit("gpsr-bench gen: --edge is not supported with --sim-feasible")
        templates = [t.strip() for t in args.templates.split(",")] if args.templates else None
        entries, skipped = generate_sim_corpus(Path(args.constants), seed=args.seed,
                                               count=args.count, templates=templates)
        header = {"_skipped": skipped, "_seed": args.seed, "_mode": "sim-feasible"}
        write_jsonl(entries, Path(args.out), header=header)
        print(f"wrote {len(entries)} sim-feasible commands to {args.out} (skipped={skipped})")
        return 0
    entries = generate_corpus(Path(args.constants), seed=args.seed, per_template=args.per_template)
    if args.edge:
        entries = entries + list(EDGE_COMMANDS)
    write_jsonl(entries, Path(args.out))
    print(f"wrote {len(entries)} commands to {args.out}")
    return 0


def cmd_tier0(args) -> int:
    entries = _filter(read_jsonl(Path(args.corpus)), args.only_class)
    known_actions, known_locations = _knowledge(Path(args.constants))
    # run_tier0 builds a fresh planner per entry via planner_factory and never touches the
    # `planner` positional argument when one is supplied -- constructing one here too would
    # be a throwaway (and, when not offline, an unused OpenAI client).
    results = run_tier0(entries, None, known_actions=known_actions,
                        known_locations=known_locations, timeout_s=args.timeout,
                        planner_factory=_make_planner)
    meta = {"tier": 0, "timeout_s": args.timeout, "only_class": args.only_class,
            "seed": _corpus_seed(entries)}
    return _finish(results, Path(args.out), Path(args.corpus), meta=meta)


def cmd_tier1(args) -> int:
    entries = _filter(read_jsonl(Path(args.corpus)), args.only_class)
    results = run_tier1(entries, group_size=args.group_size, timeout_s=args.timeout,
                        mock_config=Path(args.mock_config), constants=Path(args.constants),
                        plan_dir=Path(args.out) / "runs", live_llm=not args.offline_planner)
    meta = {"tier": 1, "group_size": args.group_size, "timeout_s": args.timeout,
            "live_llm": not args.offline_planner, "only_class": args.only_class,
            "seed": _corpus_seed(entries)}
    return _finish(results, Path(args.out), Path(args.corpus), meta=meta)


def cmd_report(args) -> int:
    data = json.loads((Path(args.out) / "report.json").read_text(encoding="utf-8"))
    results = [BenchResult(**r) for r in data["results"]]
    corpus = Path(data["corpus"]) if data.get("corpus") else None
    print(write_report(results, Path(args.out), corpus_path=corpus, meta=data.get("meta")).read_text())
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="gpsr-bench", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    g = sub.add_parser("gen", help="generate a seeded corpus")
    g.add_argument("--seed", type=int, required=True)
    g.add_argument("--per-template", type=int, default=3)
    g.add_argument("--constants", default=str(DEFAULT_CONSTANTS))
    g.add_argument("--edge", action="store_true", help="append the hand-written edge commands")
    g.add_argument("--sim-feasible", action="store_true",
                    help="generate a corpus excluding SIM_INFEASIBLE templates/followups")
    g.add_argument("--count", type=int, default=40, help="entry count (--sim-feasible only)")
    g.add_argument("--templates", default=None,
                    help="comma-separated template names to draw from (--sim-feasible only)")
    g.add_argument("--out", required=True)
    g.set_defaults(func=cmd_gen)

    for name, func in (("tier0", cmd_tier0), ("tier1", cmd_tier1)):
        t = sub.add_parser(name)
        t.add_argument("--corpus", required=True)
        t.add_argument("--out", required=True)
        t.add_argument("--constants", default=str(DEFAULT_CONSTANTS))
        t.add_argument("--only-class", default=None, help="comma-separated feasibility classes, e.g. A,B")
        t.add_argument("--timeout", type=float, default=180.0 if name == "tier0" else 300.0)
        if name == "tier1":
            t.add_argument("--group-size", type=int, default=3)
            t.add_argument("--mock-config", default=str(DEFAULT_MOCK_CONFIG))
            t.add_argument("--offline-planner", action="store_true", help="GPSR_OFFLINE_PLANNER=1 (no LLM)")
        t.set_defaults(func=func)

    r = sub.add_parser("report", help="re-render SUMMARY.md from report.json")
    r.add_argument("--out", required=True)
    r.set_defaults(func=cmd_report)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
