"""Bench result model and the template x tier pass matrix."""
from __future__ import annotations

import json
import subprocess
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

VERDICTS = ("PASS", "FAIL", "REFEREE", "TIMEOUT", "ERROR")
PHYSICAL_VERDICTS = ("PASS", "FAIL", "REFEREE", "UNKNOWN")
CLASSES = ("A", "B", "C")


def _clean_pass(result: BenchResult) -> bool:
    """Whether one result proves both planner and physical success.

    Tier-2 results carry both verdict fields.  A legacy tier-0/tier-1 result has no
    physical field and is therefore counted by its ordinary planner verdict only;
    this keeps the report useful for those tiers while making a physical-truth
    streak impossible to claim from an old, planner-only tier-2 report.
    """
    if result.verdict != "PASS":
        return False
    # Tier 2 is the physical-truth tier.  A pre-truth report can contain a
    # plain planner PASS but has no evidence for the second half of the claim.
    if result.tier == 2:
        return result.planner_verdict == "PASS" and result.physical_verdict == "PASS"
    if result.planner_verdict is None and result.physical_verdict is None:
        return True
    return result.planner_verdict == "PASS" and result.physical_verdict == "PASS"


def pass_streak(results: Iterable[BenchResult], required: int | None = None) -> dict[str, Any]:
    """Return auditable consecutive clean-pass counts in result order.

    ``required`` is optional so generic tier reports can expose the measured streak
    without imposing a gate.  When provided, ``met`` records whether that exact
    requirement was reached.  A REFEREE result, planner failure, physical failure,
    timeout, or missing physical verdict breaks the streak.
    """
    ordered = list(results)
    longest = trailing = current = 0
    first_failure: int | None = None
    clean_count = 0
    for index, result in enumerate(ordered):
        if _clean_pass(result):
            clean_count += 1
            current += 1
            longest = max(longest, current)
        else:
            if first_failure is None:
                first_failure = index
            current = 0
    for result in reversed(ordered):
        if _clean_pass(result):
            trailing += 1
        else:
            break
    return {
        "required": required,
        "total": len(ordered),
        "clean_passes": clean_count,
        "longest": longest,
        "trailing": trailing,
        "first_failure_index": first_failure,
        "first_failure_entry_id": (ordered[first_failure].entry_id
                                    if first_failure is not None else None),
        "met": (longest >= required) if required is not None else None,
    }


@dataclass
class BenchResult:
    entry_id: str
    template: str
    feasibility: str
    tier: int
    verdict: str
    detail: str = ""
    seconds: float = 0.0
    plan: list[str] = field(default_factory=list)
    # K3 (task-K, live-manipulation sim findings, F2): True when any of this
    # run's step_methods (bench/events.py TaskResult.step_methods) was
    # "referee_fallback" -- surfaces a referee-assisted grasp/step in the
    # per-run record instead of it hiding behind an indistinguishable PASS.
    referee_assisted: bool = False
    # Physical-truth verdict (bench/truth.py, tier2 only). ``planner_verdict`` is the
    # executor/planner-gate verdict that ``verdict`` used to be; ``physical_verdict`` is
    # PASS/FAIL/REFEREE/UNKNOWN from sim physics truth (``run_truth.json``);
    # ``physical_checks`` the per-postcondition evidence; ``physical_detail`` the
    # human-readable why. ``verdict`` is their combination (``truth.combine_verdicts``).
    # All default to "absent" so report.json files written before this field existed
    # (and tier0/tier1 results) still load and render as planner-only.
    planner_verdict: str | None = None
    physical_verdict: str | None = None
    physical_checks: list[dict] = field(default_factory=list)
    physical_detail: str = ""
    # Human-readable planner/executor-gate evidence.  Tier-2 ``detail`` may also
    # contain physical-truth evidence for backwards compatibility; keep this
    # field separate so reports and the viewer never show physical failures as
    # planner failures.
    planner_detail: str = ""


def matrix(results: Iterable[BenchResult]) -> dict[tuple[str, str], dict[int, tuple[int, int]]]:
    """Pass/total per (template, feasibility class) per tier.

    Keyed by (template, class) rather than template alone: feasibility is per ENTRY (the
    worst of template + follow-ups), so a template with mixed-class entries would otherwise
    have its class column arbitrarily decided by whichever entry's result was folded last.
    """
    table: dict[tuple[str, str], dict[int, list[int]]] = defaultdict(lambda: defaultdict(lambda: [0, 0]))
    for r in results:
        cell = table[(r.template, r.feasibility)][r.tier]
        cell[1] += 1
        if r.verdict == "PASS":
            cell[0] += 1
    return {k: {tier: (p, n) for tier, (p, n) in tiers.items()} for k, tiers in table.items()}


def class_totals(results: Iterable[BenchResult]) -> dict[int, dict[str, tuple[int, int]]]:
    """Pass/total per feasibility class per tier -- the spec's own success-criterion figure
    (\">=95% of class A+B corpus commands produce a validator-clean plan\") is otherwise not
    readable off the report at all."""
    table: dict[int, dict[str, list[int]]] = defaultdict(lambda: defaultdict(lambda: [0, 0]))
    for r in results:
        cell = table[r.tier][r.feasibility]
        cell[1] += 1
        if r.verdict == "PASS":
            cell[0] += 1
    return {t: {c: (p, n) for c, (p, n) in classes.items()} for t, classes in table.items()}


def _ab_pass_rate(classes: dict[str, tuple[int, int]]) -> tuple[int, int, float]:
    passed = sum(classes.get(c, (0, 0))[0] for c in ("A", "B"))
    total = sum(classes.get(c, (0, 0))[1] for c in ("A", "B"))
    pct = (100.0 * passed / total) if total else 0.0
    return passed, total, pct


def _git_commit() -> str | None:
    """Best-effort `git rev-parse HEAD` of the repo this file lives in -- never raises."""
    try:
        out = subprocess.run(["git", "rev-parse", "HEAD"], cwd=str(Path(__file__).resolve().parent),
                             capture_output=True, text=True, timeout=5)
        return out.stdout.strip() if out.returncode == 0 else None
    except Exception:  # noqa: BLE001 - metadata only, never fail the run over it
        return None


def runs_section(results: Iterable[BenchResult]) -> str:
    """Render the per-run listing for SUMMARY.md.

    tier2 stores its contact sheet's relative path inside ``BenchResult.detail`` as a
    ``| sheet=<relpath>`` suffix (no schema change); when present it is rendered as a
    markdown link, otherwise the bullet is just the id and verdict.
    """
    lines = ["## Runs", ""]
    for r in results:
        sheet = None
        if " | sheet=" in r.detail:
            _, sheet = r.detail.rsplit(" | sheet=", 1)
        bullet = f"- {r.entry_id} **{r.verdict}**"
        if sheet:
            bullet += f" — [sheet]({sheet})"
        if r.referee_assisted:
            bullet += " _(referee-assisted)_"
        if r.planner_verdict is not None or r.physical_verdict is not None:
            bullet += f" (planner {r.planner_verdict or '-'} / physical {r.physical_verdict or '-'})"
        lines.append(bullet)
    return "\n".join(lines) + "\n"


def physical_section(results: Iterable[BenchResult]) -> str:
    """Render the per-run planner-vs-physical table for SUMMARY.md (empty string when no
    result carries a physical-truth verdict -- tier0/tier1 and pre-truth tier2 reports)."""
    results = [r for r in results if r.planner_verdict is not None or r.physical_verdict is not None]
    if not results:
        return ""
    lines = ["## Physical truth", "",
             "| run | verdict | planner | physical | physical detail |",
             "|---|---|---|---|---|"]
    for r in results:
        detail = (r.physical_detail or "").replace("|", "\\|").replace("\n", " ")
        lines.append(f"| {r.entry_id} | **{r.verdict}** | {r.planner_verdict or '-'} | "
                     f"{r.physical_verdict or '-'} | {detail} |")
    return "\n".join(lines) + "\n"


def write_report(results: Iterable[BenchResult], out_dir: Path, *, corpus_path: Path | None = None,
                 meta: dict[str, Any] | None = None,
                 required_streak: int | None = None) -> Path:
    results = list(results)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    streak = pass_streak(results, required=required_streak)
    tiers = sorted({r.tier for r in results})
    totals = {str(t): dict(Counter(r.verdict for r in results if r.tier == t)) for t in tiers}
    report_meta = dict(meta) if meta else {}
    report_meta.setdefault("timestamp", datetime.now(timezone.utc).isoformat())
    report_meta.setdefault("commit", _git_commit())
    report_meta.setdefault("corpus", str(corpus_path) if corpus_path else None)
    (out_dir / "report.json").write_text(json.dumps({
        "corpus": str(corpus_path) if corpus_path else None,
        "meta": report_meta,
        "totals": totals,
        "streak": streak,
        "results": [asdict(r) for r in results],
    }, indent=2) + "\n", encoding="utf-8")

    m = matrix(results)
    by_class = class_totals(results)
    lines = ["# GPSR bench summary", ""]
    if corpus_path:
        lines += [f"Corpus: `{corpus_path}`", ""]
    if report_meta:
        meta_bits = ", ".join(f"{k}={v}" for k, v in report_meta.items() if v is not None)
        lines += [f"Run: {meta_bits}", ""]
    header = "| template | class | " + " | ".join(f"T{t}" for t in tiers) + " |"
    lines += [header, "|" + "---|" * (2 + len(tiers))]
    for template, klass in sorted(m, key=lambda tc: (tc[1], tc[0])):
        cells = [f"{m[(template, klass)][t][0]}/{m[(template, klass)][t][1]}" if t in m[(template, klass)] else "-"
                for t in tiers]
        lines.append(f"| {template} | {klass} | " + " | ".join(cells) + " |")
    lines += ["", "## Totals", ""]
    for t in tiers:
        lines.append(f"- T{t}: " + ", ".join(f"{v} {totals[str(t)].get(v, 0)}" for v in VERDICTS))
        tier_results = [r for r in results if r.tier == t]
        planner = Counter(r.planner_verdict for r in tier_results if r.planner_verdict is not None)
        physical = Counter(r.physical_verdict for r in tier_results if r.physical_verdict is not None)
        if planner:
            lines.append(f"- T{t} planner: " + ", ".join(f"{v} {planner.get(v, 0)}" for v in VERDICTS))
        if physical:
            lines.append(f"- T{t} physical: " + ", ".join(
                f"{v} {physical.get(v, 0)}" for v in PHYSICAL_VERDICTS))
    lines += ["", "## Totals by class", ""]
    for t in tiers:
        classes = by_class.get(t, {})
        lines.append(f"T{t}:")
        for c in CLASSES:
            p, n = classes.get(c, (0, 0))
            lines.append(f"- class {c}: {p}/{n}")
        ab_p, ab_n, ab_pct = _ab_pass_rate(classes)
        lines.append(f"- Class A+B pass rate: {ab_p}/{ab_n} ({ab_pct:.0f} %)")
    lines += ["", "## Failures", ""]
    for r in results:
        if r.verdict != "PASS":
            lines.append(f"- T{r.tier} `{r.entry_id}` **{r.verdict}** — {r.detail}")
    lines += ["", "## Consecutive clean passes", "",
              "A clean pass requires planner `PASS` and physical-truth `PASS` when those "
              "fields are present; `REFEREE` breaks the streak.",
              f"- Clean passes: {streak['clean_passes']}/{streak['total']}",
              f"- Longest streak: {streak['longest']}",
              f"- Trailing streak: {streak['trailing']}"]
    if streak["first_failure_entry_id"] is not None:
        lines.append(f"- First failure: {streak['first_failure_entry_id']} "
                     f"(index {streak['first_failure_index']})")
    if required_streak is not None:
        status = "PASS" if streak["met"] else "INCOMPLETE"
        lines.append(f"- Required streak: {required_streak} ({status})")
    physical_table = physical_section(results)
    if physical_table:
        lines += ["", physical_table.rstrip("\n")]
    lines += ["", runs_section(results).rstrip("\n")]
    summary = out_dir / "SUMMARY.md"
    summary.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return summary
