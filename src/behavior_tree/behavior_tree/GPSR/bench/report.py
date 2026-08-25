"""Bench result model and the template x tier pass matrix."""
from __future__ import annotations

import json
import subprocess
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

VERDICTS = ("PASS", "FAIL", "TIMEOUT", "ERROR")
CLASSES = ("A", "B", "C")


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
        lines.append(bullet)
    return "\n".join(lines) + "\n"


def write_report(results: Iterable[BenchResult], out_dir: Path, *, corpus_path: Path | None = None,
                 meta: dict[str, Any] | None = None) -> Path:
    results = list(results)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
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
    lines += ["", runs_section(results).rstrip("\n")]
    summary = out_dir / "SUMMARY.md"
    summary.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return summary
