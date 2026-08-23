"""Bench result model and the template x tier pass matrix."""
from __future__ import annotations

import json
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Iterable

VERDICTS = ("PASS", "FAIL", "TIMEOUT", "ERROR")


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


def matrix(results: Iterable[BenchResult]) -> dict[str, dict[int, tuple[int, int]]]:
    table: dict[str, dict[int, list[int]]] = defaultdict(lambda: defaultdict(lambda: [0, 0]))
    for r in results:
        cell = table[r.template][r.tier]
        cell[1] += 1
        if r.verdict == "PASS":
            cell[0] += 1
    return {t: {tier: (p, n) for tier, (p, n) in tiers.items()} for t, tiers in table.items()}


def write_report(results: Iterable[BenchResult], out_dir: Path, *, corpus_path: Path | None = None) -> Path:
    results = list(results)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    tiers = sorted({r.tier for r in results})
    totals = {str(t): dict(Counter(r.verdict for r in results if r.tier == t)) for t in tiers}
    (out_dir / "report.json").write_text(json.dumps({
        "corpus": str(corpus_path) if corpus_path else None,
        "totals": totals,
        "results": [asdict(r) for r in results],
    }, indent=2) + "\n", encoding="utf-8")

    feas = {r.template: r.feasibility for r in results}
    m = matrix(results)
    lines = ["# GPSR bench summary", ""]
    if corpus_path:
        lines += [f"Corpus: `{corpus_path}`", ""]
    header = "| template | class | " + " | ".join(f"T{t}" for t in tiers) + " |"
    lines += [header, "|" + "---|" * (2 + len(tiers))]
    for template in sorted(m, key=lambda t: (feas[t], t)):
        cells = [f"{m[template][t][0]}/{m[template][t][1]}" if t in m[template] else "-" for t in tiers]
        lines.append(f"| {template} | {feas[template]} | " + " | ".join(cells) + " |")
    lines += ["", "## Totals", ""]
    for t in tiers:
        lines.append(f"- T{t}: " + ", ".join(f"{v} {totals[str(t)].get(v, 0)}" for v in VERDICTS))
    lines += ["", "## Failures", ""]
    for r in results:
        if r.verdict != "PASS":
            lines.append(f"- T{r.tier} `{r.entry_id}` **{r.verdict}** — {r.detail}")
    summary = out_dir / "SUMMARY.md"
    summary.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return summary
