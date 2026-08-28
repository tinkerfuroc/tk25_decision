# tools/gpsr_ui/corpus.py
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

# Tier dirs hold their runs in "runs", plus historical siblings such as
# "runs-invalidated-20260826" which we surface as separate pseudo-tiers.
_RUNS_PREFIX = "runs"


@dataclass(frozen=True)
class Attempt:
    dir_name: str
    entry_id: str
    slug: str | None
    path: Path
    verdict: str | None
    seconds: float | None
    detail: str
    is_current: bool
    mtime: float


@dataclass
class Entry:
    entry_id: str
    template: str | None = None
    feasibility: str | None = None
    text: str | None = None
    attempts: list[Attempt] = field(default_factory=list)


@dataclass
class Tier:
    name: str
    path: Path
    entries: list[Entry] = field(default_factory=list)


def split_dir_name(name: str) -> tuple[str, str | None]:
    """`s2026-002-x.attempt7-rpp` -> (`s2026-002-x`, `attempt7-rpp`)."""
    entry_id, sep, slug = name.partition(".")
    return (entry_id, slug if sep else None)


def _read_run_json(run_dir: Path) -> dict:
    try:
        return json.loads((run_dir / "run.json").read_text())
    except (OSError, ValueError):
        return {}


def _attempt(run_dir: Path) -> Attempt:
    entry_id, slug = split_dir_name(run_dir.name)
    data = _read_run_json(run_dir)
    seconds = data.get("seconds")
    return Attempt(
        dir_name=run_dir.name,
        entry_id=entry_id,
        slug=slug,
        path=run_dir,
        verdict=data.get("verdict"),
        seconds=float(seconds) if isinstance(seconds, (int, float)) else None,
        detail=data.get("detail") or "",
        is_current=slug is None,
        mtime=run_dir.stat().st_mtime,
    )


def _corpus_metadata(tier_dir: Path) -> dict[str, dict]:
    """entry_id -> corpus.jsonl record, for template/feasibility/text."""
    out: dict[str, dict] = {}
    path = tier_dir / "corpus.jsonl"
    try:
        text = path.read_text()
    except OSError:
        return out
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except ValueError:
            continue
        if isinstance(rec, dict) and isinstance(rec.get("id"), str):
            out[rec["id"]] = rec
    return out


def _build_tier(name: str, runs_dir: Path, meta: dict[str, dict]) -> Tier:
    by_entry: dict[str, Entry] = {}
    for run_dir in sorted(p for p in runs_dir.iterdir() if p.is_dir()):
        attempt = _attempt(run_dir)
        entry = by_entry.get(attempt.entry_id)
        if entry is None:
            rec = meta.get(attempt.entry_id, {})
            entry = Entry(
                entry_id=attempt.entry_id,
                template=rec.get("template"),
                feasibility=rec.get("feasibility"),
                text=rec.get("text"),
            )
            by_entry[attempt.entry_id] = entry
        entry.attempts.append(attempt)

    for entry in by_entry.values():
        # Current attempt first; archived attempts newest-first behind it.
        entry.attempts.sort(key=lambda a: (a.is_current, a.mtime), reverse=True)

    return Tier(
        name=name,
        path=runs_dir,
        entries=sorted(by_entry.values(), key=lambda e: e.entry_id),
    )


def list_tiers(bench_root: Path) -> list[Tier]:
    tiers: list[Tier] = []
    try:
        tier_dirs = sorted(p for p in bench_root.iterdir() if p.is_dir())
    except OSError:
        return []

    for tier_dir in tier_dirs:
        meta = _corpus_metadata(tier_dir)
        for runs_dir in sorted(
            p for p in tier_dir.iterdir()
            if p.is_dir() and p.name.startswith(_RUNS_PREFIX)
        ):
            suffix = runs_dir.name[len(_RUNS_PREFIX):].lstrip("-")
            name = tier_dir.name if not suffix else f"{tier_dir.name}/{suffix}"
            tier = _build_tier(name, runs_dir, meta)
            if tier.entries:
                tiers.append(tier)
    return tiers


def find_run(bench_root: Path, tier: str, dir_name: str) -> Path | None:
    """Resolve a (tier, dir_name) pair to a run dir, refusing traversal."""
    if "/" in dir_name or dir_name in {"", ".", ".."}:
        return None
    for candidate in list_tiers(bench_root):
        if candidate.name != tier:
            continue
        for entry in candidate.entries:
            for attempt in entry.attempts:
                if attempt.dir_name == dir_name:
                    return attempt.path
    return None
