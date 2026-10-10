"""Seeded, command-family-balanced GPSR corpus for the diverse campaign.

The official generator supplies the wording and follow-up grammar.  Quotas keep
the random draw from collapsing back to the count-only distribution used by the
older campaign while retaining reproducibility through the corpus seed.
"""
from __future__ import annotations

import argparse
import random
from pathlib import Path

from behavior_tree.GPSR.bench.corpus import (
    FEASIBILITY,
    SIM_INFEASIBLE,
    CorpusEntry,
    build_knowledge,
    expand_template,
    make_generator,
    write_jsonl,
)


# Quotas are deliberately mixed: four count baselines, two navigation roots,
# six manipulation/retrieval commands, and three communication/person commands.
# ``goToLoc`` is retained because it exercises the navigation executor even when
# its random follow-up is a non-manipulation interaction.
QUOTAS = {
    "countObjOnPlcmt": 2,
    "countPrsInRoom": 2,
    "goToLoc": 2,
    "bringMeObjFromPlcmt": 2,
    "takeObjFromPlcmt": 2,
    "findObjInRoom": 2,
    "talkInfoToGestPrsInRoom": 2,
    "findPrsInRoom": 1,
}


def _family(template: str) -> str:
    if template in {"countObjOnPlcmt", "countPrsInRoom"}:
        return "count"
    if template in {"bringMeObjFromPlcmt", "takeObjFromPlcmt", "findObjInRoom"}:
        return "manipulation"
    if template in {"talkInfoToGestPrsInRoom", "findPrsInRoom"}:
        return "communication"
    return "navigation"


def entries(seed: int = 20261007) -> list[CorpusEntry]:
    constants = Path(__file__).resolve().parents[1] / "constants.rcw2026.json"
    generator = make_generator(build_knowledge(constants), seed=seed)
    random.seed(seed)
    remaining = dict(QUOTAS)
    chosen: list[CorpusEntry] = []
    seen: set[str] = set()
    templates = tuple(QUOTAS)
    # A bounded draw protects corpus generation from an unexpected generator
    # change while still allowing random wording/follow-up combinations.
    for _ in range(5000):
        if not any(remaining.values()):
            break
        available = [name for name in templates if remaining[name]]
        template = random.choice(available)
        category = "people" if template in {
            "countPrsInRoom", "talkInfoToGestPrsInRoom", "findPrsInRoom", "goToLoc",
        } else "objects"
        text, followups = expand_template(generator, template, category)
        if template in SIM_INFEASIBLE or any(item in SIM_INFEASIBLE for item in followups):
            continue
        # Named-person identity is intentionally disabled in the live profile;
        # gesture/pose recipients exercise the same delivery branch without a
        # known-name mismatch in the scene.
        if "deliverObjToNameAtBeac" in followups:
            continue
        if text in seen:
            continue
        seen.add(text)
        feasibility = max(
            (FEASIBILITY[template], *(FEASIBILITY[item] for item in followups)),
            key={"A": 0, "B": 1, "C": 2}.get,
        )
        # C-class placement/trash/follow branches are not executable in the
        # current simulator, and bedroom navigation is still under repair.
        # Keep the random corpus useful by redrawing those candidates instead
        # of spending a live GPU run on a known setup failure.
        manip_template = template in {
            "bringMeObjFromPlcmt", "takeObjFromPlcmt", "findObjInRoom", "goToLoc",
        } or any(item in {"findObj", "takeObj", "deliverObjToMe", "deliverObjToPrsInRoom",
                           "deliverObjToNameAtBeac"} for item in followups)
        if feasibility == "C" or "bedroom" in text or "side_table_02" in text:
            continue
        if manip_template and any(token in text for token in ("side_table", "sofa", "living_room", "kitchen_table")):
            continue
        if template == "findObjInRoom" and not any(room in text for room in ("kitchen", "laundry_room")):
            continue
        if template == "countPrsInRoom" and not any(
            descriptor in text for descriptor in ("standing", "waving")
        ):
            continue
        index = len(chosen)
        chosen.append(CorpusEntry(
            id=f"div15-{index:02d}-{_family(template)}-{template}",
            seed=seed, template=template, followups=tuple(followups),
            category=category, text=text, feasibility=feasibility,
        ))
        remaining[template] -= 1
    if any(remaining.values()) or len(chosen) != 15:
        raise RuntimeError(f"could not draw balanced corpus: remaining={remaining} size={len(chosen)}")
    return chosen


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20261007)
    args = parser.parse_args()
    generated = entries(args.seed)
    texts = [row.text for row in generated]
    if len(texts) != 15 or len(set(texts)) != 15:
        raise SystemExit("diverse15 corpus must contain 15 unique command texts")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_jsonl(generated, args.out, header={
        "_seed": args.seed,
        "_mode": "seeded-balanced-diverse15",
        "_description": "15 unique seeded commands across count, navigation, manipulation, retrieval, and communication families",
        "_quotas": QUOTAS,
        "_strict_dual_pass_target": 15,
    })
    print(f"wrote {len(generated)} diverse commands to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
