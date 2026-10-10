"""Seeded, template-labelled GPSR command corpus built on the vendored official generator."""
from __future__ import annotations

import json
import random
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence

from behavior_tree.GPSR._official_cmd_gen import CommandGenerator

_FOLLOWUP_RE = re.compile(r"\{FOLLOWUP:(\w+)\}")

# Feasibility in the rcw2026 simulation (spec section "Sim-feasibility classification").
# A: checkable with truth pose + announce events today.
# B: executable but the object-pose oracle is blocked until YCB rigid bodies resolve.
# C: unsupported sim content (moving actors, clothing, trash bin) or stubbed executor (place, open).
FEASIBILITY: dict[str, str] = {
    # top-level
    "goToLoc": "A", "findPrsInRoom": "A", "findObjInRoom": "A", "meetPrsAtBeac": "A",
    "countObjOnPlcmt": "A", "countPrsInRoom": "A", "tellObjPropOnPlcmt": "A",
    "tellCatPropOnPlcmt": "A", "talkInfoToGestPrsInRoom": "A", "greetClothDscInRm": "C",
    "greetNameInRm": "A", "meetNameAtLocThenFindInRm": "A",
    "takeObjFromPlcmt": "B", "bringMeObjFromPlcmt": "B",
    "tellPrsInfoInLoc": "C", "followNameFromBeacToRoom": "C", "guideNameFromBeacToBeac": "C",
    "guidePrsFromBeacToBeac": "C", "guideClothPrsFromBeacToBeac": "C",
    "countClothPrsInRoom": "C", "tellPrsInfoAtLocToPrsAtLoc": "C", "followPrsAtLoc": "C",
    # follow-ups
    "findObj": "A", "findPrs": "A", "meetName": "A", "talkInfo": "A",
    "takeObj": "B", "deliverObjToMe": "B", "deliverObjToPrsInRoom": "B", "deliverObjToNameAtBeac": "B",
    "placeObjOnPlcmt": "C", "putObjInTrash": "C", "followPrs": "C", "followPrsToRoom": "C",
    "guidePrsToBeacon": "C",
}
_RANK = {"A": 0, "B": 1, "C": 2}

# Templates/follow-ups the sim-hybrid bench (manipulation real, everything else mocked) cannot
# execute: moving-actor following/guiding, clothing description, and the trash-bin placement.
SIM_INFEASIBLE: frozenset[str] = frozenset({
    "followNameFromBeacToRoom", "followPrsAtLoc", "followPrs", "followPrsToRoom",
    "guideNameFromBeacToBeac", "guidePrsFromBeacToBeac", "guideClothPrsFromBeacToBeac",
    "guidePrsToBeacon", "greetClothDscInRm", "countClothPrsInRoom", "putObjInTrash",
})


@dataclass(frozen=True)
class Knowledge:
    names: list[str]
    locations: list[str]
    placement_locations: list[str]
    rooms: list[str]
    objects: list[str]
    object_categories_singular: list[str]
    object_categories_plural: list[str]


@dataclass(frozen=True)
class CorpusEntry:
    id: str
    seed: int
    template: str
    followups: tuple[str, ...]
    category: str
    text: str
    feasibility: str


def build_knowledge(constants_path: Path) -> Knowledge:
    raw = json.loads(Path(constants_path).read_text(encoding="utf-8"))
    rooms = [r for r in raw.get("search_spots", {}) if not r.startswith("_")]
    locations = [p for p in raw["possible_poses"] if p != "command_point" and p not in rooms]
    # Flat surfaces the grammar may place/count objects on: everything that is not a seat.
    placement = [p for p in locations if p not in {"sofa"}]
    return Knowledge(
        names=["Alex", "Sarah", "John", "Emma", "Liam", "Olivia"],
        locations=locations,
        placement_locations=placement,
        rooms=rooms,
        objects=[o for o in raw["possible_objects"] if not o.startswith("_")],
        object_categories_singular=["food", "drink", "kitchen item"],
        object_categories_plural=["foods", "drinks", "kitchen items"],
    )


def make_generator(kb: Knowledge, *, seed: int) -> CommandGenerator:
    random.seed(seed)
    return CommandGenerator(kb)


def _expand_followup(gen: CommandGenerator, name: str, category: str, followups: list[str]) -> str:
    """Expand one follow-up template, resolving its own nested FOLLOWUP markers first, then
    filling its own placeholders in isolation -- mirroring the vendored _generate_followup,
    which fills each follow-up's placeholders before it is spliced into its parent, so repeated
    bare placeholders (e.g. {art}) at different nesting levels are sampled independently."""
    text = gen.followup_templates[name]
    while True:
        match = _FOLLOWUP_RE.search(text)
        if not match:
            break
        sub_name = gen._sample_followup(match.group(1), category)
        followups.append(sub_name)
        expanded = _expand_followup(gen, sub_name, category, followups)
        text = text.replace(match.group(0), expanded, 1)
    return gen.insert_all_placeholders(text)


def expand_template(gen: CommandGenerator, template: str, category: str) -> tuple[str, tuple[str, ...]]:
    """Expand one named template exactly as generate_command_start does, recording follow-up names."""
    text = gen.templates[template]
    followups: list[str] = []
    while True:
        match = _FOLLOWUP_RE.search(text)
        if not match:
            break
        name = gen._sample_followup(match.group(1), category)
        followups.append(name)
        expanded = _expand_followup(gen, name, category, followups)
        text = text.replace(match.group(0), expanded, 1)
    text = gen.insert_all_placeholders(text)
    text = gen._resolve_duplicates(text)
    text = gen._fix_articles(text)
    return text, tuple(followups)


def _feasibility(template: str, followups: Sequence[str]) -> str:
    worst = max([FEASIBILITY[template], *[FEASIBILITY[f] for f in followups]], key=_RANK.__getitem__)
    return worst


def generate_corpus(
    constants_path: Path, *, seed: int, per_template: int, templates: Sequence[str] | None = None
) -> list[CorpusEntry]:
    kb = build_knowledge(constants_path)
    gen = make_generator(kb, seed=seed)
    people = [name for name, _ in gen.person_cmd_list]
    objects = [name for name, _ in gen.object_cmd_list]
    chosen = list(templates) if templates else sorted(set(people) | set(objects))
    entries: list[CorpusEntry] = []
    index = 0
    for template in chosen:
        category = "objects" if template in objects and template not in people else "people"
        for _ in range(per_template):
            if template in people and template in objects:
                category = random.choice(["people", "objects"])
            text, followups = expand_template(gen, template, category)
            entry_id = f"c{index:03d}-{template}" + ("-" + "-".join(followups) if followups else "")
            entries.append(CorpusEntry(
                id=entry_id, seed=seed, template=template, followups=followups,
                category=category, text=text, feasibility=_feasibility(template, followups),
            ))
            index += 1
    return entries


def write_jsonl(entries: Iterable[CorpusEntry], path: Path, *, header: dict | None = None) -> None:
    """Write entries as one JSON object per line. If `header` is given, it is written first as
    its own line (its keys are expected to start with "_" so `read_jsonl` can recognise and
    skip it as metadata rather than a CorpusEntry)."""
    with Path(path).open("w", encoding="utf-8") as fh:
        if header is not None:
            fh.write(json.dumps(header, ensure_ascii=False) + "\n")
        for entry in entries:
            fh.write(json.dumps(asdict(entry), ensure_ascii=False) + "\n")


def read_jsonl(path: Path) -> list[CorpusEntry]:
    entries = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith('{"_'):
            # Header/metadata line (e.g. written by `write_jsonl(..., header=...)`), not a
            # CorpusEntry -- skip it.
            continue
        raw = json.loads(line)
        raw["followups"] = tuple(raw["followups"])
        entries.append(CorpusEntry(**raw))
    return entries


def generate_sim_corpus(
    constants_path: Path, *, seed: int, count: int, templates: Sequence[str] | None = None
) -> tuple[list[CorpusEntry], dict[str, int]]:
    """Like `generate_corpus`, but round-robins templates and skips any expansion whose
    template or follow-ups intersect `SIM_INFEASIBLE`, re-drawing (advancing to the next
    template in rotation) until `count` sim-feasible entries have been produced.

    Returns `(entries, skipped)` where `skipped` maps the specific SIM_INFEASIBLE name that
    triggered each skip to how many times it did.
    """
    kb = build_knowledge(constants_path)
    gen = make_generator(kb, seed=seed)
    people = [name for name, _ in gen.person_cmd_list]
    objects = [name for name, _ in gen.object_cmd_list]
    chosen = list(templates) if templates else sorted(set(people) | set(objects))
    if not chosen:
        raise ValueError("generate_sim_corpus: no templates to draw from")

    entries: list[CorpusEntry] = []
    skipped: dict[str, int] = {}
    draw = 0
    max_draws = max(count * 50, 1000)
    while len(entries) < count:
        if draw >= max_draws:
            raise RuntimeError(
                f"generate_sim_corpus: could not reach count={count} sim-feasible entries "
                f"after {draw} draws (skipped={skipped}); the templates filter may exclude "
                "every sim-feasible command"
            )
        template = chosen[draw % len(chosen)]
        category = "objects" if template in objects and template not in people else "people"
        if template in people and template in objects:
            category = random.choice(["people", "objects"])
        text, followups = expand_template(gen, template, category)
        draw += 1

        trigger = template if template in SIM_INFEASIBLE else next(
            (f for f in followups if f in SIM_INFEASIBLE), None)
        if trigger is not None:
            skipped[trigger] = skipped.get(trigger, 0) + 1
            continue

        entry_id = f"s{seed}-{len(entries):03d}-{template}"
        entries.append(CorpusEntry(
            id=entry_id, seed=seed, template=template, followups=followups,
            category=category, text=text, feasibility=_feasibility(template, followups),
        ))
    return entries, skipped


def _edge(i: int, text: str, feasibility: str) -> CorpusEntry:
    return CorpusEntry(id=f"e{i:02d}", seed=-1, template="edge", followups=(), category="edge",
                       text=text, feasibility=feasibility)


EDGE_COMMANDS: list[CorpusEntry] = [
    _edge(0, "go to the kitchen table and find a person", "A"),            # run18 canary
    _edge(1, "head over to the sofa and tell me what you see", "A"),        # synonym the grammar never emits
    _edge(2, "bring me the soup from the laundry desk", "B"),              # object not at its default location
    _edge(3, "find a unicorn in the bedroom", "A"),                        # unknown object
    _edge(4, "guide Alex from the kitchen to the bedroom then to the sofa", "C"),  # two-hop guide
    _edge(5, "count how many mugs are on the shelf and then on the side table", "A"),  # two counts
    _edge(6, "go to the laundry desk and count the mugs on the side table", "A"),  # spoken waypoint name uses spaces
]
