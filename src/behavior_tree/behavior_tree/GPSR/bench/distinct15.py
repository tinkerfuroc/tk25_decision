"""Build the explicit fifteen-command GPSR diversity corpus.

The live tier-2 oracle can prove a physical result for ``counted(...)`` targets
(the simulator writes the spawned entities and the announced number to each
run directory).  Pure conversational targets have no physical postcondition,
so they are useful for planner-only coverage but cannot contribute to a strict
planner+physical dual pass.  This corpus therefore uses two count families,
with different category/location/gesture combinations, rather than repeating
one seeded command.

Usage::

    python3 bench/distinct15.py \
      --out gpsr_runs/bench/t2-15-distinct/corpus.jsonl

The output is ordinary bench JSONL and can be consumed by ``gpsr-bench tier2``
or by the live battery wrapper.  ``--check`` validates that every command is
unique and that all locations/descriptors exist in ``constants.rcw2026.json``.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from behavior_tree.GPSR.bench.corpus import CorpusEntry, write_jsonl


# Keep the text in the same grammar emitted by _official_cmd_gen.  In
# particular, room/location names use the rcw2026 underscore spelling because
# that is what the live parser and scene applier use as their stable keys.
# The command text itself is deliberately unique on every row.
COMMANDS: tuple[tuple[str, str, str], ...] = (
    # kitchen_table is boxed in by four chairs in the RCW arena; keep the
    # diversity run on free-face viewpoints so a known camera occlusion does
    # not dominate the dual-pass qualification.
    # Three food assets do not fit on the small side table. The kitchen-item
    # assets have compact footprints and are validated by gpsr_spawn's
    # placement planner for this surface.
    ("count-obj-00", "countObjOnPlcmt", "tell me how many kitchen items there are on the side_table"),
    ("count-obj-01", "countObjOnPlcmt", "tell me how many drinks there are on the laundry_desk"),
    ("count-obj-02", "countObjOnPlcmt", "tell me how many foods there are on the shelf"),
    ("count-obj-03", "countObjOnPlcmt", "tell me how many kitchen items there are on the shelf_02"),
    ("count-obj-04", "countObjOnPlcmt", "tell me how many drinks there are on the side_table"),
    # The second small side table has the same capacity constraint as
    # side_table; use the compact kitchen-item set there.
    ("count-obj-05", "countObjOnPlcmt", "tell me how many kitchen items there are on the side_table_02"),
    ("count-obj-06", "countObjOnPlcmt", "tell me how many kitchen items there are on the laundry_desk"),
    ("count-obj-07", "countObjOnPlcmt", "tell me how many drinks there are on the shelf_02"),
    ("count-person-00", "countPrsInRoom", "tell me how many standing persons are in the living_room"),
    ("count-person-01", "countPrsInRoom", "tell me how many sitting persons are in the kitchen"),
    ("count-person-02", "countPrsInRoom", "tell me how many waving persons are in the laundry_room"),
    ("count-person-03", "countPrsInRoom", "tell me how many persons pointing to the right are in the bedroom"),
    ("count-person-04", "countPrsInRoom", "tell me how many persons pointing to the left are in the living_room"),
    ("count-person-05", "countPrsInRoom", "tell me how many persons raising their right arm are in the kitchen"),
    ("count-person-06", "countPrsInRoom", "tell me how many persons raising their left arm are in the bedroom"),
)


def _constants_keys(path: Path) -> tuple[set[str], set[str]]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    poses = raw.get("possible_poses", {})
    pose_keys = {str(k) for k in poses if not str(k).startswith("_")}
    rooms = raw.get("search_spots", {})
    room_keys = {str(k) for k in rooms if not str(k).startswith("_")}
    return pose_keys, room_keys


def validate(constants: Path) -> None:
    texts = [text for _, _, text in COMMANDS]
    if len(texts) != 15 or len(set(texts)) != len(texts):
        raise ValueError("distinct15 corpus must contain fifteen unique command texts")
    pose_keys, room_keys = _constants_keys(constants)
    expected_poses = {"kitchen_table", "laundry_desk", "shelf", "shelf_02", "side_table", "side_table_02"}
    if not expected_poses <= pose_keys:
        raise ValueError(f"constants missing placement keys: {sorted(expected_poses - pose_keys)}")
    expected_rooms = {"kitchen", "laundry_room", "living_room", "bedroom"}
    if not expected_rooms <= room_keys:
        raise ValueError(f"constants missing room keys: {sorted(expected_rooms - room_keys)}")


def entries(seed: int = 20261006) -> list[CorpusEntry]:
    return [CorpusEntry(
        id=f"d15-{index:02d}-{entry_id}",
        seed=seed,
        template=template,
        followups=(),
        category="objects" if template == "countObjOnPlcmt" else "people",
        text=text,
        feasibility="A",
    ) for index, (entry_id, template, text) in enumerate(COMMANDS)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--constants", type=Path,
                        default=Path(__file__).resolve().parents[1] / "constants.rcw2026.json")
    parser.add_argument("--seed", type=int, default=20261006)
    args = parser.parse_args()
    validate(args.constants)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_jsonl(entries(args.seed), args.out, header={
        "_seed": args.seed,
        "_mode": "explicit-distinct15",
        "_description": "15 unique physical-truth-countable GPSR commands",
        "_strict_dual_pass_target": 15,
    })
    print(f"wrote {len(COMMANDS)} unique commands to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
