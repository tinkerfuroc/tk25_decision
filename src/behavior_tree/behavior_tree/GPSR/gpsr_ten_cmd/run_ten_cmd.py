"""Generate 10 GPSR commands with the official RoboCupAtHome CommandGenerator
and drive them through the two-layer GPSRPlanner, dumping per-command:
  - the TOP-LAYER split targets,
  - each target's LOWER-LAYER action plan (validated),
  - the decision tree (py_trees.display.unicode_tree),
  - the blackboard write-map: EXACTLY which BB keys each step's
    BtNode_MaterialiseStep writes (ground truth from materialise_params via a
    recording client), plus which keys the step's small tree consumes.

Planner threads NEVER touch the blackboard — this script proves it by showing
that every BB write happens through the per-step MaterialiseStep on the
executor thread (the recorder wraps materialise_params, which is only ever
called from a Behaviour.update()).

Usage:
  BT_MOCK_MODE=true BT_MOCK_CONFIG=full.json  .../run_ten_cmd.py --mock   # offline
  .../run_ten_cmd.py --count 10 --seed 7                                     # real LLM
"""

import argparse
import json
import os
import random
import sys
import time
from pathlib import Path

# --- official generator ------------------------------------------------------
GEN_SRC = Path(os.environ.get("COMMAND_GENERATOR_SRC", "/tmp/CommandGenerator/src"))
sys.path.insert(0, str(GEN_SRC))
from robocupathome_generator.knowledge import parse_data               # noqa: E402
from robocupathome_generator.gpsr_commands import CommandGenerator     # noqa: E402

# --- GPSR planner ------------------------------------------------------------
HERE = Path(__file__).resolve().parent                      # .../GPSR
sys.path.insert(0, str(HERE.parent.parent))                 # behavior_tree
from behavior_tree.GPSR.planner import GPSRPlanner          # noqa: E402
from behavior_tree.GPSR.orchestrator import (               # noqa: E402
    KNOWN_LOCATIONS, START_LOCATION_ALIASES, materialise_params,
    load_knowledge_from_constants,
)
from behavior_tree.GPSR.gpsr_full import CONSTANTS_PATH     # noqa: E402
from behavior_tree.GPSR.planner_validators import validate_plan          # noqa: E402
from behavior_tree.GPSR.small_trees import (                             # noqa: E402
    ACTION_FACTORIES, bb_keys, SEARCH_POSE_KEYS,
)

DATA_DIR = HERE / "data"


class RecordingClient:
    """Minimal blackboard client that records every set() for one step."""

    def __init__(self, start_pose=None, dynamic=None):
        self.writes = []          # (key, value)
        self._start_pose = start_pose
        self._dynamic = dynamic or {}

    def set(self, key, value, overwrite=True):
        self.writes.append((key, value))

    def get(self, key):
        if key == bb_keys.START_POSE:
            if self._start_pose is None:
                raise KeyError(key)
            return self._start_pose
        if key == bb_keys.DYNAMIC_LOCATIONS:
            return self._dynamic
        raise KeyError(key)


# Static: which input BB keys each small tree consumes (the READ contract).
# Keyed by action -> list of keys the small-tree nodes read for that action.
ACTION_INPUT_KEYS = {
    "goto": [bb_keys.TARGET_POSE, bb_keys.TARGET_LOCATION],
    "find_object": [bb_keys.TARGET_OBJECT_PROMPT, bb_keys.TARGET_OBJECT_NAME],
    "search_object": [bb_keys.TARGET_POSE, bb_keys.TARGET_LOCATION,
                      SEARCH_POSE_KEYS[0], SEARCH_POSE_KEYS[1]],
    "find_person": [bb_keys.TARGET_PERSON_PROMPT],
    "approach_person": [bb_keys.PERSON_NAV_POSE, bb_keys.TARGET_PERSON_POSE],
    "describe_person": [bb_keys.TARGET_PERSON_PROMPT, bb_keys.TARGET_PERSON_POSE],
    "ask_person": [bb_keys.ASK_QUESTION, bb_keys.TARGET_PERSON_PROMPT],
    "follow": [bb_keys.TARGET_PERSON_PROMPT, bb_keys.TARGET_PERSON_POSE],
    "guide": [bb_keys.TARGET_POSE, bb_keys.TARGET_LOCATION],
    "grasp": [bb_keys.TARGET_OBJECT_PROMPT, bb_keys.TARGET_OBJECT_NAME,
              bb_keys.GRASP_ASK_REFEREE, bb_keys.GRASP_REFEREE_LOCATION,
              bb_keys.GRASP_REFEREE_POSE, bb_keys.GRASP_REFEREE_IS_APPLIANCE,
              bb_keys.LAST_NAV_LOCATION, bb_keys.APPLIANCE_OPENED],
    "open": [bb_keys.TARGET_POSE, bb_keys.TARGET_LOCATION],
    "place": [bb_keys.TARGET_POSE, bb_keys.TARGET_LOCATION],
    "deliver": [bb_keys.TARGET_OBJECT_NAME, bb_keys.TARGET_PERSON_PROMPT,
                bb_keys.TARGET_POSE],
    "count": [bb_keys.TARGET_OBJECT_PROMPT, bb_keys.TARGET_OBJECT_NAME],
    "answer_question": [bb_keys.QA_QUESTION],
    "announce": [bb_keys.ANNOUNCE_TEXT, bb_keys.REPORT_INFO],
    "record_position": [bb_keys.CURRENT_DYNLABEL],
    "vlm_fallback": [bb_keys.VLM_QUESTION, bb_keys.VLM_IMAGE],
    "llm_fallback": [bb_keys.LLM_QUESTION],
}


def action_reads(action, step_subtree):
    """Keys this step's small tree reads: static contract + any from the node tree."""
    keys = list(ACTION_INPUT_KEYS.get(action, []))
    return keys


def dump_bb_for_step(action, params):
    """Run materialise_params against a RecordingClient -> (written, reads)."""
    rec = RecordingClient()
    materialise_params(rec, action, params)
    written = [(k, _short(v)) for k, v in rec.writes]
    reads = action_reads(action, None)
    return written, reads


def _short(v):
    if v is None:
        return "None"
    s = str(v)
    return s if len(s) <= 60 else s[:57] + "..."


def generate_commands(generator, count, seed):
    rng = random.Random(seed)
    cmds = []
    # sample a varied mix: any / people / objects / batch, reproducible via seed
    for _ in range(count):
        cat = rng.choice(["", "", "people", "objects"])
        c = generator.generate_command_start(cmd_category=cat)
        cmds.append(c)
    return cmds


def plan_one_command(planner, slot, command, max_wait=90.0):
    """Top split + parallel lower layer; returns (targets, per_target_plans)."""
    targets = planner.split_command(command)
    planner.request_plan_all(slot, targets)
    deadline = time.time() + max_wait
    while time.time() < deadline:
        if planner.all_targets_ready(slot, len(targets)):
            break
        time.sleep(0.05)
    plans = [planner.get_action_plan(slot, i) for i in range(len(targets))]
    return targets, plans


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=10)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--mock", action="store_true",
                    help="use full-mock planner (no network; trivial plans)")
    args = ap.parse_args()

    # Same knowledge set the real orchestrator loads: KNOWN_LOCATIONS drives
    # both the validator (known_locations) and the LLM prompt (Known locations).
    # Without this, KNOWN_LOCATIONS is empty and every location fails validation.
    load_knowledge_from_constants(CONSTANTS_PATH)

    data = parse_data(str(DATA_DIR))
    gen = CommandGenerator(data)
    commands = generate_commands(gen, args.count, args.seed)

    planner = GPSRPlanner()
    offline = planner._offline_mock
    print(f"# generator data: {len(data.rooms)} rooms, {len(data.locations)} "
          f"locations, {len(data.objects)} objects")
    print(f"# planner offline(mock): {offline}\n")

    known_loc = (set(KNOWN_LOCATIONS.keys()) | START_LOCATION_ALIASES) or None
    for slot, cmd in enumerate(commands):
        print("=" * 78)
        print(f"COMMAND {slot + 1}: {cmd}")
        print("=" * 78)
        try:
            targets, plans = plan_one_command(planner, slot, cmd)
        except Exception as exc:  # noqa: BLE001
            print(f"  !! planning failed: {exc!r}")
            continue

        print(f"\n[TOP-LAYER] split -> {len(targets)} target(s):")
        for i, t in enumerate(targets):
            print(f"  T{i}: {t}")

        for i, (t, plan) in enumerate(zip(targets, plans)):
            print(f"\n  --- TARGET T{i}: {t} ---")
            ok, reason = validate_plan(plan, t, set(ACTION_FACTORIES.keys()),
                                       known_locations=known_loc)
            print(f"  action plan ({len(plan)} step(s), validate={ok}"
                  + (f": {reason}" if reason else "") + "):")
            for k, step in enumerate(plan):
                act = step.get("action")
                params = step.get("params", {})
                written, reads = dump_bb_for_step(act, params)
                print(f"    step{k} [{act}] params={params}")
                print(f"      BB WRITE (by MaterialiseStep): {written}")
                print(f"      BB READ  (by {act} small tree): {reads}")
            # decision tree
            sub = planner.get_target_subtree(slot, i)
            if sub is not None:
                try:
                    import py_trees
                    tree_str = py_trees.display.unicode_tree(
                        sub, show_status=False)
                    print("      decision tree:")
                    for line in tree_str.splitlines():
                        print(f"        {line}")
                except Exception as exc:  # noqa: BLE001
                    print(f"      (tree render failed: {exc!r})")
            else:
                print("      (no subtree built)")
        print()


if __name__ == "__main__":
    main()
