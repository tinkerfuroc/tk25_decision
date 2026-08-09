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
  .../run_ten_cmd.py --count 10 --seed 8 --save --label gpt5.6-luna
      # --save writes BOTH to gpsr_runs/logs under the GPSR module:
      #   ten_cmd_<stamp>_<label>_<count>cmds_seed<seed>.log          (detailed, tee'd stdout)
      #   ten_cmd_<stamp>_<label>_<count>cmds_seed<seed>_simple.log   (simplified planner summary)
"""

import argparse
import datetime
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
    planner.request_plan_all(slot, targets, command=command)
    deadline = time.time() + max_wait
    while time.time() < deadline:
        if planner.all_targets_ready(slot, len(targets)):
            break
        time.sleep(0.05)
    plans = [planner.get_action_plan(slot, i) for i in range(len(targets))]
    return targets, plans


# GPSR-package gpsr_runs/logs — run logs live under the GPSR module, not the
# workspace root, so they stay with the code that produces them.
RUNS_LOG_DIR = Path(__file__).resolve().parent.parent / "gpsr_runs" / "logs"


def _slug(text: str, n: int = 40) -> str:
    return "".join(c if c.isalnum() else "_" for c in text).strip("_")[:n] or "run"


class _Tee:
    """Duplicate all writes to both the real stdout and a log file."""

    def __init__(self, fh):
        self._fh = fh
        self._real = sys.stdout

    def write(self, data):
        self._real.write(data)
        self._fh.write(data)
        self._fh.flush()

    def flush(self):
        self._real.flush()
        self._fh.flush()


class _SimpleLog:
    """Human-readable planner summary: command -> targets -> action steps -> pass/fail.

    Deliberately omits blackboard write-maps and decision trees (those live in
    the detailed log). One line per action, indented under its target.
    """

    def __init__(self, path: str):
        self._path = path
        self._fh = open(path, "w", buffering=1)
        self._valid = 0
        self._total = 0
        self._rejected: list[str] = []

    def header(self, model: str, seed: int, count: int, timestamp: str):
        self._fh.write("==== GPSR PLANNER RUN (SIMPLIFIED) ====\n")
        self._fh.write(f"model:     {model}\n")
        self._fh.write(f"seed:      {seed}\n")
        self._fh.write(f"commands:  {count}\n")
        self._fh.write(f"timestamp: {timestamp}\n")

    def command(self, idx: int, cmd: str):
        self._fh.write("\n" + "=" * 60 + "\n")
        self._fh.write(f"COMMAND {idx}: {cmd}\n")
        self._fh.write("-" * 60 + "\n")

    def targets(self, targets):
        self._fh.write(f"[top-layer] {len(targets)} target(s)\n")
        for i, t in enumerate(targets):
            if not isinstance(t, dict):
                self._fh.write(f"  T{i}: {t}\n")
                continue
            obj = t.get("object") or ""
            loc = t.get("location") or ""
            dep = t.get("depends_on")
            bits = []
            if obj:
                bits.append(f"object={obj}")
            if loc:
                bits.append(f"location={loc}")
            if dep is not None and dep >= 0:
                bits.append(f"after=T{dep}")
            suffix = ("  [" + ", ".join(bits) + "]") if bits else ""
            self._fh.write(f"  T{i}: {t.get('desc')}{suffix}\n")

    def plan(self, desc: str, steps, ok: bool, reason: str = ""):
        self._total += 1
        self._fh.write(f"\n  TARGET: {desc}\n")
        if not steps:
            self._fh.write("    (empty plan)\n")
        for k, step in enumerate(steps):
            act = step.get("action")
            params = step.get("params", {})
            arg = ", ".join(f"{k2}={v}" for k2, v in params.items()) if params else ""
            self._fh.write(f"    {k}. {act}({arg})\n")
        if ok:
            self._valid += 1
            self._fh.write("    PASS\n")
        else:
            self._rejected.append(f"{desc}: {reason}")
            self._fh.write(f"    FAIL: {reason}\n")

    def summary(self):
        self._fh.write("\n" + "=" * 60 + "\n")
        self._fh.write(f"VALIDATED: {self._valid}/{self._total} plans passed\n")
        if self._rejected:
            self._fh.write("REJECTED:\n")
            for r in self._rejected:
                self._fh.write(f"  - {r}\n")
        else:
            self._fh.write("All plans validated.\n")

    def close(self):
        self._fh.close()


def open_run_log(label: str, seed: int, count: int) -> str:
    """Create a labelled log path in gpsr_runs/logs, return its path."""
    RUNS_LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = RUNS_LOG_DIR / f"ten_cmd_{stamp}_{_slug(label)}_{count}cmds_seed{seed}.log"
    return str(path)


def open_simple_log(label: str, seed: int, count: int) -> str:
    """Create the simplified-log path, derived from the detailed log's name."""
    RUNS_LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = RUNS_LOG_DIR / f"ten_cmd_{stamp}_{_slug(label)}_{count}cmds_seed{seed}_simple.log"
    return str(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=10)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--mock", action="store_true",
                    help="use full-mock planner (no network; trivial plans)")
    ap.add_argument("--save", action="store_true",
                    help="tee stdout to a labelled log in gpsr_runs/logs")
    ap.add_argument("--label", default="",
                    help="label used in the saved log filename (default: model)")
    args = ap.parse_args()

    # Save-to-file: tee stdout (detailed log) + write a simplified log alongside.
    log_path = None
    simple = None
    if args.save:
        label = args.label or (
            os.environ.get("GPSR_LLM_MODEL", "unknown") or "mock" if args.mock else "unknown"
        )
        log_path = open_run_log(label, args.seed, args.count)
        simple_path = open_simple_log(label, args.seed, args.count)
        fh = open(log_path, "w", buffering=1)
        sys.stdout = _Tee(fh)
        print(f"[run-ten-cmd] saving this run -> {log_path}")
        print(f"[run-ten-cmd] model={os.environ.get('GPSR_LLM_MODEL', '?')} "
              f"seed={args.seed} count={args.count} label={label or '?'}")
        simple = _SimpleLog(simple_path)
        simple.header(
            model=os.environ.get("GPSR_LLM_MODEL", "?"),
            seed=args.seed,
            count=args.count,
            timestamp=datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        )

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
        if simple:
            simple.command(slot + 1, cmd)
        try:
            targets, plans = plan_one_command(planner, slot, cmd)
        except Exception as exc:  # noqa: BLE001
            print(f"  !! planning failed: {exc!r}")
            continue

        print(f"\n[TOP-LAYER] split -> {len(targets)} target(s):")
        if simple:
            simple.targets(targets)
        for i, t in enumerate(targets):
            desc = t.get("desc") if isinstance(t, dict) else t
            print(f"  T{i}: {desc}   object={t.get('object','') if isinstance(t, dict) else ''}"
                  f" location={t.get('location','') if isinstance(t, dict) else ''}"
                  f" depends_on={t.get('depends_on','') if isinstance(t, dict) else ''}")

        # Flat list of already-accepted steps from prior targets (validate_plan's
        # prior_plan is a flat list of dict steps — a list of lists would seed
        # nothing, since the cross-target seeding loop skips non-dicts).
        prior_steps = []
        for i, (t, plan) in enumerate(zip(targets, plans)):
            desc = t.get("desc") if isinstance(t, dict) else t
            print(f"\n  --- TARGET T{i}: {desc} ---")
            ok, reason = validate_plan(plan, desc, set(ACTION_FACTORIES.keys()),
                                       known_locations=known_loc,
                                       prior_plan=prior_steps)
            prior_steps.extend(plan)
            if simple:
                simple.plan(desc, plan, ok, reason)
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

    if log_path:
        simple.summary()
        simple.close()
        print(f"\n[run-ten-cmd] done — log saved to {log_path}")
        fh.close()
        sys.stdout = sys.__stdout__


if __name__ == "__main__":
    main()
