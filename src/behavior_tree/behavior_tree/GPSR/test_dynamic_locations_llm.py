"""Probe: can the LLM planner fix TWO OR MORE dynamic locations in one plan?

The production planner currently knows only the single auto-captured
``start_position``. This standalone probe temporarily extends the action
vocabulary with a *labelled* ``record_position(label=...)`` (backed at runtime
by BtNode_CaptureCurrentPose — see test_capture_pose.py, which proves the node
can hold many labels at once) and a note that ``goto`` may target a previously
recorded label. We then feed commands that genuinely require 2+ runtime-fixed
locations and inspect the plans.

For each plan we check self-consistency: every goto to a non-map location has a
matching earlier record_position(label=...).

Run::

    source /home/tinker/tk25_ws/install/setup.zsh
    BT_MOCK_MODE=true \
        /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python \
        src/behavior_tree/behavior_tree/GPSR/test_dynamic_locations_llm.py
"""

import json
import os
import sys
import textwrap
from pathlib import Path

os.environ.setdefault("BT_MOCK_MODE", "true")
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import openai
import cmd_understanding_test as cut
from config import OPENAI_API_KEY


# --- temporarily teach the planner about labelled dynamic locations ---------
DYNAMIC_VOCAB = textwrap.dedent("""

    DYNAMIC LOCATIONS (test extension):
    - record_position(label: str)
        Capture the robot's CURRENT pose and remember it under ``label`` for
        the rest of this task. Use when the command refers to a place that is
        not in the known-locations list but is "here / where I am / where this
        person is / this spot". Each label is independent — you may record
        several different labels.
    - goto(location: str) may target either a known location OR any ``label``
        you recorded earlier in THIS plan with record_position.

    Rule: every goto to a recorded label must come AFTER its record_position.
""").strip()


def patch_catalogue():
    cut.ACTION_CATALOGUE_DESCRIPTION = (
        cut.ACTION_CATALOGUE_DESCRIPTION + "\n\n" + DYNAMIC_VOCAB
    )


KNOWN = None  # filled in main()


def consistency(plan):
    """Return (ok, notes): every goto to a non-known loc needs a prior record."""
    recorded = set()
    notes = []
    ok = True
    for i, s in enumerate(plan):
        if not isinstance(s, dict):
            continue
        act, params = s.get("action"), (s.get("params") or {})
        if act == "record_position":
            lab = params.get("label")
            if lab:
                recorded.add(lab.lower())
        elif act == "goto":
            loc = (params.get("location") or "").lower()
            if loc and loc not in KNOWN and loc not in recorded:
                ok = False
                notes.append(f"step {i}: goto({loc!r}) has no prior record_position")
    return ok, notes


COMMANDS = [
    # 2 dynamic: my spot + the person's spot
    "remember where I am standing, then go to the living_room and find a waving "
    "person and remember where they are, then go back to where I was and tell me "
    "you found them",
    # 2 dynamic: two marked spots, then shuttle between them
    "mark your current position as the pickup point, go to the kitchen, then "
    "remember the kitchen as the dropoff point, and finally drive back to the "
    "pickup point",
    # 3 dynamic: here, the bedroom person, the office person
    "record your current location, go to the bedroom and record where the "
    "sitting person is, then go to the office and record where the standing "
    "person is, then return to your starting location",
]


def main() -> int:
    global KNOWN
    patch_catalogue()
    kb = cut.load_user_vocabulary()
    KNOWN = {x.lower() for x in (kb.rooms + kb.locations)} | {"start_position"}
    known_obj_names = list(
        json.loads((HERE / "constants.json").read_text())["possible_objects"].keys()
    )
    client = openai.OpenAI(api_key=OPENAI_API_KEY, base_url="https://openrouter.ai/api/v1")

    all_ok = True
    for i, cmd in enumerate(COMMANDS, 1):
        r = cut.run_one(client, cmd, kb, known_obj_names)
        print(f"\n[{i}] {cmd}")
        if "error" in r:
            print(f"    ERROR: {r['error']}")
            all_ok = False
            continue
        plan = r.get("plan") or []
        steps = " -> ".join(
            f"{s.get('action')}({','.join(f'{k}={v}' for k,v in (s.get('params') or {}).items())})"
            for s in plan if isinstance(s, dict)
        )
        print(f"    {steps}")
        # how many DISTINCT dynamic labels did it fix?
        labels = {
            (s.get("params") or {}).get("label", "").lower()
            for s in plan if isinstance(s, dict) and s.get("action") == "record_position"
        } - {""}
        ok, notes = consistency(plan)
        all_ok = all_ok and ok and len(labels) >= 2
        print(f"    dynamic labels fixed: {sorted(labels)}  ({len(labels)})")
        print(f"    self-consistent: {ok}" + (f"  issues={notes}" if notes else ""))

    verdict = ("fixed 2+ consistent dynamic locations on every command"
               if all_ok else "did not satisfy 2+/consistency on all commands")
    print(f"\n{'PASS' if all_ok else 'CHECK'}: LLM {verdict}.")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
