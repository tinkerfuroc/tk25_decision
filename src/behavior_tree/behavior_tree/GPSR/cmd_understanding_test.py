"""Command-understanding test for the GPSR planner.

Runs the official RoboCup@Home command generator (vendored as
``_official_cmd_gen.py``) against the user vocabulary defined in
``constants.json``, then feeds each generated command into the same OpenAI call
that ``BtNode_PlanActions`` (orchestrator.py) makes — but without any ROS / BT
overhead. Validates the returned plan structure and prints a summary table plus
a markdown report.

Run with the decision venv directly (does not require ROS sourced):

    /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python \
        src/behavior_tree/behavior_tree/GPSR/cmd_understanding_test.py
"""

import argparse
import json
import os
import random
import sys
import textwrap
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import openai

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from _official_cmd_gen import CommandGenerator  # vendored upstream
from config import (
    OPENAI_API_KEY,
    OPENAI_MODEL,
    OPENAI_TEMPERATURE,
    OPENAI_MAX_TOKENS,
)
from planner_validators import validate_plan as shared_validate_plan


CONSTANTS_PATH = HERE / "constants.json"
REPORT_PATH = HERE / "cmd_understanding_results.md"
NUM_COMMANDS = 30
SEED = 42

ACTION_FACTORY_NAMES = {
    "goto",
    "find_object",
    "find_person",
    "approach_person",
    "describe_person",
    "ask_person",
    "report_answer",
    "follow",
    "guide",
    "grasp",
    "place",
    "deliver",
    "count",
    "answer_question",
    "announce",
    "record_position",
    "vlm_fallback",
    "llm_fallback",
    "report_view",
}


# ---------------------------------------------------------------------------
# Build a Knowledge stub matching the upstream Knowledge dataclass shape, but
# populated with the user's actual KNOWN_LOCATIONS / KNOWN_OBJECT_PROMPTS so
# generated commands stay inside the planner's vocabulary.
# ---------------------------------------------------------------------------

@dataclass
class StubKnowledge:
    names: List[str]
    locations: List[str]
    placement_locations: List[str]
    rooms: List[str]
    objects: List[str]
    object_categories_singular: List[str]
    object_categories_plural: List[str]


def load_user_vocabulary() -> StubKnowledge:
    raw = json.loads(CONSTANTS_PATH.read_text())
    rooms_raw = list(raw.get("egpsr_rooms", {}).keys())
    poses_raw = list(raw.get("possible_poses", {}).keys())

    # Rooms = top-level rooms only.
    rooms = [r for r in rooms_raw if r not in {"QA_point"}]
    # Locations = furniture / placements (everything in possible_poses that is not a room)
    locations = [p for p in poses_raw if p not in rooms and p != "QA_point"]
    # Placement locations = those that look like flat-surface placements.
    placement_locations = ["shelf", "desk_lamp"]

    objects = list(raw.get("possible_objects", {}).keys())

    return StubKnowledge(
        names=["Alex", "Sarah", "John", "Emma", "Liam", "Olivia"],
        locations=locations or ["shelf"],
        placement_locations=placement_locations,
        rooms=rooms,
        objects=objects,
        object_categories_singular=["fruit", "drink", "snack"],
        object_categories_plural=["fruits", "drinks", "snacks"],
    )


# ---------------------------------------------------------------------------
# Planner prompt — copied verbatim from orchestrator.py so the test mirrors the
# exact LLM call BtNode_PlanActions makes. If orchestrator.py changes, update
# this block.
# ---------------------------------------------------------------------------

ACTION_CATALOGUE_DESCRIPTION = textwrap.dedent("""
    Available atomic actions and their parameter schemas:

    - goto(location: str)
        Navigate to a known room or placement. ``location`` must be one of
        the known locations, the special location ``start_position`` (the
        place where the robot received the command — use it to come back to
        the operator), OR any label fixed earlier in this plan with
        ``record_position(label=...)``.
    - find_object(object: str, location?: str)
        Visually search for an object from where the robot is standing. If
        the command names a search location, you MUST emit an explicit
        goto(location=...) step BEFORE this one (and pass the same
        ``location`` here for bookkeeping). ``object`` may be a specific known name (e.g.
        ``coke``) OR a category noun (e.g. ``drink``, ``fruit``, ``snack``).
        When the command was phrased with a category ("bring me a drink",
        "fetch a fruit", "tell me how many snacks"), pass the category noun
        through verbatim. Do not substitute a concrete instance — the vision
        module needs the category to enumerate all matches.
    - find_person(descriptor: str)
        Locate a person matching ``descriptor`` (e.g. "waving person",
        "person in a red shirt", "John") and store their position. Use
        "waving person" when the command mentions a waving/gesture cue. This
        only LOCATES — it does not move the robot.
    - approach_person()
        Walk the robot up to the person located by the most recent
        ``find_person``. Emit this after ``find_person`` whenever the next
        step needs the robot standing next to the person (describe_person,
        follow, guide, handing something over).
    - describe_person()
        Look at the person currently in view and speak a description of their
        VISIBLE appearance only — what they look like, their clothing, pose,
        gesture, or what they are holding. This is vision only: it CANNOT learn
        a spoken fact such as the person's name, age, or favourite drink — use
        ``ask_person`` for those. Plan ``find_person`` then ``approach_person``
        FIRST, then this.
    - ask_person(question: str)
        Ask the person the robot is standing next to a spoken question and
        record their answer. Use for any fact you can ONLY get by asking — the
        person's name, age, favourite drink, where they live. ``question`` is
        the literal sentence to speak ("What is your name?"). Plan
        ``find_person`` then ``approach_person`` FIRST. The answer is remembered
        for a later ``report_answer``.
    - report_answer()
        Speak the answer captured by the most recent ``ask_person`` back to the
        person in front of the robot. Use for "ask the person X and tell ME X":
        ``ask_person`` at the person, then ``goto(location=start_position)``,
        then ``report_answer``.
    - follow(person?: str)
        Continuously follow a person. ``person`` is optional and informational.
        Plan ``find_person`` then ``approach_person`` before it so the robot
        starts next to them. If the command says "follow X to the Y" (or
        "follow them to the Y"), emit ``follow(person=X)`` THEN
        ``goto(location=Y)`` so the destination is reached when the person stops.
    - guide(location: str)
        Lead a person to ``location``. MUST be preceded by ``find_person``
        then ``approach_person`` in the same plan (locate the person and walk
        to them before leading). Never use ``guide`` to express "go
        yourself" — that is ``goto``.
    - grasp(object: str)
        Pick up an object that is currently in view of the arm/vision system.
        Always plan find_object + goto first.
    - place(location: str)
        Place the currently-held object at ``location``. The robot navigates
        to ``location`` itself — do not add a separate ``goto`` before it.
    - deliver(object: str, recipient: str, recipient_location?: str)
        Hand the currently-held object to ``recipient``. The robot navigates
        to ``recipient_location`` (if given), then detects and approaches the
        recipient by itself — do not add find_person for the recipient, and
        do not add a separate ``goto`` to ``recipient_location`` before it.
        For "deliver/bring it to me", set ``recipient_location=start_position``.
    - count(object: str)
        Count visible instances of ``object`` and announce the number.
        count() performs its own visual scan — NEVER emit find_object for
        the same object before count. If the command names a location, emit
        goto first, then count directly.
    - answer_question()
        Listen to a question and answer it.
    - announce(text: str)
        Speak the literal ``text``. Use for "tell me the time/day", team
        info, reporting results, explaining a refusal — any spoken output.
        ``text`` must be the final resolved phrase.
    - record_position(label: str)
        Capture the robot's CURRENT pose and remember it under ``label`` for
        the rest of this task, so a later ``goto(location=<label>)`` can return
        to it. Use for a place that is NOT in the known-locations list but is
        referred to as "here / where I am / where this person is / this spot".
        Labels are independent — record as many distinct ones as the command
        needs. (The operator's spot is already captured automatically as
        ``start_position``; you don't need record_position for that.)
    - vlm_fallback(question: str)
        LAST-RESORT visual fallback: look at the scene and answer ``question``
        with a vision model, speaking the answer where it looks. Use ONLY when a
        clause needs the robot to LOOK at something and NO specific action
        covers it (colour/state/contents of a thing). Navigate there first.
        Never follow it with a templated announce. For "go look at X and tell
        ME": vlm_fallback at X → goto(start_position) → report_view. Prefer
        count / describe_person / find_object when they fit.
    - report_view()
        Speak what the most recent ``vlm_fallback`` saw back to the operator
        (visual twin of report_answer). Use after returning to start_position
        for "go look at X, then tell ME".
    - llm_fallback(question: str)
        LAST-RESORT general-knowledge fallback: answer a NON-visual question
        (date/day/time, a fact, arithmetic) with a language model. ``question``
        is the literal question; the robot speaks the answer itself.

    Compose composite commands by emitting multiple atomic actions in order.
""").strip()

SYSTEM_PROMPT = textwrap.dedent("""
    You are the task-planning module of a household service robot competing in
    RoboCup@Home GPSR. You translate a free-form natural-language instruction
    into an ordered list of atomic actions the robot can execute.

    Respond with JSON only, in this exact shape:
    {
      "reasoning": "<short explanation of the plan>",
      "plan": [
        {"action": "<action_name>", "params": {<key>: <value>, ...}},
        ...
      ]
    }

    Use only the action names listed below. Use only known locations and objects.

    Hard planning rules — your plan WILL be rejected if you violate any:
    1. Do not silently drop any clause from the command. Every clause must
       map to a step, OR the plan must end with an ``announce(text=...)``
       step that names the clause you could not execute.
    2. ``guide(location=X)`` is only valid AFTER a ``find_person`` step in
       the same plan. To move the robot itself, use ``goto(location=X)``.
    3. When the command refers to an OBJECT CATEGORY ("a drink", "a fruit",
       "a snack"), pass the category noun verbatim as the ``object`` param
       of ``find_object`` / ``count`` / ``grasp``. Do NOT pick a concrete
       instance — the robot needs the category to find any matching item.
    4. The ``text`` field of ``announce`` must be the literal final phrase.
       NEVER include angle-bracket placeholders like ``<day>``,
       ``<country>``, ``<name>``. If you cannot resolve the value, do not
       emit the step — or emit an ``announce`` that explains why.
    5. When the command says "follow X to the Y" or "follow them to the Y",
       emit ``follow`` then ``goto(location=Y)``.
    6. If the command is impossible, return an empty plan with a reasoning
       that explains why. Empty-with-reasoning is preferred to a partial
       plan that is wrong.
    7. When the command names the place to search ("find X in the kitchen",
       "how many X on the shelf"), the FIRST step for that clause must be
       an explicit ``goto(location=...)``. Navigation is never implicit.
    8. ``count`` does its own scanning. Never emit ``find_object`` for an
       object that a ``count`` step will count.
    9. The place where the robot received the command is the known location
       ``start_position``. Use it for clauses addressed to ME (the operator):
       - "deliver/bring it to me": set ``recipient_location=start_position``
         on the ``deliver`` step. ``deliver`` drives there itself — do NOT
         add a separate ``goto`` before it.
       - "tell me / say to me": emit ``goto(location=start_position)`` before
         the final ``announce`` (``announce`` does not move the robot).
    10. ``deliver`` and ``place`` navigate to their own destination
       (``recipient_location`` / ``location``). NEVER put a ``goto`` to that
       same place immediately before them — it would drive there twice. This
       applies to every recipient, not just ME.
    11. If the command reports a result to ME ("tell me ...", "show me ...")
       and the robot had to leave to do the task, the final ``announce`` MUST
       be immediately preceded by ``goto(location=start_position)`` so the
       robot reports back at the operator. (For "bring/give me", use
       ``deliver`` with ``recipient_location=start_position`` instead.)
    12. For a place not in the known list but named at runtime ("here",
       "this spot", "where I am / where this person is"), emit
       ``record_position(label=<name>)`` to fix it, and only ``goto`` that
       label AFTER the record_position step.
    13. Choose person-info actions by HOW the fact is obtained, not by the word
       "tell":
       - VISIBLE traits — what the person is WEARING or HOLDING, their POSE,
         GESTURE, hair/clothing colour, general appearance — use
         ``describe_person``. The robot SEES these; never ask for them.
       - Facts obtainable ONLY by asking — the person's NAME, AGE, FAVOURITE
         DRINK, where they are from — use ``ask_person(question=...)``. NEVER
         use ``describe_person`` for these, and NEVER a bare ``announce``.
       For "ask the person X and tell ME X" (X askable): ``ask_person`` →
       ``goto(location=start_position)`` → ``report_answer``.
    14. Do not refuse a clause just because no exact action fits. If it needs
       LOOKING at the scene, use ``vlm_fallback(question=...)``; if it is a
       general non-visual question (date/day/time, a fact), use
       ``llm_fallback(question=...)``. Last resorts only — prefer a specific
       action when one fits.
""").strip()


def build_user_prompt(command: str, kb: StubKnowledge, known_obj_names: List[str],
                      failure_msg: str = None) -> str:
    from datetime import datetime
    known_loc = ", ".join(sorted(kb.rooms + kb.locations))
    known_obj = ", ".join(sorted(known_obj_names))
    body = (
        f"Current date and time: {datetime.now().strftime('%A, %B %d, %Y, %H:%M')}\n"
        f"Known locations: {known_loc}\n"
        f"Known objects: {known_obj}\n\n"
        f"{ACTION_CATALOGUE_DESCRIPTION}\n\n"
        f"Command:\n{command}\n\n"
        f"Completed steps so far:\n[]\n"
    )
    if failure_msg:
        # Mirror orchestrator._build_planner_user_prompt's rephrase-on-failure
        # path so the offline test re-plans the same way the runtime does.
        body += (
            f"\nYour previous plan was REJECTED: {failure_msg}\n"
            "Fix exactly that problem and return a corrected plan.\n"
        )
    body += "\nReturn the JSON plan now."
    return body


# ---------------------------------------------------------------------------
# Plan validation
# ---------------------------------------------------------------------------

CATEGORY_WORDS = frozenset({
    "drink", "drinks",
    "fruit", "fruits",
    "snack", "snacks",
    "food", "foods",
})


def validate_plan(plan: List[Dict[str, Any]], kb: StubKnowledge, command: str) -> Dict[str, Any]:
    """Return a dict of structural checks + the shared validator's verdict."""
    issues = []
    known_loc_set = set(kb.rooms) | set(kb.locations) | {"start_position"}
    # Runtime-recorded dynamic labels count as known locations for the
    # structural check (the shared validator also enforces record-before-goto).
    known_loc_set |= {
        (s.get("params") or {}).get("label", "")
        for s in plan
        if isinstance(s, dict) and s.get("action") == "record_position"
    } - {""}
    known_obj_set = set(kb.objects) | set(CATEGORY_WORDS)

    unknown_actions = []
    unknown_locations = []
    unknown_objects = []

    for step in plan:
        if not isinstance(step, dict):
            issues.append(f"non-dict step: {step!r}")
            continue
        action = step.get("action")
        params = step.get("params", {}) or {}
        if action not in ACTION_FACTORY_NAMES:
            unknown_actions.append(action)
        loc = params.get("location") or params.get("recipient_location")
        if loc and loc not in known_loc_set:
            unknown_locations.append(loc)
        obj = params.get("object")
        if obj and obj not in known_obj_set:
            unknown_objects.append(obj)

    # Shared validator (the one the orchestrator uses at runtime). It runs on
    # the plan AFTER the orchestrator's clean step that drops malformed/None
    # action entries — mirror that here so structural noise doesn't double-fail
    # the guard.
    cleaned_for_guard = [
        s for s in plan
        if isinstance(s, dict) and s.get("action") in ACTION_FACTORY_NAMES
    ]
    shared_ok, shared_reason = shared_validate_plan(
        cleaned_for_guard, command, ACTION_FACTORY_NAMES, CATEGORY_WORDS,
        known_locations=known_loc_set,
    )

    return {
        "n_steps": len(plan),
        "empty": len(plan) == 0,
        "unknown_actions": unknown_actions,
        "unknown_locations": unknown_locations,
        "unknown_objects": unknown_objects,
        "structural_issues": issues,
        "shared_ok": shared_ok,
        "shared_reason": shared_reason,
        "ok": (
            len(plan) > 0
            and not unknown_actions
            and not unknown_locations
            and not unknown_objects
            and not issues
            and shared_ok
        ),
    }


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def run_one(client: openai.OpenAI, command: str, kb: StubKnowledge,
            known_obj_names: List[str], failure_msg: str = None) -> Dict[str, Any]:
    user_prompt = build_user_prompt(command, kb, known_obj_names, failure_msg)
    t0 = time.time()
    try:
        resp = client.chat.completions.create(
            model=OPENAI_MODEL,
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": user_prompt},
            ],
            temperature=OPENAI_TEMPERATURE,
            max_tokens=max(OPENAI_MAX_TOKENS, 1024),
            response_format={"type": "json_object"},
        )
        raw = resp.choices[0].message.content.strip()
        parsed = json.loads(raw)
        elapsed = time.time() - t0
    except Exception as exc:  # noqa: BLE001
        return {
            "command": command,
            "elapsed_s": time.time() - t0,
            "error": repr(exc),
        }

    plan = parsed.get("plan", [])
    if not isinstance(plan, list):
        return {
            "command": command,
            "elapsed_s": elapsed,
            "error": f"non-list plan: {plan!r}",
            "reasoning": parsed.get("reasoning"),
        }

    validation = validate_plan(plan, kb, command)
    return {
        "command": command,
        "elapsed_s": elapsed,
        "reasoning": parsed.get("reasoning"),
        "plan": plan,
        "validation": validation,
    }


def fmt_actions(plan: List[Dict[str, Any]]) -> str:
    if not plan:
        return "(empty)"
    return " > ".join(
        f"{s.get('action')}({json.dumps(s.get('params', {}), separators=(',', ':'))})"
        for s in plan
    )


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="GPSR command-understanding test.")
    p.add_argument("--seed", type=int, default=SEED,
                   help=f"Random seed for command generation (default {SEED}).")
    p.add_argument("--n", type=int, default=NUM_COMMANDS,
                   help=f"Number of commands to generate (default {NUM_COMMANDS}).")
    p.add_argument("--command", action="append", default=[],
                   help=("Skip the generator — run a literal command through the "
                         "planner. Repeatable. Mixes with --commands-file."))
    p.add_argument("--commands-file", type=Path, default=None,
                   help="File with one command per line. Lines starting with # are skipped.")
    p.add_argument("--no-report", action="store_true",
                   help="Skip writing the markdown report file.")
    p.add_argument("--graphviz", type=Path, default=None,
                   help=("Emit a graphviz DOT file showing each command and "
                         "its planned action chain. If the `dot` binary is on "
                         "PATH, also renders alongside .svg and .png."))
    return p.parse_args()


def _dot_escape(s: str) -> str:
    return s.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")


def write_graphviz(commands: List[str], results: List[Dict[str, Any]], dot_path: Path) -> None:
    """Emit a DOT cluster-per-command showing the planned action chain."""
    lines = [
        "digraph gpsr_cmd_understanding {",
        '  rankdir=LR;',
        '  graph [fontname="Helvetica", labelloc="t", '
        f'label="GPSR command understanding — {len(commands)} commands"];',
        '  node  [fontname="Helvetica", shape=box, style="rounded,filled"];',
        '  edge  [fontname="Helvetica", fontsize=10];',
        "",
    ]
    for i, (cmd, r) in enumerate(zip(commands, results), 1):
        v = r.get("validation") or {}
        err = r.get("error")
        if err:
            cluster_color = '"#ffd6d6"'  # transport error → red
            status = f"ERROR: {err}"
        elif v.get("shared_ok") and v.get("ok"):
            cluster_color = '"#d6ffd6"'  # OK → green
            status = f"OK ({v['n_steps']} step{'s' if v['n_steps']!=1 else ''})"
        elif v.get("empty"):
            cluster_color = '"#f0f0f0"'  # empty → grey
            status = "EMPTY plan"
        elif not v.get("shared_ok"):
            cluster_color = '"#ffe6c2"'  # guard rejection → orange
            status = f"GUARD: {v.get('shared_reason','?')[:80]}"
        else:
            cluster_color = '"#fff2b3"'  # other issue → yellow
            status = "ISSUE"

        cluster_label = f"{i:02d}. {cmd} — {status}"
        lines.append(f"  subgraph cluster_{i} {{")
        lines.append(f'    label="{_dot_escape(cluster_label)}";')
        lines.append(f"    style=filled; color={cluster_color};")
        lines.append(f"    node [fillcolor=white];")

        plan = r.get("plan") or []
        if not plan:
            lines.append(f'    n{i}_0 [label="(empty)", fillcolor="#f0f0f0"];')
        else:
            prev_id = None
            for j, step in enumerate(plan):
                action = step.get("action") if isinstance(step, dict) else None
                params = step.get("params") or {} if isinstance(step, dict) else {}
                param_pairs = "\\n".join(f"{k}={v}" for k, v in params.items())
                fill = "white"
                if action not in ACTION_FACTORY_NAMES:
                    fill = '"#ffd6d6"'  # unknown action → red leaf
                label = f"{action or '?'}"
                if param_pairs:
                    label += f"\\n{param_pairs}"
                node_id = f"n{i}_{j}"
                lines.append(f'    {node_id} [label="{_dot_escape(label)}", fillcolor={fill}];')
                if prev_id:
                    lines.append(f"    {prev_id} -> {node_id};")
                prev_id = node_id
        lines.append("  }")
        lines.append("")

    lines.append("}")
    dot_path.write_text("\n".join(lines))


def main() -> int:
    args = _parse_args()
    random.seed(args.seed)
    kb = load_user_vocabulary()
    known_obj_names = list(json.loads(CONSTANTS_PATH.read_text())["possible_objects"].keys())
    gen = CommandGenerator(kb)

    # Source the command list: literal --command flags > --commands-file > generator.
    commands: List[str] = list(args.command)
    if args.commands_file:
        for line in args.commands_file.read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#"):
                commands.append(line)

    if commands:
        print(f"Using {len(commands)} user-supplied command(s):")
        for c in commands:
            print(f"  - {c}")
        print()
    else:
        print(f"Generating {args.n} commands using the official generator "
              f"(seed={args.seed})…")
        print(f"  rooms      = {kb.rooms}")
        print(f"  locations  = {kb.locations}")
        print(f"  objects    = {kb.objects}")
        print()
        half = args.n // 2
        for _ in range(half):
            commands.append(gen.generate_command_start("people"))
        for _ in range(args.n - half):
            commands.append(gen.generate_command_start("objects"))
        random.shuffle(commands)
    n_total = len(commands)

    client = openai.OpenAI(api_key=OPENAI_API_KEY, base_url="https://openrouter.ai/api/v1")

    results: List[Dict[str, Any]] = []
    for i, cmd in enumerate(commands, 1):
        print(f"[{i:02d}/{n_total}] {cmd}")
        result = run_one(client, cmd, kb, known_obj_names)
        results.append(result)
        if "error" in result:
            print(f"   ERROR: {result['error']}")
        else:
            v = result["validation"]
            status = "OK" if v["ok"] else "FAIL"
            issues = []
            if v["empty"]:
                issues.append("empty plan")
            if v["unknown_actions"]:
                issues.append(f"unknown action(s) {v['unknown_actions']}")
            if v["unknown_locations"]:
                issues.append(f"unknown loc(s) {v['unknown_locations']}")
            if v["unknown_objects"]:
                issues.append(f"unknown obj(s) {v['unknown_objects']}")
            if not v["shared_ok"]:
                issues.append(f"guard: {v['shared_reason']}")
            tag = f" — {'; '.join(issues)}" if issues else ""
            print(f"   {status} ({v['n_steps']} steps){tag}")
            print(f"   plan: {fmt_actions(result['plan'])}")
        print()

    # --- summary ---
    n_ok = sum(1 for r in results if r.get("validation", {}).get("ok"))
    n_empty = sum(1 for r in results if r.get("validation", {}).get("empty"))
    n_err = sum(1 for r in results if "error" in r)
    n_unknown_action = sum(1 for r in results if r.get("validation", {}).get("unknown_actions"))
    n_unknown_loc = sum(1 for r in results if r.get("validation", {}).get("unknown_locations"))
    n_unknown_obj = sum(1 for r in results if r.get("validation", {}).get("unknown_objects"))
    n_guard_fail = sum(
        1 for r in results
        if r.get("validation") and not r["validation"]["shared_ok"]
    )

    print("=" * 60)
    print(f"Summary: {n_ok}/{n_total} fully OK")
    print(f"  empty plan:           {n_empty}")
    print(f"  LLM/transport errors: {n_err}")
    print(f"  unknown actions:      {n_unknown_action}")
    print(f"  unknown locations:    {n_unknown_loc}")
    print(f"  unknown objects:      {n_unknown_obj}")
    print(f"  guard rejections:     {n_guard_fail}")

    if not args.no_report:
        write_report(commands, results, kb, n_ok, n_empty, n_err,
                     n_unknown_action, n_unknown_loc, n_unknown_obj, n_guard_fail)
        print(f"\nMarkdown report written to: {REPORT_PATH}")
    if args.graphviz:
        write_graphviz(commands, results, args.graphviz)
        print(f"Graphviz DOT written to: {args.graphviz}")
        import shutil, subprocess
        if shutil.which("dot"):
            for fmt in ("svg", "png"):
                out = args.graphviz.with_suffix(f".{fmt}")
                subprocess.run(["dot", f"-T{fmt}", str(args.graphviz), "-o", str(out)], check=True)
                print(f"  -> rendered {out}")
        else:
            print("  (graphviz `dot` not found on PATH — DOT only)")
    return 0 if n_err == 0 else 1


def write_report(commands, results, kb, n_ok, n_empty, n_err,
                 n_unknown_action, n_unknown_loc, n_unknown_obj, n_guard_fail):
    lines = []
    lines.append("# GPSR Command-Understanding Test")
    lines.append("")
    lines.append(f"- Generator: official RoboCup@Home `CommandGenerator` "
                 f"(vendored as `_official_cmd_gen.py`)")
    lines.append(f"- Planner: `BtNode_PlanActions` LLM call from `orchestrator.py`")
    lines.append(f"- Model: `{OPENAI_MODEL}` via OpenRouter")
    lines.append(f"- Commands tested: {NUM_COMMANDS}")
    lines.append(f"- Random seed: {SEED}")
    lines.append("")
    lines.append("## Vocabulary fed to generator + planner")
    lines.append(f"- **Rooms:** {', '.join(kb.rooms)}")
    lines.append(f"- **Locations:** {', '.join(kb.locations)}")
    lines.append(f"- **Objects:** {', '.join(kb.objects)}")
    lines.append("")
    lines.append("## Summary")
    lines.append("")
    lines.append(f"| Metric | Count |")
    lines.append(f"|---|---|")
    lines.append(f"| Fully OK | {n_ok} / {NUM_COMMANDS} |")
    lines.append(f"| Empty plans | {n_empty} |")
    lines.append(f"| LLM/transport errors | {n_err} |")
    lines.append(f"| Commands with unknown actions | {n_unknown_action} |")
    lines.append(f"| Commands with unknown locations | {n_unknown_loc} |")
    lines.append(f"| Commands with unknown objects | {n_unknown_obj} |")
    lines.append(f"| Guard-validator rejections | {n_guard_fail} |")
    lines.append("")
    lines.append("## Per-command results")
    lines.append("")
    for i, r in enumerate(results, 1):
        cmd = r["command"]
        lines.append(f"### {i}. `{cmd}`")
        if "error" in r:
            lines.append(f"- **ERROR:** {r['error']}")
            lines.append("")
            continue
        v = r["validation"]
        status = "OK" if v["ok"] else "FAIL"
        lines.append(f"- **Status:** {status}  ({v['n_steps']} step(s), {r['elapsed_s']:.2f}s)")
        if r.get("reasoning"):
            lines.append(f"- **Reasoning:** {r['reasoning']}")
        lines.append(f"- **Plan:**")
        for step in r["plan"]:
            lines.append(f"  - `{step.get('action')}({json.dumps(step.get('params', {}), separators=(',', ':'))})`")
        if v["unknown_actions"]:
            lines.append(f"- ⚠ unknown actions: {v['unknown_actions']}")
        if v["unknown_locations"]:
            lines.append(f"- ⚠ unknown locations: {v['unknown_locations']}")
        if v["unknown_objects"]:
            lines.append(f"- ⚠ unknown objects: {v['unknown_objects']}")
        if not v["shared_ok"]:
            lines.append(f"- ⛔ guard rejection: {v['shared_reason']}")
        lines.append("")
    REPORT_PATH.write_text("\n".join(lines))


if __name__ == "__main__":
    sys.exit(main())
