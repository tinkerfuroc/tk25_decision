"""Physical-truth verdict for tier-2 bench runs (pure functions, no ROS).

The executor's postcondition facts (``held()``, ``placed()``, ``counted()``, ...) are the
robot's BELIEFS: a grasp that reported success, a ``place`` that merely opened the gripper,
a count whose gate only checks provenance. Recordings of runs scored PASS showed nothing
grasped, a count of 0 accepted, and objects dropped on the floor. This module re-scores
each split target's postconditions against sim physics truth written by the sim-side probe
into ``<run_dir>/run_truth.json``::

    {"schema_version": 1, "samples": int, "any_object_retained": bool|null,
     "objects": {"<object_id>": {"first_xyz": [x,y,z], "last_xyz": [x,y,z],
                                 "ever_retained": bool, "retained_at_end": bool,
                                 "max_lift_m": f, "max_xy_disp_m": f, "samples": int}},
     "robot": {"last_xyz": [x,y,z], "last_yaw_deg": f, "samples": int} | null,
     "surfaces": {"<spot>": {"surface_xyz": [x,y,z], "half_extent_xy": [hx,hy]}},
     "persons": {"<room>": {"xyz": [x,y,z]}}}

``object_id`` may be the short id from ``spawned.json`` (``cmd_sugar_box_0``) or the prim
path (``/World/Scenario/cmd_sugar_box_0``); both are matched on the path suffix.

Everything here is a BENCH ORACLE used strictly after the fact to score a run -- never a
runtime input of the stack or the behaviour tree.

Verdict vocabulary per check: ``PASS`` / ``FAIL`` (physics decided), ``UNKNOWN`` (a
physical check exists for the fact but the data to decide it is missing), ``PLANNER``
(no physical check for this fact; the planner gate's own verdict stands).
"""
from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any, Iterable

TRUTH_FILE = "run_truth.json"
LEGACY_GRASP_TRUTH_FILE = "grasp_truth.json"

# Tolerances (metres). ``placed``: XY slack beyond the surface half-extent and the Z band
# above the surface plane an object may rest in (tall objects / stacked on another).
PLACED_XY_SLACK_M = 0.15
PLACED_Z_BELOW_M = 0.05
PLACED_Z_ABOVE_M = 0.30
DELIVERED_RADIUS_M = 1.2
AT_ROBOT_RADIUS_M = 0.6
# A person-directed answer is only physically useful when the robot is close enough
# to the person and its base is facing them.  These are deliberately looser than the
# navigation controller's standoff target; the oracle checks communication intent,
# rather than requiring a particular approach controller implementation.
PERSON_SPEECH_RADIUS_M = 1.5
PERSON_SPEECH_HEADING_TOLERANCE_DEG = 55.0
# A delivered object must have LEFT the hand: fix8 run s70718-000 ended with
# the soup still in the gripper (z=1.16 m, 0.28 m from the robot) after the
# approach to the person aborted, and the old check scored PASS because the
# robot stood near the operator's start pose. An object this high and this
# close to the robot at the end was never handed over (the sim referee hold
# releases on gripper open, so a real hand-over drops it).
STILL_IN_HAND_MIN_Z_M = 0.3
STILL_IN_HAND_MAX_XY_M = 0.6

# Grasp truth is MOTION-based. The probe's ``retained_by_gripper`` is only evaluated for
# the first scenario-declared object, so bench-spawned entities always publish
# ``ever_retained: False`` -- non-decisive. An object was carried when it was lifted
# >= LIFT_CARRIED_M, or displaced >= XY_CARRIED_M while lifted >= LIFT_MIN_M; it was never
# touched when lift < LIFT_STILL_M and displacement < XY_STILL_M; anything in between
# (nudged/pushed, never lifted clear) is a FAIL too -- not carried. ``ever_retained: True``
# remains an additional PASS signal.
LIFT_CARRIED_M = 0.05
LIFT_MIN_M = 0.02
XY_CARRIED_M = 0.15
LIFT_STILL_M = 0.02
XY_STILL_M = 0.05

# Recipients / locations that mean "the operator's start pose" (the command point).
OPERATOR_ALIASES = frozenset({"me", "operator", "start_position", "start", "command_point",
                              "start_point", "instruction_point"})
_OPERATOR_POSE_KEYS = ("command_point", "start_position", "instruction_point")

PHYSICAL_TRUTH_UNAVAILABLE = "physical truth unavailable"

_FACT_RE = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*\((.*)\)\s*$", re.DOTALL)
_OBJECT_ID_RE = re.compile(r"^cmd_(.+?)(?:_(\d+))?$")
# Announced-count sources, in priority order (see ``announced_count``).
# The SPOKEN result (announce/buffer leaves of the count subtree) is
# authoritative: since the count sweep keeps the max over several views, the
# per-view "counted N objects" lines below can end on a 0 view AFTER the
# spoken value (fix4b run 002: announced 2, scored "counted 0 objects").
_LOG_SPOKEN_COUNT_RES = (
    re.compile(r"buffered: I counted (\d+)\b"),
    re.compile(r"Finished announcing I counted (\d+)\b"),
)
_LOG_COUNT_RES = (
    re.compile(r"parsed count=(\d+) from VLM answer"),
    re.compile(r"\bcounted (\d+) objects\b"),
    re.compile(r"synthetic count=(\d+)"),
)
_ANNOUNCE_I_COUNTED_RE = re.compile(r"^\s*I counted\s+(\d+)\b", re.IGNORECASE)
_ANNOUNCE_COUNT_SENTENCE_RE = re.compile(r"\b(visible|there (?:are|is)|count)", re.IGNORECASE)
# Plan narration / intentions that mention counting but carry no result.
_ANNOUNCE_NOT_A_RESULT_RE = re.compile(
    r"\b(my plan|I will|tasks?\b|first,|then,|let me|please wait|counting now|look again)", re.IGNORECASE)
_NUMBER_WORDS = {
    "zero": 0, "no": 0, "none": 0, "one": 1, "a single": 1, "two": 2, "three": 3, "four": 4,
    "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10, "eleven": 11,
    "twelve": 12, "thirteen": 13, "fourteen": 14, "fifteen": 15, "sixteen": 16,
    "seventeen": 17, "eighteen": 18, "nineteen": 19, "twenty": 20,
}
_NUMBER_TOKEN_RE = re.compile(  # "**0 persons**" (markdown bold) must still match
    r"(?<!\w)(\d+|" + "|".join(re.escape(w) for w in _NUMBER_WORDS) + r")(?!\w)",
    re.IGNORECASE,
)
_PERSON_TOKENS = ("person", "people", "guest", "human", "someone", "somebody")
_ANSWER_BOILERPLATE = re.compile(
    r"^(?:hi,? i am|the door is open|going to |i am trying to go|"
    r"i am planning|please give me|here is my plan|i have \d+ tasks?|"
    r"starting task|my plan:|next:|i am ready|i will |report the result)",
    re.IGNORECASE,
)


# --------------------------------------------------------------------------- loaders

def _read_json(path: Path) -> Any:
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, ValueError):
        return None


def load_truth(run_dir: Path) -> dict | None:
    """``run_truth.json`` as a dict, or ``None`` when absent/unparseable.

    Falls back to the older ``grasp_truth.json`` (the closed-on-air probe, which only knows
    ``any_object_retained``/``max_xy_disp_m``) by lifting it into the run_truth schema with an
    empty ``objects`` map -- so ``held()`` can still FAIL on an explicit "nothing was ever
    retained" and everything else stays UNKNOWN.
    """
    run_dir = Path(run_dir)
    data = _read_json(run_dir / TRUTH_FILE)
    if isinstance(data, dict):
        return data
    legacy = _read_json(run_dir / LEGACY_GRASP_TRUTH_FILE)
    if isinstance(legacy, dict):
        return {
            "schema_version": 0, "samples": legacy.get("samples"),
            "any_object_retained": legacy.get("any_object_retained"),
            "max_xy_disp_m": legacy.get("max_xy_disp_m"),
            "objects": {}, "robot": None, "surfaces": {}, "persons": {},
            "legacy_source": LEGACY_GRASP_TRUTH_FILE,
        }
    return None


def load_constants(path: Path | None) -> dict:
    """The subset of ``constants.*.json`` the checks need; missing file -> empty maps."""
    data = _read_json(Path(path)) if path else None
    if not isinstance(data, dict):
        data = {}
    out = {}
    for key in ("possible_poses", "categories", "search_spots", "possible_objects"):
        val = data.get(key)
        out[key] = {k: v for k, v in val.items() if not str(k).startswith("_")} \
            if isinstance(val, dict) else {}
    return out


def newest_events_file(run_dir: Path) -> Path | None:
    """The run's ``debug/*/events.jsonl`` (newest by mtime when several exist)."""
    debug = Path(run_dir) / "debug"
    if not debug.is_dir():
        return None
    candidates = [p / "events.jsonl" for p in debug.iterdir() if (p / "events.jsonl").is_file()]
    return max(candidates, key=lambda p: p.stat().st_mtime) if candidates else None


def _event_records(run_dir: Path, task_slot: int = 1) -> list[dict]:
    """Read the run's telemetry events for one task slot.

    Physical checks historically only consumed ``split.accepted`` and gate events.  The
    communication oracle also needs the materialized action plan and the step outcomes,
    so keep this tiny loader local to this module instead of making the report layer know
    about the telemetry format.
    """
    path = newest_events_file(run_dir)
    if path is None:
        return []
    from behavior_tree.GPSR.bench.events import slot_of

    out: list[dict] = []
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return out
    for line in lines:
        try:
            event = json.loads(line)
        except (TypeError, json.JSONDecodeError):
            continue
        if not isinstance(event, dict):
            continue
        slot = slot_of(event.get("task_id"))
        if slot is not None and slot != task_slot:
            continue
        out.append(event)
    return out


def _materialized_steps(events: list[dict]) -> dict[int, list[dict]]:
    """Return the last materialized action list for each target index."""
    by_target: dict[int, list[dict]] = {}
    for event in events:
        if event.get("event_type") != "plan.materialized":
            continue
        payload = event.get("payload") or {}
        try:
            index = int(payload.get("target_index"))
        except (TypeError, ValueError):
            continue
        steps = payload.get("steps")
        if isinstance(steps, list):
            by_target[index] = [step for step in steps if isinstance(step, dict)]
    return by_target


def _step_outcomes(events: list[dict]) -> list[dict]:
    """Return action step outcomes in telemetry order."""
    out: list[dict] = []
    for event in events:
        if event.get("event_type") != "step.finished":
            continue
        payload = event.get("payload") or {}
        if isinstance(payload, dict) and payload.get("action"):
            out.append(payload)
    return out


def _is_successful_outcome(value: Any) -> bool:
    """Whether a telemetry step outcome represents a completed action."""
    return str(value or "").strip().lower() in {"succeeded", "success", "pass", "valid"}


def plan_execution_audit(run_dir: Path, targets: list[dict] | None,
                         *, task_slot: int = 1) -> dict:
    """Audit the *materialized* plans captured by telemetry.

    The executor's ``task.finished`` status only says that the behaviour tree reached a
    terminal state.  It does not prove that an answer target contained an explicit
    speech action, or that a person-directed answer retained the person hand-off.  This
    audit therefore checks the final materialized plan for each target before accepting
    a planner PASS.  It deliberately returns no opinion when there is no materialized
    telemetry (old/synthetic runs), preserving compatibility with planner-only reports.

    Person-directed answers require both ``approach_person`` and ``announce``.  A
    ``llm_fallback`` action may produce speech internally, but it is an information
    gathering/fallback action and is not a substitute for the explicit announce step
    required by the GPSR command contract.
    """
    events = _event_records(Path(run_dir), task_slot)
    materialized = _materialized_steps(events)
    if not materialized:
        return {"verdict": "PASS", "issues": [], "audited": False}
    clean_targets = [t for t in (targets or []) if isinstance(t, dict)]
    issues: list[dict] = []

    # Flatten all materialized actions in target order.  A person search in an earlier
    # target may establish the pose for a later answer target, so the approach check is
    # intentionally cross-target as well as same-target.
    prior_actions: list[tuple[int, str]] = []
    for index in sorted(materialized):
        steps = materialized[index]
        for step in steps:
            action = str(step.get("action") or "")
            prior_actions.append((index, action))
        target = clean_targets[index] if index < len(clean_targets) else None
        if target is None:
            continue
        answer = any(parse_fact(fact)[0] == "answered"
                     for fact in (target.get("postconditions") or [])
                     if isinstance(fact, str))
        if not answer:
            continue
        steps_actions = [str(step.get("action") or "") for step in steps]
        if "announce" not in steps_actions:
            issues.append({
                "target": index,
                "kind": "missing_announce",
                "reason": (f"answer target {index} has no explicit announce step "
                           f"(actions: {', '.join(steps_actions) or 'none'})"),
            })
        if _answer_targets_person(index, target, clean_targets):
            # Only actions from this target and earlier targets count.  The final
            # materialized map is already the accepted/replanned plan, so an approach
            # that existed only in an abandoned revision cannot satisfy this check.
            has_approach = any(action == "approach_person"
                               for target_index, action in prior_actions
                               if target_index <= index)
            if not has_approach:
                issues.append({
                    "target": index,
                    "kind": "missing_approach",
                    "reason": f"person-directed answer target {index} has no preceding approach_person",
                })

    return {"verdict": "FAIL" if issues else "PASS", "issues": issues,
            "audited": True}


_STARTUP_REPEAT_PATTERNS = {
    "ready": re.compile(r"^hi,? i am tinker\. i am ready for gpsr\. please open the door\.?$", re.I),
    "entering": re.compile(r"^the door is open\. entering the arena\.?$", re.I),
    "command_point": re.compile(r"^going to command point\.?$", re.I),
    # The startup node has used both phrasings over time; both are the same
    # activation and must count toward replay detection.
    "plan": re.compile(r"^i am planning, please wait(?: for a while)?\.?$", re.I),
}

# These are the node labels used by the startup and batch-flow trees.  The
# console tree renderer prints the complete tree on every tick, including the
# old ``Finished announcing ...`` feedback.  Counting those substrings counts
# snapshots, rather than executions.  Structured telemetry and the snapshot
# parser below therefore count a node only when its state transitions into
# SUCCESS.
_STARTUP_NODE_KEYS = {
    "ready": re.compile(r"^announce ready for gpsr$", re.I),
    "entering": re.compile(r"^announce entering arena$", re.I),
    "command_point": re.compile(r"^announce going$", re.I),
    "plan": re.compile(r"^announce planning task(?: \d+)?$", re.I),
}


def _startup_activation_counts(events: list[dict]) -> dict[str, int]:
    """Count startup/planning node activations from structured tree events.

    ``tree.tick_observed`` is intentionally ignored: it contains a full view of
    every visited node on every tick.  ``tree.node_states_changed`` is a delta
    stream, so a SUCCESS transition represents a completed node activation.  A
    tree can be regenerated after a dynamic plan is materialized; retaining the
    previous status by node id prevents that regeneration from being mistaken
    for a replay of the startup subtree.
    """
    counts = {key: 0 for key in _STARTUP_NODE_KEYS}
    names: dict[str, str] = {}
    states: dict[str, str] = {}
    for event in events:
        kind = event.get("event_type")
        payload = event.get("payload") or {}
        if not isinstance(payload, dict):
            continue
        if kind == "tree.generated":
            for node in payload.get("nodes", ()):
                if isinstance(node, dict):
                    node_id = node.get("id") or node.get("node_id")
                    name = node.get("name")
                    if node_id and isinstance(name, str):
                        names[str(node_id)] = name
            continue
        if kind != "tree.node_states_changed":
            continue
        for node in payload.get("nodes", ()):
            if not isinstance(node, dict):
                continue
            node_id = str(node.get("id") or node.get("node_id") or "")
            if not node_id:
                continue
            name = node.get("name") or names.get(node_id, "")
            if isinstance(name, str):
                names[node_id] = name
            status = str(node.get("status") or "")
            feedback = str(node.get("feedback") or "")
            previous = states.get(node_id)
            states[node_id] = status
            if status != "SUCCESS" or previous == "SUCCESS":
                continue
            for key, pattern in _STARTUP_NODE_KEYS.items():
                # ``announce going`` is also used by per-task navigation.  The
                # command-point startup activation is identified by its spoken
                # feedback, which is present in the structured delta.
                if key == "command_point" and "going to command point" not in feedback.lower():
                    continue
                if pattern.search(str(name)):
                    counts[key] += 1
                    break
    return counts


def _startup_snapshot_counts(log_lines: list[str]) -> dict[str, int]:
    """Count SUCCESS transitions in console tree snapshots.

    This is the degraded path for runs without structured telemetry.  A snapshot
    is delimited by the root tree header; repeated snapshots retain the same
    ``[✓]`` state and consequently do not increment the transition count.  A
    genuine replay first returns a node to a non-success state and then reaches
    ``[✓]`` again, which is counted.
    """
    counts = {key: 0 for key in _STARTUP_NODE_KEYS}
    if not log_lines:
        return counts
    block: list[str] = []
    blocks: list[list[str]] = []
    for line in log_lines:
        if "{-} GPSR orchestrator" in line and block:
            blocks.append(block)
            block = []
        block.append(line)
    if block:
        blocks.append(block)
    if len(blocks) <= 1:
        return counts
    previous: dict[str, str] = {}
    # Keep the final occurrence in a block; nested plan trees can contain an
    # ``announce`` label unrelated to startup, while these labels are unique in
    # the executor root.
    line_patterns = {
        key: re.compile(r"-->\s+" + pattern.pattern.strip("^") .strip("$")
                        + r"\s+\[(?P<status>[^\]]+)\]", re.I)
        for key, pattern in _STARTUP_NODE_KEYS.items()
    }
    for snapshot in blocks:
        current: dict[str, str] = {}
        for line in snapshot:
            for key, pattern in line_patterns.items():
                match = pattern.search(line)
                if match:
                    if key == "command_point" and "Finished announcing Going to command point" not in line:
                        continue
                    current[key] = match.group("status").strip()
        for key, status in current.items():
            if status == "✓" and previous.get(key) != "✓":
                counts[key] += 1
            previous[key] = status
    return counts


def execution_integrity_audit(run_dir: Path, *, task_slot: int = 1) -> dict:
    """Audit startup/replan evidence in ``announcements.txt`` and telemetry.

    Repeating the arena-entry/command-point sequence is a runtime integrity failure: it
    means the executor was re-entering its startup tree instead of executing the one
    accepted plan.  Likewise, a second materialized plan for a target is only justified
    after a failed step/target.  Keep this as an evidence audit so a viewer can explain
    why a terminal ``succeeded`` status was downgraded.
    """
    run_dir = Path(run_dir)
    issues: list[dict] = []
    ann_path = run_dir / "announcements.txt"
    # ``announcements.txt`` is the clean speech artifact, but the orchestrator
    # trace is the authoritative execution timeline.  A restarted root can
    # print the same startup subtree into the trace even when speech capture is
    # incomplete, so inspect both artifacts and retain their provenance in the
    # finding shown by the report/viewer.
    log_path = run_dir / "orchestrator.log"
    counts: dict[str, int] = {key: 0 for key in _STARTUP_REPEAT_PATTERNS}
    try:
        lines = ann_path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        lines = []
    try:
        log_lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        log_lines = []
    # ``orchestrator.log`` is a tree trace, not a speech event stream.  A full
    # tree snapshot is printed every tick and includes old SUCCESS feedback, so
    # substring counts are deliberately retained only as raw forensic data.
    raw_log_counts = {key: 0 for key in _STARTUP_REPEAT_PATTERNS}
    log_needles = {
        "ready": "Hi, I am Tinker. I am ready for GPSR. Please open the door",
        "entering": "The door is open. Entering the arena",
        "command_point": "Going to command point",
        "plan": "I am planning, please wait",
    }
    for line in log_lines[-200_000:]:
        for key, needle in log_needles.items():
            if needle.lower() in line.lower():
                raw_log_counts[key] += 1
    events = _event_records(run_dir, task_slot)
    structured_counts = _startup_activation_counts(events)
    snapshot_counts = _startup_snapshot_counts(log_lines[-200_000:])
    # Structured deltas are authoritative when present.  The snapshot parser
    # supplies a conservative fallback for older runs whose telemetry omitted
    # node-state deltas; it also remains transition-based, so stale snapshots do
    # not inflate the result.
    has_tree_deltas = any(event.get("event_type") == "tree.node_states_changed"
                          for event in events)
    if has_tree_deltas:
        log_counts = {
            key: max(structured_counts.get(key, 0), snapshot_counts.get(key, 0))
            for key in _STARTUP_REPEAT_PATTERNS
        }
    else:
        # A plain speech trace (without the console tree header) has no stale
        # snapshot structure to parse, so each matching line is an event in
        # that degraded format.  Keep this path for hardware-era logs and the
        # incomplete-capture guard; normal orchestrator traces take the
        # transition-based snapshot path above.
        log_counts = (raw_log_counts if log_lines and not any(snapshot_counts.values())
                      else snapshot_counts)
    # ``announcements.txt`` is the recorded speech stream.  It used to be
    # generated from every tree snapshot, which produced the exact failure
    # this audit is meant to catch: a run could actually replay the startup
    # speech hundreds of times while the transition parser saw one SUCCESS
    # activation.  Treat repeated speech as evidence even when a structured
    # tree trace exists.  Current runs extract one line per speech activation;
    # stale snapshots therefore remain harmless because they never enter this
    # artifact.
    ann_counts = {key: 0 for key in _STARTUP_REPEAT_PATTERNS}
    for line in lines:
        text = line.strip()
        for key, pattern in _STARTUP_REPEAT_PATTERNS.items():
            if pattern.search(text):
                ann_counts[key] += 1
    counts = ann_counts
    # One occurrence is expected.  A second arena-entry or command-point announcement
    # cannot be explained by a normal task plan and is enough to invalidate execution.
    for key in ("ready", "entering", "command_point"):
        # Use the larger observation from the speech file and trace.  The
        # source list tells a reviewer exactly which artifacts supplied it.
        observed = max(counts[key], log_counts[key])
        if observed > 1:
            sources = []
            if counts[key] > 1:
                sources.append("announcements.txt")
            if log_counts[key] > 1:
                sources.append("orchestrator.log")
            issues.append({"kind": "startup_repeat", "reason":
                           f"{key} node activated {observed} times "
                           f"(sources: {', '.join(sources)})",
                           "sources": sources, "announcement_count": counts[key],
                           "orchestrator_count": log_counts[key],
                           "raw_orchestrator_count": raw_log_counts[key]})
    observed_plan = max(counts["plan"], log_counts["plan"])
    if observed_plan > 1:
        sources = []
        if counts["plan"] > 1:
            sources.append("announcements.txt")
        if log_counts["plan"] > 1:
            sources.append("orchestrator.log")
        issues.append({"kind": "planning_repeat", "reason":
                       f"planning node activated {observed_plan} times "
                       f"(sources: {', '.join(sources)})",
                       "sources": sources, "announcement_count": counts["plan"],
                       "orchestrator_count": log_counts["plan"],
                       "raw_orchestrator_count": raw_log_counts["plan"]})

    # Replanning is allowed after an actual failed action.  A plan replacement without
    # such evidence is an unexplained loop and must not count as execution success.
    seen: set[int] = set()
    failed_since: dict[int, bool] = {}
    revisions: dict[int, int] = {}
    current_index: int | None = None
    for event in events:
        kind = event.get("event_type")
        payload = event.get("payload") or {}
        try:
            index = int(payload.get("target_index")) if isinstance(payload, dict) else None
        except (TypeError, ValueError):
            index = None
        if kind == "plan.materialized" and index is not None:
            current_index = index
            revisions[index] = revisions.get(index, 0) + 1
            if index in seen and not failed_since.get(index, False):
                issues.append({"kind": "replan_without_failure", "target": index,
                               "reason": (f"target {index} was materialized {revisions[index]} "
                                          "times without a failed step/target between plans")})
            seen.add(index)
            failed_since[index] = False
        elif kind == "target.failed" and index is not None:
            failed_since[index] = True
        elif kind == "step.finished":
            # Current telemetry puts target_index on plan.materialized but not on
            # step.finished.  Attribute the step to the most recent materialized
            # target so a failed step correctly authorizes its following replan.
            step_target = index if index is not None else current_index
            if (step_target is not None and isinstance(payload, dict)
                    and not _is_successful_outcome(payload.get("outcome"))):
                failed_since[step_target] = True
    return {"verdict": "FAIL" if issues else "PASS", "issues": issues,
            "counts": counts, "orchestrator_counts": log_counts,
            "raw_orchestrator_counts": raw_log_counts,
            "structured_counts": structured_counts,
            "snapshot_counts": snapshot_counts,
            "sources": [name for name, present in (("announcements.txt", bool(lines)),
                                                     ("orchestrator.log", bool(log_lines)),
                                                     ("events.jsonl", bool(events))) if present],
            "audited": bool(lines or log_lines or events)}


def _pose_from_payload(payload: dict) -> tuple[float, float, float, float] | None:
    """Extract ``(x, y, yaw_deg, confidence)`` from optional speech telemetry.

    Current recordings do not publish this field, which intentionally yields ``None``.
    Accept the common flat and nested forms so adding speech-time snapshots to the
    recorder does not require another scorer change.
    """
    candidates = []
    for key in ("robot_pose", "speech_pose", "pose"):
        value = payload.get(key)
        if isinstance(value, dict):
            candidates.append(value)
        elif isinstance(value, (list, tuple)):
            candidates.append({"xyz": value})
    robot = payload.get("robot")
    if isinstance(robot, dict):
        candidates.append(robot)
    for value in candidates:
        xyz = value.get("xyz") or value.get("position")
        if isinstance(xyz, dict):
            xyz = [xyz.get("x"), xyz.get("y"), xyz.get("z", 0.0)]
        if not isinstance(xyz, (list, tuple)) or len(xyz) < 2:
            if "x" in value and "y" in value:
                xyz = [value.get("x"), value.get("y"), value.get("z", 0.0)]
            else:
                continue
        try:
            x, y = float(xyz[0]), float(xyz[1])
        except (TypeError, ValueError):
            continue
        orientation = value.get("orientation")
        raw_yaw = value.get("yaw_deg", value.get("yaw"))
        if raw_yaw is None and isinstance(orientation, dict):
            raw_yaw = orientation.get("yaw_deg", orientation.get("yaw"))
        if raw_yaw is None:
            continue
        try:
            yaw = float(raw_yaw)
        except (TypeError, ValueError):
            continue
        confidence = value.get("confidence", 1.0)
        try:
            confidence = float(confidence)
        except (TypeError, ValueError):
            confidence = 1.0
        return x, y, yaw, confidence
    return None


def _speech_pose(events: list[dict]) -> tuple[float, float, float, float] | None:
    """Find the robot pose captured at the answer speech action, if any."""
    # A fallback/VLM query can return text, but it is an information-gathering
    # action rather than the explicit GPSR speech hand-off.  Only an announce
    # action is valid evidence for the pose at which a person-directed answer
    # was spoken.
    answer_actions = {"announce"}
    answer_kinds = {"speech.finished", "speech.spoken", "answer.spoken", "announce.finished"}
    found = None
    for event in events:
        kind = event.get("event_type")
        payload = event.get("payload") or {}
        action = str(payload.get("action") or "") if isinstance(payload, dict) else ""
        if kind in answer_kinds and action and action not in answer_actions:
            continue
        if kind in answer_kinds or (kind == "step.finished" and action in answer_actions):
            pose = _pose_from_payload(payload) if isinstance(payload, dict) else None
            if pose is not None:
                found = pose
    return found


def read_split_targets(run_dir: Path, task_slot: int = 1) -> tuple[list[dict], dict[tuple[int, str], str]]:
    """``(targets, gate_verdicts)`` from the run's events file.

    ``targets`` is the ``payload.targets`` list of the LAST ``split.accepted`` event for
    ``task_slot`` (a replanned split supersedes an earlier one); ``gate_verdicts`` maps
    ``(target_index, fact)`` to the last ``gate.verified`` postcondition verdict seen for it,
    so a physical check can carry the planner gate's own opinion alongside its own.
    """
    from behavior_tree.GPSR.bench.events import slot_of  # local: keep module import-light

    targets: list[dict] = []
    gates: dict[tuple[int, str], str] = {}
    path = newest_events_file(run_dir)
    if path is None:
        return targets, gates
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return targets, gates
    for line in lines:
        line = line.strip()
        if not line:
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            continue
        slot = slot_of(event.get("task_id"))
        if slot is not None and slot != task_slot:
            continue
        kind = event.get("event_type")
        payload = event.get("payload") or {}
        if kind == "split.accepted":
            found = payload.get("targets")
            if isinstance(found, list):
                targets = [t for t in found if isinstance(t, dict)]
        elif kind == "gate.verified" and payload.get("phase") == "postcondition":
            try:
                index = int(payload.get("target_index"))
            except (TypeError, ValueError):
                continue
            fact = payload.get("fact")
            if isinstance(fact, str):
                gates[(index, fact)] = str(payload.get("verdict"))
    return targets, gates


# --------------------------------------------------------------------------- naming

def parse_fact(fact: str) -> tuple[str, list[str]]:
    """``"placed(food, side_table_02)"`` -> ``("placed", ["food", "side_table_02"])``."""
    match = _FACT_RE.match(str(fact))
    if not match:
        return str(fact).strip(), []
    args = [a.strip() for a in match.group(2).split(",")] if match.group(2).strip() else []
    return match.group(1), args


def norm_key(noun: str) -> str:
    """Object-key form: lower-case, spaces/hyphens -> ``_``."""
    return re.sub(r"[\s\-]+", "_", str(noun).strip().lower()).strip("_")


def _singular(noun: str) -> str:
    if noun.endswith("ies"):
        return noun[:-3] + "y"
    if noun.endswith(("shes", "ches", "sses", "xes", "zes")):
        return noun[:-2]
    if noun.endswith("s") and not noun.endswith("ss"):
        return noun[:-1]
    return noun


def object_key_of(object_id: str) -> str:
    """``"/World/Scenario/cmd_sugar_box_0"`` -> ``"sugar_box"``; a non-``cmd_`` id yields its
    last path component unchanged (base-scenario prims)."""
    tail = str(object_id).rstrip("/").rsplit("/", 1)[-1]
    match = _OBJECT_ID_RE.match(tail)
    return match.group(1) if match else tail


def resolve_members(noun: str, constants: dict) -> tuple[list[str], bool]:
    """``(member object keys, is_category)`` for a fact argument.

    A category noun (``"kitchen items"``, ``"kitchen_item"``, ``"food"``) resolves through
    ``constants["categories"]`` (singular/plural/underscore variants); anything else is
    treated as a single object key (``"sugar box"`` -> ``["sugar_box"]``).
    """
    categories = constants.get("categories") or {}
    raw = str(noun).strip().lower()
    spaced = re.sub(r"[_\-]+", " ", raw)
    spaced = re.sub(r"\s+", " ", spaced).strip()
    variants = []
    for v in (raw, spaced, _singular(spaced), norm_key(raw), norm_key(_singular(spaced))):
        if v and v not in variants:
            variants.append(v)
    for v in variants:
        if v in categories and isinstance(categories[v], list):
            return [norm_key(m) for m in categories[v]], True
    lowered = {norm_key(k): v for k, v in categories.items() if isinstance(v, list)}
    for v in variants:
        if norm_key(v) in lowered:
            return [norm_key(m) for m in lowered[norm_key(v)]], True
    return [norm_key(_singular(spaced)) if norm_key(_singular(spaced)) in
            (constants.get("possible_objects") or {}) else norm_key(raw)], False


def is_person_noun(noun: str) -> bool:
    tokens = re.split(r"[\s_\-]+", str(noun).strip().lower())
    return any(any(tok.startswith(p) for p in _PERSON_TOKENS) for tok in tokens)


def _key_matches(object_id: str, member_keys: Iterable[str]) -> bool:
    key = norm_key(object_key_of(object_id))
    tokens = set(key.split("_"))
    for member in member_keys:
        if key == member:
            return True
        # Base-scenario/asset-style ids ("ycb_010_tomato_soup_can"): every token of the
        # member key present in the id's tokens.
        member_tokens = [t for t in member.split("_") if t]
        if member_tokens and all(t in tokens for t in member_tokens):
            return True
    return False


def candidate_objects(truth: dict | None, noun: str, constants: dict) -> dict[str, dict]:
    """Truth objects (id -> record) whose object key matches ``noun`` (or any member when
    ``noun`` is a category)."""
    objects = (truth or {}).get("objects") or {}
    if not isinstance(objects, dict):
        return {}
    members, _ = resolve_members(noun, constants)
    return {oid: rec for oid, rec in objects.items()
            if isinstance(rec, dict) and _key_matches(oid, members)}


def _xyz(value: Any) -> tuple[float, float, float] | None:
    try:
        x, y, z = (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError, IndexError, KeyError):
        return None
    return x, y, z


def _pose_xy(constants: dict, name: str) -> tuple[float, float] | None:
    poses = constants.get("possible_poses") or {}
    keys = [name]
    if name in OPERATOR_ALIASES:
        keys = list(_OPERATOR_POSE_KEYS) + [name]
    for key in keys:
        pose = poses.get(key)
        if not isinstance(pose, dict):
            continue
        point = pose.get("point") if isinstance(pose.get("point"), dict) else pose
        try:
            return float(point["x"]), float(point["y"])
        except (TypeError, ValueError, KeyError):
            continue
    return None


def _fmt_xyz(xyz: tuple[float, float, float]) -> str:
    return "({:.2f}, {:.2f}, {:.2f})".format(*xyz)


# --------------------------------------------------------------------------- checks

def _check(fact: str, verdict: str, reason: str, **extra) -> dict:
    out = {"fact": fact, "verdict": verdict, "reason": reason}
    out.update(extra)
    return out


def _person_descriptor_matches(descriptor: str, pose: str) -> bool:
    """Match the official command-generator descriptor to a scene pose."""
    wanted = norm_key(descriptor)
    actual = norm_key(pose)
    aliases = {
        "person_raising_their_left_arm": "raising_left_arm",
        "person_raising_left_arm": "raising_left_arm",
        "person_raising_their_right_arm": "raising_right_arm",
        "person_raising_right_arm": "raising_right_arm",
        "person_pointing_to_the_left": "pointing_left",
        "person_pointing_left": "pointing_left",
        "person_pointing_to_the_right": "pointing_right",
        "person_pointing_right": "pointing_right",
        "waving_person": "waving",
        "sitting_person": "sitting",
        "standing_person": "standing",
        "lying_person": "lying",
    }
    wanted = aliases.get(wanted, wanted)
    actual = aliases.get(actual, actual)
    return wanted == actual or wanted.endswith("_" + actual) or actual.endswith("_" + wanted)


def _communication_scene_check(fact: str, target: dict, spawned: dict | None,
                              scene_plan: dict | None, gate: str | None) -> dict:
    """Score person-found facts against the command's spawned person asset.

    The planner gate records the detector result; this oracle confirms that the
    requested descriptor actually existed in the requested room, preventing a
    planner-only PASS when scene setup silently spawned a different person.
    """
    if gate not in {"VALID", "PASS"}:
        return _check(fact, "PLANNER", "planner gate did not validate person detection")
    if not isinstance(spawned, dict) or not isinstance(scene_plan, dict):
        return _check(fact, "UNKNOWN", "no spawned person scene evidence")
    _, args = parse_fact(fact)
    descriptor = args[0] if args else "person"
    location = norm_key(str(target.get("location") or ""))
    items = present_entities(spawned, scene_plan)
    matches = [item for item in items
               if item.get("kind") == "person"
               and (not location or norm_key(item.get("where", "")) == location)
               and _person_descriptor_matches(descriptor, item.get("pose", ""))]
    if matches:
        item = matches[0]
        return _check(fact, "PASS", f"scene contains {item['pose']} person in {item['where']}")
    return _check(fact, "FAIL", f"requested {descriptor} person was not spawned in {location or 'the scene'}")


def _answer_targets_person(target_index: int, target: dict, targets: list[dict]) -> bool:
    """Whether an ``answered`` target is directed to the found person.

    The split contract often loses the original command's subject.  For example,
    ``find a person ... and say what day is tomorrow`` becomes a person-found target
    followed by a bare ``say what day`` target.  Treat that as person-directed when it
    remains in the same room, while excluding explicit operator/report-to-me answers.
    """
    text = " ".join(str(target.get(key) or "") for key in ("desc", "description", "object"))
    text = re.sub(r"[_-]+", " ", text).lower()
    operator_directed = bool(re.search(
        r"\b(?:operator|me|user|commander|command point|report|tell me|announce the result)\b",
        text,
    ))
    person_words = r"(?:person|people|human|guest|someone|somebody|waving|pointing|standing|lying)"
    speech_words = r"(?:tell|say|speak|talk|announce|answer|inform|report)"
    explicit_person = bool(re.search(rf"\b{speech_words}\b[^.]*\b{person_words}\b|"
                                     rf"\b{person_words}\b[^.]*\b{speech_words}\b", text))
    preconditions = target.get("preconditions") or []
    if any(str(fact).strip().lower().startswith("person_found(") for fact in preconditions):
        return not operator_directed
    if explicit_person and not operator_directed:
        return True

    # A follow-up target may have no person precondition (the planner bug in the audited
    # run), but its location and ordering still identify the person-directed utterance.
    target_location = norm_key(str(target.get("location") or ""))
    for prior_index, prior in enumerate(targets[:target_index]):
        facts = prior.get("postconditions") or []
        if not any(str(fact).strip().lower().startswith("person_found(") for fact in facts):
            continue
        prior_location = norm_key(str(prior.get("location") or ""))
        if target_location and prior_location and target_location == prior_location:
            return not operator_directed
        # Some splitters omit the follow-up location entirely.  A speech action after a
        # person-found target still addresses that person unless it explicitly names the
        # operator/report channel.
        if re.search(rf"\b{speech_words}\b", text) and not operator_directed:
            return True
    return False


def _person_target_location(target_index: int, target: dict, targets: list[dict]) -> str:
    location = norm_key(str(target.get("location") or ""))
    if location:
        return location
    for prior in reversed(targets[:target_index]):
        if any(str(fact).strip().lower().startswith("person_found(")
               for fact in prior.get("postconditions") or []):
            location = norm_key(str(prior.get("location") or ""))
            if location:
                return location
    return ""


def _communication_answer_check(
    fact: str,
    target_index: int,
    target: dict,
    targets: list[dict],
    run_dir: Path,
    truth: dict | None,
    gate: str | None,
    execution: dict,
) -> dict:
    """Score a spoken answer, including the physical person-communication contract."""
    if gate not in {"VALID", "PASS"}:
        return _check(fact, "PLANNER", "planner gate did not validate the answer")
    artifact = _answer_artifact_check(fact, run_dir, gate)
    if artifact["verdict"] != "PASS" or not _answer_targets_person(target_index, target, targets):
        return artifact

    # The postcondition may be established by llm_fallback/vlm_fallback in the
    # planner's generic fact model, but those nodes do not constitute a
    # person-directed utterance.  Once materialized telemetry is available,
    # require the explicit announce action before physical communication can
    # pass.  ``plan_execution_audit`` applies the same rule to the planner
    # verdict; keeping it here prevents a physical pass if that audit is
    # bypassed or an older report supplied a planner PASS.
    materialized = execution.get("materialized") or {}
    if target_index in materialized:
        target_steps = materialized[target_index]
        if not any(str(step.get("action")) == "announce" for step in target_steps):
            actions = ", ".join(str(step.get("action") or "") for step in target_steps)
            return _check(fact, "FAIL",
                          f"person-directed answer has no explicit announce step"
                          f" (actions: {actions or 'none'})",
                          requires_announce=True)

    # A spoken answer to a person is not complete until the approach action has been
    # planned and completed before the answer target.  This catches planner-only PASSes
    # like the two audited runs, where the robot merely entered the room.
    materialized = execution.get("materialized") or {}
    approach_targets = [index for index, steps in materialized.items()
                        if any(str(step.get("action")) == "approach_person" for step in steps)]
    answer_actions = {"announce"}
    approach_before_answer = any(index < target_index for index in approach_targets)
    if not approach_before_answer and target_index in materialized:
        steps = materialized[target_index]
        approach_positions = [pos for pos, step in enumerate(steps)
                              if str(step.get("action")) == "approach_person"]
        answer_positions = [pos for pos, step in enumerate(steps)
                            if str(step.get("action")) in answer_actions]
        approach_before_answer = bool(approach_positions and answer_positions
                                      and min(approach_positions) < min(answer_positions))
    if not approach_before_answer:
        return _check(fact, "FAIL", "person-directed answer has no preceding approach_person step",
                      requires_approach=True)

    outcomes = [step for step in execution.get("outcomes") or []
                if str(step.get("action")) == "approach_person"]
    if not outcomes:
        return _check(fact, "UNKNOWN", "approach_person was planned but has no completion event",
                      requires_approach=True, critical=True)
    outcome = str(outcomes[-1].get("outcome") or "").lower()
    if outcome not in {"succeeded", "success", "pass", "valid"}:
        return _check(fact, "FAIL", f"approach_person outcome was {outcome or 'unknown'}",
                      requires_approach=True)

    pose = execution.get("speech_pose")
    location = _person_target_location(target_index, target, targets)
    persons = truth.get("persons") if isinstance(truth, dict) else None
    person = persons.get(location) if isinstance(persons, dict) and location else None
    person_xyz = _xyz(person.get("xyz")) if isinstance(person, dict) else None
    if pose is None or person_xyz is None:
        return _check(fact, "UNKNOWN",
                      "approach_person succeeded but speech-time robot/person geometry is unavailable",
                      requires_approach=True, critical=True)
    robot_x, robot_y, robot_yaw, confidence = pose
    dx, dy = person_xyz[0] - robot_x, person_xyz[1] - robot_y
    distance = math.hypot(dx, dy)
    wanted = math.degrees(math.atan2(dy, dx))
    heading_error = abs((wanted - robot_yaw + 180.0) % 360.0 - 180.0)
    note = (f"spoken to {location or 'person'} at {distance:.2f} m, "
            f"heading error {heading_error:.1f} deg")
    if distance > PERSON_SPEECH_RADIUS_M:
        return _check(fact, "FAIL", note + f" (> {PERSON_SPEECH_RADIUS_M:.1f} m)",
                      requires_approach=True, distance_m=distance,
                      heading_error_deg=heading_error)
    if heading_error > PERSON_SPEECH_HEADING_TOLERANCE_DEG:
        return _check(fact, "FAIL", note + f" (> {PERSON_SPEECH_HEADING_TOLERANCE_DEG:.0f} deg)",
                      requires_approach=True, distance_m=distance,
                      heading_error_deg=heading_error)
    return _check(fact, "PASS", note, requires_approach=True,
                  distance_m=distance, heading_error_deg=heading_error,
                  confidence=confidence)


def _answer_artifact_check(fact: str, run_dir: Path, gate: str | None) -> dict:
    """Confirm that the validated answer was actually spoken in the recording."""
    if gate not in {"VALID", "PASS"}:
        return _check(fact, "PLANNER", "planner gate did not validate the answer")
    path = Path(run_dir) / "announcements.txt"
    if not path.is_file():
        return _check(fact, "UNKNOWN", "no announcements.txt answer artifact")
    try:
        lines = [line.strip() for line in path.read_text(encoding="utf-8", errors="replace").splitlines()]
    except OSError:
        return _check(fact, "UNKNOWN", "answer artifact unreadable")
    answers = [line for line in lines if line and not _ANSWER_BOILERPLATE.match(line)]
    if answers:
        return _check(fact, "PASS", f"spoken answer recorded: {answers[-1]}")
    return _check(fact, "FAIL", "planner validated an answer but no spoken answer was recorded")


def _num(value: Any) -> float | None:
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def motion_verdict(rec: dict) -> tuple[str, str]:
    """``(PASS|FAIL|UNKNOWN, reason)`` -- was this truth object physically carried?

    See the ``LIFT_*``/``XY_*`` thresholds above. ``ever_retained: True`` is a PASS on its
    own; ``False`` says nothing (bench-spawned entities always publish it). Missing motion
    fields -> UNKNOWN.
    """
    lift, disp = _num(rec.get("max_lift_m")), _num(rec.get("max_xy_disp_m"))
    motion = ""
    if lift is not None or disp is not None:
        motion = "lift {} m, xy {} m".format(
            "?" if lift is None else f"{lift:.2f}", "?" if disp is None else f"{disp:.2f}")
    if rec.get("ever_retained") is True:
        return "PASS", "retained by the gripper" + (f" ({motion})" if motion else "")
    if lift is None or disp is None:
        return "UNKNOWN", "no motion data" + (f" ({motion})" if motion else "")
    if lift >= LIFT_CARRIED_M or (disp >= XY_CARRIED_M and lift >= LIFT_MIN_M):
        return "PASS", f"lifted/carried ({motion})"
    if lift < LIFT_STILL_M and disp < XY_STILL_M:
        return "FAIL", f"never moved ({motion})"
    return "FAIL", f"nudged but never lifted clear ({motion})"


def _carried(cands: dict[str, dict]) -> tuple[dict[str, dict], dict[str, str], dict[str, str]]:
    """Split candidates into ``(carried, failed, unknown)`` by ``motion_verdict``; the latter
    two map id -> reason."""
    carried: dict[str, dict] = {}
    failed: dict[str, str] = {}
    unknown: dict[str, str] = {}
    for oid, rec in sorted(cands.items()):
        verdict, reason = motion_verdict(rec)
        if verdict == "PASS":
            carried[oid] = rec
        elif verdict == "FAIL":
            failed[oid] = reason
        else:
            unknown[oid] = reason
    return carried, failed, unknown


def check_held(fact: str, obj: str, truth: dict | None, constants: dict) -> dict:
    if truth is None:
        return _check(fact, "UNKNOWN", PHYSICAL_TRUTH_UNAVAILABLE)
    cands = candidate_objects(truth, obj, constants)
    carried, failed, unknown = _carried(cands)
    if carried:
        oid = next(iter(carried))
        return _check(fact, "PASS", f"{oid} {motion_verdict(carried[oid])[1]}", objects=list(carried))
    if cands and not unknown:
        return _check(fact, "FAIL", "never retained by gripper: " + "; ".join(
            f"{oid} {reason}" for oid, reason in failed.items()), objects=sorted(cands))
    if truth.get("legacy_source") and truth.get("any_object_retained") is False:
        # grasp_truth.json (closed-on-air probe): no per-object motion, only the global flag.
        note = "never retained by gripper (no object was retained at all per sim truth)"
        disp = _num(truth.get("max_xy_disp_m"))
        if disp is not None:
            note += f"; target moved only {disp * 1000:.0f}mm"
        return _check(fact, "FAIL", note, objects=sorted(cands))
    if not cands:
        return _check(fact, "UNKNOWN", f"no truth object matches '{obj}'")
    return _check(fact, "UNKNOWN", "motion undecidable: " + "; ".join(
        f"{oid} {reason}" for oid, reason in {**failed, **unknown}.items()), objects=sorted(cands))


def _on_surface(last: tuple[float, float, float], surface: dict) -> tuple[bool, str]:
    sxyz = _xyz(surface.get("surface_xyz"))
    half = surface.get("half_extent_xy")
    try:
        hx, hy = float(half[0]), float(half[1])
    except (TypeError, ValueError, IndexError):
        hx = hy = None
    if sxyz is None or hx is None:
        return False, "surface geometry incomplete"
    dx, dy, dz = last[0] - sxyz[0], last[1] - sxyz[1], last[2] - sxyz[2]
    ok_x = abs(dx) <= hx + PLACED_XY_SLACK_M
    ok_y = abs(dy) <= hy + PLACED_XY_SLACK_M
    ok_z = -PLACED_Z_BELOW_M <= dz <= PLACED_Z_ABOVE_M
    offset = f"dx={dx:+.2f} dy={dy:+.2f} dz={dz:+.2f} m (half-extent {hx:.2f}x{hy:.2f})"
    if ok_x and ok_y and ok_z:
        return True, offset
    if dz < -PLACED_Z_BELOW_M:
        offset += "; below the surface (dropped?)"
    return False, offset


def check_placed(fact: str, obj: str, loc: str, truth: dict | None, constants: dict) -> dict:
    if truth is None:
        return _check(fact, "UNKNOWN", PHYSICAL_TRUTH_UNAVAILABLE)
    surfaces = truth.get("surfaces") or {}
    surface = surfaces.get(loc) if isinstance(surfaces, dict) else None
    if not isinstance(surface, dict):
        return _check(fact, "UNKNOWN", f"surface '{loc}' not in truth surfaces")
    cands = candidate_objects(truth, obj, constants)
    if not cands:
        return _check(fact, "UNKNOWN", f"no truth object matches '{obj}'")
    # Only the object(s) the gripper actually carried (motion-based, see motion_verdict)
    # can have been placed: an object that was ALREADY on the destination surface and
    # never moved must not score a PASS for "placed".
    carried, failed, unknown = _carried(cands)
    if not carried and not unknown:
        return _check(fact, "FAIL", "nothing was placed: candidate object(s) never retained "
                      "by gripper (" + "; ".join(f"{oid} {r}" for oid, r in failed.items()) + ")",
                      objects=sorted(cands))
    pool = carried or {oid: cands[oid] for oid in unknown}
    failures = []
    for oid, rec in sorted(pool.items()):
        last = _xyz(rec.get("last_xyz"))
        if last is None:
            failures.append(f"{oid}: no last_xyz")
            continue
        ok, offset = _on_surface(last, surface)
        if ok:
            return _check(fact, "PASS", f"{oid} rests on {loc}: last {_fmt_xyz(last)} {offset}",
                          objects=[oid])
        failures.append(f"{oid} last {_fmt_xyz(last)} not on {loc}: {offset}")
    if all(f.endswith("no last_xyz") for f in failures):
        return _check(fact, "UNKNOWN", "; ".join(failures), objects=sorted(pool))
    return _check(fact, "FAIL", "; ".join(failures), objects=sorted(pool))


def _recipient_xy(recipient: str, target: dict, truth: dict, constants: dict) -> tuple[tuple[float, float] | None, str]:
    name = norm_key(recipient)
    persons = truth.get("persons") or {}
    if name in OPERATOR_ALIASES:
        # The spawned operator's body (sim scene "operator" placement) when the
        # probe recorded one; else the command point from constants.
        rec = persons.get("operator") if isinstance(persons, dict) else None
        xyz = _xyz(rec.get("xyz")) if isinstance(rec, dict) else None
        if xyz is not None:
            return (xyz[0], xyz[1]), "operator"
        xy = _pose_xy(constants, name)
        return xy, ("operator start pose" if xy else "operator start pose not in constants")
    for room in (name, norm_key(target.get("location") or "")):
        rec = persons.get(room) if isinstance(persons, dict) and room else None
        xyz = _xyz(rec.get("xyz")) if isinstance(rec, dict) else None
        if xyz is not None:
            return (xyz[0], xyz[1]), f"person in {room}"
    xy = _pose_xy(constants, name)
    if xy:
        return xy, f"pose {name}"
    return None, f"recipient '{recipient}' location unresolved"


def check_delivered(fact: str, obj: str, recipient: str, target: dict, truth: dict | None,
                    constants: dict) -> dict:
    if truth is None:
        return _check(fact, "UNKNOWN", PHYSICAL_TRUTH_UNAVAILABLE)
    held = check_held(f"held({obj})", obj, truth, constants)
    if held["verdict"] == "FAIL":
        return _check(fact, "FAIL", "nothing to deliver: " + held["reason"], objects=held.get("objects", []))
    if held["verdict"] == "UNKNOWN":
        return _check(fact, "UNKNOWN", held["reason"])
    robot = truth.get("robot") or {}
    last = _xyz(robot.get("last_xyz")) if isinstance(robot, dict) else None
    if last is None:
        return _check(fact, "UNKNOWN", "robot last pose not in truth")
    xy, where = _recipient_xy(recipient, target, truth, constants)
    if xy is None:
        return _check(fact, "UNKNOWN", where)
    dist = ((last[0] - xy[0]) ** 2 + (last[1] - xy[1]) ** 2) ** 0.5
    note = f"{held['reason']}; robot ended {dist:.2f} m from {where}"
    if dist > DELIVERED_RADIUS_M:
        return _check(fact, "FAIL", note + f" (> {DELIVERED_RADIUS_M} m)", objects=held.get("objects", []))
    # The OBJECT must have arrived too, not just the robot (fix5 run 004: the
    # handed-over banana fell to the floor at the kitchen table, the robot drove
    # back empty-handed and the old check -- robot pose only -- scored PASS).
    objects = (truth.get("objects") or {}) if isinstance(truth, dict) else {}
    ends = []
    for oid in held.get("objects", []):
        end = _xyz((objects.get(oid) or {}).get("last_xyz"))
        if end is not None:
            ends.append((oid, end, ((end[0] - xy[0]) ** 2 + (end[1] - xy[1]) ** 2) ** 0.5))
    if not ends:
        return _check(fact, "UNKNOWN", note + "; carried object's final position unknown",
                      objects=held.get("objects", []))
    oid, end, odist = min(ends, key=lambda e: e[2])
    note += f"; {oid} ended {odist:.2f} m from {where} at z={end[2]:.2f}"
    to_robot = ((end[0] - last[0]) ** 2 + (end[1] - last[1]) ** 2) ** 0.5
    if end[2] > STILL_IN_HAND_MIN_Z_M and to_robot < STILL_IN_HAND_MAX_XY_M:
        return _check(fact, "FAIL", note + f" -- still in the robot's hand ({to_robot:.2f} m from the robot): "
                      "never handed over", objects=held.get("objects", []))
    if odist <= DELIVERED_RADIUS_M:
        return _check(fact, "PASS", note, objects=held.get("objects", []))
    return _check(fact, "FAIL", note + f" -- the object never reached the recipient (dropped?)",
                  objects=held.get("objects", []))


def check_at_robot(fact: str, loc: str, truth: dict | None, constants: dict) -> dict:
    if truth is None:
        return _check(fact, "UNKNOWN", PHYSICAL_TRUTH_UNAVAILABLE)
    robot = truth.get("robot") or {}
    last = _xyz(robot.get("last_xyz")) if isinstance(robot, dict) else None
    if last is None:
        return _check(fact, "UNKNOWN", "robot last pose not in truth")
    xy = _pose_xy(constants, norm_key(loc))
    if xy is None:
        return _check(fact, "UNKNOWN", f"no nav pose for '{loc}' in constants")
    dist = ((last[0] - xy[0]) ** 2 + (last[1] - xy[1]) ** 2) ** 0.5
    note = f"robot ended {dist:.2f} m from {loc}"
    if dist <= AT_ROBOT_RADIUS_M:
        return _check(fact, "PASS", note)
    return _check(fact, "FAIL", note + f" (> {AT_ROBOT_RADIUS_M} m)")


# --------------------------------------------------------------------------- counting

def announced_count(run_dir: Path) -> tuple[int | None, str]:
    """``(count, source)`` -- the number the robot actually reported.

    Primary source: ``orchestrator.log`` leaf feedback of the count subtree, which is the
    exact blackboard value the announce step then speaks (``small_trees.py``: ``parse vlm
    count`` -> "parsed count=N from VLM answer"; ``count detections`` -> "counted N
    objects"; mock vision -> "synthetic count=N"). The LAST match in file order wins, so a
    detector's "counted 0 objects -- deferring to fallback" is superseded by the VLM
    fallback's parsed value. Fallback source: ``announcements.txt`` -- an "I counted N" line
    (detector path), else the last count-shaped sentence (contains "visible"/"there are"/
    "count") whose first number or number-word is taken ("There are **0 persons** visible",
    "Zero red bowl ... are visible").
    """
    run_dir = Path(run_dir)
    log = run_dir / "orchestrator.log"
    found: tuple[int, str] | None = None
    spoken: tuple[int, str] | None = None
    if log.is_file():
        try:
            with log.open("r", encoding="utf-8", errors="replace") as fh:
                for line in fh:
                    for rx in _LOG_SPOKEN_COUNT_RES:
                        m = rx.search(line)
                        if m:
                            spoken = (int(m.group(1)), f"orchestrator.log: {m.group(0)}")
                    for rx in _LOG_COUNT_RES:
                        m = rx.search(line)
                        if m:
                            found = (int(m.group(1)), f"orchestrator.log: {m.group(0)}")
        except OSError:
            found = spoken = None
    if spoken is not None:
        return spoken
    if found is not None:
        return found
    ann = run_dir / "announcements.txt"
    if not ann.is_file():
        return None, "no count source (no orchestrator.log count feedback, no announcements.txt)"
    try:
        lines = ann.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return None, "announcements.txt unreadable"
    for line in reversed(lines):
        m = _ANNOUNCE_I_COUNTED_RE.match(line)
        if m:
            return int(m.group(1)), f"announcements.txt: {line.strip()}"
    for line in reversed(lines):
        if not _ANNOUNCE_COUNT_SENTENCE_RE.search(line):
            continue
        # fix4 run 001: the plan announcement "Here is my plan. I will complete
        # 2 tasks. First, count how many ..." is not a count RESULT -- its
        # "2" was taken as the announced count. Only result sentences qualify.
        if _ANNOUNCE_NOT_A_RESULT_RE.search(line):
            continue
        m = _NUMBER_TOKEN_RE.search(line)
        if not m:
            continue
        token = m.group(1).lower()
        value = int(token) if token.isdigit() else _NUMBER_WORDS.get(token)
        if value is not None:
            return value, f"announcements.txt: {line.strip()}"
    return None, "no count sentence found in announcements.txt or orchestrator.log"


def present_entities(spawned: dict | None, scene_plan: dict | None) -> list[dict]:
    """Entities physically present in the scene during the run, as
    ``{"id", "key", "where", "kind", "pose"}`` records.

    From ``spawned.json``: every ``entities[]`` record with ``ok: true`` (spawned for this
    run -- ``cleared`` is set by the POST-run clear and is irrelevant here) plus every
    ``skipped[]`` record with reason "already present" (the base scenario / a previous
    manifest already had that asset at that spot, so it IS in the scene). ``not_attempted``
    and ``ok: false`` records are absent from the scene. ``kind`` comes from
    ``scene-plan.json``'s item of the same id (``"object"``/``"person"``), falling back to
    the id itself (``cmd_person_*``).
    """
    kinds: dict[str, str] = {}
    wheres: dict[str, str] = {}
    poses: dict[str, str] = {}
    for item in (scene_plan or {}).get("items") or []:
        if isinstance(item, dict) and item.get("id"):
            kinds[item["id"]] = str(item.get("kind") or "")
            wheres[item["id"]] = str(item.get("spot") or item.get("room") or "")
            poses[item["id"]] = str(item.get("pose") or "")
    out: dict[str, dict] = {}
    if not isinstance(spawned, dict):
        return []

    def include(rec: dict) -> None:
        entity_id = str(rec.get("id") or "")
        if not entity_id or entity_id in out:
            return
        out[entity_id] = {
            "id": entity_id,
            "key": norm_key(object_key_of(entity_id)),
            "where": str(rec.get("where") or wheres.get(entity_id, "")),
            "kind": kinds.get(entity_id) or ("person" if "person" in entity_id else "object"),
            "pose": str(rec.get("pose") or poses.get(entity_id, "")),
        }

    # A scene applier can report a pre-existing entity in both arrays: it is
    # physically present, and the requested spawn was skipped as a duplicate.
    # Count the entity once, preferring the richer successful entity record.
    for rec in spawned.get("entities") or []:
        if isinstance(rec, dict) and rec.get("ok") is True:
            include(rec)
    for rec in spawned.get("skipped") or []:
        if isinstance(rec, dict) and "already present" in str(rec.get("reason") or ""):
            include(rec)
    return list(out.values())


def _count_location(target: dict, plan_steps: list[dict] | None) -> str:
    loc = str(target.get("location") or "").strip()
    if loc:
        return loc
    for step in plan_steps or []:
        params = step.get("params") or {}
        if step.get("action") == "count" and params.get("location"):
            return str(params["location"])
    for step in reversed(plan_steps or []):
        params = step.get("params") or {}
        if step.get("action") == "goto" and params.get("location"):
            return str(params["location"])
    return ""


def _person_count_pose(noun: str) -> str | None:
    """Return the pose requested by a person-count noun, if it is specific.

    Count nouns arrive in natural language (for example, ``"persons pointing to
    the right"``), while scene metadata stores canonical pose keys.  Keep bare
    ``persons`` generic and only filter when the noun contains an explicit pose.
    """
    text = re.sub(r"[_-]+", " ", str(noun).strip().lower())
    patterns = (
        (r"\bpoint(?:ing)?\s+(?:to\s+the\s+)?right\b", "pointing_right"),
        (r"\bpoint(?:ing)?\s+(?:to\s+the\s+)?left\b", "pointing_left"),
        (r"\brais(?:e|ed|ing)\s+(?:their\s+)?right\s+arm\b", "raising_right_arm"),
        (r"\brais(?:e|ed|ing)\s+(?:their\s+)?left\s+arm\b", "raising_left_arm"),
        (r"\bwav(?:e|ed|ing)\b", "waving"),
        (r"\bsitt(?:ing|ed)\b", "sitting"),
        (r"\bstand(?:ing|s)?\b", "standing"),
        (r"\b(?:lie|lying|lay|laying)\b", "lying"),
    )
    for pattern, pose in patterns:
        if re.search(pattern, text):
            return pose
    return None


def expected_count(noun: str, location: str, entities: list[dict], constants: dict) -> tuple[int | None, str]:
    """``(expected, description)`` -- how many matching entities were in the scene at
    ``location`` (a furniture spot, or a room whose ``search_spots`` all count)."""
    loc = norm_key(location)
    if not loc:
        return None, "count location unknown"
    spots = {loc}
    room_spots = (constants.get("search_spots") or {}).get(loc)
    if isinstance(room_spots, list):
        spots.update(norm_key(s) for s in room_spots)
    if is_person_noun(noun):
        pose = _person_count_pose(noun)
        matched = [e for e in entities if e["kind"] == "person" and norm_key(e["where"]) in spots
                   and (pose is None or _person_descriptor_matches(pose, e.get("pose", "")))]
        label = f"{pose} person(s)" if pose else "person(s)"
        return len(matched), f"{len(matched)} {label} spawned in {location}"
    members, is_category = resolve_members(noun, constants)
    matched = [e for e in entities if e["kind"] != "person" and norm_key(e["where"]) in spots
               and _key_matches(e["id"], members)]
    label = f"{noun} (category)" if is_category else f"{noun}"
    return len(matched), f"{len(matched)} x {label} spawned on {location}"


def check_counted(fact: str, noun: str, target: dict, run_dir: Path, *, spawned: dict | None,
                  scene_plan: dict | None, constants: dict, plan_steps: list[dict] | None = None) -> dict:
    if spawned is None:
        return _check(fact, "UNKNOWN", "no spawned.json: expected count cannot be derived")
    entities = present_entities(spawned, scene_plan)
    location = _count_location(target, plan_steps)
    expected, desc = expected_count(noun, location, entities, constants)
    if expected is None:
        return _check(fact, "UNKNOWN", desc)
    announced, source = announced_count(run_dir)
    if announced is None:
        return _check(fact, "UNKNOWN", f"announced count not found ({source}); expected {desc}",
                      expected=expected)
    note = f"announced {announced}, {desc} [{source}]"
    if announced == expected:
        return _check(fact, "PASS", note, expected=expected, announced=announced)
    return _check(fact, "FAIL", note, expected=expected, announced=announced)


# --------------------------------------------------------------------------- driver

def physical_checks(run_dir: Path, targets: list[dict] | None, *, constants: dict | Path | None = None,
                    truth: dict | None = None, gate_verdicts: dict[tuple[int, str], str] | None = None,
                    plan_steps: list[dict] | None = None, task_slot: int = 1) -> list[dict]:
    """One check dict per postcondition of every split target, in target order.

    Each dict: ``fact``, ``target`` (index), ``verdict`` (PASS/FAIL/UNKNOWN/PLANNER),
    ``reason``, plus ``gate`` (the planner gate's own verdict for that fact, when known)
    and check-specific extras (``objects``, ``expected``, ``announced``).
    ``constants`` may be a loaded dict (from ``load_constants``) or a path.
    """
    run_dir = Path(run_dir)
    if not isinstance(constants, dict):
        constants = load_constants(constants)
    if truth is None:
        truth = load_truth(run_dir)
    spawned = _read_json(run_dir / "spawned.json")
    scene_plan = _read_json(run_dir / "scene-plan.json")
    if not isinstance(spawned, dict):
        spawned = None
    if not isinstance(scene_plan, dict):
        scene_plan = None
    gate_verdicts = gate_verdicts or {}
    targets = [t for t in (targets or []) if isinstance(t, dict)]
    events = _event_records(run_dir, task_slot)
    execution = {
        "materialized": _materialized_steps(events),
        "outcomes": _step_outcomes(events),
        "speech_pose": _speech_pose(events),
    }
    last_index = len(targets) - 1
    checks: list[dict] = []
    for index, target in enumerate(targets):
        for fact in target.get("postconditions") or []:
            if not isinstance(fact, str):
                continue
            name, args = parse_fact(fact)
            obj = args[0] if args else str(target.get("object") or "")
            if name == "held":
                check = check_held(fact, obj, truth, constants)
            elif name == "placed":
                loc = args[1] if len(args) > 1 else str(target.get("location") or "")
                check = check_placed(fact, obj, loc, truth, constants)
            elif name == "delivered":
                recipient = args[1] if len(args) > 1 else "operator"
                check = check_delivered(fact, obj, recipient, target, truth, constants)
            elif name == "counted":
                check = check_counted(fact, obj, target, run_dir, spawned=spawned,
                                      scene_plan=scene_plan, constants=constants,
                                      plan_steps=plan_steps)
            elif name == "person_found":
                check = _communication_scene_check(fact, target, spawned, scene_plan,
                                                   gate_verdicts.get((index, fact)))
            elif name == "answered":
                check = _communication_answer_check(
                    fact, index, target, targets, run_dir, truth,
                    gate_verdicts.get((index, fact)), execution,
                )
            elif name == "at_robot":
                if index == last_index:
                    check = check_at_robot(fact, obj, truth, constants)
                else:
                    check = _check(fact, "PLANNER", "only the final robot pose is known")
            else:
                check = _check(fact, "PLANNER", "no physical check for this fact")
            check["target"] = index
            gate = gate_verdicts.get((index, fact))
            if gate is not None:
                check["gate"] = gate
            checks.append(check)
    return checks


def combine_verdicts(planner_verdict: str, checks: list[dict], referee_assisted: bool,
                     ) -> tuple[str, str, str]:
    """``(physical_verdict, final_verdict, physical_detail)``.

    * ``physical_verdict``: FAIL if any check FAILed; else UNKNOWN when a critical check
      lacks evidence; else REFEREE for a referee-assisted run; else PASS if at least one
      check PASSed; else UNKNOWN (nothing decidable, e.g. no ``run_truth.json``).
    * ``final_verdict``: ``planner_verdict`` when it is not PASS; otherwise the physical
      verdict when it is PASS/FAIL/REFEREE; otherwise PASS with the detail
      "physical truth unavailable".  Critical unknowns (such as missing speech-time
      geometry for a person-directed answer) remain UNKNOWN so a separate PASS check
      cannot turn them into a clean pass.
    """
    fails = [c for c in checks if c.get("verdict") == "FAIL"]
    passes = [c for c in checks if c.get("verdict") == "PASS"]
    critical_unknowns = [c for c in checks
                         if c.get("verdict") == "UNKNOWN" and c.get("critical")]
    if fails:
        physical = "FAIL"
        detail = "; ".join(f"{c['fact']}: {c['reason']}" for c in fails)
    elif critical_unknowns:
        # Some checks are advisory (for example, an intermediate navigation target only
        # has planner evidence), but a missing speech-time pose is decisive for a
        # person-directed answer.  Do not let a separate scene PASS mask that gap.
        physical = "UNKNOWN"
        detail = "; ".join(f"{c['fact']}: {c['reason']}" for c in critical_unknowns)
    elif referee_assisted:
        physical = "REFEREE"
        detail = "referee-assisted run" + (
            " (" + "; ".join(f"{c['fact']}: {c['reason']}" for c in passes) + ")" if passes else "")
    elif passes:
        physical = "PASS"
        detail = "; ".join(f"{c['fact']}: {c['reason']}" for c in passes)
    else:
        physical = "UNKNOWN"
        unknowns = [c for c in checks if c.get("verdict") == "UNKNOWN"]
        detail = PHYSICAL_TRUTH_UNAVAILABLE
        if unknowns:
            detail += " (" + "; ".join(f"{c['fact']}: {c['reason']}" for c in unknowns) + ")"
    if planner_verdict != "PASS":
        final = planner_verdict
    elif physical in ("PASS", "FAIL", "REFEREE"):
        final = physical
    elif critical_unknowns:
        final = "UNKNOWN"
    else:
        final = "PASS"
    return physical, final, detail
