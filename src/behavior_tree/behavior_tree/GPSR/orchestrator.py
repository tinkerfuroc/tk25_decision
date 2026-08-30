"""GPSR orchestrator.

Receives a natural-language command, asks an LLM to break it into an ordered
list of atomic actions, then dispatches each action to the corresponding small
tree from ``small_trees.py``. Adds a self-monitor guard after every step and a
self-correction sub-tree that re-prompts the LLM for a fresh plan when a step
fails.

Blackboard contract:
    Inputs:
        gpsr/command            (str) — raw user instruction
    Internal:
        gpsr/plan               (list[dict]) — ordered actions to run
        gpsr/plan_index         (int) — next action to dispatch
        gpsr/current_action     (str) — name of the action being dispatched
        gpsr/current_params     (dict) — its parameters
        gpsr/state_log          (list[str]) — completed-step descriptions
        gpsr/correction_count   (int) — bumped on every self-correction
        gpsr/last_failure       (str) — feedback text from the failed step
"""

import copy
import json
import math
import os
import random
import re
import textwrap
import threading
import time
import uuid
from typing import Any, Callable, Dict, Iterator, List, Mapping, Optional, Sequence, Tuple

import py_trees
import rclpy
from py_trees.behaviour import Behaviour
from py_trees.blackboard import Blackboard
from py_trees.common import Access, Status
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

import openai

from .config import (
    OPENAI_API_KEY,
    OPENAI_MODEL,
    OPENAI_TEMPERATURE,
    OPENAI_MAX_TOKENS,
)
from ..config import is_full_mock_mode
from .planner_validators import validate_plan
from .validators import (
    Fact,
    VerificationContext,
    Verdict,
    apply_fact_transitions,
    canonical_fact,
    check_all,
    parse_fact,
    _normalize,
)
from .small_trees import (
    ACTION_FACTORIES,
    bb_keys,
    BtNode_AnnounceFromBB,
    BtNode_BlackboardSet,
    BtNode_BlackboardCopy,
    SEARCH_POSE_KEYS,
    create_goto,
)
from .action_contracts import (
    ACTION_CONTRACTS,
    IDENTICAL_PLAN_ERROR_PREFIX,
    UNRECOVERABLE_ERROR_PREFIX,
    contract_for as _contract_for,
    established_facts as _step_established_facts,
    self_established_facts as _self_established_facts,
)
from .telemetry import get_default_telemetry
from .supervision.models import SupervisionMode
from .supervision.runtime import get_default_supervisor, wrap_action_factory


def _openai_client():
    if openai is None:
        raise RuntimeError(
            "GPSR planning requires the optional 'openai' Python package"
        )
    if not OPENAI_API_KEY:
        raise RuntimeError(
            "Set OPENROUTER_API_KEY (or OPENAI_API_KEY) before starting GPSR"
        )
    return openai.OpenAI(
        api_key=OPENAI_API_KEY,
        base_url="https://openrouter.ai/api/v1",
    )


def _offline_planner_enabled() -> bool:
    """Resolve planner mocking independently from execution mocking.

    Full execution mock remains offline by default for CI compatibility.
    Operators can set ``GPSR_OFFLINE_PLANNER=0`` to exercise the real
    orchestrator API while every ROS/Tinker boundary stays mocked.
    """
    override = os.environ.get("GPSR_OFFLINE_PLANNER")
    if override is None or not override.strip():
        return is_full_mock_mode()
    value = override.strip().lower()
    if value in {"1", "true", "yes", "on"}:
        return True
    if value in {"0", "false", "no", "off"}:
        return False
    raise ValueError(
        "GPSR_OFFLINE_PLANNER must be one of 1/0, true/false, yes/no, or on/off"
    )

# ---------------------------------------------------------------------------
# Knowledge available to the LLM (poses + objects loaded once at import).
# Each entry maps the GPSR-vocabulary name -> resolved PoseStamped or vision
# prompt string. The orchestrator uses these to materialise concrete BB values
# when popping a step from the plan.
# ---------------------------------------------------------------------------

KNOWN_LOCATIONS: Dict[str, PoseStamped] = {}
KNOWN_OBJECT_PROMPTS: Dict[str, str] = {}
# J5 (round-3 adversarial review, testing session 3a): normalised (see
# validators._normalize) arena object NAMES only -- the instance vocabulary
# the gate's category-membership rule checks a detector's class-prefix match
# against (validators._label_matches). Same source as KNOWN_OBJECT_PROMPTS
# (constants.json "possible_objects"), populated alongside it.
KNOWN_OBJECT_NAMES: set = set()
# object name -> the location it usually lives at (used when a fetch command
# names no location). Populated from constants.json "default_locations".
DEFAULT_OBJECT_LOCATIONS: Dict[str, str] = {}
# room/location name -> ordered list of pose names to sweep when the in-room
# spot is unknown (the override case). Populated from constants.json
# "search_spots"; a location with no entry falls back to [itself].
ROOM_SEARCH_SPOTS: Dict[str, List[str]] = {}
# Appliances/containers with doors the robot CANNOT open or reach into itself: a
# grasp there is ALWAYS bypassed to the ask-referee branch (the referee opens the
# door and hands the item over). Forced into NO_GRASP_LOCATIONS on every load so a
# constants.json "no_grasp_locations" override can never drop them.
ALWAYS_NO_GRASP: set = {"refrigerator", "fridge", "washing_machine", "dishwasher"}
# Furniture the robot must NOT grasp from — a grasp there is bypassed straight to
# the ask-referee branch. Populated from constants.json "no_grasp_locations"
# (default shelf / cabinet / coat_rack) UNION the always-bypass appliances above.
# Matched case-insensitively, and both the underscore and space spellings
# ("coat_rack" / "coat rack", "washing_machine" / "washing machine") are recognised.
NO_GRASP_LOCATIONS: set = {"shelf", "cabinet", "coat_rack"} | ALWAYS_NO_GRASP

# Names the planner may use for "where the robot stood when it received the
# command". Resolved from the blackboard (bb_keys.START_POSE, captured by
# create_record_position at command start) instead of constants.json.
# I5 (round-3 adversarial review, M2): "me"/"the user"/"the operator"/
# "command point" are common ways the split layer's own postconditions name
# the SAME destination -- kept in sync with
# planner_validators.START_LOCATION_WORDS (that module's is_start_alias is
# the single source of truth other code should prefer; this set additionally
# drives runtime pose resolution, resolve_pose() below).
START_LOCATION_ALIASES = {
    "start_position", "instruction_point", "start", "operator",
    "me", "the_user", "the_operator", "command_point",
}


def _target_desc(t: Any) -> str:
    """A target's human-readable description: ``desc`` if structured, else itself."""
    if isinstance(t, dict):
        return str(t.get("desc") or "")
    return str(t)


def _parse_pose_stamped(json_dict: dict) -> PoseStamped:
    point = json_dict["point"]
    orientation = json_dict["orientation"]
    return PoseStamped(
        header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
        pose=Pose(
            position=Point(x=point["x"], y=point["y"], z=0.0),
            orientation=Quaternion(
                x=orientation['x'], y=orientation['y'],
                z=orientation['z'], w=orientation['w'],
            ),
        ),
    )


def load_knowledge_from_constants(constants_path: str) -> None:
    """Populate KNOWN_LOCATIONS / KNOWN_OBJECT_PROMPTS from constants.json."""
    KNOWN_LOCATIONS.clear()
    KNOWN_OBJECT_PROMPTS.clear()
    KNOWN_OBJECT_NAMES.clear()
    DEFAULT_OBJECT_LOCATIONS.clear()
    ROOM_SEARCH_SPOTS.clear()
    with open(constants_path, "r") as fh:
        constants = json.load(fh)

    def _try_pose(key, value):
        """Parse a pose entry, skipping (with a warning) malformed/empty ones.

        A placeholder like ``"command_point": {}`` the operator hasn't filled in
        yet must NOT crash the whole load — it just isn't a known location until
        its point/orientation are set.
        """
        try:
            return _parse_pose_stamped(value)
        except (KeyError, TypeError):
            print(f"[gpsr] constants: location {key!r} has no valid "
                  "point/orientation yet — skipping it (fill it in to enable).")
            return None

    for key, value in constants.get("possible_poses", {}).items():
        pose = _try_pose(key, value)
        if pose is not None:
            KNOWN_LOCATIONS[key] = pose
    for key, value in constants.get("egpsr_rooms", {}).items():
        if key in KNOWN_LOCATIONS:
            continue  # possible_poses wins
        pose = _try_pose(key, value)
        if pose is not None:
            KNOWN_LOCATIONS[key] = pose
    for key, value in constants.get("possible_objects", {}).items():
        KNOWN_OBJECT_PROMPTS[key] = value
        KNOWN_OBJECT_NAMES.add(_normalize(str(key)))
    # Object default locations (skip _comment-style keys).
    for key, value in constants.get("default_locations", {}).items():
        if str(key).startswith("_"):
            continue
        DEFAULT_OBJECT_LOCATIONS[str(key).lower()] = str(value)
    # Per-room search-spot sweep lists (skip _comment-style keys).
    for key, value in constants.get("search_spots", {}).items():
        if str(key).startswith("_"):
            continue
        if isinstance(value, list) and value:
            ROOM_SEARCH_SPOTS[str(key).lower()] = [str(v) for v in value]
    # Rooms as first-class navigable locations: alias each search_spots
    # room (that has no pose of its own) to its first spot's pose. This
    # puts rooms in the planner prompt's known-location list and in the
    # validators, and makes resolve_pose hit them BEFORE the dynamic-
    # location registry. Without this the 2026-08-27 battery's LLM plans
    # worked around the missing rooms with record_position(label=room) at
    # the START pose — registering e.g. 'bedroom' at the robot's feet —
    # and every goto(room) navigated to the spot the robot already stood
    # on (zero movement in all four room-targeted runs).
    for room, spots in ROOM_SEARCH_SPOTS.items():
        if room in KNOWN_LOCATIONS:
            continue  # an explicit possible_poses/egpsr_rooms entry wins
        for spot in spots:
            pose = KNOWN_LOCATIONS.get(spot)
            if pose is not None:
                KNOWN_LOCATIONS[room] = pose
                break
    # No-grasp furniture (grasp there -> ask referee). Config-driven; defaults to
    # shelf / cabinet / coat_rack when the key is absent.
    ng = constants.get("no_grasp_locations")
    if isinstance(ng, list) and ng:
        NO_GRASP_LOCATIONS.clear()
        NO_GRASP_LOCATIONS.update(str(x).lower() for x in ng if not str(x).startswith("_"))
    # Appliances the robot can never open/reach into are ALWAYS bypassed, whatever
    # the config said (a partial override must not silently re-enable grasping from
    # a fridge / washing machine / dishwasher).
    NO_GRASP_LOCATIONS.update(ALWAYS_NO_GRASP)


# ---------------------------------------------------------------------------
# LLM planning
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
        ``find_person`` then ``approach_person`` FIRST so the robot is next to
        them. The answer is buffered, so for "ask the person X and tell ME X"
        follow with ``goto(location=start_position)`` then a text-less
        ``announce`` to report it back at the operator (see ``announce``).
    - follow(person?: str)
        Continuously follow a person. ``person`` is optional and informational.
        Plan ``find_person`` then ``approach_person`` before it so the robot
        starts next to them. If the command says "follow X to the Y" (or
        "follow them to the Y"), emit ``follow(person=X)`` THEN
        ``goto(location=Y)`` so the destination is reached when the person stops.
        ONLY when the command explicitly says follow / accompany / come with;
        "find/locate/meet <person> in <room>" is goto + find_person, never follow.
    - guide(location: str)
        Lead a person to ``location``. MUST be preceded by ``find_person``
        then ``approach_person`` in the same plan (locate the person and walk
        to them before leading). Never use ``guide`` to express "go
        yourself" — that is ``goto``. ONLY when the command says guide/lead/take
        <person> to <location>.
    - grasp(object: str, from_shelf?: bool, from_cabinet?: bool, from_coat_rack?: bool)
        Pick up an object from the surface in front of the robot. ``grasp`` moves
        the arm to the table-grasp scan pose, detects the object on the table
        with the ARM (RealSense) camera ITSELF, and picks it up — you do NOT add
        a separate find/scan step before it and you do NOT use the head camera.
        Always plan a ``goto(location)`` first so the robot is at the surface
        where the object is. The robot CANNOT safely grasp from a SHELF, a
        CABINET, or a COAT RACK — set the matching flag (``from_shelf`` /
        ``from_cabinet`` / ``from_coat_rack`` = true) when the object is on one of
        those (the command says so, or the object's Default location is one of
        them). It then skips the grasp and asks a human referee to hand the
        object over instead.
    - open(location: str)
        Ask a human referee to OPEN a closed container/appliance the robot
        cannot open itself — a REFRIGERATOR / FRIDGE, a WASHING MACHINE, or a
        DISHWASHER (and closed CABINETS). Emit ``goto(location)`` FIRST (the robot
        must already be standing at it), then ``open(location=<same>)`` before any
        find/scan/grasp of what is inside. The robot does NOT drive during
        ``open``. Whatever is inside must still be retrieved with ``grasp`` — which
        is itself an ask-referee handover for these locations (see below).
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
        Count visible instances of ``object`` and announce the number on the
        spot. count() performs its own visual scan — NEVER emit find_object for
        the same object before count. If the command names a location, emit
        goto first, then count directly. If the command says to tell ME the
        number ("tell me how many ... in the kitchen"), the count happens away
        from the operator, so follow it with ``goto(location=start_position)``
        then a text-less ``announce`` so the operator actually hears the result
        (see ``announce``).
    - answer_question()
        Listen to a question and answer it.
    - announce(text?: str)
        Speak. TWO modes:
        (a) ``announce(text=...)`` speaks the literal ``text`` — use for fixed
            output you know at plan time: team info, explaining a refusal, etc.
            ``text`` must be the final resolved phrase.
        (b) ``announce()`` with NO text speaks the LAST RESULT the robot
            gathered — the number from ``count``, the description from
            ``describe_person``, the answer from ``ask_person``, the observation
            from ``vlm_fallback`` (whichever ran most recently). This is the
            generalized "go gather X, come back and tell ME X" reporter: do the
            perception action, then ``goto(location=start_position)``, then
            ``announce()`` with no text. NEVER hand-write the result into a
            templated ``announce(text=...)`` — you cannot know it at plan time.
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
        with a vision model, and SPEAK the answer where it is looking. Use ONLY
        when a clause needs the robot to LOOK at something and NO specific
        action above covers it — e.g. "what colour is the X", "is the door
        open", "what is on the table", "what is the person holding". Navigate
        (goto) to the right place first. If the command says to tell ME and the
        robot had to go elsewhere to look, follow it with
        ``goto(location=start_position)`` then a text-less ``announce`` to report
        the observation back at the operator (see ``announce``). Prefer the
        specific actions (count, describe_person, find_object) whenever they fit.
    - llm_fallback(question: str)
        LAST-RESORT general-knowledge fallback: answer a NON-visual question
        with a language model. Use ONLY for questions not about the robot's
        surroundings and not covered above — e.g. "what is the date / day /
        time", simple facts or arithmetic. ``question`` is the literal question.
        (The robot speaks the answer itself; do not add an announce after it.)

    Compose composite commands by emitting multiple atomic actions in order.
    Example: "bring me the coke from the kitchen" =>
        [
          {"action": "goto", "params": {"location": "kitchen"}},
          {"action": "grasp", "params": {"object": "coke"}},
          {"action": "deliver", "params": {"object": "coke", "recipient": "me", "recipient_location": "start_position"}}
        ]
    Example: "fetch me a coke" (no location given -> use coke's default location) =>
        [
          {"action": "goto", "params": {"location": "kitchen"}},
          {"action": "grasp", "params": {"object": "coke"}},
          {"action": "deliver", "params": {"object": "coke", "recipient": "me", "recipient_location": "start_position"}}
        ]
    Example: "get the plate from the dishwasher and bring it to me" (closed
    appliance -> open + ask-referee grasp) =>
        [
          {"action": "goto", "params": {"location": "dishwasher"}},
          {"action": "open", "params": {"location": "dishwasher"}},
          {"action": "grasp", "params": {"object": "plate", "ask_referee": true}},
          {"action": "deliver", "params": {"object": "plate", "recipient": "me", "recipient_location": "start_position"}}
        ]
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

    Use only the action names listed below, and only known LOCATIONS. OBJECTS
    are NOT restricted: the vision system is open-vocabulary, so pass through
    whatever object word the command names (e.g. "bottle", "cup", "remote",
    "coke"). The "Known objects" list is only the typical arena items as a hint —
    NEVER refuse or announce "I cannot find a known object matching X"; just use
    X as the object. Only refuse if there is no LOCATION you can resolve.

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
    6. NEVER return an empty plan, and never refuse. Always emit at least a
       best-effort plan of known actions. If part of the command is impossible,
       unknown, or unclear, still emit the steps you CAN do and finish with an
       ``announce(text=...)`` that explains the part you could not do. A
       non-empty plan is ALWAYS required — even "I could not find a known
       location for X" must be expressed as an ``announce`` step, not an empty
       plan.
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
       and the robot had to leave to do the task, the result MUST be reported
       back AT the operator: do the gathering action (count / describe_person /
       ask_person / vlm_fallback — it buffers its result), then
       ``goto(location=start_position)``, then a text-less ``announce`` (no
       ``text`` param) which speaks that buffered result. Do NOT hand-write the
       result into a templated ``announce(text=...)`` (you cannot know it at
       plan time), and a perception action's own on-spot line does NOT satisfy
       "tell ME" — the text-less ``announce`` at start_position does. (For
       "bring/give me", use ``deliver`` with
       ``recipient_location=start_position`` instead.)
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
         use ``describe_person`` for these, and NEVER a bare ``announce`` (it
         speaks but never listens).
       For "ask the person X and tell ME X" (X askable): ``ask_person`` →
       ``goto(location=start_position)`` → text-less ``announce``.
    14. Do not refuse a clause just because no exact action fits. If it needs
       LOOKING at the scene, use ``vlm_fallback(question=...)``; if it is a
       general non-visual question (date/day/time, a fact), use
       ``llm_fallback(question=...)``. These are last resorts — always prefer a
       specific action (count, describe_person, find_object, ask_person,
       announce) when one fits. Only fall back when nothing else does. For "go
       and look at X, then tell ME": ``vlm_fallback`` at X →
       ``goto(location=start_position)`` → text-less ``announce``.
    15. Objects are NOT bound to a fixed room. When the command names WHERE to
       fetch / find / count / grasp an object ("a coke FROM THE LIVING ROOM",
       "the apple ON THE SHELF"), search THAT named location. NEVER refuse, and
       NEVER return an empty plan, just because the object is one you would
       normally expect somewhere else — the location named in the command always
       wins over any default. Only when the command gives NO location may you
       fall back to the object's usual place (the "Default object locations"
       list above).
    16. To FETCH / BRING / PICK UP an object: emit ``goto(location)`` then
       ``grasp(object)`` (then ``deliver`` if it goes to someone). ``grasp``
       moves the arm to the table-grasp pose and detects the object on the table
       with the ARM camera itself — do NOT add any find/scan step before grasp
       and do NOT use the head camera to look for it. ``location`` is the place
       named in the command, or — if none is named — the object's default
       location from the "Default object locations" list. Plain ``goto`` +
       ``find_object`` / ``count`` are still correct for NON-grasp finds and
       counts (e.g. "how many cokes are in the kitchen").
    17. The robot CANNOT safely grasp from a SHELF, a CABINET, or a COAT RACK.
       If the object being grasped is on one of these — the command says
       "shelf"/"bookcase", "cabinet"/"cupboard", or "coat rack"/"coatrack", or
       the object's Default location (list above) is one of them — set the
       matching flag on the ``grasp`` step: ``from_shelf`` / ``from_cabinet`` /
       ``from_coat_rack`` = true. The robot will then ask a referee to hand the
       object over instead of attempting the grasp.
    18. The robot ALSO cannot open or reach into a REFRIGERATOR / FRIDGE, a
       WASHING MACHINE, or a DISHWASHER. When an object to fetch/find/count is
       inside one of these, the plan MUST be:
       ``goto(location)`` → ``open(location)`` → (optional find/count) →
       ``grasp(object, ask_referee=true)``. The ``open`` step asks a referee to
       open the door, and the ``grasp`` step (with ``ask_referee=true``) asks the
       referee to hand the object over — the robot never opens the door or reaches
       inside itself. Only add ``open`` for these closed containers (and closed
       cabinets), never for open surfaces like a table or shelf.
""").strip()


def _extract_json_object(text: str) -> Optional[Dict[str, Any]]:
    """Best-effort parse of a JSON object from a model reply.

    Handles models that don't strictly honour ``response_format`` (DeepSeek and
    other reasoning models): tries a direct ``json.loads`` first, then strips a
    ```json fenced block, then falls back to the outermost ``{...}`` span.
    Returns the dict, or ``None`` if nothing parses.
    """
    if not text:
        return None
    try:
        obj = json.loads(text)
        return obj if isinstance(obj, dict) else None
    except Exception:
        pass
    # ```json ... ``` fenced block
    fence = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", text, re.DOTALL)
    if fence:
        try:
            obj = json.loads(fence.group(1))
            return obj if isinstance(obj, dict) else None
        except Exception:
            pass
    # outermost { ... } span
    start = text.find("{")
    end = text.rfind("}")
    if 0 <= start < end:
        try:
            obj = json.loads(text[start:end + 1])
            return obj if isinstance(obj, dict) else None
        except Exception:
            pass
    return None


def _build_planner_user_prompt(
    command: str,
    state_log: List[str],
    failure_msg: Optional[str] = None,
    nonce: Optional[str] = None,
) -> str:
    from datetime import datetime
    known_loc = ", ".join(sorted(KNOWN_LOCATIONS.keys())) or "(none)"
    known_obj = ", ".join(sorted(KNOWN_OBJECT_PROMPTS.keys())) or "(none)"
    default_loc = ", ".join(
        f"{k}={v}" for k, v in sorted(DEFAULT_OBJECT_LOCATIONS.items())
    ) or "(none)"
    body = (
        f"Current date and time: {datetime.now().strftime('%A, %B %d, %Y, %H:%M')}\n"
        f"Known locations: {known_loc}\n"
        f"Known objects (HINT ONLY — any object word is allowed, not just these): {known_obj}\n"
        f"Default object locations (where each object usually is, used only when "
        f"a fetch/find command names NO location): {default_loc}\n\n"
        f"{ACTION_CATALOGUE_DESCRIPTION}\n\n"
        f"Command:\n{command}\n\n"
        f"Completed steps so far:\n{json.dumps(state_log, indent=2)}\n"
    )
    if failure_msg:
        body += (
            f"\nThe previous attempt failed with: {failure_msg}\n"
            "Re-plan from the current state. Do not repeat completed steps.\n"
        )
    if nonce:
        # A fresh nonce every call makes each planning request byte-unique, so
        # re-issuing an identical command (or re-planning) cannot return a
        # cached / deterministic copy of a previous refusal — the model
        # re-evaluates from scratch. The token itself carries no meaning.
        body += f"\n(Planning request id: {nonce} — ignore, ensures a fresh plan.)"
    body += "\nReturn the JSON plan now."
    return body


def _clean_plan(plan_raw: Any) -> Tuple[List[Dict[str, Any]], List[str]]:
    """Keep only well-formed {action, params} steps using known actions.

    Consecutive identical steps (same action AND params) are collapsed to one:
    a replan that emits ``place, place`` would execute the second against a
    now-empty gripper and fail for a reason that has nothing to do with the
    command.
    """
    cleaned: List[Dict[str, Any]] = []
    dropped: List[str] = []
    if not isinstance(plan_raw, list):
        return cleaned, [f"<{type(plan_raw).__name__}>"]
    for step in plan_raw:
        if not isinstance(step, dict):
            dropped.append(f"<{type(step).__name__}>")
            continue
        action = step.get("action")
        params = step.get("params", {}) or {}
        if action not in ACTION_FACTORIES:
            dropped.append(str(action))
            continue
        if cleaned and cleaned[-1]["action"] == action and cleaned[-1]["params"] == params:
            dropped.append(f"duplicate:{action}")
            continue
        cleaned.append({"action": action, "params": params})
    return cleaned, dropped


def _fallback_plan(command: str) -> List[Dict[str, Any]]:
    """Guaranteed non-empty plan when the LLM cannot produce a valid one.

    Never let the robot silently refuse: emit a single spoken acknowledgement so
    the operator hears a response and the command always has *a* plan. Only hit
    after every planning attempt failed — realistic commands never reach here.

    ``params["acknowledgement"] = True`` marks this announce as a planning
    -failure apology, not a real answer: the cached entry this plan is stored
    under is READY with a non-None error (``"all N attempts failed …"``, see
    GPSRTargetPlanner.plan_target), and the executor's REQUESTING branch only
    skips a cached entry whose error starts with IDENTICAL_PLAN_ERROR_PREFIX —
    this one does not, so it IS swapped in and executed. Without the flag,
    validators._action_verdict's answered-fallback (announce establishes
    answered(question)) would turn that planning failure into a postcondition
    gate PASS.
    """
    return [{
        "action": "announce",
        "params": {
            "text": "I heard your command but could not work out a complete "
                    "plan for it. I will skip it for now.",
            "acknowledgement": True,
        },
    }]


def _offline_mock_plan(command: str) -> List[Dict[str, Any]]:
    """Return a deterministic, network-free plan for the all-mock preset."""
    text = "Mock mode: planner bypassed; no network request was made."
    if command:
        text += f" Command received: {command}"
    return [{"action": "announce", "params": {"text": text}}]


class BtNode_PlanActions(Behaviour):
    """Plan a command into actions, with internal retries and a guaranteed plan.

    The planning thread loops up to ``max_attempts``: each attempt calls the LLM
    (fresh nonce, temperature rising per attempt), then cleans + validates the
    result locally. The *reason* a try was rejected (bad JSON, empty plan,
    validator complaint) is fed back into the next prompt so the model fixes it
    rather than resampling the same dead end. If every attempt fails it falls
    back to a non-empty acknowledgement plan — so ``update()`` ALWAYS returns
    SUCCESS with a non-empty plan and the orchestrator never silently refuses.
    """

    def __init__(
        self,
        name: str = "Plan actions",
        rephrase_on_failure: bool = False,
        max_attempts: int = 4,
    ):
        super().__init__(name)
        self._offline_mock = _offline_planner_enabled()
        self._client_oai = None
        if not self._offline_mock:
            self._client_oai = _openai_client()
        self._bb = None
        self._thread: Optional[threading.Thread] = None
        self._plan_result: Optional[List[Dict[str, Any]]] = None
        self._fell_back: bool = False
        self._attempts_used: int = 0
        self._rephrase_on_failure = rephrase_on_failure
        self._max_attempts = max(1, int(max_attempts))
        self._telemetry = get_default_telemetry()
        self._task_id: str | None = None
        self._last_raw_response: str = ""
        self._reasoning: str = ""
        self._attempt_ids: dict[int, str] = {}

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.COMMAND, access=Access.READ)
        self._bb.register_key(bb_keys.PLAN, access=Access.WRITE)
        self._bb.register_key(bb_keys.PLAN_INDEX, access=Access.WRITE)
        self._bb.register_key(bb_keys.STATE_LOG, access=Access.READ)
        self._bb.register_key(bb_keys.LAST_FAILURE, access=Access.WRITE)
        self._bb.register_key(bb_keys.CORRECTION_COUNT, access=Access.WRITE)
        self._bb.register_key(bb_keys.RUN_ID, access=Access.READ)
        self._bb.register_key(bb_keys.TASK_ID, access=Access.READ)
        self._bb.register_key(bb_keys.PLAN_REVISION, access=Access.WRITE)

    def _emit(self, event_type: str, payload: dict[str, Any], *, phase: str = "planning") -> None:
        telemetry = self._telemetry or get_default_telemetry()
        if telemetry is None:
            return
        try:
            self._task_id = self._task_id or self._bb.get(bb_keys.TASK_ID)
        except Exception:
            pass
        telemetry.emit(event_type, payload, task_id=self._task_id, phase=phase)

    def _call_llm(
        self, user_prompt: str, temperature: float, *, attempt: int = 0,
        nonce: str | None = None, command: str = "",
    ) -> Tuple[Optional[dict], Optional[str]]:
        """One LLM round-trip → (parsed JSON dict, error string). Exactly one is set."""
        started_ns = time.monotonic_ns()
        attempt_id = f"{self._task_id or 'task'}/attempt-{attempt}-{uuid.uuid4().hex[:10]}"
        self._attempt_ids[attempt] = attempt_id
        self._emit("planner.request", {
            "attempt_id": attempt_id,
            "attempt": attempt,
            "model": OPENAI_MODEL,
            "temperature": temperature,
            "max_tokens": max(OPENAI_MAX_TOKENS, 8192),
            "nonce": nonce,
            "command": command,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": user_prompt},
            ],
        })
        try:
            # A fresh random seed every call makes the provider treat this as a
            # brand-new request and sample anew — it defeats any response
            # caching / dedup of identical consecutive requests (the "re-issue
            # the same command and it refuses again until you say something else"
            # symptom). Combined with the per-call nonce in the prompt, no two
            # planning calls are ever identical.
            kwargs = dict(
                model=OPENAI_MODEL,
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": user_prompt},
                ],
                temperature=temperature,
                seed=random.randint(1, 2_000_000_000),
                # Reasoning models spend tokens thinking before the JSON; a high
                # cap avoids truncation and costs nothing on a short reply.
                max_tokens=max(OPENAI_MAX_TOKENS, 8192),
                response_format={"type": "json_object"},
            )
            # OpenRouter reasoning-effort knob for the reasoning-capable planner
            # (openai/gpt-5.6-luna and friends). High effort = deeper planning.
            # extra_body (not the typed kwarg) so the request works on SDK 2.30.
            if "gpt-5.6-luna" in OPENAI_MODEL:
                kwargs["extra_body"] = {"reasoning": {"effort": "high"}}
            try:
                resp = self._client_oai.chat.completions.create(**kwargs)
            except Exception as exc:  # a model/provider that rejects `seed`?
                if "seed" not in repr(exc).lower():
                    raise
                kwargs.pop("seed", None)
                resp = self._client_oai.chat.completions.create(**kwargs)
            msg = resp.choices[0].message
            content = (getattr(msg, "content", None) or "").strip()
            reasoning = (getattr(msg, "reasoning", None) or "").strip()
            raw = content
            if not raw:
                raw = reasoning
            self._last_raw_response = raw
            self._reasoning = reasoning
            usage = getattr(resp, "usage", None)
            usage_dict = {}
            if usage is not None:
                usage_dict = {
                    key: getattr(usage, key) for key in
                    ("prompt_tokens", "completion_tokens", "total_tokens")
                    if getattr(usage, key, None) is not None
                }
            self._emit("planner.response", {
                "attempt": attempt, "attempt_id": attempt_id, "model": OPENAI_MODEL,
                "latency_ms": round((time.monotonic_ns() - started_ns) / 1_000_000, 3),
                "raw_content": content, "reasoning": reasoning,
                "finish_reason": getattr(resp.choices[0], "finish_reason", None),
                "provider_request_id": getattr(resp, "id", None),
                "usage": usage_dict,
            })
            parsed = _extract_json_object(raw)
            if parsed is None:
                error = ("your reply was not parseable JSON "
                              f"(content was {'empty' if not raw else 'non-JSON'}). "
                              "Reply with ONLY the JSON object.")
                self._emit("planner.error", {"attempt": attempt, "attempt_id": attempt_id, "latency_ms": round((time.monotonic_ns() - started_ns) / 1_000_000, 3), "error": error})
                return None, error
            return parsed, None
        except Exception as exc:  # noqa: BLE001 — surface anything for retry
            error = f"LLM call error: {exc!r}"
            self._emit("planner.error", {"attempt": attempt, "attempt_id": attempt_id, "latency_ms": round((time.monotonic_ns() - started_ns) / 1_000_000, 3), "error": error})
            return None, error

    def initialise(self):
        self._plan_result = None
        self._fell_back = False
        self._attempts_used = 0
        self._last_raw_response = ""
        self._reasoning = ""
        self._attempt_ids = {}
        try:
            command = self._bb.get(bb_keys.COMMAND)
        except KeyError:
            command = ""
        try:
            state_log = self._bb.get(bb_keys.STATE_LOG) or []
        except KeyError:
            state_log = []
        try:
            seed_failure = self._bb.get(bb_keys.LAST_FAILURE)
        except KeyError:
            seed_failure = None
        try:
            self._task_id = self._bb.get(bb_keys.TASK_ID)
        except KeyError:
            self._task_id = None
        self._emit("task.command_received", {"command": command, "replan": self._rephrase_on_failure}, phase="intake")
        if not self._rephrase_on_failure:
            # Initial plan for a fresh command: plan from the command ALONE.
            # Never carry session history (a previous command's completed/failed
            # steps) into a new command's plan — that cross-command bleed is what
            # made a re-issued command behave differently from the first issue.
            seed_failure = None
            state_log = []

        if self._offline_mock:
            # Full mock must be deterministic and must never touch the network.
            # Keep the normal blackboard/update path so dispatch and completion
            # are still exercised by offline integration tests.
            self._attempts_used = 0
            self._fell_back = False
            self._plan_result = _offline_mock_plan(command or "")
            self.feedback_message = "Offline mock planner (OpenRouter disabled)"
            self._thread = None
            return

        known_locs = set(KNOWN_LOCATIONS.keys())
        known_loc_arg = (known_locs | START_LOCATION_ALIASES) if known_locs else None
        known_actions = set(ACTION_FACTORIES.keys())
        max_attempts = self._max_attempts

        def _call():
            last_reason = seed_failure
            for attempt in range(max_attempts):
                # Fresh nonce each try → no cached/deterministic refusal. Warm the
                # sampler as attempts climb so the model explores a new plan.
                nonce = uuid.uuid4().hex[:8]
                temperature = min(0.9, OPENAI_TEMPERATURE + 0.2 * attempt)
                if last_reason and attempt == 0:
                    temperature = min(0.9, OPENAI_TEMPERATURE + 0.5)
                prompt = _build_planner_user_prompt(
                    command, state_log, last_reason, nonce=nonce,
                )
                parsed, err = self._call_llm(
                    prompt, temperature, attempt=attempt + 1, nonce=nonce,
                    command=command or "",
                )
                if err is not None:
                    last_reason = err
                    print(f"[planner] attempt {attempt+1}/{max_attempts} -> {err}")
                    continue
                cleaned, dropped = _clean_plan(parsed.get("plan", []))
                raw_actions = [
                    s.get("action") if isinstance(s, dict) else f"<{type(s).__name__}>"
                    for s in (parsed.get("plan", []) or [])
                ]
                print(f"[planner] attempt {attempt+1}/{max_attempts}: raw {raw_actions} "
                      f"| kept {[s['action'] for s in cleaned]} | dropped {dropped}")
                if not cleaned:
                    last_reason = (
                        "you returned an EMPTY plan (or only unknown actions). You "
                        "MUST return a NON-EMPTY plan of the known actions — never "
                        "refuse. If part of the command is impossible, still emit the "
                        "doable steps and finish with announce(text=...) explaining "
                        "what you could not do."
                    )
                    self._emit("plan.validated", {
                        "attempt": attempt + 1, "attempt_id": self._attempt_ids.get(attempt + 1), "accepted": False,
                        "raw_plan": parsed.get("plan") if isinstance(parsed, dict) else None,
                        "cleaned_plan": cleaned, "dropped": dropped,
                        "reason": last_reason,
                    })
                    continue
                ok, reason = validate_plan(
                    cleaned, command or "", known_actions,
                    known_locations=known_loc_arg,
                )
                if not ok:
                    last_reason = reason
                    print(f"[planner] attempt {attempt+1}/{max_attempts} REJECTED: {reason}")
                    self._emit("plan.validated", {
                        "attempt": attempt + 1, "attempt_id": self._attempt_ids.get(attempt + 1), "accepted": False,
                        "raw_plan": parsed.get("plan"), "cleaned_plan": cleaned,
                        "dropped": dropped, "reason": reason,
                    })
                    continue
                # Accepted.
                self._attempts_used = attempt + 1
                self._plan_result = cleaned
                self._emit("plan.validated", {
                    "attempt": attempt + 1, "attempt_id": self._attempt_ids.get(attempt + 1), "accepted": True,
                    "raw_plan": parsed.get("plan"), "cleaned_plan": cleaned,
                    "dropped": dropped, "reasoning": self._reasoning,
                })
                print(f"[planner] accepted on attempt {attempt+1}: "
                      f"{[s['action'] for s in cleaned]}")
                return
            # Every attempt failed → guaranteed non-empty fallback plan.
            self._attempts_used = max_attempts
            self._fell_back = True
            self._plan_result = _fallback_plan(command or "")
            print(f"[planner] all {max_attempts} attempts failed "
                  f"(last reason: {last_reason}) -> fallback acknowledgement plan")

        self._thread = threading.Thread(target=_call, daemon=True)
        self._thread.start()
        self.feedback_message = "LLM planning..."

    def update(self):
        if self._plan_result is None:
            return Status.RUNNING

        plan = self._plan_result
        self._bb.set(bb_keys.PLAN, plan, overwrite=True)
        self._bb.set(bb_keys.PLAN_INDEX, 0, overwrite=True)
        try:
            previous_revision = int(self._bb.get(bb_keys.PLAN_REVISION) or 0)
        except (KeyError, TypeError, ValueError):
            previous_revision = 0
        revision = previous_revision + 1
        self._bb.set(bb_keys.PLAN_REVISION, revision, overwrite=True)
        # Clear the stale failure so it isn't fed into the next planning call.
        self._bb.set(bb_keys.LAST_FAILURE, "", overwrite=True)
        try:
            count = self._bb.get(bb_keys.CORRECTION_COUNT)
        except KeyError:
            count = 0
        self._bb.set(bb_keys.CORRECTION_COUNT, count, overwrite=True)
        if self._offline_mock:
            tag = " [OFFLINE MOCK]"
        else:
            tag = " [FALLBACK]" if self._fell_back else ""
        self.feedback_message = (
            f"Planned {len(plan)} step(s) in {self._attempts_used} attempt(s){tag}: "
            f"{[s['action'] for s in plan]}"
        )
        if previous_revision > 0:
            self._emit("plan.superseded", {
                "old_revision": previous_revision,
                "new_revision": revision,
                "reason": "self_correction" if self._rephrase_on_failure else "replanned",
            })
        self._emit("plan.committed", {
            "plan": plan, "plan_revision": revision,
            "attempts": self._attempts_used, "fallback": self._fell_back,
            "offline_mock": self._offline_mock,
        })
        return Status.SUCCESS

    def terminate(self, new_status):
        self._thread = None


# ---------------------------------------------------------------------------
# Step pop / parameter materialisation
# ---------------------------------------------------------------------------

def _norm_loc(name: Any) -> str:
    """Normalise a location name for matching: lowercase, spaces -> underscores.

    The generator / operator / LLM all say "living room", while the map waypoint
    (constants.json possible_poses) is stored as ``living_room``. Without this
    normalisation a perfectly valid goto(location="living room") would never
    resolve to a pose at runtime. Both the underscore and space spellings match.
    """
    return str(name).lower().replace(" ", "_").strip()


def resolve_pose(bb_client, name: Any) -> Optional[PoseStamped]:
    """Resolve a location name to a PoseStamped, or None if unknown.

    Resolution order: start-position aliases (pose captured at command
    start) → known map locations (exact then case- and space/underscore-
    insensitive) → runtime-recorded dynamic-location registry (labels fixed by
    an earlier record_position step).
    """
    if not name:
        return None
    key = _norm_loc(name)
    if key in START_LOCATION_ALIASES:
        try:
            return bb_client.get(bb_keys.START_POSE)
        except KeyError:
            return None
    if name in KNOWN_LOCATIONS:
        return KNOWN_LOCATIONS.get(name)
    for known_name, pose in KNOWN_LOCATIONS.items():
        if _norm_loc(known_name) == key:
            return pose
    try:
        registry = bb_client.get(bb_keys.DYNAMIC_LOCATIONS) or {}
    except KeyError:
        registry = {}
    hit = registry.get(key)
    if hit is not None:
        return hit
    # Room fallback: a ROOM name with no mapped pose of its own (rcw2026
    # ships placements in possible_poses but no egpsr_rooms) resolves to
    # the room's first search spot that has a pose. Without this, every
    # goto(room) left TARGET_POSE untouched and the robot "navigated" to
    # its own current pose — zero movement across the whole 2026-08-27
    # battery for all four room-targeted commands (goals within 10 cm of
    # spawn, drifting with AMCL, vs the real kitchen_table waypoint in the
    # one placement-targeted run).
    for spot in ROOM_SEARCH_SPOTS.get(key, ()):  # ordered sweep list
        pose = KNOWN_LOCATIONS.get(spot)
        if pose is not None:
            return pose
    return None


def materialise_params(bb_client, action: str, params: Dict[str, Any]) -> None:
    """Translate the LLM's params into the BB keys the small trees consume.

    Shared by the legacy ``BtNode_PopNextAction`` and the two-layer
    ``BtNode_MaterialiseStep`` so every dispatched step runs the exact same
    safeguarded parameter resolution (appliance must-do referee-open,
    no-grasp shelf/cabinet/coat-rack inference, search-spot sweep, ...).
    """
    # Location → PoseStamped lookup (see resolve_pose for the order).
    loc_name = params.get("location") or params.get("recipient_location")
    if loc_name:
        bb_client.set(bb_keys.TARGET_LOCATION, loc_name, overwrite=True)
        pose = resolve_pose(bb_client, loc_name)
        if pose is not None:
            bb_client.set(bb_keys.TARGET_POSE, pose, overwrite=True)
        else:
            # Leaving the previous TARGET_POSE in place makes the next goto
            # a silent navigate-to-self (see resolve_pose's room fallback
            # note). Say so loudly — an unresolvable location is a data bug
            # in constants.json, not a runtime condition to paper over.
            print(
                f"[gpsr] WARNING: location {loc_name!r} resolved to NO pose; "
                "the previous TARGET_POSE (if any) will be reused. Add the "
                "location to possible_poses/egpsr_rooms or search_spots."
            )

    # search_object: resolve the room's sweep spots into SEARCH_POSE_0..N.
    # location is optional — when omitted, fall back to the object's default
    # location (DEFAULT_OBJECT_LOCATIONS). A location with no explicit
    # search-spot list sweeps just itself. Unused slots are cleared to None
    # so the sweep guards them out. The resolved fallback is written back into
    # a local copy of params so the contract nav-record block below (which
    # reads params[contract.self_establishes["at_robot"]]) sees it too.
    if action == "search_object":
        params = dict(params)
        loc = params.get("location")
        obj = params.get("object")
        if not loc and obj:
            loc = DEFAULT_OBJECT_LOCATIONS.get(str(obj).lower())
            params["location"] = loc
        if loc:
            bb_client.set(bb_keys.TARGET_LOCATION, loc, overwrite=True)
        spot_names = ROOM_SEARCH_SPOTS.get(str(loc).lower(), [loc]) if loc else []
        for i, search_key in enumerate(SEARCH_POSE_KEYS):
            pose = resolve_pose(bb_client, spot_names[i]) if i < len(spot_names) else None
            bb_client.set(search_key, pose, overwrite=True)

    # Track where the robot is ABOUT TO navigate, so a following grasp can
    # tell it is a shelf grasp even before that navigation finishes. Which
    # actions and which param is the action contract's business, not this
    # function's.
    #
    # J4 (round-3 adversarial review, M10): this is PENDING_NAV_LOCATION, not
    # LAST_NAV_LOCATION -- materialisation happens before the step has run,
    # so writing the gate-evidence key here let a goto/self-navigating action
    # that never actually succeeded still verify at_robot(<dest>) VALID.
    # LAST_NAV_LOCATION (the at_robot() gate's evidence) is written only on
    # the step-finished path once the step actually SUCCEEDS -- see
    # ``record_nav_on_success`` / ``BtNode_LogStepResult``.
    try:
        contract = _contract_for(action)
    except KeyError:
        contract = None
    if contract is not None and "last_nav_location" in contract.records:
        nav_param = contract.self_establishes.get("at_robot")
        nav_value = params.get(nav_param) if nav_param else None
        if nav_value is not None and str(nav_value).strip():
            bb_client.set(bb_keys.PENDING_NAV_LOCATION, str(nav_value), overwrite=True)

    # grasp: decide whether to bypass the real grasp and ask a referee. The
    # robot cannot safely grasp from a shelf, cabinet, or coat rack. Truthy
    # from ANY signal: an explicit planner flag (from_shelf / from_cabinet /
    # from_coat_rack / ask_referee), a no-grasp word in the grasp step's
    # location/surface, OR the location the robot last navigated to (covers
    # default-furniture objects even if the planner omits the flag).
    if action == "grasp":
        def _is_no_grasp(v) -> bool:
            s = str(v or "").lower()
            return any(loc in s or loc.replace("_", " ") in s
                       for loc in NO_GRASP_LOCATIONS)
        try:
            # J4: shelf/no-grasp inference is an intent check, not a fact
            # verification -- it deliberately still reads the materialised
            # (pending) nav target rather than waiting on step-finished
            # success, same as before this change.
            last_nav = bb_client.get(bb_keys.PENDING_NAV_LOCATION)
        except KeyError:
            last_nav = ""
        ask_referee = bool(
            params.get("from_shelf")
            or params.get("from_cabinet")
            or params.get("from_coat_rack")
            or params.get("ask_referee")
            or _is_no_grasp(params.get("location"))
            or _is_no_grasp(params.get("surface"))
            or _is_no_grasp(last_nav)
        )
        bb_client.set(bb_keys.GRASP_ASK_REFEREE, ask_referee, overwrite=True)
        # When bypassing to the referee, drive to the no-grasp furniture
        # first (the grasp small tree's ex_machina branch gotos this pose).
        # Figure out WHICH furniture: an explicit flag maps 1:1, otherwise
        # take the no-grasp word that matched location / surface / last_nav.
        referee_loc, referee_pose = "", None
        if ask_referee:
            flag_map = {
                "from_shelf": "shelf",
                "from_cabinet": "cabinet",
                "from_coat_rack": "coat_rack",
            }
            for flag, furn in flag_map.items():
                if params.get(flag):
                    referee_loc = furn
                    break
            if not referee_loc:
                for cand in (params.get("location"), params.get("surface"), last_nav):
                    s = str(cand or "").lower()
                    for furn in NO_GRASP_LOCATIONS:
                        if furn in s or furn.replace("_", " ") in s:
                            referee_loc = furn
                            break
                    if referee_loc:
                        break
            if referee_loc:
                referee_pose = resolve_pose(bb_client, referee_loc)
        bb_client.set(bb_keys.GRASP_REFEREE_LOCATION,
                      referee_loc.replace("_", " "), overwrite=True)
        bb_client.set(bb_keys.GRASP_REFEREE_POSE, referee_pose, overwrite=True)
        # Closed appliances (fridge / washing machine / dishwasher) can't be
        # opened by the robot: flag it so the grasp ask-referee branch always
        # asks the referee to open it first, even if the planner omitted a
        # separate open() step.
        bb_client.set(bb_keys.GRASP_REFEREE_IS_APPLIANCE,
                      referee_loc in ALWAYS_NO_GRASP, overwrite=True)

    # record_position: stash the label so the small tree registers the
    # captured pose under it.
    if action == "record_position":
        bb_client.set(
            bb_keys.CURRENT_DYNLABEL,
            str(params.get("label") or "").strip(),
            overwrite=True,
        )

    # Object → vision prompt + name
    obj_name = params.get("object")
    if obj_name:
        bb_client.set(bb_keys.TARGET_OBJECT_NAME, obj_name, overwrite=True)
        prompt = KNOWN_OBJECT_PROMPTS.get(obj_name, obj_name)
        bb_client.set(bb_keys.TARGET_OBJECT_PROMPT, prompt, overwrite=True)

    # Person descriptor
    person = params.get("descriptor") or params.get("person") or params.get("recipient")
    if person:
        bb_client.set(bb_keys.TARGET_PERSON_PROMPT, person, overwrite=True)

    # announce: a literal ``text`` is spoken as-is; an announce with NO text
    # reports the latest gathered result buffered in REPORT_INFO by the most
    # recent count / describe_person / ask_person / vlm_fallback. This is the
    # generalized "go gather X then come back and tell ME X" reporter that
    # replaced the per-result report_* actions.
    if action == "announce":
        text = params.get("text") or params.get("message") or ""
        if not text:
            try:
                text = str(bb_client.get(bb_keys.REPORT_INFO) or "")
            except KeyError:
                text = ""
        bb_client.set(bb_keys.ANNOUNCE_TEXT, text, overwrite=True)

    # ask_person carries the literal question to speak
    if action == "ask_person":
        question = params.get("question") or params.get("text") or "Please tell me."
        bb_client.set(bb_keys.ASK_QUESTION, question, overwrite=True)

    # fallbacks carry the literal question to answer
    if action == "vlm_fallback":
        bb_client.set(
            bb_keys.VLM_QUESTION,
            params.get("question") or params.get("text") or "Describe what you see.",
            overwrite=True,
        )
    if action == "llm_fallback":
        bb_client.set(
            bb_keys.LLM_QUESTION,
            params.get("question") or params.get("text") or "",
            overwrite=True,
        )


def record_nav_on_success(bb_client, action: str, params: Dict[str, Any]) -> None:
    """Write LAST_NAV_LOCATION -- the at_robot() gate's evidence -- iff a
    navigating step actually SUCCEEDED.

    J4 (round-3 adversarial review, M10): called from the step-finished path
    (``BtNode_LogStepResult``), never at materialisation (``materialise_params``
    only ever records the INTENDED destination, to ``PENDING_NAV_LOCATION``).
    Without this split, a goto/self-navigating action that failed still left
    the materialised destination in the gate's evidence, so at_robot(<dest>)
    verified VALID for a navigation that never happened.
    """
    try:
        contract = _contract_for(action)
    except KeyError:
        contract = None
    if contract is None or "last_nav_location" not in contract.records:
        return
    nav_param = contract.self_establishes.get("at_robot")
    nav_value = params.get(nav_param) if nav_param else None
    if nav_value is not None and str(nav_value).strip():
        bb_client.set(bb_keys.LAST_NAV_LOCATION, str(nav_value), overwrite=True)


class BtNode_PopNextAction(Behaviour):
    """Pop ``plan[plan_index]``, resolve its params into BB targets.

    Returns:
        SUCCESS — a step is ready to execute.
        FAILURE — plan exhausted; outer loop should terminate.
    """

    def __init__(self, name: str = "Pop next action"):
        super().__init__(name)
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.TASK_ID, access=Access.READ)
        self._bb.register_key(bb_keys.PLAN_REVISION, access=Access.READ)
        self._bb.register_key(bb_keys.PLAN_INDEX, access=Access.READ)
        self._bb.register_key(bb_keys.PLAN, access=Access.READ)
        self._bb.register_key(bb_keys.PLAN_INDEX, access=Access.WRITE)
        self._bb.register_key(bb_keys.CURRENT_ACTION, access=Access.WRITE)
        self._bb.register_key(bb_keys.CURRENT_PARAMS, access=Access.WRITE)
        self._bb.register_key(bb_keys.TARGET_POSE, access=Access.WRITE)
        self._bb.register_key(bb_keys.TARGET_LOCATION, access=Access.WRITE)
        self._bb.register_key(bb_keys.TARGET_OBJECT_NAME, access=Access.WRITE)
        self._bb.register_key(bb_keys.TARGET_OBJECT_PROMPT, access=Access.WRITE)
        self._bb.register_key(bb_keys.TARGET_PERSON_PROMPT, access=Access.WRITE)
        self._bb.register_key(bb_keys.ANNOUNCE_TEXT, access=Access.WRITE)
        self._bb.register_key(bb_keys.REPORT_INFO, access=Access.READ)
        self._bb.register_key(bb_keys.ASK_QUESTION, access=Access.WRITE)
        self._bb.register_key(bb_keys.VLM_QUESTION, access=Access.WRITE)
        self._bb.register_key(bb_keys.LLM_QUESTION, access=Access.WRITE)
        self._bb.register_key(bb_keys.START_POSE, access=Access.READ)
        self._bb.register_key(bb_keys.DYNAMIC_LOCATIONS, access=Access.READ)
        self._bb.register_key(bb_keys.CURRENT_DYNLABEL, access=Access.WRITE)
        # written by goto/search_object at materialisation, read back by
        # grasp (shelf inference) -- J4: the intended-destination signal, not
        # the at_robot() gate's success-only LAST_NAV_LOCATION.
        self._bb.register_key(bb_keys.PENDING_NAV_LOCATION, access=Access.WRITE)
        self._bb.register_key(bb_keys.PENDING_NAV_LOCATION, access=Access.READ)
        self._bb.register_key(bb_keys.GRASP_ASK_REFEREE, access=Access.WRITE)
        self._bb.register_key(bb_keys.GRASP_REFEREE_LOCATION, access=Access.WRITE)
        self._bb.register_key(bb_keys.GRASP_REFEREE_POSE, access=Access.WRITE)
        self._bb.register_key(bb_keys.GRASP_REFEREE_IS_APPLIANCE, access=Access.WRITE)
        for search_pose_key in SEARCH_POSE_KEYS:
            self._bb.register_key(search_pose_key, access=Access.WRITE)

    def update(self):
        try:
            plan = self._bb.get(bb_keys.PLAN) or []
            index = self._bb.get(bb_keys.PLAN_INDEX)
        except KeyError:
            self.feedback_message = "Plan not initialised"
            return Status.FAILURE

        if index >= len(plan):
            self.feedback_message = f"Plan exhausted ({index}/{len(plan)})"
            return Status.FAILURE

        step = plan[index]
        action = step.get("action")
        params = step.get("params", {}) or {}

        self._bb.set(bb_keys.CURRENT_ACTION, action, overwrite=True)
        self._bb.set(bb_keys.CURRENT_PARAMS, params, overwrite=True)
        materialise_params(self._bb, action, params)
        self._bb.set(bb_keys.PLAN_INDEX, index + 1, overwrite=True)
        self.feedback_message = f"step {index+1}/{len(plan)}: {action}({params})"
        telemetry = get_default_telemetry()
        if telemetry is not None:
            try:
                plan_revision = self._bb.get(bb_keys.PLAN_REVISION) or 1
                step_id = f"plan-r{plan_revision}/step-{index:04d}"
                if index == 0:
                    telemetry.emit(
                        "task.execution_started",
                        {"plan_revision": plan_revision, "plan_length": len(plan)},
                        task_id=self._bb.get(bb_keys.TASK_ID),
                        phase="execution",
                    )
                telemetry.emit(
                    "step.started",
                    {
                        "step_index": index,
                        "step_id": step_id,
                        "plan_revision": plan_revision,
                        "action": action,
                        "params": params,
                        "plan_length": len(plan),
                    },
                    task_id=self._bb.get(bb_keys.TASK_ID),
                    phase="execution",
                )
            except Exception:
                pass
        return Status.SUCCESS


class BtNode_MaterialiseStep(Behaviour):
    """Materialise ``plan_key[step_index]`` into the BB targets a small tree needs.

    Generalized from ``BtNode_PopNextAction`` for the two-layer executor: the
    step index is baked into the node, so there is no index bookkeeping and the
    node can be embedded once per step inside a per-target subtree. Reads the
    step's ``{action, params}`` from the blackboard key named by ``plan_key``
    (a per-target slot like ``gpsr/saved_target_plan_<slot>_<i>``), writes
    CURRENT_ACTION / CURRENT_PARAMS, and runs the shared ``materialise_params``.
    """

    def __init__(self, name: str, plan_key: str, step_index: int):
        super().__init__(name)
        self._plan_key = plan_key
        self._step_index = int(step_index)
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(self._plan_key, access=Access.READ)
        self._bb.register_key(bb_keys.CURRENT_ACTION, access=Access.WRITE)
        self._bb.register_key(bb_keys.CURRENT_PARAMS, access=Access.WRITE)
        self._bb.register_key(bb_keys.PLAN_INDEX, access=Access.WRITE)
        self._bb.register_key(bb_keys.TARGET_POSE, access=Access.WRITE)
        self._bb.register_key(bb_keys.TARGET_LOCATION, access=Access.WRITE)
        self._bb.register_key(bb_keys.TARGET_OBJECT_NAME, access=Access.WRITE)
        self._bb.register_key(bb_keys.TARGET_OBJECT_PROMPT, access=Access.WRITE)
        self._bb.register_key(bb_keys.TARGET_PERSON_PROMPT, access=Access.WRITE)
        self._bb.register_key(bb_keys.ANNOUNCE_TEXT, access=Access.WRITE)
        self._bb.register_key(bb_keys.REPORT_INFO, access=Access.READ)
        self._bb.register_key(bb_keys.ASK_QUESTION, access=Access.WRITE)
        self._bb.register_key(bb_keys.VLM_QUESTION, access=Access.WRITE)
        self._bb.register_key(bb_keys.LLM_QUESTION, access=Access.WRITE)
        self._bb.register_key(bb_keys.START_POSE, access=Access.READ)
        self._bb.register_key(bb_keys.DYNAMIC_LOCATIONS, access=Access.READ)
        self._bb.register_key(bb_keys.CURRENT_DYNLABEL, access=Access.WRITE)
        # written by goto/search_object at materialisation, read back by
        # grasp (shelf inference) -- J4: the intended-destination signal, not
        # the at_robot() gate's success-only LAST_NAV_LOCATION.
        self._bb.register_key(bb_keys.PENDING_NAV_LOCATION, access=Access.WRITE)
        self._bb.register_key(bb_keys.PENDING_NAV_LOCATION, access=Access.READ)
        self._bb.register_key(bb_keys.GRASP_ASK_REFEREE, access=Access.WRITE)
        self._bb.register_key(bb_keys.GRASP_REFEREE_LOCATION, access=Access.WRITE)
        self._bb.register_key(bb_keys.GRASP_REFEREE_POSE, access=Access.WRITE)
        self._bb.register_key(bb_keys.GRASP_REFEREE_IS_APPLIANCE, access=Access.WRITE)
        for search_pose_key in SEARCH_POSE_KEYS:
            self._bb.register_key(search_pose_key, access=Access.WRITE)

    def update(self):
        try:
            plan = self._bb.get(self._plan_key) or []
        except KeyError:
            self.feedback_message = f"Plan key {self._plan_key} not initialised"
            return Status.FAILURE
        if self._step_index >= len(plan):
            self.feedback_message = (
                f"step {self._step_index} out of range (plan has {len(plan)})"
            )
            return Status.FAILURE
        step = plan[self._step_index]
        action = step.get("action")
        params = step.get("params", {}) or {}
        self._bb.set(bb_keys.CURRENT_ACTION, action, overwrite=True)
        self._bb.set(bb_keys.CURRENT_PARAMS, params, overwrite=True)
        self._bb.set(bb_keys.PLAN_INDEX, self._step_index + 1, overwrite=True)
        materialise_params(self._bb, action, params)
        self.feedback_message = f"step {self._step_index}: {action}({params})"
        return Status.SUCCESS


class BtNode_ActionRouter(Behaviour):
    """Guard that succeeds only if ``CURRENT_ACTION`` matches this branch."""

    def __init__(self, expected_action: str):
        super().__init__(f"router:{expected_action}")
        self._expected = expected_action
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.CURRENT_ACTION, access=Access.READ)

    def update(self):
        try:
            current = self._bb.get(bb_keys.CURRENT_ACTION)
        except KeyError:
            return Status.FAILURE
        return Status.SUCCESS if current == self._expected else Status.FAILURE


# ---------------------------------------------------------------------------
# Self-monitor + self-correction
# ---------------------------------------------------------------------------

class BtNode_LogStepResult(Behaviour):
    """Append ``current_action(params) <result>`` to the state log."""

    def __init__(self, name: str, succeeded: bool):
        super().__init__(name)
        self._succeeded = succeeded
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.CURRENT_ACTION, access=Access.READ)
        self._bb.register_key(bb_keys.CURRENT_PARAMS, access=Access.READ)
        self._bb.register_key(bb_keys.STATE_LOG, access=Access.WRITE)
        self._bb.register_key(bb_keys.LAST_FAILURE, access=Access.WRITE)
        self._bb.register_key(bb_keys.TASK_ID, access=Access.READ)
        self._bb.register_key(bb_keys.PLAN_REVISION, access=Access.READ)
        self._bb.register_key(bb_keys.PLAN_INDEX, access=Access.READ)
        self._bb.register_key(bb_keys.SUPERVISOR_STEP_DISPOSITION, access=Access.READ)
        self._bb.register_key(bb_keys.SUPERVISOR_STEP_DISPOSITION, access=Access.WRITE)
        # J4: the step-finished path is where at_robot() gate evidence is
        # actually written -- only once the step SUCCEEDED.
        self._bb.register_key(bb_keys.LAST_NAV_LOCATION, access=Access.WRITE)

    def update(self):
        try:
            action = self._bb.get(bb_keys.CURRENT_ACTION)
            params = self._bb.get(bb_keys.CURRENT_PARAMS)
        except KeyError:
            action, params = "unknown", {}
        if self._succeeded and action != "unknown":
            record_nav_on_success(self._bb, action, params)
        try:
            log = self._bb.get(bb_keys.STATE_LOG)
        except KeyError:
            log = None
        if log is None:
            log = []
        try:
            disposition = self._bb.get(bb_keys.SUPERVISOR_STEP_DISPOSITION)
        except KeyError:
            disposition = None
        if self._succeeded and disposition:
            verdict = "SUPERSEDED"
            self._bb.set(
                bb_keys.SUPERVISOR_STEP_DISPOSITION, None, overwrite=True
            )
        else:
            verdict = "SUCCEEDED" if self._succeeded else "FAILED"
        log.append(f"{action}({params}) {verdict}")
        self._bb.set(bb_keys.STATE_LOG, log, overwrite=True)
        if not self._succeeded:
            self._bb.set(
                bb_keys.LAST_FAILURE,
                f"{action}({params}) failed",
                overwrite=True,
            )
        telemetry = get_default_telemetry()
        if telemetry is not None:
            try:
                plan_revision = self._bb.get(bb_keys.PLAN_REVISION) or 1
                plan_index = int(self._bb.get(bb_keys.PLAN_INDEX) or 1) - 1
                telemetry.emit(
                    "step.finished",
                    {
                        "step_index": plan_index,
                        "step_id": f"plan-r{plan_revision}/step-{max(0, plan_index):04d}",
                        "plan_revision": plan_revision,
                        "action": action,
                        "params": params,
                        "outcome": verdict.lower(),
                        "feedback": getattr(self, "feedback_message", ""),
                    },
                    task_id=self._bb.get(bb_keys.TASK_ID),
                    phase="execution",
                )
            except Exception:
                pass
        return Status.SUCCESS


class BtNode_SupervisorBarrier(Behaviour):
    """Stop a target at the safe boundary after a supervisor global decision.

    The supervised action finishes first, its state is logged, then this node
    turns the typed intervention into a target-level FAILURE.  The dynamic
    executor consumes the request and swaps in the validated remaining plan;
    no running subtree is ever mutated mid-tick.
    """

    def __init__(self, name: str = "supervisor replan barrier"):
        super().__init__(name)
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.REPLAN_REQUEST, access=Access.READ)

    def update(self):
        try:
            request = self._bb.get(bb_keys.REPLAN_REQUEST) or {}
        except KeyError:
            request = {}
        if request.get("level") != "supervisor":
            return Status.SUCCESS
        self.feedback_message = str(
            request.get("reason") or "supervisor requested a global replan"
        )
        return Status.FAILURE


_TARGET_GATE_EVIDENCE_KEYS = (
    ("last_nav_location", bb_keys.LAST_NAV_LOCATION),
    ("object_detection", bb_keys.TARGET_OBJECT_DETECTION),
    ("person_detection", bb_keys.TARGET_PERSON_DETECTION),
    ("waving_persons", bb_keys.ALL_WAVING_PERSONS),
    ("target_person_pose", bb_keys.TARGET_PERSON_POSE),
    ("target_object_name", bb_keys.TARGET_OBJECT_NAME),
    ("target_object_prompt", bb_keys.TARGET_OBJECT_PROMPT),
    ("target_person_prompt", bb_keys.TARGET_PERSON_PROMPT),
    ("count_value", bb_keys.COUNT_VALUE),
    ("count_target", bb_keys.TARGET_OBJECT_NAME),
    ("count_query", bb_keys.TARGET_OBJECT_PROMPT),
    ("qa_answer", bb_keys.QA_ANSWER),
    ("person_answer", bb_keys.PERSON_ANSWER),
    ("llm_answer", bb_keys.LLM_ANSWER),
    ("vlm_answer", bb_keys.VLM_ANSWER),
    ("qa_question", bb_keys.QA_QUESTION),
    ("ask_question", bb_keys.ASK_QUESTION),
    ("vlm_question", bb_keys.VLM_QUESTION),
    ("llm_question", bb_keys.LLM_QUESTION),
)

# I1 (round-3 adversarial review, H1): question/answer evidence is
# target-scoped but was only ever cleared when the executor advanced to a
# DIFFERENT target -- a same-target replan (e.g. a failed ask_person
# followed by a plain announce() of the answer) left the earlier attempt's
# llm_question/ask_question on the blackboard, where the answered() gate's
# provenance check could still see it and poison an otherwise-correct
# replan. These evidence names are cleared on EVERY _swap_in, including a
# same-target continuation; the target's other perception/count evidence
# still follows the existing target-index-change clearing below.
_TARGET_GATE_QA_EVIDENCE_NAMES = frozenset({
    "qa_answer", "person_answer", "llm_answer", "vlm_answer",
    "qa_question", "ask_question", "vlm_question", "llm_question",
})


def _target_gate_evidence(bb) -> Dict[str, Any]:
    evidence: Dict[str, Any] = {}
    for evidence_key, bb_key in _TARGET_GATE_EVIDENCE_KEYS:
        try:
            value = bb.get(bb_key)
        except (KeyError, AttributeError):
            continue
        if value is not None:
            evidence[evidence_key] = value
    if evidence.get("count_target") is None and evidence.get("count_query") is not None:
        evidence["count_target"] = evidence["count_query"]
    if evidence.get("waving_persons") is not None:
        evidence["person_provenance"] = "waving_specialist"
        # The specialist has no named-person identity; expose it through the
        # common person artifact channel without inventing a name.
        evidence.setdefault("person_detection", evidence["waving_persons"])
    return evidence


def _target_gate_facts(bb) -> List[str]:
    try:
        facts = bb.get(bb_keys.FACTS)
    except (KeyError, AttributeError):
        facts = []
    return list(facts or [])


def _emit_gate_verified(
    bb, slot: int, target_index: int, phase: str, fact: str,
    verdict: Verdict, confidence: float, reason: str,
) -> None:
    """I6 (round-3 adversarial review): the gate's VALID/INVALID rationale
    used to exist only in memory (a transient ``feedback_message``) -- emit
    it as a ``gate.verified`` telemetry event per fact, plus one print line,
    so a bench/debugger consumer can see WHY a gate failed after the fact
    (``bench/events.py``'s ``parse_events`` keeps the last INVALID/UNKNOWN
    reason per target from exactly this event -- see I6). Telemetry must
    never break gate evaluation, so this is best-effort/guarded throughout.
    """
    print(f"[gate:{target_index}:{phase}] {fact} {verdict.value} ({reason})")
    telemetry = get_default_telemetry()
    if telemetry is None:
        return
    try:
        task_id = bb.get(bb_keys.TASK_ID)
    except Exception:  # noqa: BLE001
        task_id = None
    try:
        telemetry.emit(
            "gate.verified",
            {
                "slot": slot, "target_index": target_index, "phase": phase,
                "fact": fact, "verdict": verdict.value, "confidence": confidence,
                "reason": reason,
            },
            task_id=task_id, phase="execution",
        )
    except Exception:  # noqa: BLE001
        pass


def _plan_established_predicates(action_plan: Sequence[Mapping[str, Any]]) -> set:
    """Predicates ANY step of ``action_plan`` establishes, per its contract.

    J1(b) (round-3 adversarial review, H4): deferral is not limited to the
    at_robot/self_establishes case -- a precondition whose PREDICATE some
    step of the target's own plan establishes (a `grasp` step establishes
    `held`, `find_object` establishes `object_seen`, ...) is deferred too,
    verified at the postcondition gate once that step has actually run.
    Predicate-level (not exact-fact) on purpose: the plan may not resolve
    the establishing step's params until it runs (e.g. an implicit `object`
    on `place`/`deliver` -- see J10), so matching on the predicate alone is
    what makes the deferral fire for the cases it exists for.
    """
    predicates: set = set()
    for step in action_plan or []:
        if not isinstance(step, Mapping):
            continue
        contract = ACTION_CONTRACTS.get(str(step.get("action")))
        if contract is None:
            continue
        for template in contract.establishes:
            predicates.add(template.split("(", 1)[0])
    return predicates


class BtNode_TargetPreconditionCheck(Behaviour):
    """Verify a target's preconditions at its execution boundary.

    A precondition the target's OWN plan establishes by itself (e.g.
    ``at_robot(kitchen_table)`` when the plan contains ``place(location=
    kitchen_table)``) is DEFERRED: not checked here, written to
    ``DEFERRED_PRECONDITIONS`` and verified by the postcondition gate once the
    establishing step has run. Checking it at entry would fail every
    self-navigating action before it could navigate. The same deferral
    applies, at PREDICATE level, to any precondition some step of the
    target's own plan establishes per its action contract (a `grasp` step
    defers `held`, `find_object` defers `object_seen`) -- see
    ``_plan_established_predicates``.
    """

    def __init__(self, name: str, preconditions: List[str], target_index: int,
                 action_plan: Sequence[Mapping[str, Any]] = (), slot: int = 0):
        super().__init__(name)
        self._preconditions = list(preconditions or [])
        self._target_index = int(target_index)
        self._slot = int(slot)
        self._self_established = {
            fact for step in (action_plan or []) for fact in _self_established_facts(step)
        }
        self._plan_established_predicates = _plan_established_predicates(action_plan)
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.FACTS, access=Access.READ)
        self._bb.register_key(bb_keys.DEFERRED_PRECONDITIONS, access=Access.WRITE)
        self._bb.register_key(bb_keys.TASK_ID, access=Access.READ)
        for _, key in _TARGET_GATE_EVIDENCE_KEYS:
            self._bb.register_key(key, access=Access.READ)

    def update(self):
        deferred: List[str] = []
        checked: List[str] = []
        for source in self._preconditions:
            fact, _err = parse_fact(source)
            canonical = canonical_fact(fact) if fact is not None else None
            if canonical is not None and (
                canonical in self._self_established
                or fact.predicate in self._plan_established_predicates
            ):
                deferred.append(canonical)
            else:
                checked.append(source)
        self._bb.set(bb_keys.DEFERRED_PRECONDITIONS, deferred, overwrite=True)
        if deferred:
            self.feedback_message = "precondition deferred: " + ", ".join(deferred)
        if not checked:
            return Status.SUCCESS
        context = VerificationContext(
            phase="precondition",
            established_facts=frozenset(_target_gate_facts(self._bb)),
            target_object="",
            target_location="",
        )
        evidence = _target_gate_evidence(self._bb)
        for source in checked:
            try:
                results, _ = check_all([source], evidence, context)
                result = results[0]
            except Exception:
                result = None
            verdict = Verdict.INVALID if result is None else result.verdict
            reason = "verifier exception" if result is None else result.evidence
            confidence = 0.0 if result is None else result.confidence
            # I6: telemetry per checked fact -- fail-fast semantics unchanged,
            # so a fact after the first failure is never reached/emitted.
            _emit_gate_verified(
                self._bb, self._slot, self._target_index, "precondition",
                source, verdict, confidence, reason,
            )
            if verdict is not Verdict.VALID:
                self.feedback_message = f"precondition unmet: {source} ({verdict.value})"
                return Status.FAILURE
        return Status.SUCCESS


def _steps_establishing(
    action_plan: Sequence[Mapping[str, Any]], canonical_facts: Sequence[str],
) -> List[Dict[str, Any]]:
    """The steps of ``action_plan`` whose contract established any of ``canonical_facts``.

    J3 (round-3 adversarial review, M8): when the postcondition gate commits
    a subset of a target's declared facts before failing, this tells the
    NEXT replan which of the target's own already-run steps produced one of
    the just-committed facts -- so ``plan_target`` can feed it back as
    ``completed_steps`` and the model is told not to repeat it.
    """
    wanted = set(canonical_facts)
    if not wanted:
        return []
    matched: List[Dict[str, Any]] = []
    for step in action_plan or []:
        if not isinstance(step, Mapping):
            continue
        produced = set(_step_established_facts(step)) | set(_self_established_facts(step))
        if produced & wanted:
            matched.append(dict(step))
    return matched


class BtNode_TargetPostconditionCheck(Behaviour):
    """Verify and publish target facts under the v1 fact-store contract.

    When ``facts_writer`` is provided it is the authoritative atomic/idempotent
    planner fact-store commit and receives only newly validated canonical facts.
    The Blackboard FACTS value is then a repairable mirror. Without a writer,
    the Blackboard is authoritative and a mirror write failure is fatal.
    """

    def __init__(
        self,
        name: str,
        postconditions: List[str],
        target_index: int,
        action_plan: List[Dict[str, Any]],
        target_object: str = "",
        completed_steps: Optional[List[Dict[str, Any]]] = None,
        target_location: str = "",
        facts_writer: Optional[Callable[[List[str]], None]] = None,
        slot: int = 0,
    ):
        super().__init__(name)
        self._postconditions = list(postconditions or [])
        self._target_index = int(target_index)
        self._slot = int(slot)
        self._action_plan = copy.deepcopy(list(completed_steps or []) + list(action_plan or []))
        self._target_object = target_object
        self._target_location = target_location
        self._facts_writer = facts_writer
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.FACTS, access=Access.READ)
        self._bb.register_key(bb_keys.FACTS, access=Access.WRITE)
        self._bb.register_key(bb_keys.DEFERRED_PRECONDITIONS, access=Access.READ)
        self._bb.register_key(bb_keys.DEFERRED_PRECONDITIONS, access=Access.WRITE)
        self._bb.register_key(bb_keys.GATE_COMPLETED_STEPS, access=Access.WRITE)
        self._bb.register_key(bb_keys.TASK_ID, access=Access.READ)
        for _, key in _TARGET_GATE_EVIDENCE_KEYS:
            self._bb.register_key(key, access=Access.READ)

    def update(self):
        try:
            deferred = list(self._bb.get(bb_keys.DEFERRED_PRECONDITIONS) or [])
        except KeyError:
            deferred = []
        sources = list(self._postconditions) + [d for d in deferred if d not in self._postconditions]
        if not sources:
            return Status.SUCCESS
        own_postconditions = set()
        for source in self._postconditions:
            fact, _err = parse_fact(source)
            if fact is not None:
                own_postconditions.add(canonical_fact(fact))
        context = VerificationContext(
            phase="postcondition",
            established_facts=frozenset(_target_gate_facts(self._bb)),
            completed_steps=tuple(self._action_plan),
            target_object=self._target_object,
            target_location=self._target_location,
            own_postconditions=frozenset(own_postconditions),
        )
        evidence = _target_gate_evidence(self._bb)
        # J3 (round-3 adversarial review, M8): verify EVERY fact -- no more
        # fail-fast on the first unmet one -- so the VALID ones can still be
        # committed even when the target overall fails. Telemetry is
        # therefore emitted for every source now, not just up to the first
        # failure (this intentionally widens the I6 comment's old "fail-fast
        # semantics unchanged" note).
        valid_facts: List[Fact] = []
        unmet: List[str] = []
        for source in sources:
            try:
                results, facts = check_all([source], evidence, context)
                result = results[0]
            except Exception:
                result, facts = None, []
            if result is None:
                verdict = Verdict.INVALID
            else:
                verdict = result.verdict
            reason = "verifier exception" if result is None else result.evidence
            confidence = 0.0 if result is None else result.confidence
            _emit_gate_verified(
                self._bb, self._slot, self._target_index, "postcondition",
                source, verdict, confidence, reason,
            )
            if verdict is Verdict.VALID:
                valid_facts.extend(facts)
            else:
                unmet.append(f"{source} ({verdict.value})")

        canonical = []
        seen = set()
        for fact in valid_facts:
            value = canonical_fact(fact)
            if value not in seen:
                seen.add(value)
                canonical.append(value)

        if canonical:
            try:
                if self._facts_writer is not None:
                    self._facts_writer(canonical)
            except Exception as exc:
                self.feedback_message = f"postcondition fact write failed: {exc}"
                return Status.FAILURE

            current = _target_gate_facts(self._bb)
            merged = apply_fact_transitions(current, canonical)
            try:
                self._bb.set(bb_keys.FACTS, merged, overwrite=True)
            except Exception as exc:
                if self._facts_writer is not None:
                    self.feedback_message = f"postcondition fact mirror write failed: {exc}"
                else:
                    self.feedback_message = f"postcondition fact write failed: {exc}"
                    return Status.FAILURE

        if unmet:
            committed_steps = _steps_establishing(self._action_plan, canonical)
            self._bb.set(bb_keys.GATE_COMPLETED_STEPS, committed_steps, overwrite=True)
            print(
                f"gate:{self._target_index}:post committed {canonical} unmet {unmet}"
            )
            self.feedback_message = "postcondition unmet: " + ", ".join(unmet)
            return Status.FAILURE

        self._bb.set(bb_keys.DEFERRED_PRECONDITIONS, [], overwrite=True)
        self._bb.set(bb_keys.GATE_COMPLETED_STEPS, [], overwrite=True)
        return Status.SUCCESS


class BtNode_BumpCorrectionCounter(Behaviour):
    """Increment correction counter; FAIL if it exceeds ``max_corrections``."""

    def __init__(self, name: str, max_corrections: int):
        super().__init__(name)
        self._max = max_corrections
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.CORRECTION_COUNT, access=Access.WRITE)
        self._bb.register_key(bb_keys.TASK_ID, access=Access.READ)

    def update(self):
        try:
            count = self._bb.get(bb_keys.CORRECTION_COUNT)
        except KeyError:
            count = 0
        count = (count or 0) + 1
        self._bb.set(bb_keys.CORRECTION_COUNT, count, overwrite=True)
        if count > self._max:
            self.feedback_message = f"correction limit reached ({count} > {self._max})"
            return Status.FAILURE
        self.feedback_message = f"correction #{count}"
        telemetry = get_default_telemetry()
        if telemetry is not None:
            try:
                telemetry.emit(
                    "correction.started",
                    {"correction_number": count, "max_corrections": self._max},
                    task_id=self._bb.get(bb_keys.TASK_ID),
                    phase="correction",
                )
            except Exception:
                pass
        return Status.SUCCESS


class BtNode_FinalizeTask(Behaviour):
    """Write an explicit terminal outcome and emit it for the debugger."""

    def __init__(self, name: str = "finalize GPSR task", max_corrections: int = 3):
        super().__init__(name)
        self._max_corrections = max_corrections
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        for key, access in (
            (bb_keys.TASK_ID, Access.READ),
            (bb_keys.PLAN, Access.READ),
            (bb_keys.PLAN_INDEX, Access.READ),
            (bb_keys.CORRECTION_COUNT, Access.READ),
            (bb_keys.LAST_FAILURE, Access.READ),
            (bb_keys.TASK_OUTCOME, Access.WRITE),
        ):
            self._bb.register_key(key, access=access)

    def update(self):
        try:
            existing_outcome = Blackboard.get(bb_keys.TASK_OUTCOME)
        except KeyError:
            existing_outcome = None
        if (
            isinstance(existing_outcome, dict)
            and existing_outcome.get("source") == "llm_supervisor"
        ):
            self.feedback_message = (
                f"{existing_outcome.get('status')}: "
                f"{existing_outcome.get('reason')}"
            )
            telemetry = get_default_telemetry()
            if telemetry is not None:
                try:
                    telemetry.emit(
                        "task.finished",
                        existing_outcome,
                        task_id=self._bb.get(bb_keys.TASK_ID),
                        phase="terminal",
                    )
                except Exception:
                    pass
            return Status.SUCCESS
        try:
            plan = self._bb.get(bb_keys.PLAN) or []
            index = int(self._bb.get(bb_keys.PLAN_INDEX) or 0)
            corrections = int(self._bb.get(bb_keys.CORRECTION_COUNT) or 0)
            failure = self._bb.get(bb_keys.LAST_FAILURE) or ""
        except (KeyError, TypeError, ValueError):
            plan, index, corrections, failure = [], 0, 0, ""
        if corrections > self._max_corrections:
            status, reason = "failed", "correction_limit"
        elif index >= len(plan):
            status, reason = "succeeded", "plan_exhausted"
        elif failure:
            status, reason = "failed", failure
        else:
            status, reason = "incomplete", "execution stopped before plan exhaustion"
        outcome = {"status": status, "reason": reason, "plan_index": index, "plan_length": len(plan), "corrections": corrections}
        self._bb.set(bb_keys.TASK_OUTCOME, outcome, overwrite=True)
        self.feedback_message = f"{status}: {reason}"
        telemetry = get_default_telemetry()
        if telemetry is not None:
            try:
                telemetry.emit("task.finished", outcome, task_id=self._bb.get(bb_keys.TASK_ID), phase="terminal")
            except Exception:
                pass
        return Status.SUCCESS


# ---------------------------------------------------------------------------
# Two-layer bridge nodes: TOP layer (split_command) and LOWER layer
# (request_plan_all). Both delegate the heavy lifting to a GPSRPlanner
# (injected at composition time; never imported here to avoid a cycle), spawn
# planner threads, and RUN until the planner reports ready. The planner threads
# never touch the Blackboard — every BB write below happens on the executor
# thread in these nodes' update(), on the same thread that ticks the tree.
# ---------------------------------------------------------------------------

class BtNode_SplitCommand(Behaviour):
    """TOP LAYER bridge: NL command -> ordered list of self-contained targets.

    Reads ``COMMAND``, spawns a daemon thread running ``planner.split_command``
    (blocking LLM split; deterministic under full-mock), RUNNING until it
    returns, then writes ``TARGETS`` / ``NUM_TARGETS`` and clears
    ``REPLAN_REQUEST``. ``rephrase_on_failure`` + failure seeding are the future
    hook for command-level replans (trigger not wired yet).
    """

    def __init__(self, name: str, planner, rephrase_on_failure: bool = False, slot: int = 0):
        super().__init__(name)
        self._planner = planner
        self._rephrase_on_failure = rephrase_on_failure
        self._slot = int(slot)
        self._bb = None
        self._thread = None
        self._targets = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.COMMAND, access=Access.READ)
        self._bb.register_key(bb_keys.TARGETS, access=Access.WRITE)
        self._bb.register_key(bb_keys.NUM_TARGETS, access=Access.WRITE)
        self._bb.register_key(bb_keys.REPLAN_REQUEST, access=Access.WRITE)
        self._bb.register_key(bb_keys.REPLAN_REQUEST, access=Access.READ)

    def initialise(self):
        self._targets = None
        try:
            command = self._bb.get(bb_keys.COMMAND)
        except KeyError:
            command = ""
        self._thread = threading.Thread(
            target=self._split_worker, args=(command,), daemon=True,
        )
        self._thread.start()
        self.feedback_message = "splitting command into targets..."

    def _split_worker(self, command: str):
        # I4 (round-3 adversarial review, M7): an unhandled exception in
        # `split_command` used to leave `self._targets` at its `None`
        # sentinel forever -- `update()` above spins RUNNING indefinitely
        # ("splitting command into targets...") with no telemetry
        # distinguishing that from a slow, still-in-flight split. Never let
        # this daemon thread die silently: fall back to the same
        # deterministic split `split_command` itself uses when every LLM
        # attempt fails.
        try:
            self._targets = self._planner.split_command(command, slot=self._slot)
        except Exception as exc:  # noqa: BLE001 -- I4: never leave the split not-ready
            print(f"[split] split_command crashed: {exc!r} -> deterministic fallback split")
            from .planner import _offline_mock_targets  # lazy: planner.py imports this module
            self._targets = _offline_mock_targets(command)

    def update(self):
        if self._targets is None:
            return Status.RUNNING
        self._bb.set(bb_keys.TARGETS, self._targets, overwrite=True)
        self._bb.set(bb_keys.NUM_TARGETS, len(self._targets), overwrite=True)
        self._bb.set(bb_keys.REPLAN_REQUEST, {}, overwrite=True)
        descs = [_target_desc(t) for t in self._targets]
        self.feedback_message = (
            f"split into {len(self._targets)} target(s): {descs}"
        )
        return Status.SUCCESS

    def terminate(self, new_status):
        self._thread = None


class BtNode_PlanAllTargets(Behaviour):
    """LOWER LAYER parallel bridge: plan EVERY target of the current command.

    Reads ``TARGETS``, calls ``planner.request_plan_all(slot, targets)`` (one
    daemon thread per target — parallel planning), RUNNING until every target
    is ready, then:
      - copies each per-target action plan to ``SAVED_TARGET_PLAN_PREFIX+<slot>_<i>``
        (the slots ``BtNode_MaterialiseStep`` reads at execution time),
      - copies ``TARGETS`` to ``SAVED_TARGETS_PREFIX+<slot>``,
      - writes the concatenated aggregate to ``gpsr/plan`` + resets
        ``gpsr/plan_index=0`` so command_logger / codegen / plan_judge / the
        plan-reading tests keep working on the flattened form.
    """

    def __init__(self, name: str, slot: int, planner):
        super().__init__(name)
        self._slot = int(slot)
        self._planner = planner
        self._bb = None
        self._targets = []

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.TARGETS, access=Access.READ)
        self._bb.register_key(bb_keys.COMMAND, access=Access.READ)
        self._bb.register_key(bb_keys.TARGETS, access=Access.WRITE)
        self._bb.register_key(bb_keys.NUM_TARGETS, access=Access.WRITE)
        self._bb.register_key(bb_keys.SAVED_TARGETS_PREFIX + str(self._slot),
                              access=Access.WRITE)
        self._bb.register_key(bb_keys.PLAN, access=Access.WRITE)
        self._bb.register_key(bb_keys.PLAN_INDEX, access=Access.WRITE)
        self._bb.register_key(bb_keys.REPLAN_REQUEST, access=Access.WRITE)

    def initialise(self):
        try:
            targets = self._bb.get(bb_keys.TARGETS) or []
        except KeyError:
            targets = []
        try:
            command = self._bb.get(bb_keys.COMMAND) or ""
        except KeyError:
            command = ""
        self._targets = list(targets)
        for i in range(len(self._targets)):
            self._bb.register_key(
                bb_keys.SAVED_TARGET_PLAN_PREFIX + f"{self._slot}_{i}",
                access=Access.WRITE,
            )
        if not self._targets:
            self.feedback_message = "no targets to plan (COMMAND was empty)"
            return
        self._planner.request_plan_all(self._slot, self._targets, command=command)
        self.feedback_message = f"planning {len(self._targets)} target(s) in parallel..."

    def update(self):
        if not self._targets:
            return Status.FAILURE
        if not self._planner.all_targets_ready(self._slot, len(self._targets)):
            return Status.RUNNING
        # Copy the per-target plans into their execution slots.
        aggregate = []
        for i, t in enumerate(self._targets):
            plan = self._planner.get_action_plan(self._slot, i)
            aggregate.extend(plan)
            self._bb.set(
                bb_keys.SAVED_TARGET_PLAN_PREFIX + f"{self._slot}_{i}",
                plan, overwrite=True,
            )
        # Store the target DESC list (not the dicts) for speech/execution readers.
        self._bb.set(bb_keys.SAVED_TARGETS_PREFIX + str(self._slot),
                     [_target_desc(t) for t in self._targets], overwrite=True)
        # Flattened aggregate for the legacy plan readers.
        self._bb.set(bb_keys.PLAN, aggregate, overwrite=True)
        self._bb.set(bb_keys.PLAN_INDEX, 0, overwrite=True)
        self._bb.set(bb_keys.NUM_TARGETS, len(self._targets), overwrite=True)
        self._bb.set(bb_keys.REPLAN_REQUEST, {}, overwrite=True)
        self.feedback_message = (
            f"planned {len(self._targets)} target(s) -> {len(aggregate)} step(s)"
        )
        return Status.SUCCESS


# ---------------------------------------------------------------------------
# Tree composition
# ---------------------------------------------------------------------------

class DynamicExecutor(py_trees.composites.Composite):
    """Runtime-changeable executor: owns ONE child = the active target subtree.

    This composite implements the executing half of the two-layer orchestrator.
    It holds exactly one child at a time — the subtree ``planner`` built for the
    current target — and *swaps in* the next target's subtree the moment it is
    ready. The swap is a real runtime mutation of the running tree
    (``tree.replace_subtree``), so a newly planned subtree starts executing
    immediately on the same tick cycle.

    State machine::

        REQUESTING --(target i ready)--> EXECUTING --(child SUCCESS + more targets)--> REQUESTING (i+1)
                     EXECUTING --(child FAILURE)--> replan_target(i) + REQUESTING (re-run i, budget)
                     EXECUTING --(last target SUCCESS)--> DONE (SUCCESS)

    Swap-safety (the whole point of the custom tick):
    - Swaps happen ONLY at the two safe tick-boundary points below — the top of
      the tick (REQUESTING, when a fresh subtree is ready) or right after the
      current child returned a terminal status. They NEVER happen from a
      sibling's ``update()``, and NEVER while the child is RUNNING.
    - A runtime-created subtree MUST be ``py_trees.trees.setup``'d before
      insertion (it needs a ROS node handle + BB clients). Done in ``_swap_in``.
    """

    def __init__(self, name: str, slot: int, planner, max_replans_per_target: int = 3):
        super().__init__(name)
        self._slot = int(slot)
        self._planner = planner
        self._max_replans = max(1, int(max_replans_per_target))
        self._tree = None
        self._node = None
        self._bb = None
        self._state = "REQUESTING"
        self._index = 0
        self._active_target_index: Optional[int] = None
        self._swap_count = 0
        self._num_targets = 0
        self._target_outcomes: Dict[str, str] = {}
        self._targets: List[Any] = []
        self._supervisor_aborted = False

    # -- lifecycle ----------------------------------------------------------

    def setup(self, **kwargs):
        # The running tree + ROS node are distributed via tree.setup(..., gpsr_tree=tree).
        # py_trees_ros forwards extra kwargs to every behaviour's setup(), so the
        # orchestrator entry point passes gpsr_tree=tree and it lands here.
        self._tree = kwargs.get("gpsr_tree")
        self._node = kwargs.get("node")
        if self._tree is None:
            raise RuntimeError(
                "DynamicExecutor requires the running tree: tree.setup(..., gpsr_tree=tree)"
            )
        if self._node is None:
            raise RuntimeError("DynamicExecutor requires a ROS node in setup kwargs")
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.TARGET_INDEX, access=Access.WRITE)
        self._bb.register_key(bb_keys.CURRENT_TARGET, access=Access.WRITE)
        self._bb.register_key(bb_keys.REPLAN_REQUEST, access=Access.WRITE)
        self._bb.register_key(bb_keys.SUPERVISOR_STEP_DISPOSITION, access=Access.WRITE)
        self._bb.register_key(bb_keys.TARGET_REPLAN_COUNT, access=Access.WRITE)
        self._bb.register_key(bb_keys.DEFERRED_PRECONDITIONS, access=Access.WRITE)
        self._bb.register_key(bb_keys.GATE_COMPLETED_STEPS, access=Access.READ)
        self._bb.register_key(bb_keys.GATE_COMPLETED_STEPS, access=Access.WRITE)
        self._bb.register_key(bb_keys.STATE_LOG, access=Access.WRITE)
        self._bb.register_key(bb_keys.FACTS, access=Access.READ)
        self._bb.register_key(bb_keys.FACTS, access=Access.WRITE)
        # Telemetry: TASK_ID is set (idempotently) on every ``_swap_in`` of
        # this executor's slot, not just once — see the stamp in
        # ``_swap_in`` (read+write access is registered here); the
        # current-step keys are read-only for the failed-step event.
        self._bb.register_key(bb_keys.TASK_ID, access=Access.READ)
        self._bb.register_key(bb_keys.TASK_ID, access=Access.WRITE)
        self._bb.register_key(bb_keys.CURRENT_ACTION, access=Access.READ)
        self._bb.register_key(bb_keys.CURRENT_PARAMS, access=Access.READ)
        self._bb.register_key(bb_keys.PLAN_INDEX, access=Access.READ)
        for _, key in _TARGET_GATE_EVIDENCE_KEYS:
            self._bb.register_key(key, access=Access.WRITE)
        self._bb.register_key(bb_keys.SAVED_TARGETS_PREFIX + str(self._slot),
                              access=Access.READ)
        self._bb.register_key(bb_keys.TARGETS, access=Access.READ)

    def _log(self, text: str) -> None:
        try:
            log = self._bb.get(bb_keys.STATE_LOG) or []
        except KeyError:
            log = []
        log.append(text)
        self._bb.set(bb_keys.STATE_LOG, log, overwrite=True)

    # -- swap machinery (safe points only) ----------------------------------

    def _swap_in(self, index: int) -> py_trees.composites.Sequence:
        """Setup + swap the ready target subtree into the live tree. Safe points only."""
        new_subtree = self._planner.get_target_subtree(self._slot, index)
        if new_subtree is None:
            return None
        self._bb.set(bb_keys.REPLAN_REQUEST, {}, overwrite=True)
        self._bb.set(bb_keys.SUPERVISOR_STEP_DISPOSITION, None, overwrite=True)
        # A swap within the active target is a continuation/replan: retain all
        # evidence.  Advancing to a new target invalidates target-scoped
        # perception/answer artifacts, but navigation remains persistent safety
        # state for subsequent grasp materialisation.
        if self._active_target_index != index:
            for evidence_name, key in _TARGET_GATE_EVIDENCE_KEYS:
                if evidence_name != "last_nav_location":
                    self._bb.set(key, None, overwrite=True)
            self._bb.set(bb_keys.DEFERRED_PRECONDITIONS, [], overwrite=True)
            self._bb.set(bb_keys.GATE_COMPLETED_STEPS, [], overwrite=True)
            self._swap_count = 0
        else:
            # I1: a same-target replan/continuation retains perception/count
            # evidence (comment above), but question/answer provenance is
            # per-attempt -- clear it so a stale question from an earlier
            # step in this same target cannot poison this replan's gate.
            for evidence_name, key in _TARGET_GATE_EVIDENCE_KEYS:
                if evidence_name in _TARGET_GATE_QA_EVIDENCE_NAMES:
                    self._bb.set(key, None, overwrite=True)
        self._active_target_index = index
        # Every materialization of the same target (LLM replan OR supervisor
        # replacement, which does not bump TARGET_REPLAN_COUNT) gets a strictly
        # increasing revision.
        self._swap_count += 1
        telemetry = get_default_telemetry()
        if telemetry is not None:
            # The blackboard is process-global and shared by every slot's
            # executor, so stamp this executor's identity on every swap
            # (idempotent per executor) rather than only when unset — otherwise
            # slots 2..N inherit slot 1's task id. Same helper as the legacy flow.
            task_id = _task_identity(self._slot)
            self._bb.set(bb_keys.TASK_ID, task_id, overwrite=True)
            revision = self._swap_count
            get_action_plan = getattr(self._planner, "get_action_plan", None)
            # Telemetry must never break execution: plan shaping + emit guarded.
            try:
                steps = list(get_action_plan(self._slot, index)) if callable(get_action_plan) else []
                telemetry.emit(
                    "plan.materialized",
                    {"slot": self._slot, "target_index": index, "revision": revision,
                     "steps": [{"action": s.get("action"), "params": s.get("params") or {}} for s in steps]},
                    task_id=task_id, phase="planning",
                )
            except Exception:
                pass
        get_facts = getattr(self._planner, "get_facts", None)
        if callable(get_facts):
            self._bb.set(
                bb_keys.FACTS,
                list(get_facts(self._slot) or []),
                overwrite=True,
            )
        # Keep the blackboard plan slot in sync with both LLM replans and
        # supervisor-supplied replacement plans before materialisation begins.
        get_action_plan = getattr(self._planner, "get_action_plan", None)
        if callable(get_action_plan):
            plan_key = bb_keys.SAVED_TARGET_PLAN_PREFIX + f"{self._slot}_{index}"
            self._bb.register_key(plan_key, access=Access.WRITE)
            self._bb.set(
                plan_key,
                get_action_plan(self._slot, index),
                overwrite=True,
            )
        # A runtime-created subtree must be setup before insertion; this wires
        # its BB clients + ROS handles. Node is forwarded so the small trees can
        # (re)discover action servers — default timeout, signal-based timeout is
        # unsafe on a thread.
        py_trees.trees.setup(root=new_subtree, node=self._node)
        if self.children:
            old_id = self.children[0].id
            # replace_subtree looks up old_id under the tree root and swaps the
            # child in place — fires the tree_update_handler for viz republish.
            self._tree.replace_subtree(old_id, new_subtree)
        else:
            self.add_child(new_subtree)
        self.current_child = new_subtree
        return new_subtree

    def _announce(self, text: str) -> None:
        from behavior_tree.TemplateNodes.Audio import BtNode_Announce
        announce = BtNode_Announce(name=self.name + "/announce", bb_source=None,
                                   message=text)
        py_trees.trees.setup(root=announce, node=self._node)
        announce.initialise()
        announce.update()

    # -- per-target event handlers ------------------------------------------

    def _on_target_success(self) -> None:
        self._target_outcomes[self._target_id(self._index)] = "SUCCEEDED"
        self._log(f"target:{self._index}:{self._current_desc()} SUCCEEDED")
        self._bb.set(bb_keys.TARGET_INDEX, self._index + 1, overwrite=True)
        self._bb.set(bb_keys.TARGET_REPLAN_COUNT, 0, overwrite=True)
        if self._index + 1 >= self._num_targets:
            self._state = "DONE"
        else:
            self._state = "REQUESTING"
            self._index += 1

    def _on_target_failure(self, reason: str, target_failed_emitted: bool = False) -> None:
        """``target_failed_emitted`` (I6): whether the caller's
        ``_emit_failed_step`` already emitted a ``target.failed`` telemetry
        event for THIS SAME failure — the SKIPPED/UNRECOVERABLE branch below
        only emits its own ``target.failed`` when that did NOT already
        happen (never double-emit for a gate failure the caller already
        reported).
        """
        self._log(f"target:{self._index}:{self._current_desc()} FAILED: {reason}")
        try:
            request = self._bb.get(bb_keys.REPLAN_REQUEST) or {}
        except KeyError:
            request = {}
        if request.get("level") == "supervisor":
            self._bb.set(bb_keys.REPLAN_REQUEST, {}, overwrite=True)
            # L1 (round-2 review): a forced-to-exhausted TARGET_REPLAN_COUNT
            # (the UNRECOVERABLE_SKIPPED branch in ``tick()`` forces it to
            # ``_max_replans`` right before calling this method) must not
            # survive into the supervisor's replacement plan -- otherwise
            # that replacement's own first genuine failure would budget-skip
            # immediately, with zero replan attempts of its own. The
            # supervisor is handing this target a fresh start either way.
            self._bb.set(bb_keys.TARGET_REPLAN_COUNT, 0, overwrite=True)
            action = request.get("action")
            if action == "abort_and_report":
                message = str(request.get("operator_message") or reason)
                if message:
                    self._announce(message)
                self._bb.set(
                    bb_keys.TARGET_INDEX, self._num_targets, overwrite=True,
                )
                self._supervisor_aborted = True
                self._state = "DONE"
                return
            replacement = request.get("replacement_plan") or []
            preserved = request.get("preserved_completed_steps")
            if isinstance(preserved, list):
                completed_steps = copy.deepcopy(preserved)
            elif isinstance(preserved, int) and not isinstance(preserved, bool) and preserved >= 0:
                get_action_plan = getattr(self._planner, "get_action_plan", None)
                original = get_action_plan(self._slot, self._index) if callable(get_action_plan) else []
                completed_steps = copy.deepcopy(original[:preserved]) if preserved <= len(original) else []
            else:
                completed_steps = []
            self._planner.replace_target_plan(
                self._slot, self._index, replacement, reason,
                completed_steps=completed_steps,
            )
            self._log(
                f"target:{self._index}:{self._current_desc()} SUPERVISOR_REPLAN "
                f"({len(replacement)} remaining step(s))"
            )
            self._state = "REQUESTING"
            return
        try:
            replans = int(self._bb.get(bb_keys.TARGET_REPLAN_COUNT) or 0)
        except KeyError:
            replans = 0
        replans += 1
        self._bb.set(bb_keys.TARGET_REPLAN_COUNT, replans, overwrite=True)
        if replans > self._max_replans:
            self._announce(f"I could not complete {self._current_desc()} after "
                           f"{self._max_replans} attempts. I will skip it.")
            self._target_outcomes[self._target_id(self._index)] = "SKIPPED"
            # L1 (round-2 review): brief says "the normal budget exceeded
            # reason, with the unrecoverable text appended" -- an
            # UNRECOVERABLE_SKIPPED tick forces the budget to exhausted on
            # THIS call (see tick()'s UNRECOVERABLE_SKIPPED branch), so this
            # is the only way `reason` itself can already carry the
            # unrecoverable marker text; append it so a log/bench grep on
            # the SKIPPED line alone (not just the preceding FAILED line)
            # can tell an unrecoverable skip from a plain budget skip.
            skip_suffix = (
                f" ({reason})"
                if isinstance(reason, str) and reason.startswith(UNRECOVERABLE_ERROR_PREFIX)
                else ""
            )
            self._log(f"target:{self._index}:{self._current_desc()} SKIPPED "
                      f"(replan budget exceeded){skip_suffix}")
            # I6 (round-3 adversarial review): the SKIPPED (budget) and
            # UNRECOVERABLE outcomes previously had no telemetry event of
            # their own -- only the STATE_LOG text line above. Emit
            # target.failed here too, UNLESS the caller's _emit_failed_step
            # already reported target.failed for this same underlying
            # failure (a gate failure) -- never double-emit.
            if not target_failed_emitted:
                telemetry = get_default_telemetry()
                if telemetry is not None:
                    try:
                        task_id = self._bb.get(bb_keys.TASK_ID)
                    except KeyError:
                        task_id = None
                    try:
                        telemetry.emit(
                            "target.failed",
                            {"slot": self._slot, "target_index": self._index, "reason": reason},
                            task_id=task_id, phase="execution",
                        )
                    except Exception:  # noqa: BLE001
                        pass
            self._bb.set(bb_keys.TARGET_INDEX, self._index + 1, overwrite=True)
            self._bb.set(bb_keys.TARGET_REPLAN_COUNT, 0, overwrite=True)
            if self._index + 1 >= self._num_targets:
                self._state = "DONE"
            else:
                self._state = "REQUESTING"
                self._index += 1
            return
        # Lower-layer replan ONLY: re-plan this target (planner threads rebuild
        # + cache the fresh subtree; they never touch the BB). The top layer
        # and the other targets are untouched. Command-level replan becomes a
        # stub writing REPLAN_REQUEST={"level":"command",...} when the trigger
        # mechanism is announced.
        # J3: hand the replan whatever the postcondition gate just committed
        # (GATE_COMPLETED_STEPS) so it does not redo work that already
        # succeeded (e.g. a grasp) just because a LATER postcondition of the
        # same target failed.
        try:
            gate_completed = list(self._bb.get(bb_keys.GATE_COMPLETED_STEPS) or [])
        except KeyError:
            gate_completed = []
        self._planner.replan_target(self._slot, self._index, reason, completed_steps=gate_completed)
        self._bb.set(bb_keys.REPLAN_REQUEST,
                     {"level": "target", "index": self._index, "reason": reason},
                     overwrite=True)
        self._state = "REQUESTING"

    def _current_desc(self) -> str:
        try:
            cur = self._bb.get(bb_keys.CURRENT_TARGET) or self._index
        except KeyError:
            cur = self._index
        if isinstance(cur, dict):
            return str(cur.get("desc") or "")
        return str(cur)

    def _target_context(self, index: int) -> Dict[str, Any]:
        target = self._targets[index] if index < len(self._targets) else {}
        if isinstance(target, dict):
            return target
        return {"id": f"t{index}", "desc": str(target), "depends_on": []}

    def _target_id(self, index: int) -> str:
        return str(self._target_context(index).get("id") or f"t{index}")

    def _blocked_by(self, index: int) -> List[Tuple[str, str]]:
        blocked = []
        for dependency in self._target_context(index).get("depends_on", []) or []:
            dep_id = str(dependency)
            outcome = self._target_outcomes.get(dep_id)
            if outcome != "SUCCEEDED":
                blocked.append((dep_id, outcome or "NOT_RUN"))
        return blocked

    def _mark_blocked(self, index: int, blocked_by: List[Tuple[str, str]]) -> None:
        target_id = self._target_id(index)
        self._target_outcomes[target_id] = "BLOCKED"
        reason = ", ".join(f"{dep}={outcome}" for dep, outcome in blocked_by)
        self._log(f"target:{index}:{self._current_desc()} BLOCKED (depends_on: {reason})")
        self._bb.set(bb_keys.TARGET_INDEX, index + 1, overwrite=True)
        if index + 1 >= self._num_targets:
            self._state = "DONE"
        else:
            self._state = "REQUESTING"
            self._index = index + 1

    # -- custom tick ---------------------------------------------------------

    def tick(self) -> Iterator[py_trees.behaviour.Behaviour]:
        """Generator tick: yield the active child each cycle, driving swaps.

        Two safe swap points, both at tick boundaries:
          (A) top of tick in REQUESTING, when the target's subtree is ready;
          (B) right after the current child returned a terminal status.
        Never swap from a sibling's update(); never swap while RUNNING.

        The executor stays RUNNING across target advances (state ``REQUESTING``
        with the next index), so the tree never re-initialises mid-command; it
        only reaches SUCCESS once the last target is done.
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)

        if self.status != py_trees.common.Status.RUNNING:
            # Fresh activation (INVALID/terminal -> RUNNING): (re)init state.
            self.initialise()
            try:
                targets = self._bb.get(bb_keys.SAVED_TARGETS_PREFIX + str(self._slot)) \
                    or self._bb.get(bb_keys.TARGETS) or []
            except KeyError:
                targets = []
            get_targets = getattr(self._planner, "get_targets", None)
            if callable(get_targets):
                self._targets = list(get_targets(self._slot) or [])
            else:
                self._targets = list(targets)
            self._num_targets = len(self._targets)
            self._target_outcomes = {}
            self._supervisor_aborted = False
            self._active_target_index = None
            self._swap_count = 0
            if not callable(getattr(self._planner, "get_facts", None)):
                self._bb.set(bb_keys.FACTS, [], overwrite=True)
            if self._num_targets == 0:
                self.feedback_message = "no saved targets to execute"
                self.stop(py_trees.common.Status.SUCCESS)
                yield self
                return
            self._index = 0
            self._state = "REQUESTING"
            self._bb.set(bb_keys.TARGET_INDEX, 0, overwrite=True)
            self._bb.set(bb_keys.TARGET_REPLAN_COUNT, 0, overwrite=True)

        # --- SWAP POINT A: REQUESTING, subtree ready -> swap + fall through ---
        if self._state == "REQUESTING":
            blocked_by = self._blocked_by(self._index)
            if blocked_by:
                self._mark_blocked(self._index, blocked_by)
                if self._state == "DONE":
                    self.stop(py_trees.common.Status.FAILURE)
                    yield self
                    return
                self.status = py_trees.common.Status.RUNNING
                yield self
                return
            # A replan the planner MARKED as identical to the plan that just
            # failed is never executed again: it burns one replan-budget slot
            # (and, if budget remains, triggers another replan whose guard
            # still compares against the same failed plan) or SKIPs at budget.
            # Readiness first: while a replan is in flight the entry is not
            # ready and its error is cleared, so only a READY marked plan skips.
            get_error = getattr(self._planner, "get_error", None)
            error = None
            if callable(get_error) and self._planner.is_target_ready(self._slot, self._index):
                error = get_error(self._slot, self._index)
            if isinstance(error, str) and error.startswith(IDENTICAL_PLAN_ERROR_PREFIX):
                self._log(f"target:{self._index}:{self._current_desc()} "
                          f"IDENTICAL_PLAN_SKIPPED ({error})")
                already_emitted = self._emit_failed_step(None, error)
                self._on_target_failure(error, target_failed_emitted=already_emitted)
                if self._state == "DONE":
                    self.stop(py_trees.common.Status.FAILURE)
                    yield self
                    return
                self.status = py_trees.common.Status.RUNNING
                yield self
                return
            # E2 (runs 003/004, 2026-08-29): a marker the planner stamped
            # UNRECOVERABLE (the deterministic escape ladder is fully
            # exhausted for this target -- no untried establisher exists for
            # any of its own postconditions) is skipped the same way an
            # IDENTICAL_PLAN marker is, but the target's replan budget is
            # FORCED to exhausted first so `_on_target_failure`'s existing
            # "budget exceeded" path fires immediately -- no further
            # replan_target call for this target, ever, no matter how much
            # budget remained. Telemetry/the bench still see the normal
            # "budget exceeded" SKIPPED reason; the FAILED log line for this
            # tick carries the unrecoverable text via `error` itself.
            if isinstance(error, str) and error.startswith(UNRECOVERABLE_ERROR_PREFIX):
                self._log(f"target:{self._index}:{self._current_desc()} "
                          f"UNRECOVERABLE_SKIPPED ({error})")
                already_emitted = self._emit_failed_step(None, error)
                self._bb.set(bb_keys.TARGET_REPLAN_COUNT, self._max_replans, overwrite=True)
                self._on_target_failure(error, target_failed_emitted=already_emitted)
                if self._state == "DONE":
                    self.stop(py_trees.common.Status.FAILURE)
                    yield self
                    return
                self.status = py_trees.common.Status.RUNNING
                yield self
                return
            subtree = self._swap_in(self._index)
            if subtree is None:
                self.feedback_message = (
                    f"waiting for target {self._index} plan...")
                self.status = py_trees.common.Status.RUNNING
                yield self
                return
            self._state = "EXECUTING"
            desc = _target_desc(
                self._planner._get_desc(self._slot, self._index)
            ) or f"target {self._index}"
            self._bb.set(bb_keys.CURRENT_TARGET, desc, overwrite=True)
            self.feedback_message = f"executing target {self._index}: {desc}"

        # Tick the active child once (sequence semantics over a single child).
        for child in self.children:
            for node in child.tick():
                yield node
                if node is child and node.status != py_trees.common.Status.RUNNING:
                    # --- SWAP POINT B: child just returned terminal ---
                    terminal = node.status
                    if terminal == py_trees.common.Status.SUCCESS:
                        self._on_target_success()
                    else:
                        reason = self._last_child_feedback(node)
                        already_emitted = self._emit_failed_step(node, reason)
                        self._on_target_failure(reason, target_failed_emitted=already_emitted)
                    if self._state == "DONE":
                        terminal_status = (
                            py_trees.common.Status.SUCCESS
                            if self._supervisor_aborted
                            else (
                                py_trees.common.Status.FAILURE
                                if any(outcome != "SUCCEEDED" for outcome in self._target_outcomes.values())
                                else py_trees.common.Status.SUCCESS
                            )
                        )
                        self.stop(terminal_status)
                        yield self
                        return
                    # Next target (or replan): stay RUNNING, swap at point A on
                    # the next tick. The terminal child stays in children[0]
                    # until then — harmless, never ticked again.
                    self.status = py_trees.common.Status.RUNNING
                    yield self
                    return
            # Child was RUNNING: stay RUNNING.
            self.status = py_trees.common.Status.RUNNING
            yield self
            return

    _GATE_REASON_PREFIXES = ("precondition unmet", "postcondition unmet")

    def _emit_failed_step(self, node, reason: str) -> bool:
        """Emit ``step.finished``/failed so the bench sees failed steps too.

        ``BtNode_LogStepResult`` only runs after a successful step; a failed
        leaf short-circuits the sequence before it. Telemetry must never break
        execution, so every read/emit here is guarded.

        ``node`` is the terminal target subtree; its ``tip()`` is the failing
        leaf. When that leaf is a target gate (pre/postcondition check) no step
        failed: CURRENT_ACTION still names the previous target's last step
        (pre gate) or a step that just logged ``succeeded`` (post gate). Those
        — and any failure with no current step at all — are reported as
        ``target.failed`` instead of being attributed to a stale step.

        ``node=None`` means no subtree ran at all (the executor refused an
        identical replan before swapping it in): always ``target.failed``.

        Returns True iff a ``target.failed`` event was emitted here (vs
        ``step.finished`` or nothing) — I6 (round-3 adversarial review):
        ``_on_target_failure``'s SKIPPED/UNRECOVERABLE path uses this to
        avoid double-emitting ``target.failed`` for the same failure.
        """
        telemetry = get_default_telemetry()
        if telemetry is None:
            return False
        try:
            task_id = self._bb.get(bb_keys.TASK_ID)
        except KeyError:
            task_id = None
        try:
            tip = node.tip() if callable(getattr(node, "tip", None)) else None
        except Exception:  # noqa: BLE001
            tip = None
        is_gate = node is None or isinstance(
            tip, (BtNode_TargetPreconditionCheck, BtNode_TargetPostconditionCheck),
        ) or str(reason or "").startswith(self._GATE_REASON_PREFIXES)
        action = None
        if not is_gate:
            try:
                action = self._bb.get(bb_keys.CURRENT_ACTION)
            except KeyError:
                action = None
        try:
            if is_gate or not action:
                telemetry.emit(
                    "target.failed",
                    {"slot": self._slot, "target_index": self._index, "reason": reason},
                    task_id=task_id, phase="execution",
                )
                return True
            params = self._bb.get(bb_keys.CURRENT_PARAMS) or {}
            step_index = int(self._bb.get(bb_keys.PLAN_INDEX) or 1) - 1
            telemetry.emit(
                "step.finished",
                {"step_index": step_index, "action": action, "params": params,
                 "outcome": "failed", "feedback": reason},
                task_id=task_id, phase="execution",
            )
        except Exception:
            pass
        return False

    @staticmethod
    def _last_child_feedback(node) -> str:
        try:
            tip = node.tip() if callable(getattr(node, "tip", None)) else None
            candidate = tip or node
            return str(getattr(candidate, "feedback_message", "") or candidate.name)
        except Exception:  # noqa: BLE001
            try:
                return node.name
            except Exception:
                return "target failed"

    def tip(self) -> Optional[py_trees.behaviour.Behaviour]:
        if self.current_child is not None:
            return self.current_child.tip()
        return None


class BtNode_GeneratePlanFile(Behaviour):
    """Freeze the just-planned command into a standalone, re-runnable .py.

    Best-effort: reads ``COMMAND`` + ``PLAN`` from the blackboard after
    ``BtNode_PlanActions`` succeeds and writes a plan module via
    ``codegen.write_plan_module`` for check-after-run / deterministic replay.
    Always returns SUCCESS — a file-write hiccup must never abort the task.
    """

    def __init__(self, name: str = "emit plan file", out_dir: str = "."):
        super().__init__(name)
        self._out_dir = out_dir
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.COMMAND, access=Access.READ)
        self._bb.register_key(bb_keys.PLAN, access=Access.READ)
        self._bb.register_key(bb_keys.TASK_ID, access=Access.READ)
        self._bb.register_key(bb_keys.PLAN_REVISION, access=Access.READ)

    def update(self):
        from pathlib import Path
        from datetime import datetime
        from .codegen import write_plan_module, safe_slug
        try:
            command = self._bb.get(bb_keys.COMMAND) or ""
            plan = self._bb.get(bb_keys.PLAN) or []
        except KeyError:
            self.feedback_message = "no command/plan to emit"
            return Status.SUCCESS
        try:
            stamp = datetime.now().strftime("%H%M%S")
            revision = self._bb.get(bb_keys.PLAN_REVISION) or 1
            task = safe_slug(str(self._bb.get(bb_keys.TASK_ID) or "task"), 20)
            out = Path(self._out_dir) / f"gpsr_plan_{stamp}_{task}_r{revision}_{safe_slug(command)}.py"
            write_plan_module(command, plan, out)
            self.feedback_message = f"wrote plan module: {out}"
            telemetry = get_default_telemetry()
            if telemetry is not None:
                telemetry.emit(
                    "artifact.created",
                    {"kind": "plan_module", "path": str(out), "plan_revision": self._bb.get(bb_keys.PLAN_REVISION) or 1},
                    task_id=self._bb.get(bb_keys.TASK_ID), phase="planning",
                )
        except Exception as exc:  # noqa: BLE001 — best-effort, never abort
            self.feedback_message = f"plan-file emit failed (ignored): {exc!r}"
        return Status.SUCCESS


# ---------------------------------------------------------------------------
# Plan rehearsal: speak the plan + draw the tree (no robot execution needed)
# ---------------------------------------------------------------------------

def describe_step(action: str, params: Optional[Dict[str, Any]]) -> str:
    """One short spoken clause for a planned step ("go to the kitchen")."""
    p = params or {}
    loc = p.get("location") or p.get("recipient_location")
    obj = p.get("object")
    person = p.get("descriptor") or p.get("person") or p.get("recipient")
    label = p.get("label")
    text = p.get("text") or p.get("message")
    if text:
        # The rehearsal wrapper adds its own sentence period — strip the
        # planner's trailing punctuation so we don't get "shelf..".
        text = str(text).strip().rstrip(".!? ")
    table = {
        "goto": f"go to the {loc}" if loc else "move to the next location",
        "find_object": f"look for the {obj}" if obj else "look for the object",
        "search_object": (
            f"go to the {loc} and look for the {obj}" if loc and obj
            else f"go and look for the {obj}" if obj
            else "go and look for the object"
        ),
        "find_person": f"find the {person}" if person else "find the person",
        "approach_person": "walk up to the person",
        "describe_person": "describe what the person looks like",
        "ask_person": (
            f"ask the person, {p.get('question')}" if p.get("question")
            else "ask the person a question"
        ),
        "follow": f"follow {person}" if person else "follow the person",
        "guide": f"guide them to the {loc}" if loc else "guide them to the destination",
        "grasp": f"pick up the {obj}" if obj else "pick up the object",
        "open": f"ask a referee to open the {loc}" if loc else "ask a referee to open it",
        "place": f"place it at the {loc}" if loc else "place the object down",
        "deliver": (
            f"deliver the {obj} to {person}" if obj and person
            else f"deliver the {obj}" if obj else "hand the object over"
        ),
        "count": f"count the {obj}" if obj else "count the objects",
        "answer_question": "answer a question",
        "announce": f"say, {text}" if text else "tell you what I found",
        "record_position": (
            f"remember this spot as {label}" if label else "remember this spot"
        ),
        "vlm_fallback": (
            f"look and answer, {p.get('question')}" if p.get("question")
            else "look at the scene and answer"
        ),
        "llm_fallback": (
            f"answer the question, {p.get('question')}" if p.get("question")
            else "answer a general question"
        ),
    }
    return table.get(action, str(action).replace("_", " "))


def build_plan_speech(plan: List[Dict[str, Any]]) -> str:
    """Turn a plan into an ordered spoken rehearsal of its small-tree steps."""
    steps = [
        describe_step(s.get("action"), s.get("params"))
        for s in (plan or [])
        if isinstance(s, dict) and s.get("action") in ACTION_FACTORIES
    ]
    if not steps:
        return "I could not form a plan for that command."
    if len(steps) == 1:
        return f"Here is my plan. I will {steps[0]}."
    parts = ["Here is my plan."]
    last = len(steps) - 1
    for i, phrase in enumerate(steps):
        lead = "First" if i == 0 else "Finally" if i == last else "Then"
        parts.append(f"{lead}, I will {phrase}.")
    return " ".join(parts)


class BtNode_BuildPlanSpeech(Behaviour):
    """Read the plan and write an ordered spoken rehearsal to PLAN_SPEECH.

    Always SUCCESS (writes a fallback sentence on an empty/missing plan so the
    downstream announce still has something to say).
    """

    def __init__(self, name: str = "build plan speech"):
        super().__init__(name)
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.PLAN, access=Access.READ)
        self._bb.register_key(bb_keys.PLAN_SPEECH, access=Access.WRITE)

    def update(self):
        try:
            plan = self._bb.get(bb_keys.PLAN) or []
        except KeyError:
            plan = []
        speech = build_plan_speech(plan)
        self._bb.set(bb_keys.PLAN_SPEECH, speech, overwrite=True)
        self.feedback_message = speech
        return Status.SUCCESS


def create_announce_plan() -> py_trees.composites.Sequence:
    """Speak the planned small-tree sequence aloud (build text, then announce)."""
    seq = py_trees.composites.Sequence("announce_plan", memory=True)
    seq.add_child(BtNode_BuildPlanSpeech())
    seq.add_child(BtNode_AnnounceFromBB("announce plan", bb_keys.PLAN_SPEECH))
    return seq


class BtNode_RenderPlanTree(Behaviour):
    """Draw the tree the robot WOULD execute for the current plan, to disk.

    Best-effort: builds each step's small tree (construction only — no ROS
    clients, no robot needed) and renders ``<out_dir>/<name>.{dot,png,svg}``.
    Always returns SUCCESS — a Graphviz hiccup must never abort the rehearsal.
    """

    def __init__(self, name: str = "render plan tree", out_dir: str = "."):
        super().__init__(name)
        self._out_dir = out_dir
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.COMMAND, access=Access.READ)
        self._bb.register_key(bb_keys.PLAN, access=Access.READ)
        self._bb.register_key(bb_keys.TASK_ID, access=Access.READ)
        self._bb.register_key(bb_keys.PLAN_REVISION, access=Access.READ)

    def update(self):
        from datetime import datetime
        from .plan_viz import render_plan_tree, planned_tree_document
        from .codegen import safe_slug
        try:
            command = self._bb.get(bb_keys.COMMAND) or ""
            plan = self._bb.get(bb_keys.PLAN) or []
        except KeyError:
            self.feedback_message = "no command/plan to render"
            return Status.SUCCESS
        if not plan:
            self.feedback_message = "empty plan — nothing to render"
            return Status.SUCCESS
        try:
            stamp = datetime.now().strftime("%H%M%S")
            plan_revision = self._bb.get(bb_keys.PLAN_REVISION) or 1
            task = safe_slug(str(self._bb.get(bb_keys.TASK_ID) or "task"), 20)
            name = f"gpsr_tree_{stamp}_{task}_r{plan_revision}_{safe_slug(command)}"
            artifacts = render_plan_tree(plan, ACTION_FACTORIES, self._out_dir, name)
            tree_document = planned_tree_document(plan, ACTION_FACTORIES, name)
            self.feedback_message = (
                f"rendered tree: {artifacts.get('png') or artifacts.get('dot')}"
            )
            telemetry = get_default_telemetry()
            if telemetry is not None:
                telemetry.emit(
                    "tree.generated",
                    {
                        "kind": "planned",
                        "plan_revision": plan_revision,
                        "tree_revision": f"planned-r{plan_revision}",
                        "artifacts": artifacts,
                        "node_count": len(tree_document.get("nodes", [])),
                        "tree": tree_document,
                    },
                    task_id=self._bb.get(bb_keys.TASK_ID), phase="tree",
                )
        except Exception as exc:  # noqa: BLE001 — best-effort, never abort
            self.feedback_message = f"tree render failed (ignored): {exc!r}"
        return Status.SUCCESS


def create_dispatcher() -> py_trees.composites.Selector:
    """Dispatcher Selector: one [Router → SmallTree] branch per registered action.

    On a successful match the small tree's status (SUCCESS/FAILURE) propagates
    up through the selector. If no router matches, the selector returns FAILURE
    (which the orchestrator interprets as "unknown action — trigger correction").
    """
    selector = py_trees.composites.Selector("dispatcher", memory=False)
    supervisor = get_default_supervisor()
    for action_name, factory in ACTION_FACTORIES.items():
        branch = py_trees.composites.Sequence(f"branch:{action_name}", memory=True)
        branch.add_child(BtNode_ActionRouter(action_name))
        branch.add_child(wrap_action_factory(action_name, factory, supervisor))
        selector.add_child(branch)
    return selector


def create_self_correction(max_corrections: int = 3) -> py_trees.composites.Sequence:
    """On step failure: log, bump counter (fail if exhausted), announce, re-plan."""
    from behavior_tree.TemplateNodes.Audio import BtNode_Announce
    seq = py_trees.composites.Sequence("self_correction", memory=True)
    seq.add_child(BtNode_LogStepResult("log failure", succeeded=False))
    seq.add_child(BtNode_BumpCorrectionCounter(
        "bump correction counter", max_corrections=max_corrections,
    ))
    seq.add_child(BtNode_Announce(
        "announce correcting", bb_source=None,
        message="I had trouble with that step. Let me reconsider.",
    ))
    seq.add_child(BtNode_PlanActions(
        name="replan after failure",
        rephrase_on_failure=True,
    ))
    return seq


def create_execute_one_step(max_corrections: int = 3) -> py_trees.composites.Sequence:
    """One iteration of the orchestrator loop: pop → dispatch+monitor → maybe correct."""
    # Pop: SUCCESS continues, FAILURE = no more steps (caller decides what to do).
    pop = BtNode_PopNextAction()

    dispatch = create_dispatcher()

    # Self-monitor: log a SUCCESS step after the dispatcher returned SUCCESS.
    monitor_then_log = py_trees.composites.Sequence("monitor+log success", memory=True)
    monitor_then_log.add_child(dispatch)
    monitor_then_log.add_child(BtNode_LogStepResult("log success", succeeded=True))

    supervisor = get_default_supervisor()
    if supervisor is not None and supervisor.config.mode is SupervisionMode.ACTIVE:
        # Effect failures are held at RUNNING by SupervisedEffect until the
        # verifier resolves them. Never enter the legacy whole-plan correction
        # path while the active supervisor owns recovery.
        dispatch_or_correct = monitor_then_log
    else:
        correction = create_self_correction(max_corrections=max_corrections)
        # memory=True: once `correction` (the replan branch) goes RUNNING it
        # must be resumed, not torn down and relaunched from `monitor_then_log`
        # every tick. With memory=False, a non-memory Selector re-enters
        # child 0 (monitor_then_log/dispatch) on EVERY tick regardless of
        # which branch is in flight: since `pop` isn't re-ticked while this
        # Sequence is RUNNING here, the same failed action's router still
        # matches, so monitor_then_log relaunches that action from scratch;
        # the moment it goes RUNNING again it becomes `current_child`, and
        # the selector's own priority-interrupt handling
        # (`previous != current_child`) invalidates the in-flight `correction`
        # branch (BtNode_PlanActions mid-replan) before it can ever complete
        # — the same livelock shape as F3 (round-2 review follow-up), bounded
        # only by `max_corrections`. memory=True makes the Selector resume
        # directly at whichever branch last went RUNNING; a fresh entry
        # (this composite's own status != RUNNING, e.g. a new step) still
        # resets to child 0 first (see Selector.tick()), so normal
        # dispatch-first-then-correct semantics are unchanged.
        dispatch_or_correct = py_trees.composites.Selector(
            "dispatch_or_correct", memory=True,
            children=[monitor_then_log, correction],
        )

    step = py_trees.composites.Sequence("execute_step", memory=True)
    step.add_child(pop)
    step.add_child(dispatch_or_correct)
    return step


def _reset_task_state(seq: py_trees.composites.Sequence) -> None:
    """Hard per-task reset of EXECUTION state (not the plan, not the start pose).

    Runs before planning a task and before executing a task, so a previous
    task's STATE_LOG / correction count / failure / nav-location / grasp-referee
    flag can never bleed into this task's plan or self-correction. PLAN_INDEX is
    reset to 0 so a restored plan starts from its first step.
    """
    seq.add_child(BtNode_BlackboardSet("reset state_log", bb_keys.STATE_LOG, []))
    seq.add_child(BtNode_BlackboardSet("reset correction", bb_keys.CORRECTION_COUNT, 0))
    seq.add_child(BtNode_BlackboardSet("reset last_failure", bb_keys.LAST_FAILURE, ""))
    seq.add_child(BtNode_BlackboardSet("reset plan revision", bb_keys.PLAN_REVISION, 0))
    seq.add_child(BtNode_BlackboardSet(
        "reset task outcome", bb_keys.TASK_OUTCOME, None,
    ))
    seq.add_child(BtNode_BlackboardSet(
        "reset supervisor disposition", bb_keys.SUPERVISOR_STEP_DISPOSITION, None,
    ))
    seq.add_child(BtNode_BlackboardSet("reset nav location", bb_keys.LAST_NAV_LOCATION, ""))
    seq.add_child(BtNode_BlackboardSet("reset pending nav location", bb_keys.PENDING_NAV_LOCATION, ""))
    seq.add_child(BtNode_BlackboardSet("reset grasp-referee", bb_keys.GRASP_ASK_REFEREE, False))
    seq.add_child(BtNode_BlackboardSet("reset appliance-opened", bb_keys.APPLIANCE_OPENED, False))
    seq.add_child(BtNode_BlackboardSet("reset plan_index", bb_keys.PLAN_INDEX, 0))


def _task_identity(slot: int) -> str:
    """Return a run-scoped task id when telemetry is active."""
    telemetry = get_default_telemetry()
    return telemetry.task_id(slot + 1) if telemetry is not None else f"task-{slot + 1}"


# --------------------------------------------------------------------------- #
# "Here's what I'm about to do": a short spoken summary of the materialized
# plan, announced right after "Starting task N now." (before execution touches
# anything). Distinct from describe_step/build_plan_speech above, which speak
# a longer step-by-step rehearsal during the PLANNING phase — this is a terse
# one-liner spoken at the START of EXECUTION, from the slot's already-saved
# plan (see create_announce_task_plan).
# --------------------------------------------------------------------------- #

_PLAN_STEP_PHRASES = {
    "announce": lambda p: "report the result",
    "ask_person": lambda p: "ask the person",
    "describe_person": lambda p: "describe the person",
    "deliver": lambda p: "hand it over",
    "follow": lambda p: "follow the person",
    "guide": lambda p: (
        f"guide the person to the {p['location']}" if p.get("location")
        else "guide the person"
    ),
    "vlm_fallback": lambda p: "look and answer",
    "record_position": lambda p: "remember this spot",
    "goto": lambda p: (
        f"go to the {p['location']}" if p.get("location") else "go to the destination"
    ),
    "count": lambda p: (
        f"count the {p['object']}" if p.get("object") else "count the objects"
    ),
    "find_person": lambda p: (
        f"find {p.get('descriptor') or p.get('person') or p.get('recipient')}"
        if (p.get("descriptor") or p.get("person") or p.get("recipient"))
        else "find the person"
    ),
    "grasp": lambda p: (
        f"pick up the {p['object']}" if p.get("object") else "pick up the object"
    ),
}


def _describe_plan_step(action: Any, params: Any) -> str:
    """One short clause for a single materialized plan step. Never raises."""
    try:
        template = _PLAN_STEP_PHRASES.get(action)
        p = params if isinstance(params, dict) else {}
        if template is not None:
            return template(p)
        return str(action or "").replace("_", " ") or "do something"
    except Exception:
        return "do something"


def describe_plan(steps: Optional[List[Dict[str, Any]]]) -> str:
    """Turn a materialized action plan into one short spoken sentence.

    ``steps`` is the same ordered list of ``{action, params}`` dicts that
    reaches ``materialise_params`` / ``BtNode_TargetPostconditionCheck``'s
    ``action_plan`` — the concrete, already-planned steps for one task. Pure
    and best-effort: malformed steps are skipped/summarized rather than
    raising, so a corrupt saved plan can never crash the announce.
    """
    phrases = []
    for step in (steps or []):
        if not isinstance(step, dict):
            continue
        action = step.get("action")
        params = step.get("params")
        phrases.append(_describe_plan_step(action, params))
    if not phrases:
        return "My plan is ready."
    truncated = len(phrases) > 8
    phrases = phrases[:8]
    body = ", then ".join(phrases)
    return f"My plan: {body}, and more." if truncated else f"My plan: {body}."


class BtNode_BuildTaskPlanSpeech(Behaviour):
    """Read one batch slot's SAVED plan and write describe_plan's summary to
    PLAN_SPEECH.

    Unlike BtNode_BuildPlanSpeech (reads the live PLAN key during planning),
    this runs at the START of the EXECUTE phase, right after "announce start
    task N" and before the restore/reset leaves pull the slot's saved plan
    back onto PLAN — so it reads the stable SAVED_PLAN_PREFIX+slot copy
    directly. The batch tree is built (in Python) before any command is even
    heard, so the plan text can't be a build-time constant; it's recomputed
    here, once, right before it's spoken. Always SUCCESS.
    """

    def __init__(self, slot: int, name: Optional[str] = None):
        super().__init__(name or f"build task {slot + 1} plan speech")
        self._slot = int(slot)
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(f"{bb_keys.SAVED_PLAN_PREFIX}{self._slot}", access=Access.READ)
        self._bb.register_key(bb_keys.PLAN_SPEECH, access=Access.WRITE)

    def update(self):
        try:
            steps = self._bb.get(f"{bb_keys.SAVED_PLAN_PREFIX}{self._slot}") or []
        except KeyError:
            steps = []
        speech = describe_plan(steps)
        self._bb.set(bb_keys.PLAN_SPEECH, speech, overwrite=True)
        self.feedback_message = speech
        return Status.SUCCESS


def create_announce_task_plan(slot: int) -> py_trees.composites.Sequence:
    """Speak batch slot ``slot``'s materialized plan aloud right as its task
    starts executing (build text, then announce)."""
    seq = py_trees.composites.Sequence(f"announce plan {slot + 1}", memory=True)
    seq.add_child(BtNode_BuildTaskPlanSpeech(slot))
    seq.add_child(BtNode_AnnounceFromBB(f"announce plan {slot + 1} (speak)", bb_keys.PLAN_SPEECH))
    return seq


def create_execute_loop(
    max_steps: int = 25, max_corrections: int = 3,
) -> py_trees.behaviour.Behaviour:
    """The dispatch + self-correction loop over a pre-set ``PLAN``.

    Runs each step until ``BtNode_PopNextAction`` returns FAILURE (plan
    exhausted). That failure bubbles up through ``Repeat`` (aborts on any child
    failure), and the outer ``FailureIsSuccess`` converts it back to SUCCESS so
    the parent treats command completion as normal success. Assumes ``PLAN`` /
    ``PLAN_INDEX`` are already set (by planning, or by restoring a saved plan).
    """
    loop_body = create_execute_one_step(max_corrections=max_corrections)
    loop = py_trees.decorators.Repeat(
        name="step loop", child=loop_body, num_success=max_steps,
    )
    return py_trees.decorators.FailureIsSuccess("plan-exhausted = done", loop)


def create_execute_command(
    max_steps: int = 25,
    max_corrections: int = 3,
    emit_plan_dir: Optional[str] = None,
    announce_plan: bool = True,
) -> py_trees.behaviour.Behaviour:
    """Plan once, then run the step loop until the plan is exhausted.

    Kept for single-command callers (dry-run, tests). The batch flow uses the
    split ``_reset_task_state`` / ``create_execute_loop`` pieces instead so it can
    plan up front and execute later.

    If ``emit_plan_dir`` is given, a standalone re-runnable ``.py`` of the
    planned tree is written there right after planning (check-after-run). If
    ``announce_plan`` (default), the robot speaks the full planned step sequence
    aloud right after planning, before executing it.
    """
    root = py_trees.composites.Sequence("execute_command", memory=True)
    _reset_task_state(root)
    root.add_child(BtNode_PlanActions(name="plan initial"))
    if emit_plan_dir is not None:
        root.add_child(BtNode_GeneratePlanFile(out_dir=emit_plan_dir))
        root.add_child(BtNode_RenderPlanTree(out_dir=emit_plan_dir))
    if announce_plan:
        # Speak the full plan ("Here is my plan. First... Then... Finally...")
        # right after planning so the operator hears it before execution starts.
        root.add_child(create_announce_plan())
    root.add_child(create_execute_loop(max_steps, max_corrections))
    root.add_child(BtNode_FinalizeTask(max_corrections=max_corrections))
    return root


# --------------------------------------------------------------------------- #
# Batch command flow: collect N commands + plans UP FRONT (at the command
# point), then execute them one by one. Replaces the old collect-execute-repeat
# loop so the robot hears/plans/announces all tasks before acting.
# --------------------------------------------------------------------------- #

def make_listen_intake(listen_timeout: float = 30.0):
    """Intake factory: for each slot, prompt the operator and listen for the
    command into ``bb_keys.COMMAND`` (real audio; typed in mock)."""
    from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_ListenAction

    def factory(slot: int) -> py_trees.behaviour.Behaviour:
        seq = py_trees.composites.Sequence(f"get command {slot + 1}", memory=True)
        seq.add_child(BtNode_Announce(
            f"prompt task {slot + 1}", bb_source=None,
            message=f"Speak loudly near to the microphone. Please tell me task number {slot + 1} after the beep.",
        ))
        seq.add_child(BtNode_ListenAction(
            f"listen task {slot + 1}", bb_dest_key=bb_keys.COMMAND,
            timeout=listen_timeout,
        ))
        seq.add_child(BtNode_AnnounceFromBB(
            f"echo task {slot + 1}", bb_keys.COMMAND, prefix="I heard: ",
        ))
        return seq

    return factory


def make_inject_intake(commands):
    """Intake factory that injects ``commands[slot]`` into ``bb_keys.COMMAND``
    (for desktop / mock e2e tests, no audio)."""
    def factory(slot: int) -> py_trees.behaviour.Behaviour:
        return BtNode_BlackboardSet(
            f"inject command {slot + 1}", bb_keys.COMMAND, commands[slot],
        )

    return factory


def _create_plan_and_save(slot: int, emit_plan_dir: Optional[str] = None,
                          announce_plan: bool = True) -> py_trees.composites.Sequence:
    """Plan the command currently in ``COMMAND``, announce it, and stash the plan
    + command into slot ``slot`` for later execution. The intake step must have
    written ``COMMAND`` first."""
    from behavior_tree.TemplateNodes.Audio import BtNode_Announce
    seq = py_trees.composites.Sequence(f"plan+save task {slot + 1}", memory=True)
    seq.add_child(BtNode_BlackboardSet(
        f"set task identity {slot + 1}", bb_keys.TASK_ID, _task_identity(slot),
    ))
    _reset_task_state(seq)
    # Bridge the dead air between hearing the command and speaking the plan: the
    # LLM planning call below takes a few seconds, so acknowledge first.
    seq.add_child(BtNode_Announce(
        f"announce planning task {slot + 1}", bb_source=None,
        message="I am planning, please wait.",
    ))
    seq.add_child(BtNode_PlanActions(name=f"plan task {slot + 1}"))
    if emit_plan_dir is not None:
        seq.add_child(BtNode_GeneratePlanFile(out_dir=emit_plan_dir))
        seq.add_child(BtNode_RenderPlanTree(out_dir=emit_plan_dir))
    if announce_plan:
        seq.add_child(BtNode_Announce(
            f"announce task {slot + 1} plan intro", bb_source=None,
            message=f"For task {slot + 1}, here is my plan.",
        ))
        seq.add_child(create_announce_plan())
    seq.add_child(BtNode_BlackboardCopy(
        f"save plan {slot}", bb_keys.PLAN, f"{bb_keys.SAVED_PLAN_PREFIX}{slot}"))
    seq.add_child(BtNode_BlackboardCopy(
        f"save command {slot}", bb_keys.COMMAND, f"{bb_keys.SAVED_COMMAND_PREFIX}{slot}"))
    return seq


def _create_execute_slot(slot: int, max_steps: int = 25,
                         max_corrections: int = 3) -> py_trees.composites.Sequence:
    """Announce the start of task ``slot``, restore its saved command + plan, then
    run the dispatch/self-correction loop over it."""
    from behavior_tree.TemplateNodes.Audio import BtNode_Announce
    seq = py_trees.composites.Sequence(f"execute task {slot + 1}", memory=True)
    seq.add_child(BtNode_BlackboardSet(
        f"set execution task identity {slot + 1}", bb_keys.TASK_ID, _task_identity(slot),
    ))
    seq.add_child(BtNode_Announce(
        f"announce start task {slot + 1}", bb_source=None,
        message=f"Starting task {slot + 1} now.",
    ))
    seq.add_child(create_announce_task_plan(slot))
    # Restore this task's command (so a mid-task self-correction re-plans the
    # right command) and its pre-made plan.
    seq.add_child(BtNode_BlackboardCopy(
        f"restore command {slot}", f"{bb_keys.SAVED_COMMAND_PREFIX}{slot}", bb_keys.COMMAND))
    seq.add_child(BtNode_BlackboardCopy(
        f"restore plan {slot}", f"{bb_keys.SAVED_PLAN_PREFIX}{slot}", bb_keys.PLAN))
    _reset_task_state(seq)
    seq.add_child(BtNode_BlackboardSet(
        f"restore plan revision {slot + 1}", bb_keys.PLAN_REVISION, 1,
    ))
    seq.add_child(create_execute_loop(max_steps, max_corrections))
    seq.add_child(BtNode_FinalizeTask(max_corrections=max_corrections))
    return seq


def create_batch_command_flow(
    num_commands: int = 3,
    make_intake=None,
    max_steps: int = 25,
    max_corrections: int = 3,
    emit_plan_dir: Optional[str] = None,
    announce_plan: bool = True,
) -> py_trees.composites.Sequence:
    """Collect ``num_commands`` commands + plans up front, then execute them.

    ``make_intake(slot)`` returns a behaviour that writes the slot's command
    into ``bb_keys.COMMAND`` (a prompt+listen sub-tree from
    :func:`make_listen_intake`, or an injection node from
    :func:`make_inject_intake` for tests). The intake phase plans + announces +
    stashes each command; the execute phase restores each and runs the loop,
    announcing before each task. The robot should already be standing at the
    command point (all tasks share that start pose).
    """
    from behavior_tree.TemplateNodes.Audio import BtNode_Announce
    if make_intake is None:
        make_intake = make_listen_intake()

    root = py_trees.composites.Sequence("batch_command_flow", memory=True)

    intake = py_trees.composites.Sequence("intake phase (collect+plan)", memory=True)
    for i in range(num_commands):
        slot = py_trees.composites.Sequence(f"intake task {i + 1}", memory=True)
        slot.add_child(make_intake(i))
        slot.add_child(_create_plan_and_save(i, emit_plan_dir, announce_plan))
        intake.add_child(slot)
    root.add_child(intake)

    root.add_child(BtNode_Announce(
        "announce start execution", bb_source=None,
        message=f"I have {num_commands} tasks. I will start executing them now.",
    ))

    execute = py_trees.composites.Sequence("execute phase", memory=True)
    for i in range(num_commands):
        execute.add_child(_create_execute_slot(i, max_steps, max_corrections))
    root.add_child(execute)
    return root


# --------------------------------------------------------------------------- #
# TWO-LAYER batch flow. Same collect-then-execute shape as the legacy flow
# above, but the intake phase runs the TOP layer (split_command) then the
# LOWER layer (request_plan_all — one thread per target, in parallel), and the
# execute phase hands each slot to a DynamicExecutor that swaps ready target
# subtrees into the RUNNING tree at runtime.
# --------------------------------------------------------------------------- #

class BtNode_BuildTargetsSpeech(Behaviour):
    """Turn the saved target list into one spoken rehearsal sentence.

    Mirrors ``BtNode_BuildPlanSpeech`` for the two-layer flow: reads the slot's
    saved targets (``SAVED_TARGETS_PREFIX+<slot>``, a list of NL descriptions)
    and writes an ordered spoken summary to ``PLAN_SPEECH`` for the announce.
    """

    def __init__(self, name: str, slot: int):
        super().__init__(name)
        self._slot = int(slot)
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.SAVED_TARGETS_PREFIX + str(self._slot),
                              access=Access.READ)
        self._bb.register_key(bb_keys.PLAN_SPEECH, access=Access.WRITE)

    def update(self):
        try:
            targets = self._bb.get(bb_keys.SAVED_TARGETS_PREFIX + str(self._slot)) or []
        except KeyError:
            targets = []
        if not targets:
            self._bb.set(bb_keys.PLAN_SPEECH, "I have no tasks.", overwrite=True)
            return Status.SUCCESS
        parts = []
        for i, t in enumerate(targets, start=1):
            lead = "First" if i == 1 else "Finally" if i == len(targets) else "Then"
            parts.append(f"{lead}, {_target_desc(t)}.")
        text = f"Here is my plan. I will complete {len(targets)} tasks. " + " ".join(parts)
        self._bb.set(bb_keys.PLAN_SPEECH, text, overwrite=True)
        self.feedback_message = text[:80]
        return Status.SUCCESS


def create_announce_targets(slot: int) -> py_trees.composites.Sequence:
    """Speak the two-layer target list aloud (build text, then announce)."""
    seq = py_trees.composites.Sequence(f"announce_targets_{slot}", memory=True)
    seq.add_child(BtNode_BuildTargetsSpeech(f"build targets speech {slot}", slot))
    seq.add_child(BtNode_AnnounceFromBB(f"announce targets {slot}", bb_keys.PLAN_SPEECH))
    return seq


def request_command_replan(bb_client, reason: str = "") -> None:
    """Stub for a COMMAND-level replan request (extension point).

    The replan-trigger mechanism is not wired yet: a failing target re-plans
    only itself inside DynamicExecutor._on_target_failure. When the trigger is
    announced, calling this asks the top layer to re-split the whole command
    (``BtNode_SplitCommand`` consumes this and re-runs). Writes a request the
    orchestrator's bridge nodes already clear on their next run.
    """
    bb_client.set(bb_keys.REPLAN_REQUEST,
                  {"level": "command", "index": -1, "reason": reason},
                  overwrite=True)


def _create_plan_and_save_new(
    slot: int,
    planner,
    announce_targets: bool = True,
    emit_plan_dir: Optional[str] = None,
) -> py_trees.composites.Sequence:
    """Two-layer intake for one slot: split -> parallel plan-all -> announce + stash.

    Reads the command (already in ``COMMAND`` from intake), runs the TOP layer
    (BtNode_SplitCommand) then the LOWER layer in parallel
    (BtNode_PlanAllTargets), announces the target list, and stashes the saved
    command + aggregate plan so the execute phase can restore + log them.
    """
    from behavior_tree.TemplateNodes.Audio import BtNode_Announce
    seq = py_trees.composites.Sequence(f"plan+save two-layer task {slot + 1}", memory=True)
    _reset_task_state(seq)
    seq.add_child(BtNode_Announce(
        f"announce planning task {slot + 1}", bb_source=None,
        message="I am planning, please wait.",
    ))
    seq.add_child(BtNode_SplitCommand(f"split command {slot + 1}", planner))
    seq.add_child(BtNode_PlanAllTargets(f"plan all targets {slot + 1}", slot, planner))
    if emit_plan_dir is not None:
        # Freeze the aggregate plan (COMMAND + PLAN are set by PlanAllTargets) to
        # a standalone replayable .py — preserves the legacy check-after-run.
        seq.add_child(BtNode_GeneratePlanFile(out_dir=emit_plan_dir))
    if announce_targets:
        seq.add_child(create_announce_targets(slot))
    seq.add_child(BtNode_BlackboardCopy(
        f"save command {slot}", bb_keys.COMMAND, f"{bb_keys.SAVED_COMMAND_PREFIX}{slot}"))
    # Aggregate plan (concatenation of per-target plans) for the legacy readers.
    seq.add_child(BtNode_BlackboardCopy(
        f"save aggregate plan {slot}", bb_keys.PLAN, f"{bb_keys.SAVED_PLAN_PREFIX}{slot}"))
    return seq


def _create_execute_slot_new(
    slot: int,
    planner,
    max_replans_per_target: int = 3,
) -> py_trees.composites.Sequence:
    """Two-layer execute for one slot: restore + run a DynamicExecutor over it."""
    from behavior_tree.TemplateNodes.Audio import BtNode_Announce
    seq = py_trees.composites.Sequence(f"execute two-layer task {slot + 1}", memory=True)
    seq.add_child(BtNode_Announce(
        f"announce start task {slot + 1}", bb_source=None,
        message=f"Starting task {slot + 1} now.",
    ))
    seq.add_child(create_announce_task_plan(slot))
    seq.add_child(BtNode_BlackboardCopy(
        f"restore command {slot}", f"{bb_keys.SAVED_COMMAND_PREFIX}{slot}", bb_keys.COMMAND))
    seq.add_child(BtNode_BlackboardCopy(
        f"restore targets {slot}", f"{bb_keys.SAVED_TARGETS_PREFIX}{slot}", bb_keys.TARGETS))
    _reset_task_state(seq)
    seq.add_child(DynamicExecutor(
        f"executor task {slot + 1}", slot, planner,
        max_replans_per_target=max_replans_per_target,
    ))
    return seq


def create_batch_command_flow_new(
    planner,
    num_commands: int = 3,
    make_intake=None,
    max_replans_per_target: int = 3,
    announce_targets: bool = True,
    emit_plan_dir: Optional[str] = None,
) -> py_trees.composites.Sequence:
    """TWO-LAYER batch command flow: split + parallel plan up front, execute later.

    ``planner`` is a GPSRPlanner (decoupled orchestrator); required. Each slot
    runs ``_create_plan_and_save_new`` (top split + parallel lower layer) in the
    intake phase, then ``_create_execute_slot_new`` (a DynamicExecutor that swaps
    ready target subtrees into the live tree at runtime) in the execute phase.
    """
    from behavior_tree.TemplateNodes.Audio import BtNode_Announce
    if make_intake is None:
        make_intake = make_listen_intake()

    root = py_trees.composites.Sequence("batch_command_flow_two_layer", memory=True)

    intake = py_trees.composites.Sequence("intake phase (split+plan parallel)", memory=True)
    for i in range(num_commands):
        slot = py_trees.composites.Sequence(f"intake task {i + 1}", memory=True)
        slot.add_child(make_intake(i))
        slot.add_child(_create_plan_and_save_new(i, planner, announce_targets,
                                                 emit_plan_dir))
        intake.add_child(slot)
    root.add_child(intake)

    root.add_child(BtNode_Announce(
        "announce start execution", bb_source=None,
        message=f"I have {num_commands} tasks. I will start executing them now.",
    ))

    execute = py_trees.composites.Sequence("execute phase", memory=True)
    for i in range(num_commands):
        execute.add_child(_create_execute_slot_new(i, planner, max_replans_per_target))
    root.add_child(execute)
    return root


COMMAND_POINT_LOCATION = "command_point"


def has_command_point() -> bool:
    """True once ``command_point`` is a filled-in pose in constants.json."""
    return COMMAND_POINT_LOCATION in KNOWN_LOCATIONS


def create_goto_command_point() -> py_trees.composites.Sequence:
    """Drive to the fixed command point to receive the next command.

    RoboCup GPSR requires the robot to be at the instruction/command point to be
    given the next task, so this runs at the top of every command round. It
    reuses the ``goto`` small tree (announce → tuck arm → navigate), pointed at
    the ``command_point`` pose from constants.json. Only add this when
    :func:`has_command_point` is True (otherwise ``command_point`` has no pose
    yet and the goto would have nothing to navigate to).
    """
    seq = py_trees.composites.Sequence("goto_command_point", memory=True)
    seq.add_child(BtNode_BlackboardSet(
        "set command-point location", bb_keys.TARGET_LOCATION, "command point",
    ))
    seq.add_child(BtNode_BlackboardSet(
        "set command-point pose", bb_keys.TARGET_POSE,
        KNOWN_LOCATIONS.get(COMMAND_POINT_LOCATION),
    ))
    seq.add_child(create_goto())
    return seq


def create_orchestrator_init(capture_pose: bool = True) -> py_trees.composites.Sequence:
    """Reset blackboard state and (optionally) record the start pose.

    ``capture_pose=False`` skips the TF lookup — used by the dry-run rehearsal,
    which only plans/announces/draws and never navigates, so it must not need
    localization (TF) up just to start.
    """
    from behavior_tree.TemplateNodes.Navigation import BtNode_CaptureCurrentPose
    from .small_trees import BtNode_BlackboardSet
    seq = py_trees.composites.Sequence("orchestrator_init", memory=True)
    seq.add_child(BtNode_BlackboardSet("clear plan", bb_keys.PLAN, []))
    seq.add_child(BtNode_BlackboardSet("clear plan_index", bb_keys.PLAN_INDEX, 0))
    seq.add_child(BtNode_BlackboardSet("clear state_log", bb_keys.STATE_LOG, []))
    seq.add_child(BtNode_BlackboardSet("clear correction", bb_keys.CORRECTION_COUNT, 0))
    seq.add_child(BtNode_BlackboardSet("clear last_failure", bb_keys.LAST_FAILURE, ""))
    seq.add_child(BtNode_BlackboardSet("clear dynamic locations", bb_keys.DYNAMIC_LOCATIONS, {}))
    if capture_pose:
        # Where the robot stands now is "start_position" for the whole command —
        # "bring it to me" navigates back here. Captured straight to START_POSE
        # (the labelled registry is for runtime record_position(label=...) steps).
        seq.add_child(BtNode_CaptureCurrentPose("capture start pose", bb_key=bb_keys.START_POSE))
    return seq
