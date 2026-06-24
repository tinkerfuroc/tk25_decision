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

import json
import math
import random
import re
import textwrap
import threading
import uuid
from typing import Any, Dict, List, Optional, Tuple

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
from .planner_validators import validate_plan
from .small_trees import (
    ACTION_FACTORIES,
    bb_keys,
    BtNode_AnnounceFromBB,
    BtNode_BlackboardSet,
    SEARCH_POSE_KEYS,
)


# ---------------------------------------------------------------------------
# Knowledge available to the LLM (poses + objects loaded once at import).
# Each entry maps the GPSR-vocabulary name -> resolved PoseStamped or vision
# prompt string. The orchestrator uses these to materialise concrete BB values
# when popping a step from the plan.
# ---------------------------------------------------------------------------

KNOWN_LOCATIONS: Dict[str, PoseStamped] = {}
KNOWN_OBJECT_PROMPTS: Dict[str, str] = {}
# object name -> the location it usually lives at (used when a fetch command
# names no location). Populated from constants.json "default_locations".
DEFAULT_OBJECT_LOCATIONS: Dict[str, str] = {}
# room/location name -> ordered list of pose names to sweep when the in-room
# spot is unknown (the override case). Populated from constants.json
# "search_spots"; a location with no entry falls back to [itself].
ROOM_SEARCH_SPOTS: Dict[str, List[str]] = {}

# Names the planner may use for "where the robot stood when it received the
# command". Resolved from the blackboard (bb_keys.START_POSE, captured by
# create_record_position at command start) instead of constants.json.
START_LOCATION_ALIASES = {"start_position", "instruction_point", "start", "operator"}


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
    DEFAULT_OBJECT_LOCATIONS.clear()
    ROOM_SEARCH_SPOTS.clear()
    with open(constants_path, "r") as fh:
        constants = json.load(fh)
    for key, value in constants.get("possible_poses", {}).items():
        KNOWN_LOCATIONS[key] = _parse_pose_stamped(value)
    for key, value in constants.get("egpsr_rooms", {}).items():
        # Don't overwrite if already present in possible_poses.
        KNOWN_LOCATIONS.setdefault(key, _parse_pose_stamped(value))
    for key, value in constants.get("possible_objects", {}).items():
        KNOWN_OBJECT_PROMPTS[key] = value
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
    - search_object(object: str, location?: str)
        Go to a place and FIND an object there so it can be picked up — the
        finder for FETCH / BRING / GRASP tasks. It navigates by itself and, if
        the location has several recorded search spots, sweeps them until the
        object is seen (then parks there with the object in view for grasp). Use
        it INSTEAD of a separate goto + find_object whenever the goal is to grasp
        or bring the object. ``location`` is OPTIONAL: pass it only when the
        command names where the object is ("a coke from the living room") — that
        overrides the default; OMIT it otherwise and the robot uses the object's
        default location. ``object`` may be a known name or a category noun.
        Follow with grasp(object). Do NOT add a goto before it.
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
    - guide(location: str)
        Lead a person to ``location``. MUST be preceded by ``find_person``
        then ``approach_person`` in the same plan (locate the person and walk
        to them before leading). Never use ``guide`` to express "go
        yourself" — that is ``goto``.
    - grasp(object: str)
        Pick up an object that is currently in view of the arm/vision system.
        Always plan ``search_object`` first (which navigates + finds it); only
        use goto + find_object before grasp if you have a specific reason not to
        sweep.
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
          {"action": "search_object", "params": {"object": "coke", "location": "kitchen"}},
          {"action": "grasp", "params": {"object": "coke"}},
          {"action": "deliver", "params": {"object": "coke", "recipient": "me", "recipient_location": "start_position"}}
        ]
    Example: "fetch me a coke" (no location given) =>
        [
          {"action": "search_object", "params": {"object": "coke"}},
          {"action": "grasp", "params": {"object": "coke"}},
          {"action": "deliver", "params": {"object": "coke", "recipient": "me", "recipient_location": "start_position"}}
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
    16. To FETCH / BRING / PICK UP an object, use ``search_object(object,
       location?)`` then ``grasp(object)`` (then ``deliver`` if it goes to
       someone). ``search_object`` navigates and finds by itself — do NOT emit a
       separate ``goto`` or ``find_object`` for that same object, and do NOT put
       a ``goto`` before it. Pass ``location`` ONLY when the command names where
       the object is (override); omit it otherwise (the robot uses the default
       location). Plain ``goto`` + ``find_object`` / ``count`` are still correct
       for NON-grasp finds and counts (e.g. "how many cokes are in the kitchen").
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
        f"Known objects: {known_obj}\n"
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
    """Keep only well-formed {action, params} steps using known actions."""
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
        if action in ACTION_FACTORIES:
            cleaned.append({"action": action, "params": params})
        else:
            dropped.append(str(action))
    return cleaned, dropped


def _fallback_plan(command: str) -> List[Dict[str, Any]]:
    """Guaranteed non-empty plan when the LLM cannot produce a valid one.

    Never let the robot silently refuse: emit a single spoken acknowledgement so
    the operator hears a response and the command always has *a* plan. Only hit
    after every planning attempt failed — realistic commands never reach here.
    """
    return [{
        "action": "announce",
        "params": {
            "text": "I heard your command but could not work out a complete "
                    "plan for it. I will skip it for now.",
        },
    }]


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
        self._client_oai = openai.OpenAI(
            api_key=OPENAI_API_KEY,
            base_url="https://openrouter.ai/api/v1",
        )
        self._bb = None
        self._thread: Optional[threading.Thread] = None
        self._plan_result: Optional[List[Dict[str, Any]]] = None
        self._fell_back: bool = False
        self._attempts_used: int = 0
        self._rephrase_on_failure = rephrase_on_failure
        self._max_attempts = max(1, int(max_attempts))

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.COMMAND, access=Access.READ)
        self._bb.register_key(bb_keys.PLAN, access=Access.WRITE)
        self._bb.register_key(bb_keys.PLAN_INDEX, access=Access.WRITE)
        self._bb.register_key(bb_keys.STATE_LOG, access=Access.READ)
        self._bb.register_key(bb_keys.LAST_FAILURE, access=Access.WRITE)
        self._bb.register_key(bb_keys.CORRECTION_COUNT, access=Access.WRITE)

    def _call_llm(self, user_prompt: str, temperature: float) -> Tuple[Optional[dict], Optional[str]]:
        """One LLM round-trip → (parsed JSON dict, error string). Exactly one is set."""
        try:
            # A fresh random seed every call makes the provider treat this as a
            # brand-new request and sample anew — it defeats any response
            # caching / dedup of identical consecutive requests (the "re-issue
            # the same command and it refuses again until you say something else"
            # symptom). Combined with the per-call nonce in the prompt, no two
            # planning calls are ever identical.
            resp = self._client_oai.chat.completions.create(
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
            msg = resp.choices[0].message
            raw = (getattr(msg, "content", None) or "").strip()
            if not raw:
                raw = (getattr(msg, "reasoning", None) or "").strip()
            parsed = _extract_json_object(raw)
            if parsed is None:
                return None, ("your reply was not parseable JSON "
                              f"(content was {'empty' if not raw else 'non-JSON'}). "
                              "Reply with ONLY the JSON object.")
            return parsed, None
        except Exception as exc:  # noqa: BLE001 — surface anything for retry
            return None, f"LLM call error: {exc!r}"

    def initialise(self):
        self._plan_result = None
        self._fell_back = False
        self._attempts_used = 0
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
        if not self._rephrase_on_failure:
            # Initial plan for a fresh command: plan from the command ALONE.
            # Never carry session history (a previous command's completed/failed
            # steps) into a new command's plan — that cross-command bleed is what
            # made a re-issued command behave differently from the first issue.
            seed_failure = None
            state_log = []

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
                parsed, err = self._call_llm(prompt, temperature)
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
                    continue
                ok, reason = validate_plan(
                    cleaned, command or "", known_actions,
                    known_locations=known_loc_arg,
                )
                if not ok:
                    last_reason = reason
                    print(f"[planner] attempt {attempt+1}/{max_attempts} REJECTED: {reason}")
                    continue
                # Accepted.
                self._attempts_used = attempt + 1
                self._plan_result = cleaned
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
        # Clear the stale failure so it isn't fed into the next planning call.
        self._bb.set(bb_keys.LAST_FAILURE, "", overwrite=True)
        try:
            count = self._bb.get(bb_keys.CORRECTION_COUNT)
        except KeyError:
            count = 0
        self._bb.set(bb_keys.CORRECTION_COUNT, count, overwrite=True)
        tag = " [FALLBACK]" if self._fell_back else ""
        self.feedback_message = (
            f"Planned {len(plan)} step(s) in {self._attempts_used} attempt(s){tag}: "
            f"{[s['action'] for s in plan]}"
        )
        return Status.SUCCESS

    def terminate(self, new_status):
        self._thread = None


# ---------------------------------------------------------------------------
# Step pop / parameter materialisation
# ---------------------------------------------------------------------------

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
        self._materialise_params(action, params)
        self._bb.set(bb_keys.PLAN_INDEX, index + 1, overwrite=True)
        self.feedback_message = f"step {index+1}/{len(plan)}: {action}({params})"
        return Status.SUCCESS

    def _resolve_pose(self, name: Any) -> Optional[PoseStamped]:
        """Resolve a location name to a PoseStamped, or None if unknown.

        Resolution order: start-position aliases (pose captured at command
        start) → known map locations (exact then case-insensitive) →
        runtime-recorded dynamic-location registry (labels fixed by an earlier
        record_position step).
        """
        if not name:
            return None
        key = str(name).lower()
        if key in START_LOCATION_ALIASES:
            try:
                return self._bb.get(bb_keys.START_POSE)
            except KeyError:
                return None
        if name in KNOWN_LOCATIONS:
            return KNOWN_LOCATIONS.get(name)
        for known_name, pose in KNOWN_LOCATIONS.items():
            if known_name.lower() == key:
                return pose
        try:
            registry = self._bb.get(bb_keys.DYNAMIC_LOCATIONS) or {}
        except KeyError:
            registry = {}
        return registry.get(key)

    def _materialise_params(self, action: str, params: Dict[str, Any]) -> None:
        """Translate the LLM's params into the BB keys the small trees consume."""
        # Location → PoseStamped lookup (see _resolve_pose for the order).
        loc_name = params.get("location") or params.get("recipient_location")
        if loc_name:
            self._bb.set(bb_keys.TARGET_LOCATION, loc_name, overwrite=True)
            pose = self._resolve_pose(loc_name)
            if pose is not None:
                self._bb.set(bb_keys.TARGET_POSE, pose, overwrite=True)

        # search_object: resolve the room's sweep spots into SEARCH_POSE_0..N.
        # location is optional — when omitted, fall back to the object's default
        # location (DEFAULT_OBJECT_LOCATIONS). A location with no explicit
        # search-spot list sweeps just itself. Unused slots are cleared to None
        # so the sweep guards them out.
        if action == "search_object":
            loc = params.get("location")
            obj = params.get("object")
            if not loc and obj:
                loc = DEFAULT_OBJECT_LOCATIONS.get(str(obj).lower())
            if loc:
                self._bb.set(bb_keys.TARGET_LOCATION, loc, overwrite=True)
            spot_names = ROOM_SEARCH_SPOTS.get(str(loc).lower(), [loc]) if loc else []
            for i, search_key in enumerate(SEARCH_POSE_KEYS):
                pose = self._resolve_pose(spot_names[i]) if i < len(spot_names) else None
                self._bb.set(search_key, pose, overwrite=True)

        # record_position: stash the label so the small tree registers the
        # captured pose under it.
        if action == "record_position":
            self._bb.set(
                bb_keys.CURRENT_DYNLABEL,
                str(params.get("label") or "").strip(),
                overwrite=True,
            )

        # Object → vision prompt + name
        obj_name = params.get("object")
        if obj_name:
            self._bb.set(bb_keys.TARGET_OBJECT_NAME, obj_name, overwrite=True)
            prompt = KNOWN_OBJECT_PROMPTS.get(obj_name, obj_name)
            self._bb.set(bb_keys.TARGET_OBJECT_PROMPT, prompt, overwrite=True)

        # Person descriptor
        person = params.get("descriptor") or params.get("person") or params.get("recipient")
        if person:
            self._bb.set(bb_keys.TARGET_PERSON_PROMPT, person, overwrite=True)

        # announce: a literal ``text`` is spoken as-is; an announce with NO text
        # reports the latest gathered result buffered in REPORT_INFO by the most
        # recent count / describe_person / ask_person / vlm_fallback. This is the
        # generalized "go gather X then come back and tell ME X" reporter that
        # replaced the per-result report_* actions.
        if action == "announce":
            text = params.get("text") or params.get("message") or ""
            if not text:
                try:
                    text = str(self._bb.get(bb_keys.REPORT_INFO) or "")
                except KeyError:
                    text = ""
            self._bb.set(bb_keys.ANNOUNCE_TEXT, text, overwrite=True)

        # ask_person carries the literal question to speak
        if action == "ask_person":
            question = params.get("question") or params.get("text") or "Please tell me."
            self._bb.set(bb_keys.ASK_QUESTION, question, overwrite=True)

        # fallbacks carry the literal question to answer
        if action == "vlm_fallback":
            self._bb.set(
                bb_keys.VLM_QUESTION,
                params.get("question") or params.get("text") or "Describe what you see.",
                overwrite=True,
            )
        if action == "llm_fallback":
            self._bb.set(
                bb_keys.LLM_QUESTION,
                params.get("question") or params.get("text") or "",
                overwrite=True,
            )


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

    def update(self):
        try:
            action = self._bb.get(bb_keys.CURRENT_ACTION)
            params = self._bb.get(bb_keys.CURRENT_PARAMS)
        except KeyError:
            action, params = "unknown", {}
        try:
            log = self._bb.get(bb_keys.STATE_LOG)
        except KeyError:
            log = None
        if log is None:
            log = []
        verdict = "SUCCEEDED" if self._succeeded else "FAILED"
        log.append(f"{action}({params}) {verdict}")
        self._bb.set(bb_keys.STATE_LOG, log, overwrite=True)
        if not self._succeeded:
            self._bb.set(
                bb_keys.LAST_FAILURE,
                f"{action}({params}) failed",
                overwrite=True,
            )
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
        return Status.SUCCESS


# ---------------------------------------------------------------------------
# Tree composition
# ---------------------------------------------------------------------------

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
            out = Path(self._out_dir) / f"gpsr_plan_{stamp}_{safe_slug(command)}.py"
            write_plan_module(command, plan, out)
            self.feedback_message = f"wrote plan module: {out}"
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

    def update(self):
        from datetime import datetime
        from .plan_viz import render_plan_tree
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
            name = f"gpsr_tree_{stamp}_{safe_slug(command)}"
            artifacts = render_plan_tree(plan, ACTION_FACTORIES, self._out_dir, name)
            self.feedback_message = (
                f"rendered tree: {artifacts.get('png') or artifacts.get('dot')}"
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
    for action_name, factory in ACTION_FACTORIES.items():
        branch = py_trees.composites.Sequence(f"branch:{action_name}", memory=True)
        branch.add_child(BtNode_ActionRouter(action_name))
        branch.add_child(factory())
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

    # Self-correction triggered only when the dispatch+monitor returned FAILURE.
    correction = create_self_correction(max_corrections=max_corrections)

    dispatch_or_correct = py_trees.composites.Selector(
        "dispatch_or_correct", memory=False,
        children=[monitor_then_log, correction],
    )

    step = py_trees.composites.Sequence("execute_step", memory=True)
    step.add_child(pop)
    step.add_child(dispatch_or_correct)
    return step


def create_execute_command(
    max_steps: int = 25,
    max_corrections: int = 3,
    emit_plan_dir: Optional[str] = None,
    announce_plan: bool = True,
) -> py_trees.behaviour.Behaviour:
    """Plan once, then run the step loop until the plan is exhausted.

    The plan-loop relies on ``BtNode_PopNextAction`` returning FAILURE when
    nothing is left to do. That failure bubbles up through ``Repeat`` (which
    aborts on any child failure), and the outer ``FailureIsSuccess`` decorator
    converts it back to SUCCESS so the parent tree treats command completion as
    normal success.

    If ``emit_plan_dir`` is given, a standalone re-runnable ``.py`` of the
    planned tree is written there right after planning (check-after-run).

    If ``announce_plan`` (default), the robot speaks the full planned step
    sequence aloud right after planning, before executing it.
    """
    plan = BtNode_PlanActions(name="plan initial")

    loop_body = create_execute_one_step(max_corrections=max_corrections)
    loop = py_trees.decorators.Repeat(
        name="step loop",
        child=loop_body,
        num_success=max_steps,
    )
    loop_done_ok = py_trees.decorators.FailureIsSuccess(
        "plan-exhausted = done", loop,
    )

    root = py_trees.composites.Sequence("execute_command", memory=True)
    # Hard per-command reset of execution state, right before planning. This runs
    # every time a command is executed (independent of create_orchestrator_init),
    # so a previous command's STATE_LOG / correction count / failure can never
    # bleed into this command's plan or self-correction.
    root.add_child(BtNode_BlackboardSet("reset state_log", bb_keys.STATE_LOG, []))
    root.add_child(BtNode_BlackboardSet("reset correction", bb_keys.CORRECTION_COUNT, 0))
    root.add_child(BtNode_BlackboardSet("reset last_failure", bb_keys.LAST_FAILURE, ""))
    root.add_child(plan)
    if emit_plan_dir is not None:
        root.add_child(BtNode_GeneratePlanFile(out_dir=emit_plan_dir))
    if announce_plan:
        # Speak the full plan ("Here is my plan. First... Then... Finally...")
        # right after planning so the operator hears it before execution starts.
        root.add_child(create_announce_plan())
    root.add_child(loop_done_ok)
    return root


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
