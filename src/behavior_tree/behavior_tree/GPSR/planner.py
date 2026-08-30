"""Two-layer GPSR planner — decoupled, callable orchestrator.

The GPSR orchestrator is split into two planning layers:

- TOP LAYER  : ``split_command`` turns a free-form natural-language command
  into an ordered list of self-contained *targets* ("fetch Susan a coke, she is
  in the living room" -> ["grab a coke", "find Susan in the living room",
  "deliver the coke to Susan"]).
- LOWER LAYER: ``plan_target`` plans ONE target into an action plan and
  pre-builds its executing subtree; ``request_plan_all`` plans every target of
  a command IN PARALLEL (one daemon thread each, saving wall-clock).

``GPSRPlanner`` is a plain Python object — NOT a behaviour — that the
orchestrator invokes repeatedly (once per slot, once per target, once per
replan). It never holds a tree or node reference.

Threading contract:
- Planner threads build subtrees and store them in the Python cache ONLY. They
  NEVER write the Blackboard — BB writes happen on the executor thread in the
  bridge nodes' ``update()``.
- LLM clients are created per-thread (``openai.OpenAI`` is not guaranteed
  thread-safe, and parallel planning spawns many threads).
- A subtree is built once and swapped in once — never reused.
"""

from __future__ import annotations

import copy
import json
import random
import re
import textwrap
import threading
import uuid
from typing import Any, Dict, List, Optional, Tuple

import py_trees
import openai

from .config import (
    OPENAI_API_KEY,
    OPENAI_MODEL,
    OPENAI_TEMPERATURE,
    OPENAI_MAX_TOKENS,
    LLM_TIMEOUT_S,
)
from .modifiable_nodes import (
    TEMPLATES,
    apply_modifications,
    group_modifications_by_step,
    validate_plan_modifications,
)
from .planner_validators import (
    validate_plan, validate_dag, uncovered_postcondition_reason, established_predicates,
    _norm_loc, DEFAULT_CATEGORY_WORDS,
)
from .validators import apply_fact_transitions, canonical_fact, parse_fact
from .small_trees import (
    ACTION_FACTORIES,
    bb_keys,
    BtNode_AnnounceFromBB,
    get_small_tree_roles,
)
from .supervision.runtime import get_default_supervisor, wrap_action_factory
from .telemetry import get_default_telemetry
from .action_contracts import (
    ACTION_CONTRACTS,
    IDENTICAL_PLAN_ERROR_PREFIX,
    UNRECOVERABLE_ERROR_PREFIX,
    established_facts,
    render_self_satisfied_rule,
    self_established_facts,
    self_navigating_destinations,
)
from .orchestrator import (
    SYSTEM_PROMPT,
    KNOWN_LOCATIONS,
    KNOWN_OBJECT_PROMPTS,
    KNOWN_OBJECT_NAMES,
    DEFAULT_OBJECT_LOCATIONS,
    ACTION_CATALOGUE_DESCRIPTION,
    START_LOCATION_ALIASES,
    BtNode_LogStepResult,
    BtNode_MaterialiseStep,
    BtNode_SupervisorBarrier,
    BtNode_TargetPreconditionCheck,
    BtNode_TargetPostconditionCheck,
    _build_planner_user_prompt,
    _clean_plan,
    _clean_plan_core,
    _offline_planner_enabled,
    _extract_json_object,
    _fallback_plan,
)


# ---------------------------------------------------------------------------
# Top-layer prompt: NL command -> ordered list of self-contained targets.
# ---------------------------------------------------------------------------

TOP_LAYER_SYSTEM_PROMPT = textwrap.dedent("""
    You are the task-splitting module of a household service robot competing in
    RoboCup@Home GPSR. A natural-language command may ask for several things at
    once. Split it into an ordered list of STRUCTURED targets — one per clause
    the robot can work on — carrying the context the lower-layer planner needs
    to plan each target WITHOUT repeating earlier work.

    Respond with JSON only, in this exact shape:
    {
      "reasoning": "<short explanation of the split>",
      "targets": [
        {
          "id": "<stable unique target id>",
          "desc": "<standalone NL instruction>",
          "object": "<concrete object noun if this clause names one, else \"\">",
          "location": "<assigned location if this clause names one, else \"\">",
          "depends_on": ["<prior target id>", ...],
          "preconditions": ["<closed-vocabulary fact>", ...],
          "postconditions": ["<closed-vocabulary fact>", ...]
        },
        ...
      ]
    }

    Split rules:
    1. Keep each target's desc self-contained: resolve pronouns and
       cross-references into concrete nouns. Example: "fetch Susan a coke, she
       is in the living room" -> ["grab a coke", "find Susan in the living
       room", "deliver the coke to Susan"]. The object "coke" is named in every
       target it touches.
    2. Preserve the order in which the command gives the clauses — do not
       reorder the work.
    3. Fill in "object" with the concrete object a clause names, and
       "location" with the location a clause ASSIGNS to it. When a later clause
       refers back to an object/location established earlier (e.g. "take the
       Fanta" after "find a fanta in the office"), resolve that reference here:
       set object=Fanta, location=office, and include the earlier target's ID in
       the depends_on LIST. This is how the assigned location survives from the
       command down to the lower layer.
    4. Set "depends_on" to a list of earlier target IDs that must complete
       BEFORE this one because they establish something this target needs.
       Multiple dependencies are allowed; use [] when independent. Conditions
       use only this closed vocabulary and exact arity:
       at_robot(location), object_seen(object), person_found(person),
       held(object), placed(object,location), delivered(object,recipient),
       counted(object), answered(question). Validators, not the LLM, determine
       whether conditions are true. Empty condition lists are allowed. The
       operator's own location (where the robot received the command) is
       "start_position" — always write at_robot(start_position) for a
       "bring/tell/report to ME" postcondition, never at_robot(me) or
       another wording.
    5. Keep each target as close to the original wording as possible; do not
       invent actions, locations, or details not present in the command.
    6. Split by DISTINCT EXECUTABLE JOBS, not by sentence connectives. A
       target is one job the robot can execute end-to-end; count the jobs the
       command names and emit one target per job, EVEN when there is no
       "then"/"and". "Bring Susan the red jacket she left in the office" names
       three jobs with no connective -> ["get the red jacket from the office",
       "find Susan", "deliver the red jacket to Susan"].
    7. Split executable state transitions even within one relocation job.
       "Move the plant from the kitchen to the balcony" MUST produce three
       targets: acquire/grasp with postcondition held(plant); transport/goto
       balcony depending on that target with precondition held(plant) and
       postcondition at_robot(balcony); place depending on transport with
       precondition held(plant) and postcondition placed(plant,balcony).
       __SELF_SATISFIED_RULE__ Preconditions list only facts an EARLIER
       target must have established. A "then"/"and" separates targets when
       it joins distinct work: "grab a coke from the kitchen, then take it to
       Susan in the living room" has two end-states.
    8. Enumeration of SEVERAL separately-named concrete objects is the one
       intra-verb-phrase case that does split: "grab a coke, some chips and a
       lemonade from the kitchen" -> three fetch targets, each carrying
       object=... and location=kitchen. A single named whole ("breakfast",
       "the mail") stays one target.
    9. Never return an empty list. If the command is genuinely one job, return
       a single-element list containing the command itself.
    10. Every postcondition must be meaningful and checkable under the closed
        vocabulary above. If a clause cannot be resolved into concrete work,
        keep it as a target anyway — the lower layer will plan an announcement.
    11. A command that COUNTS or ASKS something (any target postcondition
        counted(...)/answered(...)) must end with a target that reports the
        result to the operator, UNLESS a target's own wording already says so
        ("tell me", "report", ...). "count how many mugs are on the shelf and
        then on the side table" MUST get a final target like {"desc": "report
        the result to the operator", "depends_on": [the counting targets' ids],
        "preconditions": [the counted(...) facts], "postconditions":
        ["at_robot(start_position)", "answered(...)"]} — a plan that gathers a
        count/answer without ever speaking it is incomplete.
    12. A PERSON is never held/placed/delivered like an object: a target
        about a person (named or described, e.g. "the person wearing a gray
        jacket") must use postconditions person_found(<p>)/at_robot(<dest>)
        (find_person + guide), never held(...)/placed(...)/delivered(...).
    13. held(x)/placed(x,l)/delivered(x,p) are for PHYSICAL OBJECTS only: to
        TELL someone information (a country, a gesture, a count, an answer),
        use answered(<what>), never held/placed/delivered with a non-object
        argument like held(country) or delivered(gesture,person).
""").strip().replace("__SELF_SATISFIED_RULE__", render_self_satisfied_rule())


# ---------------------------------------------------------------------------
# Lower-layer prompt: ONE target -> {"plan": [...]}.
# The full single-command action catalogue + all 18 hard planning rules stay
# exactly valid for a single target, so the lower layer reuses SYSTEM_PROMPT
# (the per-call user prompt already scopes the context to the one target).
# ---------------------------------------------------------------------------

LOWER_LAYER_SYSTEM_PROMPT = textwrap.dedent("""
    You are planning ONE target of a larger GPSR command. The target below is a
    single goal; plan only the steps needed to achieve it, given the full-command
    context in the user prompt.

    Cross-target rules:
    - A "Location context" line names the location the top layer ASSIGNED to the
      target's object/person. Plan against THAT location — never fall back to an
      object's predefined default location when an assigned location exists.
    - "Prior targets" are earlier clauses of the same command, already planned
      and already (or about to be) executed. Start from the robot's state AFTER
      they complete. Do NOT repeat what they do: no re-navigating to a location
      a prior target already reached, no re-finding/re-grasping an object a
      prior target already obtained (the robot is already holding it), no
      re-finding a person a prior target already located and is standing next to.
    - Runtime gates enforce target preconditions and postconditions. Plan only
      the missing state delta. On a pre/postcondition failure, inspect the
      verified facts below and do not blindly repeat successful earlier work.
    - If a prior target will have already delivered the result to the operator,
      still emit any remaining steps so the plan is complete — but skip the
      repeated fetch/grasp.
    - A room named in the command (e.g. "the living room", "the kitchen") is
      legitimate even if it has no hardcoded waypoint in the known-location
      list: plan ``record_position(label=<room>)`` then ``goto(location=<room>)``
      so the pose is captured at runtime. NEVER refuse a step with
      "cannot find a known location" when the command names a real room or
      place — record it instead.
    - If part of the target is impossible, still emit the doable steps and
      finish with ``announce(text=...)`` explaining what you could not do —
      but that announce is a REFUSAL, not an answer: set
      ``"acknowledgement": true`` in its ``params`` (alongside ``text``).
      This is DIFFERENT from the "tell ME" reporting pattern above (a
      text-less ``announce()`` after a real gathering step, or
      ``announce(text=...)`` stating an actual result you gathered) — those
      never carry ``acknowledgement``. Only a spoken apology / "I could not
      ..." does.

    OPTIONAL small-tree modifications: when a step's generic small tree cannot
    express the behaviour the command needs, you may attach a TYPED modification
    to that step instead of inventing a new action. This is the ONLY way to
    specialise a tree. Return a top-level ``modifications`` array alongside
    ``plan``:
    {
      "modifications": [
        {
          "action": "<the action this modifies, e.g. find_person>",
          "template": "<one of the templates below>",
          "target_node_id": "<a node id from that action's serialized tree>",
          "step_index": <optional integer; REQUIRED when the plan has >1 step
                         of this action>,
          "params": { "<template-specific params>" },
          "reason": "<short why — becomes part of the audit record>"
        }
      ]
    }

    Allowed templates (closed set — never invent others):
    - "attribute-person-specialist" (action find_person): params
      {"gate": "<attribute, e.g. red jacket>", "prompt": "<optional pinned
      vision prompt>"} — add a descriptor-gated specialist branch so a
      person described by an attribute (not just a name) is detected.
    - "person-specialist" (action find_person): params
      {"name": "<named person>"} — pin the generalist scan to a named person.
    - "vlm-template" (action count ONLY): params
      {"question_template": "<must contain the {value} placeholder>"} — swap
      the VLM question the count fallback asks. For vlm_fallback, the question
      is set directly via vlm_fallback(question="...") — no modification
      needed or accepted.
    - "announce-text" (actions whose tree has a literal-message announce):
      params {"text": "<spoken text>"} — change a spoken announcement.
    - "pan-tilt-sweep" (actions find_person / find_object / describe_person):
      params {"pan_deg": [floats], "tilt_deg": [floats]} — override the scan
      sweep ranges. Each list is 1..5 values; pan_deg values must be within
      [-120, 120] degrees, tilt_deg values within [-45, 60] degrees.
    - "search-spots" (action search_object): params {"capacity": <int 1..32>}
      — scale the room sweep capacity.

    The ``target_node_id`` must be one of the concrete ids listed under
    "Modifiable tree targets" in the user prompt — the id of the node in the
    target action's serialized small tree that carries the role the template
    applies to (e.g. ``small/find_person/root/3`` for find_person's sweep).
    The "Modifiable tree targets" block is authoritative; only emit an id you
    see there. If a modification is not applicable, OMIT it — the generic tree
    still runs.

    WHEN to modify: a person described by an ATTRIBUTE or GESTURE — "the
    person wearing a red jacket", "the man in the blue shirt", "the person
    raising their left arm" — is not handled well by the generic find_person
    tree, which only special-cases waving. For such a descriptor, emit an
    ``attribute-person-specialist`` modification pinning the attribute into the
    vision prompt. A bare NAME needs no modification.
""").strip() + "\n\n" + SYSTEM_PROMPT


# ---------------------------------------------------------------------------
# Offline mock (full-mock preset): deterministic, network-free planning.
# ---------------------------------------------------------------------------

def _normalise_targets(raw: List[Any]) -> List[Dict[str, Any]]:
    """Normalize a list of top-layer targets to the canonical schema."""
    if not isinstance(raw, list):
        return []
    retained = []
    for item in raw or []:
        if isinstance(item, dict):
            desc = str(item.get("desc") or "").strip()
            if not desc:
                continue
            retained.append((item, desc))
        elif isinstance(item, str) and item.strip():
            retained.append(({}, item.strip()))

    ids = [
        (item.get("id").strip() if isinstance(item.get("id"), str) and item.get("id").strip()
         else f"t{i}")
        for i, (item, _desc) in enumerate(retained)
    ]

    def normalize_deps(value: Any) -> list[Any]:
        if value is None or value == -1:
            return []
        values = value if isinstance(value, (list, tuple)) else [value]
        result = []
        seen = set()
        for dep in values:
            if isinstance(dep, int) and not isinstance(dep, bool):
                dep_value = ids[dep] if 0 <= dep < len(ids) else dep
            elif isinstance(dep, str):
                dep_value = dep.strip()
            else:
                dep_value = dep
            if isinstance(dep_value, str) and dep_value not in seen:
                seen.add(dep_value)
            elif isinstance(dep_value, str):
                continue
            result.append(dep_value)
        return result

    def normalize_conditions(value: Any) -> Any:
        if not isinstance(value, (list, tuple)):
            return value if value is not None else []
        result = []
        for item in value:
            if isinstance(item, str):
                item = item.strip()
                result.append(item)
            else:
                result.append(item)
        return result

    normalized = []
    for i, (item, desc) in enumerate(retained):
        normalized.append({
            "id": ids[i],
            "desc": desc,
            "object": str(item.get("object") or "").strip(),
            "location": str(item.get("location") or "").strip(),
            "depends_on": normalize_deps(item.get("depends_on")),
            "preconditions": normalize_conditions(item.get("preconditions")),
            "postconditions": normalize_conditions(item.get("postconditions")),
        })

    # Per the split prompt contract, a target precondition is only meaningful
    # as cross-target sequencing: it must be established by an EARLIER
    # target's postconditions AND still be alive at this point in the
    # command's fact timeline. J1 (round-3 adversarial review, H4): the
    # ledger is not a monotonic union -- ``validators.apply_fact_transitions``
    # applies the SAME retraction rules the runtime gate uses (placing/
    # delivering an object retracts ``held(object)``; re-holding it retracts
    # a stale ``placed``/``delivered``), so a precondition an ancestor
    # target established but a LATER ancestor then retracted is not an
    # acceptable precondition either -- the plan must re-establish it, not
    # rely on a fact that is no longer true. Unparseable conditions are left
    # alone -- ``_validate_target_contract`` rejects those with a retry.
    ledger: list[str] = []
    retracted_by: dict[str, tuple[str, str]] = {}
    for target in normalized:
        alive = set(ledger)
        preconditions = target.get("preconditions")
        if isinstance(preconditions, list):
            kept: list[Any] = []
            dropped: list[Any] = []
            for condition in preconditions:
                fact = None
                if isinstance(condition, str):
                    fact, _error = parse_fact(condition)
                if fact is None:
                    kept.append(condition)
                    continue
                canonical = canonical_fact(fact)
                if canonical in alive:
                    kept.append(condition)
                elif canonical in retracted_by:
                    retractor_id, retract_fact = retracted_by[canonical]
                    print(
                        f"[split] target {target['id']} precondition {canonical} "
                        f"retracted by {retractor_id} {retract_fact} — dropped "
                        "(plan must re-establish)"
                    )
                    dropped.append(condition)
                else:
                    dropped.append(condition)
            if dropped:
                print(f"[split] target {target['id']}: dropped unestablishable precondition(s): {dropped}")
            target["preconditions"] = kept

        postconditions = target.get("postconditions")
        if isinstance(postconditions, list):
            post_texts = [str(item) for item in postconditions if isinstance(item, str)]
            new_ledger = apply_fact_transitions(ledger, post_texts)
            removed = [item for item in ledger if item not in new_ledger]
            for removed_fact in removed:
                retract_text = _retracting_addition(removed_fact, post_texts) or "?"
                retracted_by[removed_fact] = (target["id"], retract_text)
            ledger = new_ledger

    return normalized


def _retracting_addition(removed_fact: str, additions: List[str]) -> Optional[str]:
    """Find which of ``additions`` caused ``removed_fact`` to be retracted.

    Mirrors ``validators.apply_fact_transitions``'s own retraction rules
    (place/deliver retract held(object); re-holding retracts a stale
    placed/delivered(object,..); a new at_robot(..) retracts the old one) —
    used only to make the ``_normalise_targets`` drop log legible.
    """
    removed, _ = parse_fact(removed_fact)
    if removed is None:
        return None
    for text in additions:
        fact, _error = parse_fact(text)
        if fact is None:
            continue
        canonical = canonical_fact(fact)
        if removed.predicate == "held" and fact.predicate in {"placed", "delivered"} and fact.args[0] == removed.args[0]:
            return canonical
        if removed.predicate in {"placed", "delivered"} and fact.predicate == "held" and fact.args[0] == removed.args[0]:
            return canonical
        if removed.predicate == "at_robot" and fact.predicate == "at_robot":
            return canonical
    return None


# X1 (round-3 fix review, source-pinned tier0 sweep): generic words that
# name "some physical thing" without a specific noun -- a fact like
# held(item) or delivered(kitchen_item, emma) is still a claim about an
# OBJECT, just an unresolved/categorical one, unlike delivered(country, ...)
# or delivered(gesture, ...) (a piece of INFORMATION, never a graspable
# thing). Checked per-token so a compound arg like "kitchen_item" (one of
# its words is "item") still counts as an object reference.
_GENERIC_OBJECT_WORDS = frozenset({"object", "it", "item", "items", "thing", "things"})


def _is_physical_object_arg(value: str) -> bool:
    """True when ``value`` (a normalized held/placed/delivered fact arg)
    names something a robot can actually grasp -- a known arena object, a
    category word, or a generic object noun -- as opposed to a piece of
    INFORMATION (a country, a gesture, a colour, ...).
    """
    normalized = str(value).strip().lower()
    if not normalized:
        return False
    if normalized in KNOWN_OBJECT_NAMES:
        return True
    tokens = [t for t in re.split(r"[_\s]+", normalized) if t]
    return any(
        token in KNOWN_OBJECT_NAMES or token in DEFAULT_CATEGORY_WORDS
        or token in _GENERIC_OBJECT_WORDS
        for token in tokens
    )


def _reject_non_object_delivery(
    targets: List[Dict[str, Any]],
) -> Optional[str]:
    """X1 (round-3 fix review, source-pinned tier0 sweep): held/placed/
    delivered are for PHYSICAL OBJECTS only. The split produced
    ``delivered(country, person_raising_their_left_arm)`` and
    ``delivered(gesture, person_at_sofa)`` for "tell <info> to <person>"
    commands, and the coverage validator then forced a physical ``deliver``
    step the lower layer could never plan (there is no country/gesture to
    grasp) -- returns a rejection reason (fed back into the split retry
    loop), or None when every held/placed/delivered fact names an object.
    """
    for target in targets:
        for field in ("preconditions", "postconditions"):
            for condition in target.get(field) or []:
                if not isinstance(condition, str):
                    continue
                fact, _err = parse_fact(condition)
                if fact is None or fact.predicate not in ("held", "placed", "delivered"):
                    continue
                obj_arg = fact.args[0]
                if not _is_physical_object_arg(obj_arg):
                    return (
                        f"target {target.get('id')!r} {field} {condition!r}: "
                        "delivered/placed/held are for physical objects; to "
                        "tell information use answered(<what>)"
                    )
    return None


def _validate_target_contract(targets: List[Dict[str, Any]]) -> Tuple[bool, Optional[str]]:
    ok, reason = validate_dag(targets)
    if not ok:
        return False, reason
    for index, target in enumerate(targets):
        for field in ("preconditions", "postconditions"):
            conditions = target.get(field)
            if not isinstance(conditions, list) or any(not isinstance(item, str) for item in conditions):
                return False, f"target {index} {field} must be a list of strings"
            for condition in conditions:
                _fact, error = parse_fact(condition)
                if error:
                    return False, f"target {index} {field} invalid fact {condition!r}: {error}"
    non_object_reason = _reject_non_object_delivery(targets)
    if non_object_reason is not None:
        return False, non_object_reason
    return True, None


_REPORT_KEYWORDS_RE = re.compile(
    r"\b(tell|report|inform|let\s+me\s+know)\b", re.IGNORECASE
)


def _append_report_target_if_needed(
    targets: List[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    """J8 (round-3 adversarial review, edge corpus e05/e06): a command that
    only COUNTS or ASKS something never speaks the result.

    "count how many mugs are on the shelf and then on the side table"
    planned ``[goto, count, goto, count]``; "go to the laundry desk and
    count the mugs on the side table" planned ``[goto, goto, count]`` --
    the counts are gathered but never announced, because no target's
    postconditions call for it.

    Deterministic split-time post-check: if ANY target's postconditions
    establish ``counted(...)``/``answered(...)`` and NO target's desc
    already signals a report to the operator ("tell me", "report", ...),
    append a synthetic target whose plan will be ``goto(start_position),
    announce()`` (a text-less announce reads the latest gathered count/
    answer off the blackboard -- see ``materialise_params``'s ``announce``
    branch). Depends on every target that established one of those facts;
    those facts become its own preconditions so the runtime gate demands
    them before the announce.
    """
    if any(_REPORT_KEYWORDS_RE.search(str(t.get("desc") or "")) for t in targets):
        return targets
    reporting_facts: List[str] = []
    reporting_ids: List[str] = []
    for target in targets:
        target_reports = False
        for condition in target.get("postconditions") or []:
            if not isinstance(condition, str):
                continue
            fact, _err = parse_fact(condition)
            if fact is not None and fact.predicate in {"counted", "answered"}:
                reporting_facts.append(condition)
                target_reports = True
        if target_reports:
            reporting_ids.append(str(target.get("id")))
    if not reporting_facts:
        return targets
    new_id = f"t{len(targets)}"
    report_target = {
        "id": new_id,
        "desc": "report the result to the operator",
        "object": "",
        "location": "start_position",
        "depends_on": reporting_ids,
        "preconditions": reporting_facts,
        "postconditions": ["at_robot(start_position)", "answered(the requested information)"],
    }
    print(f"[split] appended report target {new_id} (depends_on {reporting_ids})")
    return targets + [report_target]


# J9 (round-3 adversarial review, edge corpus guideClothPrs): mirrors
# bench/corpus.py's ``Knowledge.names`` -- the canonical GPSR arena name
# list this repo's corpus generator draws from -- kept here as a plain
# set (no import of bench/, which depends on planner.py, not the reverse).
_ARENA_PERSON_NAMES = frozenset({
    "alex", "sarah", "john", "emma", "liam", "olivia",
})
_GENERIC_PERSON_WORDS_RE = re.compile(r"\b(person|persons|someone|guest)\b", re.IGNORECASE)


def _desc_names_a_person(desc: str) -> bool:
    # H-1 (round-3 fix review): callers now also feed a normalised fact arg
    # (parse_fact's ``_normalize`` joins words with "_", e.g.
    # "person_wearing_a_gray_jacket") -- \b doesn't break on "_" (a \w char),
    # so the generic-word regex would silently miss it. Un-join first.
    text = str(desc).replace("_", " ")
    if _GENERIC_PERSON_WORDS_RE.search(text):
        return True
    tokens = re.findall(r"[A-Za-z']+", text)
    return any(tok.lower() in _ARENA_PERSON_NAMES for tok in tokens)


def _reject_person_object_handling(
    targets: List[Dict[str, Any]],
) -> Optional[str]:
    """J9 (round-3 adversarial review, edge corpus guideClothPrs): a person
    is GUIDED, never grasped/placed/delivered like an object.

    Split produced ``['Take the person wearing a gray jacket from the sofa',
    'Take the person ... to the side_table', 'Place the person ... at the
    side_table']`` for a guide command -- object-handling verbs applied to a
    PERSON. Deterministic reject-and-retry: a target whose HANDLED thing --
    ``fact.args[0]`` of a held/placed/delivered postcondition (the object
    being grasped/placed/delivered, never a ``delivered(x,recipient)``
    recipient arg), or ``target["object"]`` -- names a person (an arena
    name, or a generic "person"/"someone"/"guest" word) is rejected, with a
    reason naming the correct pattern (find_person + guide).

    H-1 (round-3 fix review): the PREVIOUS version rejected on the desc
    naming a person anywhere, which also matches an ordinary
    "deliver spam to the person raising their left arm" target -- the desc
    names the RECIPIENT, not the handled thing, so it must not be rejected.
    Returns the rejection reason, or None when nothing is wrong.
    """
    for target in targets:
        desc = str(target.get("desc") or "")
        obj = str(target.get("object") or "")
        for condition in target.get("postconditions") or []:
            if not isinstance(condition, str):
                continue
            fact, _err = parse_fact(condition)
            if fact is None or fact.predicate not in {"held", "placed", "delivered"}:
                continue
            handled = fact.args[0] if fact.args else ""
            if _desc_names_a_person(handled) or _desc_names_a_person(obj):
                return (
                    f"target {target.get('id')!r} ({desc!r}): persons are "
                    "guided, not grasped: use find_person + guide "
                    "(postconditions person_found(<p>), at_robot(<dest>))"
                )
    return None


def _self_navigates_toward(target: Dict[str, Any], y: str) -> bool:
    """J11: does ``target``'s own postcondition establisher self-navigate to ``y``?

    A ``placed(_,Y)`` postcondition is established by ``place``, self-
    navigating to its own ``location`` param -- true when that Y matches. A
    ``delivered(_,r)`` postcondition is established by ``deliver``, self-
    navigating to ``recipient_location`` -- since the split has no lower-
    layer params yet, matched against the recipient arg itself (a location-
    shaped recipient, e.g. "bring the spam to the laundry_room"), the
    target's own top-layer-assigned ``location``, or the location's name
    appearing in the target's desc.
    """
    y_norm = _norm_loc(y)
    for condition in target.get("postconditions") or []:
        if not isinstance(condition, str):
            continue
        fact, _err = parse_fact(condition)
        if fact is None or len(fact.args) != 2:
            continue
        if fact.predicate == "placed":
            if _norm_loc(fact.args[1]) == y_norm:
                return True
        elif fact.predicate == "delivered":
            if _norm_loc(fact.args[1]) == y_norm:
                return True
            loc = str(target.get("location") or "")
            if loc and _norm_loc(loc) == y_norm:
                return True
            desc = str(target.get("desc") or "")
            if re.search(re.escape(y_norm).replace("_", r"[_\s]"), desc, re.IGNORECASE):
                return True
    return False


def _merge_transport_targets(targets: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """J11 (round-3 adversarial review, tier0 #1/#2): fold a pure-transport
    target into the self-navigating place/deliver target that follows it.

    Rule 7 splits "take X to Y then place it on Y" into a transport target
    (postcondition only ``at_robot(Y)``) followed by a place/deliver target
    at Y — but nothing stops the lower layer from planning the TRANSPORT
    target itself as the self-navigating place/deliver (003/004: both
    targets planned ``place``, a duplicate ``placed()``; 035: duplicate
    ``deliver()``; 029: "bring the spam to the laundry_room" planned as a
    recipient-less deliver — 4 rejections then fallback).

    When target N's postconditions are ONLY ``at_robot(Y)``, target N+1
    depends on N, and N+1's own postcondition establisher self-navigates to
    the SAME Y, N is pure duplicate work: drop it, keep N+1's contract
    (desc/pre/postconditions untouched), and re-point ``depends_on`` — N+1
    absorbs N's own dependencies, and anything else that depended on N now
    depends on N+1 instead.
    """
    merged = list(targets)
    changed = True
    while changed:
        changed = False
        for i in range(len(merged) - 1):
            target_n = merged[i]
            target_n1 = merged[i + 1]
            post_n = [c for c in target_n.get("postconditions") or [] if isinstance(c, str)]
            if len(post_n) != 1:
                continue
            fact_n, _err = parse_fact(post_n[0])
            if fact_n is None or fact_n.predicate != "at_robot":
                continue
            y = fact_n.args[0]
            n_id = str(target_n.get("id"))
            n1_deps = [str(d) for d in (target_n1.get("depends_on") or [])]
            if n_id not in n1_deps:
                continue
            if not _self_navigates_toward(target_n1, y):
                continue
            new_n1_deps = [d for d in n1_deps if d != n_id]
            for dep in target_n.get("depends_on") or []:
                dep = str(dep)
                if dep not in new_n1_deps:
                    new_n1_deps.append(dep)
            target_n1["depends_on"] = new_n1_deps
            n1_id = str(target_n1.get("id"))
            for other in merged:
                if other is target_n or other is target_n1:
                    continue
                deps = other.get("depends_on")
                if isinstance(deps, list) and n_id in [str(d) for d in deps]:
                    other["depends_on"] = [
                        n1_id if str(d) == n_id else d for d in deps
                    ]
            print(f"[split] merged transport target {n_id} into {n1_id} "
                  "(self-navigating place/deliver)")
            merged = [t for t in merged if t is not target_n]
            changed = True
            break
    return merged


def _log_split_acceptance(
    targets: List[Dict[str, Any]], slot: Optional[int] = None,
) -> None:
    """J14 (round-3 adversarial review, tier0 #7): audit an accepted split.

    The old ``[split] accepted on attempt N: [descs]`` line showed only
    descriptions -- never each target's preconditions/postconditions/
    depends_on/object/location, so no run could be audited for contract
    mistakes (a wrong precondition, a merge/reject that changed the shape,
    ...). Prints one line per target and emits a ``split.accepted``
    telemetry event ``{slot, targets: [...]}`` (kept by ``bench/events.py``
    so tier0/tier2 reports can show the full contract, not just the descs).
    """
    for t in targets:
        print(
            f"[split] {t.get('id')} {t.get('desc')!r} "
            f"pre={t.get('preconditions') or []} post={t.get('postconditions') or []} "
            f"depends_on={t.get('depends_on') or []} "
            f"object={t.get('object') or ''!r} location={t.get('location') or ''!r}"
        )
    telemetry = get_default_telemetry()
    if telemetry is None:
        return
    try:
        task_id = telemetry.task_id(slot + 1) if slot is not None else None
        telemetry.emit(
            "split.accepted",
            {"slot": slot, "targets": [dict(t) for t in targets]},
            task_id=task_id, phase="planning",
        )
    except Exception:  # noqa: BLE001 -- telemetry must never break the split
        pass


def _dependency_ancestor_targets(
    targets: List[Dict[str, Any]], index: int,
) -> List[Dict[str, Any]]:
    """Return declared dependency ancestors in stable target-list order."""
    if index <= 0:
        return []
    by_id = {str(target["id"]): target for target in targets[:index]}
    needed: set[str] = set()

    def visit(target_id: str) -> None:
        if target_id in needed:
            return
        needed.add(target_id)
        for dependency in by_id[target_id].get("depends_on", []):
            if dependency in by_id:
                visit(dependency)

    current = targets[index]
    for dependency in current.get("depends_on", []):
        visit(dependency)
    return [target for target in targets[:index] if target["id"] in needed]


# First action (in registry order) whose ``establishes`` names each predicate.
# goto is listed before search_object in the registry so at_robot -> goto.
_ESTABLISHER_FOR_PREDICATE: Dict[str, str] = {}
for _name, _contract in ACTION_CONTRACTS.items():
    for _tmpl in _contract.establishes:
        _pred = _tmpl.split("(", 1)[0]
        _ESTABLISHER_FOR_PREDICATE.setdefault(_pred, _name)


def _canonical_param_value(v: Any) -> str:
    """Canonical string form of one param value for ``_canonical_plan``.

    LOW-1 (round-2 review, task H, 2026-08-29): a plain ``str(v)`` treats a
    dict-valued param with a different key insertion order as "different"
    even though dict key order is never semantically meaningful here --
    ``json.dumps(v, sort_keys=True)`` normalises that (recursively, for any
    nested dicts too) while leaving list/tuple ELEMENT order untouched, since
    a list genuinely encodes an ordered sequence (e.g. ``pan_deg: [-90, 90]``
    vs ``[90, -90]`` are legitimately different sweeps) -- that half of the
    pattern is a known, non-blocking gap the review flagged, not something
    this can safely resolve without per-param semantics. Values ``json``
    can't serialise (rare for a plan param) fall back to plain ``str(v)``,
    matching the prior behaviour exactly.
    """
    try:
        return json.dumps(v, sort_keys=True, default=str)
    except TypeError:
        return str(v)


def _canonical_plan(
    plan: List[Dict[str, Any]],
    modifications: Optional[List[Dict[str, Any]]] = None,
) -> tuple:
    """Order/param-order-insensitive-within-a-step identity for a plan.

    Used to detect a regenerated replan that is IDENTICAL to the plan that
    just failed, so `plan_target` can reject it and force the model to
    actually change something instead of burning the replan budget re-running
    the same doomed steps.

    H1 (round-2 review, sim run 003, 2026-08-29): also folds in a canonical
    form of ``modifications`` (a RAW directive list -- the flat form
    ``parsed.get("modifications")`` carries, or `_flatten_modifications`'s
    view of the grouped form `_store` caches) -- a regenerated plan that
    repeats the same steps but attaches a genuinely different modification
    (e.g. a wider pan-tilt-sweep) is a legitimately different strategy and
    must NOT be rejected as identical. Each directive canonicalises to
    ``(template, target_node_id, sorted params items)`` -- the free-text
    ``reason`` is deliberately excluded (same reason, worded differently, is
    still the same strategy). ``modifications=None`` (the default) keeps
    every existing plan-only caller's identity unchanged.
    """
    plan_part = tuple(
        (str(s.get("action")),
         tuple(sorted((str(k), _canonical_param_value(v)) for k, v in (s.get("params") or {}).items())))
        for s in plan or []
    )
    mods_part = tuple(sorted(
        (
            str(m.get("template")),
            str(m.get("target_node_id")),
            tuple(sorted((str(k), _canonical_param_value(v)) for k, v in (m.get("params") or {}).items())),
        )
        for m in (modifications or [])
        if isinstance(m, dict)
    ))
    return (plan_part, mods_part)


def _alternatives_for_reason(failure_reason: str) -> str:
    """Tell the LLM what WOULD be acceptable when its regenerated plan repeats the failed one.

    Parses the first ``predicate(...)`` fact in ``failure_reason`` (e.g. from
    ``postcondition unmet: object_seen(kitchen item) (UNKNOWN)``) and names
    every registry action whose ``establishes`` covers that predicate, plus
    the generic escape hatches. Empty when no fact is found — the identical
    rejection then stands on its own.
    """
    match = re.search(r"([a-z_]+)\(([^()]*)\)", str(failure_reason or ""))
    if match is None:
        return ""
    fact, _err = parse_fact(match.group(0))
    if fact is None:
        return ""
    establishers = [
        name for name, c in ACTION_CONTRACTS.items()
        if any(t.split("(", 1)[0] == fact.predicate for t in c.establishes)
    ]
    parts = []
    if establishers:
        parts.append("try a DIFFERENT location or strategy with " + " / ".join(establishers))
    parts.append("or fall back to vlm_fallback / llm_fallback, or announce what you could not do")
    return "Acceptable alternatives: " + ", ".join(parts) + "."


def _establish_arg_names(action: str, predicate: str) -> List[str]:
    """Param names for ``action``'s ``establishes`` template naming ``predicate``.

    e.g. ``find_object`` establishes ``object_seen(object)`` -> ``["object"]``.
    Empty when ``action`` does not establish ``predicate`` at all.
    """
    contract = ACTION_CONTRACTS.get(action)
    if contract is None:
        return []
    for template in contract.establishes:
        pred, _, arglist = template.partition("(")
        if pred == predicate:
            return [a.strip() for a in arglist.rstrip(")").split(",") if a.strip()]
    return []


def _last_goto_location(failed_plans: List[List[Dict[str, Any]]]) -> Optional[str]:
    """The most recent ``goto(location=...)`` step across ``failed_plans``.

    Scans plans most-recent-first, and within a plan, steps most-recent-first
    — a target that keeps failing on an identical plan has an equivalent
    ``goto`` in its OWN just-failed plan too, so this recovers "where the
    robot was" for a self-navigating escape action (``search_object``) even
    though ``_escape_plan`` is only handed the failed-plan history.
    """
    for plan in reversed(failed_plans or []):
        for step in reversed(plan or []):
            if isinstance(step, dict) and step.get("action") == "goto":
                loc = (step.get("params") or {}).get("location")
                if loc:
                    return str(loc)
    return None


# L1 (Task D review): a registry action may only be an escape CANDIDATE when
# every one of its establish-template's own arg names is one of these three —
# a `question`-bearing predicate (e.g. answered(question)) can only be
# materialised from the exact, un-normalised question text, which
# `parse_fact` has already lower-cased/underscored away by the time a fact
# reaches here; escaping to it would emit a garbled param (see L1 finding).
_ESCAPE_ALLOWED_ARG_NAMES = {"object", "person", "location"}


def _flatten_modifications(
    grouped: Optional[Any],
) -> List[Dict[str, Any]]:
    """H1 (round-2 review): flatten `_store`'s per-step-grouped modifications
    (``dict[step_index -> list[directive]]``, as ``group_modifications_by_step``
    produces and the cache stores) back into one raw directive list, in
    step-index order -- the canonicalisable form `_canonical_plan`'s
    ``modifications`` argument expects. Also accepts an already-flat list
    (passed through unchanged, dicts only) so a caller need not know which
    shape a cache entry's ``"modifications"`` happens to hold. ``None``/empty
    -> ``[]``.
    """
    if not grouped:
        return []
    if isinstance(grouped, dict):
        flat: List[Dict[str, Any]] = []
        for idx in sorted(grouped.keys()):
            flat.extend(m for m in (grouped.get(idx) or []) if isinstance(m, dict))
        return flat
    return [m for m in grouped if isinstance(m, dict)]


def _used_actions(failed_plans: List[List[Dict[str, Any]]]) -> set:
    """L2 (round-2 review): action names already tried anywhere across
    ``failed_plans`` (a strategy is "tried" once, anywhere, not per-plan) --
    factored out so ``_escape_plan`` and ``_escape_no_untried_establisher_
    reason`` (the exhaustion helper) can never independently drift on what
    "used" means."""
    return {
        str(step.get("action"))
        for plan in (failed_plans or [])
        for step in (plan or [])
        if isinstance(step, dict)
    }


def _escape_candidates(
    predicate: str, used_actions: set, materialisable_only: bool,
) -> List[str]:
    """L2 (round-2 review): registry actions establishing ``predicate`` that
    are not in ``used_actions``, in registry order.

    ``materialisable_only=True`` (``_escape_plan``'s own use -- choosing
    what to actually MATERIALISE as an escape step) additionally requires
    every one of the candidate's establish-template arg names to be in
    ``_ESCAPE_ALLOWED_ARG_NAMES`` (a `question`-bearing predicate can only be
    materialised from the exact, un-normalised question text, which
    ``parse_fact`` has already lower-cased/underscored away by the time a
    fact reaches here -- see that constant's docstring).

    ``materialisable_only=False`` (H1, round-2 review; the exhaustion
    helper's use -- answering "has this STRATEGY been tried") drops that
    filter entirely: an action establishing the predicate and not yet used
    IS an untried establisher, whether or not ``_escape_plan`` could ever
    materialise a step for it. Without this distinction every one of
    ``answered(question)``'s six establishers (``ask_person``,
    ``answer_question``, ``count``, ``announce``, ``vlm_fallback``,
    ``llm_fallback``) carries a `question` arg -- ineligible for
    materialisation -- so the OLD helper (copying ``_escape_plan``'s filter
    verbatim) always answered "exhausted" for ``answered(...)`` after a
    single replan, even though none of those five alternatives was ever
    tried.
    """
    result = []
    for name, contract in ACTION_CONTRACTS.items():
        if name in used_actions:
            continue
        if not any(t.split("(", 1)[0] == predicate for t in contract.establishes):
            continue
        if materialisable_only:
            arg_names = _establish_arg_names(name, predicate)
            if not arg_names or any(a not in _ESCAPE_ALLOWED_ARG_NAMES for a in arg_names):
                continue
        result.append(name)
    return result


def _verified_at_robot_location(facts: Optional[List[str]]) -> Optional[str]:
    """The location argument of a verified ``at_robot(...)`` fact, if any.

    L2 (Task D review), last resort for a self-navigating escape step's
    location: the planner's own ``get_facts(slot)`` ledger records the most
    recently gate-verified ``at_robot`` fact — ground truth for "where the
    robot currently is" even when no target/ancestor location or failed-plan
    ``goto`` resolves one.
    """
    for raw in facts or []:
        fact, _error = parse_fact(str(raw))
        if fact is not None and fact.predicate == "at_robot" and fact.args:
            return fact.args[0]
    return None


def _nearest_ancestor_location(
    target: Optional[Dict[str, Any]],
    all_targets: Optional[List[Dict[str, Any]]],
) -> Optional[str]:
    """The closest (in list order) dependency ancestor's own ``location``.

    L2 (Task D review), second-to-last resort for a self-navigating escape
    step's location: a target's own assigned location and the last ``goto``
    in its OWN failed-plan history both come up empty when the goto lived in
    an ANCESTOR's plan instead (battery run 004: t1's only failed plan was
    ``['find_object']``, no goto of its own — the goto was in t0's plan).
    Ancestors nearer in list order are preferred (most likely to describe
    where the robot currently is); ancestors with no declared location are
    skipped.
    """
    if not target or not all_targets:
        return None
    pos = next(
        (i for i, t in enumerate(all_targets) if _is_same_target(t, target)), None,
    )
    if pos is None:
        return None
    for ancestor in reversed(_dependency_ancestor_targets(all_targets, pos)):
        loc = str(ancestor.get("location") or "").strip()
        if loc:
            return loc
    return None


def _escape_step_for(
    action: str,
    fact: "Fact",
    target_obj: str,
    target_loc: str,
    failed_plans: List[List[Dict[str, Any]]],
    ancestor_loc: Optional[str] = None,
    verified_loc: Optional[str] = None,
    known_locations: Optional[set] = None,
) -> Optional[Dict[str, Any]]:
    """Materialise one step for ``action`` establishing ``fact``.

    Params start from the ``establishes`` template's arg names zipped with
    ``fact.args`` (e.g. ``object_seen(pudding_box)`` + ``find_object`` ->
    ``{"object": "pudding_box"}``) — the fact args are already normalised
    (lowercased, spaces -> underscores) by ``parse_fact``, so an ``object``
    param is then overridden with the top layer's authoritative,
    un-normalised ``target_obj`` when one is available (the earlier failed
    steps used that exact string; escaping to a normalised variant would
    make the new step look like it targets a DIFFERENT object). A
    self-navigating action (``search_object``) additionally gets its own
    ``location`` param resolved, in order (L2, Task D review):
    ``target_loc``, the last ``goto`` seen in ``failed_plans``, the nearest
    dependency ancestor's own ``location`` (``ancestor_loc``), a verified
    ``at_robot(...)`` fact from the planner's fact ledger (``verified_loc``).
    NEVER left implicit: when none of these resolves, the whole step (and so
    the whole escape) is abandoned (``None``) — plan_validators.py rejects an
    implicit-navigation ``search_object`` anyway, and emitting one only to
    have it rejected downstream burns a validate_plan cycle for no gain; the
    caller falls straight through to the existing IDENTICAL_PLAN marker path
    instead.

    M-2 (round-2 review): when ``known_locations`` is given (already
    normalised, e.g. ``planner.py``'s ``known_loc_arg``), the RESOLVED
    location is also checked against it — same reasoning as above: a
    location with no known pose (e.g. only ever reached via
    ``record_position``, never one of ``KNOWN_LOCATIONS``) makes
    ``validate_plan``'s "unknown location" rule reject the escape step
    downstream anyway, so bail out here (``None``) instead of burning that
    cycle. ``known_locations=None`` (the default) skips the check entirely.
    """
    arg_names = _establish_arg_names(action, fact.predicate)
    if len(arg_names) != len(fact.args):
        return None
    params: Dict[str, Any] = dict(zip(arg_names, fact.args))
    if target_obj and "object" in params:
        params["object"] = target_obj
    contract = ACTION_CONTRACTS[action]
    loc_param = contract.self_establishes.get("at_robot")
    if loc_param and loc_param not in params:
        loc = (
            target_loc
            or _last_goto_location(failed_plans)
            or ancestor_loc
            or verified_loc
        )
        if not loc:
            return None
        if known_locations is not None and _norm_loc(loc) not in known_locations:
            return None
        params[loc_param] = loc
    return {"action": action, "params": params}


def _escape_plan(
    target: Optional[Dict[str, Any]],
    target_obj: str,
    target_loc: str,
    failure_reason: str,
    failed_plans: List[List[Dict[str, Any]]],
    all_targets: Optional[List[Dict[str, Any]]] = None,
    facts: Optional[List[str]] = None,
    known_locations: Optional[set] = None,
    tried_escapes: Optional[set] = None,
) -> Optional[List[Dict[str, Any]]]:
    """Deterministic fallback plan for a target the LLM is stuck replanning.

    Evidence (battery run 004, D1 in the Task D brief): after one
    ``find_object`` failure, the LLM kept returning the identical
    ``[goto, find_object]`` plan for every replan across the whole budget —
    ``search_object`` (a genuinely different, untried strategy) was never
    tried. This closes that gap with NO LLM call, working from the TARGET
    (H1, Task D review), not the reason text:

    (1) if a ``pred(args)`` fact parses out of ``failure_reason`` (the same
        regex/parse ``_alternatives_for_reason`` uses) AND that fact is one
        of the target's OWN declared postconditions, use it — this is the
        common case, where the reason is a structured postcondition-gate
        message. A reason like ``"precondition unmet: held(spam)"`` is
        deliberately SKIPPED here even when ``held(...)`` parses cleanly:
        the escape is postcondition-only (M3, Task D review) — re-planning a
        target's own PREcondition is not this function's job (and would
        need the chained "prepend a canonical establisher for what the
        chosen action itself requires" logic this fix REMOVES — see below).
    (2) otherwise (including most real leaf-failure messages, which rarely
        contain a fact at all — see the H1 evidence run) iterate the
        target's own canonical postconditions in DECLARED order and pick the
        first whose predicate still has an untried establisher.

    Either way: every registry action whose ``establishes`` covers the
    chosen predicate is a candidate, MINUS the ones already used in ANY of
    ``failed_plans`` (by action name — a strategy is "tried" once, anywhere,
    not per-plan) and MINUS any whose establish-template arg names are not
    all in ``_ESCAPE_ALLOWED_ARG_NAMES`` (L1, Task D review). The FIRST
    remaining candidate (registry order) is materialised.

    M3 (Task D review): the escape is POSTCONDITION-ONLY — no more
    requires-chaining. If the chosen action itself ``requires`` something
    unresolved (e.g. ``deliver`` requires ``held(object)``), the resulting
    step is simply incomplete; the caller's ``validate_plan`` (which already
    runs on the escape, same as the LLM path) rejects it and the existing
    IDENTICAL_PLAN marker path applies — no different than any other invalid
    escape candidate.

    Returns ``None`` when no untried, eligible establisher exists — e.g.
    ``held`` (after ``find_object`` and ``grasp`` both failed) is
    established ONLY by ``grasp``, and if the plan history shows ``grasp``
    already failed too there is nothing left to try.

    ``known_locations`` (M-2, round-2 review) is forwarded to
    ``_escape_step_for``, which bails out (``None``) when the resolved
    self-navigating location has no known pose — see that function's
    docstring.

    M-5 (round-2 review): this only ever materialises ONE step (see the
    ``return [chosen_step]`` below) — ``_ESCAPE_ALLOWED_ARG_NAMES`` plus
    single-step materialisation mean a target whose postconditions are e.g.
    ``[object_seen(x), held(x)]`` escapes to a lone ``search_object``, which
    then fails coverage (``held`` still uncovered) and falls to the marker
    path anyway. In practice this only ever fires as the
    ``find_object``<->``search_object`` swap the evidence run needed; a
    multi-step escape (append the failed plan's tail, or chain further
    establishers) is a real widening, not a bug fix, and is intentionally
    out of scope here.

    ``tried_escapes`` (M2, round-2 review) is a set of action names from
    escape candidates this cache entry already offered and had REJECTED
    (by ``validate_plan`` downstream, or a subtree-build failure) -- added
    to ``used_actions`` alongside ``failed_plans`` so a rejected candidate
    is never re-selected (and re-rejected) on the next identical replan.
    """
    used_actions = _used_actions(failed_plans) | set(tried_escapes or ())

    def candidates_for(predicate: str) -> List[str]:
        return _escape_candidates(predicate, used_actions, materialisable_only=True)

    own_postconditions = _canonical_postconditions(target)
    fact = None
    match = re.search(r"([a-z_]+)\(([^()]*)\)", str(failure_reason or ""))
    if match is not None:
        parsed_fact, _err = parse_fact(match.group(0))
        if parsed_fact is not None and canonical_fact(parsed_fact) in own_postconditions:
            fact = parsed_fact
    if fact is None:
        for raw in (target or {}).get("postconditions") or []:
            candidate_fact, _error = parse_fact(str(raw))
            if candidate_fact is None:
                continue
            if candidates_for(candidate_fact.predicate):
                fact = candidate_fact
                break
    if fact is None:
        return None
    candidates = candidates_for(fact.predicate)
    if not candidates:
        return None
    chosen = candidates[0]
    target_obj = target_obj or str((target or {}).get("object") or "")
    target_loc = target_loc or str((target or {}).get("location") or "")
    chosen_step = _escape_step_for(
        chosen, fact, target_obj, target_loc, failed_plans,
        ancestor_loc=_nearest_ancestor_location(target, all_targets),
        verified_loc=_verified_at_robot_location(facts),
        known_locations=known_locations,
    )
    if chosen_step is None:
        return None
    return [chosen_step]


def _escape_no_untried_establisher_reason(
    target: Optional[Dict[str, Any]],
    failed_plans: List[List[Dict[str, Any]]],
    tried_escapes: Optional[set] = None,
) -> Optional[str]:
    """E2 (runs 003/004, 2026-08-29): non-``None`` when the escape ladder is exhausted.

    Called by ``_try_escape`` only after ``_escape_plan`` has already
    returned ``None`` on the final identical-plan attempt, to distinguish
    the truly UNRECOVERABLE case -- no registry action can establish ANY of
    the target's own declared postconditions without repeating one already
    used in ``failed_plans``/``tried_escapes`` -- from the other reasons
    ``_escape_plan`` can come back empty: an eligible establisher exists but
    its materialised escape step was rejected by ``validate_plan``
    downstream, or a self-navigating step's location could not be resolved
    (``_escape_step_for`` returning ``None``). Neither of those is
    unrecoverable -- a future replan attempt, or a different failure
    reason, could still find a way through.

    H1 (round-2 review): "untried establisher" here means "has this
    STRATEGY been tried at all" -- unlike ``_escape_plan``'s OWN candidate
    search, this check does NOT apply the materialisation arg-name filter
    (``_ESCAPE_ALLOWED_ARG_NAMES``). An establisher that ``_escape_plan``
    itself could never materialise a step for (e.g. any of
    ``answered(question)``'s establishers, which all carry a `question`
    arg) is still a real, untried alternative the LLM could reach on its
    own -- so its existence must fall through to the ordinary IDENTICAL_
    PLAN marker (a future replan may yet try it), never UNRECOVERABLE.

    ``tried_escapes`` (M2, round-2 review) mirrors ``_escape_plan``'s own
    parameter -- a validation-rejected escape action counts as "used" here
    too, so a second identical replan after a rejected escape correctly
    sees the ladder as exhausted instead of looping forever.

    A target that declares no postconditions at all is deliberately NOT
    unrecoverable here (returns ``None``) -- there is nothing to say is
    exhausted, and this function's only caller already restricts itself to
    the final identical-plan attempt, so an ordinary IDENTICAL_PLAN marker
    still applies to that degenerate case.

    Returns the ``"no untried establisher for <facts>"`` reason text (never
    just a bool) so both the log line and the stored
    ``UNRECOVERABLE_ERROR_PREFIX: <reason>`` marker can reuse the exact same
    string.
    """
    own_postconditions = sorted(_canonical_postconditions(target))
    if not own_postconditions:
        return None
    used_actions = _used_actions(failed_plans) | set(tried_escapes or ())

    for raw in own_postconditions:
        fact, _error = parse_fact(raw)
        if fact is None:
            continue
        if _escape_candidates(fact.predicate, used_actions, materialisable_only=False):
            return None
    return f"no untried establisher for {own_postconditions}"


def _deterministic_target_intent(target: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Compile metadata into validator-only intent steps without cache reads."""
    intents: List[Dict[str, Any]] = []
    desc = str(target.get("desc") or "").lower()
    postconditions = target.get("postconditions") or []
    for condition in postconditions:
        fact, _error = parse_fact(condition)
        if fact is None:
            continue
        params = {"location": fact.args[0]} if fact.predicate == "at_robot" and fact.args else {}
        action = _ESTABLISHER_FOR_PREDICATE.get(fact.predicate)
        if action:
            intents.append({"action": action, "params": params})
    if not intents and ("find" in desc and "person" in desc):
        intents.append({"action": "find_person", "params": {}})
    return intents


def _deterministic_prior_plan(
    targets: List[Dict[str, Any]], index: int,
) -> List[Dict[str, Any]]:
    """Build stable prior intent context from dependency metadata only."""
    return [
        step
        for target in _dependency_ancestor_targets(targets, index)
        for step in _deterministic_target_intent(target)
    ]


def _flatten_prior_plans(planner, slot: int, index: int) -> List[Dict[str, Any]]:
    """Return deterministic dependency intents; never inspect concurrent cache."""
    targets = planner._get_slot_context(slot).get("targets", [])
    return _deterministic_prior_plan(targets, index)


def _is_same_target(a: Optional[Dict[str, Any]], b: Optional[Dict[str, Any]]) -> bool:
    """Identity match for target dicts, tolerant of copies carrying the same id."""
    if a is None or b is None:
        return False
    if a is b:
        return True
    a_id, b_id = a.get("id"), b.get("id")
    return bool(a_id) and a_id == b_id


def _canonical_postconditions(target: Optional[Dict[str, Any]]) -> set:
    """Canonical ``predicate(args)`` facts for a target's declared postconditions.

    Unparsable conditions are silently skipped — ``_validate_target_contract``
    is where those are rejected with a retry; by the time a target reaches
    here its postconditions are already known-parseable, but this stays
    defensive for callers (tests, ad hoc dicts) that skip that validation.
    """
    result: set = set()
    for raw in (target or {}).get("postconditions") or []:
        fact, _error = parse_fact(str(raw))
        if fact is not None:
            result.add(canonical_fact(fact))
    return result


def _foreign_facts(
    target: Optional[Dict[str, Any]],
    all_targets: Optional[List[Dict[str, Any]]],
    include_ancestors: bool = False,
) -> Tuple[set, List[Tuple[str, str]]]:
    """Foreign (off-limits) postcondition facts for ``target``.

    Shared by ``_render_contract_block`` (advisory prompt text) and
    ``_drop_foreign_contract_steps`` (the deterministic guard) so the two
    can never disagree (I-2, round-2 review): before this factoring the
    guard folded a retracted ancestor fact through ``apply_fact_transitions``
    (M1, Task D review) but the prompt's "owned by" text did not, so a model
    that followed the prompt literally could still be told an already-
    retracted ancestor fact (e.g. ``held(spam)`` after ``placed(spam,
    table)``) was off-limits, or worse, never told a still-foreign one was.

    Returns ``(foreign_fact_set, owned_pairs)``:

    - ``foreign_fact_set``: the plain union the guard matches
      ``established_facts(step)`` against (minus ``target``'s own
      postconditions, applied by the caller).
    - ``owned_pairs``: ``[(owner_target_id, fact), ...]`` — one entry per
      still-foreign fact, attributed to the target that (still) owns it —
      used by the prompt to render one "owned by <id>" line per owner, in
      the same order the facts were folded in.

    SUCCESSORS (targets AFTER ``target`` in ``all_targets`` list order) are
    ALWAYS foreign, attributed by their raw declared postconditions — they
    have not executed yet, so nothing of theirs has been retracted.

    ANCESTORS (``_dependency_ancestor_targets(all_targets, pos)`` —
    DECLARED ``depends_on`` only, never bare list position, per I-1) are
    foreign ONLY when ``include_ancestors`` is True (the INITIAL plan — see
    ``_drop_foreign_contract_steps``'s own docstring for why). Their raw
    postconditions are folded through ``apply_fact_transitions`` IN
    DEPENDENCY ORDER so a fact one ancestor's postcondition retracts (e.g.
    ``placed(spam,table)`` retracting ``held(spam)``) is never attributed
    to — or counted as foreign from — the ancestor that no longer holds it
    (M1, Task D review).
    """
    if not target or not all_targets:
        return set(), []
    own_post_canon = _canonical_postconditions(target)
    pos = next(
        (i for i, t in enumerate(all_targets) if _is_same_target(t, target)), None,
    )
    successor_targets = all_targets[pos + 1:] if pos is not None else []
    foreign: set = set()
    owned_pairs: List[Tuple[str, str]] = []
    for other in successor_targets:
        owner = str(other.get("id") or "")
        for fact in sorted(_canonical_postconditions(other) - own_post_canon):
            foreign.add(fact)
            owned_pairs.append((owner, fact))
    if include_ancestors and pos is not None:
        ancestor_targets = _dependency_ancestor_targets(all_targets, pos)
        # Fold every ancestor's raw postconditions through the SAME
        # transition ledger the runtime uses, in dependency order, tracking
        # which ancestor currently "owns" each still-effective fact — a
        # later ancestor's postcondition can retract an earlier one's
        # (M1, Task D review).
        state: List[str] = []
        owner_by_fact: Dict[str, str] = {}
        for other in ancestor_targets:
            owner = str(other.get("id") or "")
            new_state = apply_fact_transitions(state, other.get("postconditions") or [])
            for fact in set(state) - set(new_state):
                owner_by_fact.pop(fact, None)
            for fact in set(new_state) - set(state):
                owner_by_fact[fact] = owner
            state = new_state
        for other in ancestor_targets:
            owner = str(other.get("id") or "")
            for fact in sorted(state):
                if owner_by_fact.get(fact) != owner or fact in own_post_canon:
                    continue
                foreign.add(fact)
                owned_pairs.append((owner, fact))
    return foreign, owned_pairs


def _successor_postcondition_facts(
    target: Optional[Dict[str, Any]], all_targets: Optional[List[Dict[str, Any]]],
) -> set:
    """Canonical postcondition facts of every target AFTER ``target`` in list order.

    J12: used to decide whether ``answered(...)`` must be treated at
    PREDICATE level (see ``_drop_foreign_contract_steps``) — deliberately
    successors only, never ancestors, per the ruling.
    """
    if not target or not all_targets:
        return set()
    pos = next(
        (i for i, t in enumerate(all_targets) if _is_same_target(t, target)), None,
    )
    if pos is None:
        return set()
    result: set = set()
    for other in all_targets[pos + 1:]:
        result |= _canonical_postconditions(other)
    return result


def _render_contract_block(
    contract: Optional[Dict[str, Any]],
    all_targets: Optional[List[Dict[str, Any]]],
    include_ancestors: bool = True,
) -> str:
    """Render the "fact contract" block for the lower-layer prompt.

    Tells the model its OWN target's declared pre/postconditions (from the
    top-layer split) plus, for every OTHER target in the same command, the
    postconditions that belong to that target and NOT to this one — facts it
    must never establish itself. See ``task-A-brief.md`` for the "bring me a
    spam" defect this closes.

    ``include_ancestors`` mirrors ``_drop_foreign_contract_steps``'s
    parameter of the same name (D2, Task D brief): on the INITIAL plan
    (``include_ancestors=True``, the caller's default) ancestors' facts are
    listed as "owned by" too — the DAG + precondition gate already guarantee
    those facts hold, so re-planning them is pure waste (the "bring the spam
    to me" t1 replanning the whole of t0 defect). On a REPLAN
    (``include_ancestors=False``, passed explicitly by the caller when a
    failure reason is present) an ancestor's fact may legitimately need
    re-establishing during recovery (e.g. "precondition unmet: held(spam)"),
    so only successors are listed — the reworded "requires" line below
    already makes that conditional-recovery case explicit.
    """
    if not contract:
        return ""
    preconditions = list(contract.get("preconditions") or [])
    postconditions = list(contract.get("postconditions") or [])
    if not preconditions and not postconditions:
        return ""
    requires_str = ", ".join(str(p) for p in preconditions) or "(none)"
    establish_str = ", ".join(str(p) for p in postconditions) or "(none)"
    lines = [
        "This target's fact contract (from the top layer):",
        "  requires (normally established by earlier targets — re-establish "
        f"only if the failure reason below says it is unmet): {requires_str}",
        f"  must establish: {establish_str}",
        "Plan ONLY the steps that establish the \"must establish\" facts from the\n"
        "\"requires\" state.",
    ]
    # I-2 (round-2 review): the same helper the guard uses (_foreign_facts)
    # so the prompt's "owned by" text and the deterministic guard can never
    # disagree about which ancestor still owns a fact (M1's retraction
    # ledger, folded through here too).
    targets = all_targets or []
    _foreign_set, owned_pairs = _foreign_facts(
        contract, targets, include_ancestors=include_ancestors,
    )
    id_to_desc = {str(t.get("id") or ""): str(t.get("desc") or "") for t in targets}
    owned_by_id: Dict[str, List[str]] = {}
    owner_order: List[str] = []
    for owner_id, fact in owned_pairs:
        if owner_id not in owned_by_id:
            owned_by_id[owner_id] = []
            owner_order.append(owner_id)
        owned_by_id[owner_id].append(fact)
    owned_lines = [
        f"  owned by {owner_id} \"{id_to_desc.get(owner_id, '')}\": "
        f"{', '.join(owned_by_id[owner_id])}"
        for owner_id in owner_order
    ]
    if owned_lines:
        lines.append(
            "Facts owned by OTHER targets of this command are listed below — "
            "never perform an action that establishes one of them:"
        )
        lines.extend(owned_lines)
    return "\n".join(lines) + "\n"


def _drop_foreign_contract_steps(
    plan: List[Dict[str, Any]],
    target: Optional[Dict[str, Any]],
    all_targets: Optional[List[Dict[str, Any]]],
    include_ancestors: bool = False,
) -> Tuple[List[Dict[str, Any]], List[str]]:
    """Deterministically drop plan steps that establish another target's fact.

    A step is dropped iff at least one of its ``established_facts(step)`` is
    (a) NOT a canonical postcondition of ``target`` and (b) IS a canonical
    postcondition of some SUCCESSOR of ``target`` in ``all_targets`` list
    order (``all_targets[pos+1:]``, ``target``'s position found the same way
    ``_is_same_target`` matches identity elsewhere) — or, when
    ``include_ancestors`` is True, also some declared-dependency ANCESTOR
    (``_dependency_ancestor_targets(all_targets, pos)`` — I-1, round-2
    review: NEVER bare list position, ``all_targets[:pos]``, which can
    include a target ``target`` never actually depends on). See
    ``_foreign_facts`` (shared with ``_render_contract_block``) for exactly
    how the ancestor set is computed and transitioned.
    Facts are compared in FULL (``at_robot(kitchen)`` vs ``at_robot(balcony)``
    never collide) — never by predicate alone. Only ``establishes``
    (action_contracts.py) counts; ``self_establishes`` (incidental
    navigation, e.g. place/deliver reaching their own location) never
    triggers a drop, since it is not consulted here at all.

    ``include_ancestors=False`` (the default): a plan may need to
    legitimately RE-establish an ancestor's fact (e.g. re-grasp after the
    precondition gate rejects with "precondition unmet: held(spam)" because
    the earlier target was skipped or the object was dropped) — dropping
    that step would make recovery impossible. This is what ``plan_target``
    passes on a REPLAN (``failure_reason`` set) and what
    ``replace_target_plan`` (supervisor) always passes.

    ``include_ancestors=True``: on the INITIAL plan (D2, Task D brief) the
    DAG + precondition gate already guarantee an ancestor's postconditions
    hold before this target runs, so a plan that re-does an ancestor's work
    (e.g. t1 "bring the spam to me" re-planning t0's whole
    ``[goto, grasp, deliver]``) is never legitimate recovery — it is pure
    duplication, and dropping those steps is safe. ``plan_target`` passes
    this on the initial plan (``failure_reason is None``). Duplicating an
    ancestor's work when recovery is NOT needed is also guarded by the
    prompt ("Prior targets ... do NOT repeat their work",
    ``_render_contract_block``) and ``validate_plan``'s prior-plan seeding —
    this is the deterministic backstop if the model ignores that.

    No LLM call, no cost — this is the deterministic guard that closes the
    "bring me a spam from the laundry_desk" defect (a plan reaching AHEAD
    into a fact only a later target should establish) even if the prompt's
    contract block (``_render_contract_block``) is ignored by the model.
    """
    if not target or not all_targets:
        return list(plan or []), []
    own_post_canon = _canonical_postconditions(target)
    # I-2 (round-2 review): shared with _render_contract_block's "owned by"
    # text via _foreign_facts, so the two can never disagree about which
    # ancestor fact still counts as foreign (M1's retraction ledger).
    foreign_post_canon, _owned_pairs = _foreign_facts(
        target, all_targets, include_ancestors=include_ancestors,
    )
    # J12 (round-3 adversarial review, tier0 #3, run 024): every OTHER
    # predicate is compared as a FULL fact (exact argument text), but
    # `answered(<free text>)` never round-trips exactly between the top
    # layer's split wording and the lower layer's own ask_person/announce
    # steps -- so a target whose plan asks+announces an answer a SUCCESSOR
    # target actually owns was never caught ("find the person.../tell me
    # the name..." -- target0 planned the whole ask+announce itself).
    # `answered` is therefore matched at PREDICATE level: any step
    # establishing answered(*) is foreign when some successor declares AT
    # LEAST ONE answered(...) postcondition and THIS target declares none
    # of its own (a target that itself owns an answered(...) fact is
    # unaffected -- its own ask/announce is legitimate).
    own_has_answered = any(f.startswith("answered(") for f in own_post_canon)
    successor_has_answered = any(
        f.startswith("answered(")
        for f in _successor_postcondition_facts(target, all_targets)
    )
    answered_is_predicate_foreign = successor_has_answered and not own_has_answered
    kept: List[Dict[str, Any]] = []
    dropped: List[str] = []
    # J10: fall back to the TARGET's own declared object when a step omits
    # it (place/deliver routinely do -- the held object is implicit) so the
    # guard is not blind to a duplicate place/place or deliver/deliver.
    target_object = str((target or {}).get("object") or "")
    for step in plan or []:
        facts = established_facts(step, target_object) if isinstance(step, dict) else []
        foreign_fact = next(
            (f for f in facts if f not in own_post_canon and (
                f in foreign_post_canon
                or (answered_is_predicate_foreign and f.startswith("answered("))
            )),
            None,
        )
        if foreign_fact is not None:
            dropped.append(f"contract:{step.get('action')}->{foreign_fact}")
            continue
        kept.append(step)
    return kept, dropped


def _drop_dangling_goto_before_self_nav(
    plan: List[Dict[str, Any]],
) -> Tuple[List[Dict[str, Any]], List[str]]:
    """Drop a ``goto`` step immediately followed by a self-navigating step.

    I-4 (round-2 review): ``_drop_foreign_contract_steps`` can reduce a plan
    like ``[goto, grasp, deliver]`` (the ``grasp`` established an ancestor's
    already-guaranteed ``held(object)``) down to ``[goto, deliver]`` — but
    ``planner_validators.validate_plan``'s "no goto immediately before a
    self-navigating action" rule (``self_navigating_destinations()`` —
    ``deliver``/``place``/``search_object``/... never hardcoded) then ALWAYS
    rejects that plan outright, burning a full LLM round-trip on the most
    common template (fetch-and-deliver). This mirrors that same rule
    deterministically, no LLM call, so the guard-reduced plan is repaired in
    place instead of handed to the model to discover is broken.

    Only ever removes a ``goto`` that directly precedes (in the ALREADY
    guard-reduced plan) a self-navigating step — a ``goto`` anywhere else is
    untouched, same as the validator it mirrors.
    """
    self_nav_dests = self_navigating_destinations()
    kept: List[Dict[str, Any]] = []
    dropped: List[str] = []
    steps = list(plan or [])
    for i, step in enumerate(steps):
        nxt = steps[i + 1] if i + 1 < len(steps) else None
        if (
            isinstance(step, dict) and step.get("action") == "goto"
            and isinstance(nxt, dict) and nxt.get("action") in self_nav_dests
        ):
            dropped.append("contract:goto->dangling")
            continue
        kept.append(step)
    return kept, dropped


def _step_canonical(step: Any) -> Optional[tuple]:
    if not isinstance(step, dict):
        return None
    return (
        str(step.get("action")),
        tuple(sorted((str(k), _canonical_param_value(v))
                      for k, v in (step.get("params") or {}).items())),
    )


def _drop_completed_duplicate_steps(
    plan: List[Dict[str, Any]],
    completed_steps: Optional[List[Dict[str, Any]]],
) -> Tuple[List[Dict[str, Any]], List[str]]:
    """H-3 (round-3 fix review): drop a re-emitted step element-wise IDENTICAL
    to one of this target's own already-completed steps.

    ``completed_steps`` (J3, M8) is the postcondition gate's partial commit
    from a PRIOR attempt of this same target -- e.g. a fully successful
    ``grasp(x)`` before ``place(x,t)`` failed. Nothing else enforced "a
    completed grasp is not re-emitted": ``_drop_foreign_contract_steps``
    only removes steps establishing a SIBLING target's fact, and
    ``validate_plan``'s prompt just asks nicely. A replan that ignores the
    prompt and re-grasps would grasp with an already-full gripper. Compared
    the same way ``_canonical_plan`` compares a whole plan for identity --
    action + param values, order-insensitive within a step -- and only the
    FIRST matching completed step per re-emitted duplicate is consumed (a
    plan that legitimately repeats an action twice, e.g. two separate
    grasps, is not over-dropped).
    """
    completed_canon = [
        c for c in (_step_canonical(s) for s in (completed_steps or [])) if c is not None
    ]
    if not completed_canon:
        return list(plan or []), []
    remaining = list(completed_canon)
    kept: List[Dict[str, Any]] = []
    dropped: List[str] = []
    for step in plan or []:
        canon = _step_canonical(step)
        if canon is not None and canon in remaining:
            remaining.remove(canon)
            dropped.append("contract:completed-step")
            continue
        kept.append(step)
    return kept, dropped


def _contract_drop_note(
    dropped: List[str],
    target: Optional[Dict[str, Any]],
    all_targets: Optional[List[Dict[str, Any]]],
) -> str:
    """Sentence naming the dropped step(s) + owning target, for ``last_reason``.

    Only used when the contract-boundary-reduced plan then fails
    ``validate_plan`` (e.g. it no longer establishes its own postcondition):
    tells the retry which steps were removed and why, so it does not just
    re-emit them.
    """
    contract_entries = [d for d in dropped if str(d).startswith("contract:")]
    if not contract_entries or not target or not all_targets:
        return ""
    owner_by_fact: Dict[str, str] = {}
    for other in all_targets:
        if _is_same_target(other, target):
            continue
        label = f"{other.get('id') or ''} \"{other.get('desc') or ''}\""
        for fact in _canonical_postconditions(other):
            owner_by_fact.setdefault(fact, label)
    parts = []
    for entry in contract_entries:
        _, _, rest = entry.partition(":")
        action, _, fact = rest.partition("->")
        owner = owner_by_fact.get(fact, "another target")
        parts.append(f"{action} (establishes {fact}, owned by {owner})")
    # `parts` cannot be empty here: contract_entries is non-empty (checked
    # above) and every entry unconditionally appends one part (owner falls
    # back to "another target" when unmatched).
    return (
        " Note: the contract-boundary guard already removed "
        + "; ".join(parts)
        + " from your last plan because that fact belongs to a DIFFERENT "
        "target of this command — do not re-emit them."
    )


def _kept_indices(before: List[Dict[str, Any]], after: List[Dict[str, Any]]) -> List[int]:
    """Return, for each step in ``after``, its index in ``before``.

    I3 (round-3 adversarial review, M6): ``_drop_foreign_contract_steps`` and
    ``_drop_dangling_goto_before_self_nav`` both keep steps by APPENDING THE
    SAME step object references from their input plan (never copies) — so
    the surviving original index for each kept step can be recovered by
    identity, without changing either function's return signature (both are
    exercised directly by many existing tests as a 2-tuple). Matched by
    ``id()``, not equality, so two structurally-identical steps at different
    positions are never confused.
    """
    before_ids = [id(step) for step in before]
    return [before_ids.index(id(step)) for step in after]


def _remap_modification_step_indices(
    mods: Optional[List[Dict[str, Any]]],
    old_to_new: Dict[int, int],
    cleaned: List[Dict[str, Any]],
) -> Tuple[List[Dict[str, Any]], List[str]]:
    """Translate each modification's ``step_index`` through the guard drops.

    I3 (round-3 adversarial review, M6): the LLM's ``step_index`` is bound to
    the plan IT wrote, but modifications are validated/grouped against
    ``cleaned`` AFTER ``_clean_plan``, the contract-boundary guard, and the
    dangling-goto drop removed steps — every mod whose target step survived
    at a DIFFERENT index was rejected "could not be matched" (burning the
    whole attempt), or worse silently misapplied to an unrelated same-action
    step. A modification whose OWN step was dropped is dropped too here
    (logged as ``mod:<template>@<old_index>`` in the returned list,
    mirroring the ``contract:``/``duplicate:`` entries in the plan-step
    ``dropped`` list). A modification without an integer ``step_index``
    (action-only matching) passes through unchanged — there is no index to
    remap.

    ``old_to_new`` must map RAW (as-written-by-the-LLM) indices to the final
    ``cleaned`` plan's indices — the caller composes this through
    ``_clean_plan_core``'s kept-raw-index list AND both guard drops (M-1,
    round-3 fix review: the "old" frame is the raw LLM plan, not the plan
    AFTER ``_clean_plan``, since ``_clean_plan`` itself can drop
    steps — e.g. consecutive duplicates — that the LLM's ``step_index``
    still counts).

    M-1: when ``old_index`` has no entry in ``old_to_new`` (its step was
    truly dropped, OR the LLM's index was never valid to begin with — e.g.
    out of range for the raw plan it wrote), fall back to matching the
    mod's own ``action`` field against a UNIQUE step of that action in
    ``cleaned`` — the pre-I3 ``modifiable_nodes._step_index_for`` behaviour.
    Ambiguous (0 or >1 matches) still drops the modification.
    """
    remapped: List[Dict[str, Any]] = []
    dropped: List[str] = []
    for mod in mods or []:
        if not isinstance(mod, dict):
            remapped.append(mod)
            continue
        old_index = mod.get("step_index")
        if not isinstance(old_index, int) or isinstance(old_index, bool):
            remapped.append(dict(mod))
            continue
        new_index = old_to_new.get(old_index)
        if new_index is None:
            action = str(mod.get("action") or "")
            matches = [
                i for i, step in enumerate(cleaned)
                if isinstance(step, dict) and str(step.get("action") or "") == action
            ]
            if action and len(matches) == 1:
                new_mod = dict(mod)
                new_mod["step_index"] = matches[0]
                remapped.append(new_mod)
                continue
            dropped.append(f"mod:{mod.get('template', '?')}@{old_index}")
            continue
        new_mod = dict(mod)
        new_mod["step_index"] = new_index
        remapped.append(new_mod)
    return remapped, dropped


def _build_lower_layer_user_prompt(
    command: str,
    desc: str,
    target_obj: str,
    target_loc: str,
    prior_targets: List[Dict[str, Any]],
    state_log: List[str],
    failure_msg: Optional[str] = None,
    nonce: Optional[str] = None,
    verified_facts: Optional[List[str]] = None,
    contract: Optional[Dict[str, Any]] = None,
    all_targets: Optional[List[Dict[str, Any]]] = None,
    include_ancestors: Optional[bool] = None,
    completed_steps: Optional[List[Dict[str, Any]]] = None,
) -> str:
    """Build the lower-layer user prompt WITH full-command + prior-target context.

    Embeds the original NL command, this target's desc, the top-layer ASSIGNED
    object/location (authoritative — wins over any default), the prior targets
    (so the worker plans only its delta), the action catalogue, and the known
    arena data. ``LOWER_LAYER_SYSTEM_PROMPT`` supplies the plan JSON contract
    and the hard rules; this supplies the situation.

    ``include_ancestors`` (M2/N1, Task D review) mirrors
    ``_drop_foreign_contract_steps``'s parameter of the same name and MUST be
    driven by the SAME predicate the caller uses for that guard
    (``plan_target`` passes ``not failure_reason`` for both). Left ``None``
    (the default, used by callers/tests that don't carry the distinction) it
    falls back to ``failure_msg is None`` — but ``plan_target`` must NEVER
    rely on that fallback: ``failure_msg`` there is ``last_reason``, which
    goes non-None as soon as attempt 1 of the INITIAL plan is rejected for
    ANY reason (a validator rejection, an empty plan, ...), which would
    silently flip the "owned by" block to the replan rule mid-initial-plan
    even though the guard (keyed on the original ``failure_reason`` argument)
    has not.
    """
    from datetime import datetime
    known_loc = ", ".join(sorted(KNOWN_LOCATIONS.keys())) or "(none)"
    known_obj = ", ".join(sorted(KNOWN_OBJECT_PROMPTS.keys())) or "(none)"
    default_loc = ", ".join(
        f"{k}={v}" for k, v in sorted(DEFAULT_OBJECT_LOCATIONS.items())
    ) or "(none)"

    body = (
        f"Full command (the whole instruction this target belongs to):\n{command}\n\n"
        f"Current target to plan:\n{desc}\n\n"
    )
    if include_ancestors is None:
        include_ancestors = failure_msg is None
    contract_block = _render_contract_block(
        contract, all_targets, include_ancestors=include_ancestors,
    )
    if contract_block:
        body += contract_block + "\n"
    if target_loc:
        body += (
            f"Location context: {target_obj or 'the target'} was assigned to "
            f"the location {target_loc!r} by the top layer. Plan against "
            f"{target_loc!r} — do NOT fall back to any predefined default.\n\n"
        )
    elif target_obj:
        body += (
            f"Object context: this target's object is {target_obj!r} (no "
            "explicit location was assigned — use the default below only if "
            "the target itself gives no location).\n\n"
        )
    if prior_targets:
        body += (
            "Prior targets of this command (already planned, and already or "
            "soon executed). Start from the state AFTER they complete; do NOT "
            "repeat their work:\n"
        )
        for i, t in enumerate(prior_targets):
            body += f"  T{i} {t.get('desc') or ''}\n"
        body += "\n"
    # J3 (round-3 adversarial review, M8): steps of THIS target's own plan
    # that already succeeded and had a fact committed for them (a
    # postcondition gate's partial commit before a LATER fact of the same
    # target failed) -- do not repeat them on replan.
    if completed_steps:
        body += "Already completed in this target (do not repeat):\n"
        for step in completed_steps:
            action = step.get("action") if isinstance(step, dict) else step
            params = step.get("params", {}) if isinstance(step, dict) else {}
            body += f"  - {action}({params})\n"
        body += "\n"
    facts = list(verified_facts or [])
    body += "Verified world-state facts\n(established by successful deterministic target gates — treat as true):\n"
    body += "".join(f"  - {fact}\n" for fact in facts) if facts else "  (none yet)\n"
    body += "\n"
    body += (
        f"Current date and time: {datetime.now().strftime('%A, %B %d, %Y, %H:%M')}\n"
        f"Known locations: {known_loc}\n"
        f"Known objects (HINT ONLY — any object word is allowed, not just these): "
        f"{known_obj}\n"
        f"Default object locations (used ONLY when a fetch/find names NO "
        f"location): {default_loc}\n\n"
        f"{ACTION_CATALOGUE_DESCRIPTION}\n\n"
        f"Target to plan (again):\n{desc}\n\n"
        f"Completed steps so far (for this target):\n{json.dumps(state_log, indent=2)}\n"
    )
    if failure_msg:
        body += (
            f"\nThe previous attempt failed with: {failure_msg}\n"
            "Re-plan from the current state. Do not repeat completed steps.\n"
        )
    if nonce:
        body += f"\n(Planning request id: {nonce} — ignore, ensures a fresh plan.)"
    body += "\n\n" + _modification_targets_blurb()
    body += "\nReturn the JSON plan now."
    return body


def _modification_targets_blurb() -> str:
    """Render the concrete valid ``target_node_id`` options per template.

    The lower-layer LLM must name a real serialized node id for any
    modification it emits; without the id it can only omit. This renders the
    authoritative map (template -> roles -> node ids) from the role audit so
    the model picks a genuinely valid target.
    """
    roles = get_small_tree_roles()
    lines = ["Modifiable tree targets (concrete target_node_id per template):"]
    for name, spec in sorted(TEMPLATES.items()):
        entries = []
        for role in spec.applies_to:
            for node_id in roles.get(role, ()):
                entries.append(f"{role} -> {node_id}")
        if not entries:
            continue
        lines.append(f"- {name}: " + "; ".join(entries))
    return "\n".join(lines)


def _offline_mock_targets(command: str) -> List[Dict[str, Any]]:
    """Deterministic top-layer split for the all-mock preset (network-free).

    Batch debug commands use ``|`` as a command separator; reuse that convention
    here so an offline integration test can drive several targets at once. A
    command with no ``|`` is a single target (the whole command). Each target is
    a structured dict (desc / object / location / depends_on), same shape the
    LLM top layer returns.
    """
    parts = [p.strip() for p in str(command or "").split("|") if p.strip()]
    targets = []
    for i, part in enumerate(parts):
        targets.append({
            "id": f"t{i}",
            "desc": part,
            "object": "",
            "location": "",
            "depends_on": [f"t{i - 1}"] if i > 0 else [],
            "preconditions": [],
            "postconditions": [],
        })
    return targets


def _offline_mock_plan(target: str) -> List[Dict[str, Any]]:
    """Deterministic, network-free lower-layer plan for one target."""
    text = "Mock mode: planner bypassed; no network request was made."
    if target:
        text += f" Target: {target}"
    return [{"action": "announce", "params": {"text": text}}]


def _call_llm(
    client: openai.OpenAI,
    system_prompt: str,
    user_prompt: str,
    temperature: float,
) -> Tuple[Optional[dict], Optional[str]]:
    """One LLM round-trip -> (parsed JSON dict, error string). Exactly one is set.

    Same contract as the legacy ``BtNode_PlanActions._call_llm``: a fresh random
    seed per call defeats provider-side response caching/dedup of identical
    requests, and the caller's nonce makes every prompt byte-unique.
    """
    try:
        kwargs = dict(
            model=OPENAI_MODEL,
            messages=[
                {"role": "system", "content": system_prompt},
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
            resp = client.chat.completions.create(**kwargs)
        except Exception as exc:  # a model/provider that rejects `seed`?
            if "seed" not in repr(exc).lower():
                raise
            kwargs.pop("seed", None)
            resp = client.chat.completions.create(**kwargs)
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


class GPSRPlanner:
    """Decoupled, callable two-layer planner.

    Invoked by the orchestrator's bridge nodes (BtNode_SplitCommand /
    BtNode_PlanAllTargets) and by DynamicExecutor on a target failure. All
    thread spawning + LLM work lives here; the executor thread only swaps
    ready subtrees into the running tree and reads/writes the Blackboard.
    """

    def __init__(self, max_attempts: int = 4):
        self._max_attempts = max(1, int(max_attempts))
        self._offline_mock = _offline_planner_enabled()
        # (slot, index) -> {"desc": str, "plan": list[dict], "subtree": Behaviour|None,
        #                   "ready": bool, "error": str|None}
        self._cache: Dict[Tuple[int, int], Dict[str, Any]] = {}
        # slot -> {"command": str, "targets": list[dict]} — full-command context
        # for the lower layer, set once per command by request_plan_all.
        self._slot_context: Dict[int, Dict[str, Any]] = {}
        self._facts: Dict[int, List[str]] = {}
        self._lock = threading.Lock()

    # -- client factory ----------------------------------------------------

    def _new_client(self) -> Optional[openai.OpenAI]:
        if self._offline_mock:
            return None
        # I4 (round-3 adversarial review, M7): no timeout + openai's default
        # (unbounded) retries meant an OpenRouter stall could hang a
        # planning attempt indefinitely, and code AFTER this call in
        # `plan_target` (grouping, fallback subtree build) is what actually
        # protects the daemon planning thread -- see the try/except wrapping
        # the whole method body below.
        return openai.OpenAI(
            api_key=OPENAI_API_KEY,
            base_url="https://openrouter.ai/api/v1",
            timeout=LLM_TIMEOUT_S,
            max_retries=1,
        )

    # -- cache ---------------------------------------------------------------

    def _store(self, slot, index, desc, plan, subtree, error, modifications=None):
        with self._lock:
            prev = self._cache.get((slot, index))
            failed_plans = list(prev.get("failed_plans") or []) if prev else []
            # M2 (round-2 review): carried forward like failed_plans -- a
            # rejected-escape action recorded via _record_tried_escape must
            # survive the wholesale entry replacement _store performs (e.g.
            # for the IDENTICAL/UNRECOVERABLE marker stored right after
            # _try_escape recorded the rejection).
            tried_escapes = set(prev.get("tried_escapes") or ()) if prev else set()
            self._cache[(slot, index)] = {
                "desc": desc,
                "plan": list(plan),
                "subtree": subtree,
                "ready": True,
                "error": error,
                "modifications": modifications,
                "failed_plans": failed_plans,
                "tried_escapes": tried_escapes,
            }

    def _get_desc(self, slot, index) -> Optional[str]:
        """M-8 (round-2 review, pre-existing): ``entry`` is read (``.get(...)``
        below) AFTER ``_lock`` is released, while ``_invalidate`` mutates the
        SAME dict object in place under the lock (``entry["ready"] = False``,
        etc.) — a benign, pre-existing race (worst case: this read observes
        a value ``_invalidate`` is mid-way through updating, e.g. ``ready``
        already False but ``error`` not yet cleared). ``get_action_plan`` and
        ``get_error`` below share this same pattern. Nothing on this branch
        worsens it (see the whole-branch review's interaction-check (c)):
        ``_failed_plan``/``_failed_plans`` copy under the lock (513b218),
        the guard/prompt/escape helpers only read the shared ``norm_targets``
        dicts (never mutated after ``request_plan_all``), ``replace_target_plan``
        builds fresh step/params dicts, and ``_store`` replaces the cache
        entry wholesale.
        """
        with self._lock:
            entry = self._cache.get((slot, index))
        return entry.get("desc") if entry else None

    def _invalidate(self, slot, index) -> None:
        """Mark (slot, index) not-ready so an in-flight replan's OLD subtree is
        never re-swapped in while the fresh plan is still being produced, and
        remember the plan that just failed so the replan can refuse to repeat it.

        H1 (round-2 review, sim run 003, 2026-08-29): ``failed_plans`` entries
        are now ``(plan, modifications)`` pairs -- ``modifications`` a flat,
        canonicalisable directive list derived (`_flatten_modifications`) from
        the grouped form `_store` caches -- so the identical-plan check
        (`plan_target`) can tell a regenerated plan with a genuinely new
        modification apart from a truly identical repeat. Dedup now compares
        the COMBINED ``(plan, modifications)`` canonical.
        """
        with self._lock:
            entry = self._cache.get((slot, index))
            if entry:
                entry["ready"] = False
                # A stale marker (e.g. IDENTICAL_PLAN_ERROR_PREFIX) must not
                # outlive the plan it described; _store rebuilds it anyway.
                entry["error"] = None
                failed_plan = list(entry.get("plan") or [])
                entry["failed_plan"] = failed_plan
                failed_mods = _flatten_modifications(entry.get("modifications"))
                failed_plans = entry.setdefault("failed_plans", [])
                if failed_plan and _canonical_plan(failed_plan, failed_mods) not in {
                    _canonical_plan(p, m) for p, m in failed_plans
                }:
                    failed_plans.append((failed_plan, failed_mods))

    def _failed_plan(self, slot, index) -> Optional[List[Dict[str, Any]]]:
        with self._lock:
            entry = self._cache.get((slot, index))
            return list(entry["failed_plan"]) if entry and entry.get("failed_plan") else None

    def _failed_plans(self, slot, index) -> List[List[Dict[str, Any]]]:
        """The plan part only of each failed-plans entry -- what the escape
        path (`_escape_plan`, `_escape_no_untried_establisher_reason`) and
        `_used_actions` need (H1, round-2 review: they keep working on
        actions/steps only, never modifications). See
        `_failed_plans_with_mods` for the pair form the identical-plan check
        (`plan_target`) needs instead."""
        with self._lock:
            entry = self._cache.get((slot, index))
            return [list(p) for p, _m in (entry.get("failed_plans") or [])] if entry else []

    def _failed_plans_with_mods(
        self, slot, index,
    ) -> List[Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]]:
        """H1 (round-2 review): like `_failed_plans` but pairs each failed
        plan with its canonicalisable (flat, ungrouped) modifications --
        used ONLY by `plan_target`'s identical-plan check."""
        with self._lock:
            entry = self._cache.get((slot, index))
            return (
                [(list(p), list(m)) for p, m in (entry.get("failed_plans") or [])]
                if entry else []
            )

    def _tried_escapes(self, slot, index) -> set:
        """M2 (round-2 review): action names of escape candidates THIS
        cache entry already offered and had rejected (by ``validate_plan``
        or a subtree-build failure) -- see ``_record_tried_escape``."""
        with self._lock:
            entry = self._cache.get((slot, index))
            return set(entry.get("tried_escapes") or ()) if entry else set()

    def _record_tried_escape(self, slot, index, action_names) -> None:
        """M2 (round-2 review): remember a validation-rejected (or subtree-
        build-failed) escape's action name(s) against this cache entry, so
        the exhaustion check (both ``_escape_plan``'s own candidate search
        and ``_escape_no_untried_establisher_reason``) treats it as "used"
        on the NEXT identical replan too. Without this, a rejected escape
        never enters ``failed_plans`` (only a stored, once-executed plan
        does), so ``_escape_plan`` would keep re-selecting -- and
        ``validate_plan`` re-rejecting -- the exact same doomed candidate
        forever (the run-003/004 symptom E2 was meant to end, for any
        target whose single-step escape can never satisfy every
        postcondition, e.g. a 2-postcondition grasp/deliver target)."""
        with self._lock:
            entry = self._cache.get((slot, index))
            if entry is None:
                return
            tried = entry.setdefault("tried_escapes", set())
            tried.update(str(a) for a in (action_names or []) if a)

    def _get_slot_context(self, slot: int) -> Dict[str, Any]:
        """The full-command context for ``slot``: {command, targets}."""
        with self._lock:
            return dict(self._slot_context.get(int(slot), {}))

    def _get_prior_targets(self, slot: int, index: int) -> List[Dict[str, Any]]:
        """Target dicts for indices < ``index`` in this slot (stable, immutable).

        Read from the slot's stored target list — never from the live cache — so
        a parallel worker sees a consistent, order-independent picture of what
        earlier targets will do.
        """
        targets = self._get_slot_context(slot).get("targets", [])
        return list(targets[:max(0, index)])

    def get_targets(self, slot: int) -> List[Dict[str, Any]]:
        """Return a deep defensive snapshot of the canonical target context."""
        return copy.deepcopy(self._get_slot_context(slot).get("targets", []))

    def record_facts(self, slot: int, facts: List[str]) -> None:
        with self._lock:
            stored = self._facts.setdefault(int(slot), [])
            self._facts[int(slot)] = apply_fact_transitions(stored, facts)

    def get_facts(self, slot: int) -> List[str]:
        with self._lock:
            return list(self._facts.get(int(slot), []))

    def reset(self) -> None:
        """Drop every cached plan/subtree (new command, test teardown)."""
        with self._lock:
            self._cache.clear()
            self._slot_context.clear()
            self._facts.clear()

    # -- top layer ----------------------------------------------------------

    def split_command(
        self, command: str, slot: Optional[int] = None,
    ) -> List[Dict[str, Any]]:
        """TOP LAYER. Blocking split of a command into structured targets.

        Returns a list of target dicts ``{desc, object, location, depends_on}``
        (see ``TOP_LAYER_SYSTEM_PROMPT``). Run inside a worker thread owned by
        ``BtNode_SplitCommand`` (offline mock returns instantly, no thread
        needed). On total LLM failure falls back to the deterministic split so
        the pipeline always proceeds.

        ``slot`` (optional) is used only for J14's acceptance audit log/
        telemetry (below) — callers that don't have one yet (ad hoc tests)
        still get the per-target log line, just with no task-scoped event.
        """
        if self._offline_mock:
            targets = _offline_mock_targets(command)
            _log_split_acceptance(targets, slot)
            return targets
        client = self._new_client()
        last_reason: Optional[str] = None
        for attempt in range(self._max_attempts):
            nonce = uuid.uuid4().hex[:8]
            temperature = min(0.9, OPENAI_TEMPERATURE + 0.2 * attempt)
            user_prompt = self._build_split_user_prompt(command, last_reason, nonce)
            parsed, err = _call_llm(
                client, TOP_LAYER_SYSTEM_PROMPT, user_prompt, temperature,
            )
            if err is not None:
                last_reason = err
                print(f"[split] attempt {attempt+1}/{self._max_attempts} -> {err}")
                continue
            raw_targets = parsed.get("targets")
            if not isinstance(raw_targets, list):
                last_reason = "your targets field must be a list"
                print(f"[split] attempt {attempt+1}/{self._max_attempts} REJECTED: {last_reason}")
                continue
            if any(isinstance(item, dict) and "depends_on" not in item
                       for item in raw_targets):
                last_reason = "each structured target must declare depends_on"
                print(f"[split] attempt {attempt+1}/{self._max_attempts} REJECTED: {last_reason}")
                continue
            targets = _normalise_targets(raw_targets)
            if not targets:
                last_reason = (
                    "you returned an EMPTY targets list. You MUST return a "
                    "NON-EMPTY list of structured targets."
                )
                print(f"[split] attempt {attempt+1}/{self._max_attempts} REJECTED: "
                      f"{last_reason}")
                continue
            targets = _append_report_target_if_needed(targets)
            person_object_reason = _reject_person_object_handling(targets)
            if person_object_reason is not None:
                last_reason = person_object_reason
                print(f"[split] attempt {attempt+1}/{self._max_attempts} REJECTED: "
                      f"{last_reason}")
                continue
            targets = _merge_transport_targets(targets)
            ok, reason = _validate_target_contract(targets)
            if not ok:
                last_reason = f"your target graph/conditions are invalid: {reason}"
                print(f"[split] attempt {attempt+1}/{self._max_attempts} REJECTED: {last_reason}")
                continue
            print(f"[split] accepted on attempt {attempt+1}: "
                  f"{[t['desc'] for t in targets]}")
            _log_split_acceptance(targets, slot)
            return targets
        print(f"[split] all {self._max_attempts} attempts failed -> "
              f"deterministic fallback split")
        return _offline_mock_targets(command)

    @staticmethod
    def _build_split_user_prompt(
        command: str,
        last_reason: Optional[str],
        nonce: str,
    ) -> str:
        body = (
            "Split the following command into self-contained targets:\n\n"
            f"{command}\n\n"
        )
        if last_reason:
            body += f"\nThe previous split was rejected: {last_reason}\n"
        if nonce:
            body += f"\n(Planning request id: {nonce} — ignore, ensures a fresh split.)"
        body += "\nReturn the JSON split now."
        return body

    # -- lower layer ----------------------------------------------------------

    def _try_escape(
        self,
        slot: int,
        index: int,
        desc: str,
        target: Optional[Dict[str, Any]],
        target_obj: str,
        target_loc: str,
        identical_marker_reason: Optional[str],
        all_targets: Optional[List[Dict[str, Any]]],
        failure_reason: Optional[str],
        known_actions: set,
        known_loc_arg: Optional[set],
        prior_plan: List[Dict[str, Any]],
    ) -> Tuple[bool, Optional[str]]:
        """D1 (Task D brief, battery run 004): try ONE deterministic escape.

        Called from ``plan_target`` only when the LLM's regenerated plan is
        IDENTICAL to the one that just failed (the final attempt). Before
        handing back the doomed identical plan, try a registry action
        establishing the failed fact that has never been tried across this
        target's ENTIRE failed-plan history — a genuinely different,
        untried strategy (e.g. ``search_object`` after ``find_object`` kept
        failing). No LLM call burns replan budget for this: it either
        produces real forward progress (stores the escape's subtree,
        ``error=None``, and returns ``(True, None)`` — the caller returns
        immediately) or returns ``(False, unrecoverable_reason)``, falling
        straight through to the existing IDENTICAL_PLAN marker path — UNLESS
        ``unrecoverable_reason`` is not ``None`` (E2, runs 003/004,
        2026-08-29): the escape ladder is fully exhausted (no untried
        establisher exists for ANY of the target's own postconditions, not
        merely the one this attempt tried), so the caller stamps an
        UNRECOVERABLE_ERROR_PREFIX marker instead of an ordinary IDENTICAL
        one — see ``plan_target``.

        Quality (round-2 review): extracted verbatim from ``plan_target``'s
        inline escape block — pure refactor, callers unaffected.

        M2 (round-2 review): ``tried_escapes`` (this cache entry's own
        rejected-escape action names, see ``_record_tried_escape``) is
        forwarded to both ``_escape_plan`` and the exhaustion helper, and a
        NEWLY rejected candidate is recorded before returning — otherwise a
        target whose single-step escape can never satisfy every
        postcondition (e.g. a 2-postcondition grasp/deliver target) would
        have ``_escape_plan`` re-select, and ``validate_plan`` re-reject,
        the exact same candidate on every subsequent identical replan.
        """
        failed_plans = self._failed_plans(slot, index)
        tried_escapes = self._tried_escapes(slot, index)
        escape = _escape_plan(
            target, target_obj, target_loc, identical_marker_reason,
            failed_plans,
            all_targets=all_targets,
            facts=self.get_facts(slot),
            known_locations=known_loc_arg,
            tried_escapes=tried_escapes,
        )
        if escape is None:
            unrecoverable_reason = _escape_no_untried_establisher_reason(
                target, failed_plans, tried_escapes=tried_escapes,
            )
            if unrecoverable_reason is not None:
                print(f"[plan:{slot}:{index}] {unrecoverable_reason} -> unrecoverable")
            return False, unrecoverable_reason
        escape_guarded, escape_dropped = _drop_foreign_contract_steps(
            escape, target, all_targets, include_ancestors=not failure_reason,
        )
        # I-4: same dangling-goto repair as the main attempt path.
        escape_guarded, escape_dangling_dropped = (
            _drop_dangling_goto_before_self_nav(escape_guarded)
        )
        escape_dropped = list(escape_dropped) + escape_dangling_dropped
        escape_ok, escape_reason = validate_plan(
            escape_guarded, desc or "", known_actions,
            known_locations=known_loc_arg,
            prior_plan=prior_plan,
            postconditions=(target or {}).get("postconditions"),
        )
        escape_subtree = None
        if escape_ok:
            try:
                escape_subtree = self.build_target_subtree(slot, index, escape_guarded)
            except Exception as exc:  # noqa: BLE001 — fall through to the marker path
                print(f"[plan:{slot}:{index}] deterministic escape subtree build "
                      f"failed: {exc!r}")
        else:
            print(f"[plan:{slot}:{index}] deterministic escape REJECTED: {escape_reason}")
        if escape_subtree is None:
            # An eligible, untried establisher existed (that is what
            # `_escape_plan` returned) — this dead end is "rejected by
            # validation" / "subtree build failed", never unrecoverable.
            # M2: but it IS now "used" for the NEXT identical replan on
            # this target — record it so `_escape_plan` never re-offers
            # (and validate_plan never re-rejects) the same candidate.
            self._record_tried_escape(
                slot, index, [s.get("action") for s in escape if isinstance(s, dict)],
            )
            return False, None
        print(f"[plan:{slot}:{index}] LLM stuck on an identical plan -> "
              f"deterministic escape {[s['action'] for s in escape_guarded]} "
              f"dropped {escape_dropped}")
        self._store(slot, index, desc, escape_guarded, escape_subtree, None)
        return True, None

    def plan_target(
        self,
        slot: int,
        index: int,
        desc: str,
        command: Optional[str] = None,
        target_obj: str = "",
        target_loc: str = "",
        prior_targets: Optional[List[Dict[str, Any]]] = None,
        failure_reason: Optional[str] = None,
        target: Optional[Dict[str, Any]] = None,
        all_targets: Optional[List[Dict[str, Any]]] = None,
        completed_steps: Optional[List[Dict[str, Any]]] = None,
    ) -> None:
        """LOWER LAYER. Blocking: plan ONE target and pre-build its subtree.

        ``command`` / ``target_obj`` / ``target_loc`` / ``prior_targets`` are the
        full-command context the TOP layer produced (see request_plan_all): the
        original NL command, the object / assigned location of THIS target, and
        the earlier targets of the same command. The lower layer plans against
        that context — the assigned location wins over any predefined default —
        and validates against prior targets so it does not repeat their work.

        ``target`` (this target's own dict, carrying its declared
        pre/postconditions) and ``all_targets`` (the full slot target list)
        drive the fact-contract boundary: the prompt tells the model which
        facts are its own to establish and which belong to sibling targets
        (``_render_contract_block``), and the deterministic
        ``_drop_foreign_contract_steps`` guard removes any step establishing
        a sibling's fact even if the model ignores the prompt. Both are
        optional (None/empty -> no-op) so callers that don't have this
        context (offline mock, ad hoc tests) are unaffected.

        Called from worker threads (request_plan_all / replan_target). Validates
        the plan the same way the legacy single-command planner did, then builds
        and caches the target's executing subtree. Never touches the Blackboard.

        I4 (round-3 adversarial review, M7): everything below is a thin
        try/except around ``_plan_target_impl`` — an unhandled exception
        ANYWHERE in that body (grouping, subtree build, ...) used to kill
        this call's daemon thread silently, leaving the cache entry
        never-ready and the executor spinning RUNNING forever ("waiting for
        target N plan..."). On ANY exception here the target is instead
        stored READY with the guaranteed fallback plan and an error of
        ``f"planner crashed: {exc!r}"`` — the entry is never left not-ready.
        """
        try:
            self._plan_target_impl(
                slot, index, desc, command=command, target_obj=target_obj,
                target_loc=target_loc, prior_targets=prior_targets,
                failure_reason=failure_reason, target=target, all_targets=all_targets,
                completed_steps=completed_steps,
            )
        except Exception as exc:  # noqa: BLE001 -- I4: never leave the entry not-ready
            error = f"planner crashed: {exc!r}"
            print(f"[plan:{slot}:{index}] {error} -> fallback acknowledgement plan")
            try:
                plan = _fallback_plan(desc)
                subtree = self.build_target_subtree(slot, index, plan)
            except Exception as build_exc:  # noqa: BLE001
                print(f"[plan:{slot}:{index}] fallback subtree build ALSO failed: "
                      f"{build_exc!r} -- entry left not-ready")
                return
            self._store(slot, index, desc, plan, subtree, error)

    def _plan_target_impl(
        self,
        slot: int,
        index: int,
        desc: str,
        command: Optional[str] = None,
        target_obj: str = "",
        target_loc: str = "",
        prior_targets: Optional[List[Dict[str, Any]]] = None,
        failure_reason: Optional[str] = None,
        target: Optional[Dict[str, Any]] = None,
        all_targets: Optional[List[Dict[str, Any]]] = None,
        completed_steps: Optional[List[Dict[str, Any]]] = None,
    ) -> None:
        """The actual planning body of ``plan_target`` -- see that docstring."""
        if self._offline_mock:
            plan = _offline_mock_plan(desc)
            subtree = self.build_target_subtree(slot, index, plan)
            self._store(slot, index, desc, plan, subtree, None)
            return

        client = self._new_client()
        state_log = (
            [] if not failure_reason
            else [f"previous attempt failed: {failure_reason}"]
        )
        last_reason: Optional[str] = failure_reason
        # A replan can be re-invoked with a failure_reason that is ITSELF a
        # previous identical-plan marker (repeated identical replans across
        # several cycles) — strip the leading prefix once so the final-attempt
        # marker below never nests ("identical to failed plan: identical to
        # failed plan: ...").
        identical_marker_reason = failure_reason
        if isinstance(identical_marker_reason, str):
            nested_prefix = f"{IDENTICAL_PLAN_ERROR_PREFIX}: "
            if identical_marker_reason.startswith(nested_prefix):
                identical_marker_reason = identical_marker_reason[len(nested_prefix):]
        known_locs = set(KNOWN_LOCATIONS.keys())
        known_loc_arg = (known_locs | START_LOCATION_ALIASES) if known_locs else None
        known_actions = set(ACTION_FACTORIES.keys())
        for attempt in range(self._max_attempts):
            nonce = uuid.uuid4().hex[:8]
            temperature = min(0.9, OPENAI_TEMPERATURE + 0.2 * attempt)
            if last_reason and attempt == 0:
                # A seeded failure warms the sampler so the model explores a
                # fresh plan instead of resampling the dead end.
                temperature = min(0.9, OPENAI_TEMPERATURE + 0.5)
            user_prompt = _build_lower_layer_user_prompt(
                command or desc,
                desc,
                target_obj,
                target_loc,
                prior_targets or [],
                state_log,
                last_reason,
                nonce=nonce,
                verified_facts=self.get_facts(slot),
                contract=target,
                all_targets=all_targets,
                include_ancestors=not failure_reason,
                completed_steps=completed_steps,
            )
            parsed, err = _call_llm(
                client, LOWER_LAYER_SYSTEM_PROMPT, user_prompt, temperature,
            )
            if err is not None:
                last_reason = err
                print(f"[plan:{slot}:{index}] attempt {attempt+1}/{self._max_attempts} "
                      f"-> {err}")
                continue
            # M-1 (round-3 fix review): the LLM's step_index is bound to its
            # RAW plan array (parsed.get("plan", [])), not the post-
            # `_clean_plan` frame -- `_clean_plan` itself can drop steps
            # (unknown action, consecutive duplicates), so the "old" frame
            # for remapping must start there too. `_clean_plan_core` returns
            # each kept step's raw index as the first leg of the composed
            # raw->final map built below.
            cleaned, dropped, clean_kept_raw_idx = _clean_plan_core(parsed.get("plan", []))
            # I3 (round-3 adversarial review, M6): the LLM's modifications
            # index into THIS plan (post `_clean_plan`, pre-guard) — capture
            # it before the contract guard / dangling-goto drop reduce it
            # further, so a modification's step_index can be translated
            # through both drops below.
            pre_guard_plan = cleaned
            cleaned, contract_dropped = _drop_foreign_contract_steps(
                cleaned, target, all_targets, include_ancestors=not failure_reason,
            )
            post_guard_plan = cleaned
            # I-4 (round-2 review): repair a plan the guard just reduced to
            # e.g. [goto, deliver] -- validate_plan's goto-before-self-nav
            # rule would otherwise always reject it, burning an LLM
            # round-trip. _contract_drop_note (below) only walks
            # `contract_dropped`, not this cleanup's entries -- a dangling
            # goto has no "owning" sibling target to name.
            cleaned, dangling_dropped = _drop_dangling_goto_before_self_nav(cleaned)
            post_dangling_plan = cleaned
            # H-3 (round-3 fix review): a re-emitted step element-wise
            # identical to one of THIS target's own already-completed steps
            # (a partial postcondition-gate commit from a prior attempt,
            # threaded through as `completed_steps`) is dropped too -- e.g.
            # a replan after `[held(x), placed(x,t)]` partially committed
            # must not re-grasp an already-held object.
            cleaned, completed_dup_dropped = _drop_completed_duplicate_steps(
                cleaned, completed_steps,
            )
            dropped = list(dropped) + contract_dropped + dangling_dropped + completed_dup_dropped
            # I3/M-1: compose the raw(LLM JSON)->new(final `cleaned`) index
            # map through all four drops: `_clean_plan_core`'s raw-index
            # list, then by identity through the three guard drops (see
            # `_kept_indices`).
            contract_kept_idx = _kept_indices(pre_guard_plan, post_guard_plan)
            dangling_kept_idx = _kept_indices(post_guard_plan, post_dangling_plan)
            completed_kept_idx = _kept_indices(post_dangling_plan, cleaned)
            mod_old_to_new = {
                clean_kept_raw_idx[contract_kept_idx[dangling_kept_idx[i]]]: new
                for new, i in enumerate(completed_kept_idx)
            }
            raw_actions = [
                s.get("action") if isinstance(s, dict) else f"<{type(s).__name__}>"
                for s in (parsed.get("plan", []) or [])
            ]
            print(f"[plan:{slot}:{index}] attempt {attempt+1}/{self._max_attempts}: "
                  f"raw {raw_actions} | kept {[s['action'] for s in cleaned]} "
                  f"| dropped {dropped}")
            raw_mods = parsed.get("modifications")
            if raw_mods:
                # A rejected modification (schema/role mismatch) is otherwise
                # unrecoverable from orchestrator.log — dump the payload that
                # was actually sent, truncated so one bad attempt can't flood
                # the log.
                mods_dump = json.dumps(raw_mods, default=str)
                if len(mods_dump) > 300:
                    mods_dump = mods_dump[:300] + "...(truncated)"
                print(f"[plan:{slot}:{index}] attempt {attempt+1}/{self._max_attempts} "
                      f"modifications: {mods_dump}")
            if not cleaned:
                last_reason = (
                    "you returned an EMPTY plan (or only unknown actions). You "
                    "MUST return a NON-EMPTY plan of the known actions — never "
                    "refuse. If part of the target is impossible, still emit the "
                    "doable steps and finish with announce(text=...) explaining "
                    "what you could not do — and set \"acknowledgement\": true in "
                    "that announce's params (I-3, round-2 review): it is a "
                    "refusal, never an answer."
                )
                continue
            # J3 (round-3 adversarial review, M8): this target's own already-
            # completed steps (a postcondition gate's partial commit) seed
            # validate_plan's cross-target state the same way an ancestor
            # target's plan does -- e.g. a preserved grasp means a fresh
            # place() is not rejected for "no held(x)".
            prior_plan = _flatten_prior_plans(self, slot, index) + list(completed_steps or [])
            ok, reason = validate_plan(
                cleaned, desc or "", known_actions,
                known_locations=known_loc_arg,
                prior_plan=prior_plan,
                postconditions=(target or {}).get("postconditions"),
            )
            if not ok:
                # The contract-boundary guard may have just removed the step(s)
                # that would have satisfied this target's own postcondition
                # (e.g. it planned only the foreign `deliver`). Tell the retry
                # what was dropped and why so it does not just re-emit them.
                last_reason = reason + _contract_drop_note(contract_dropped, target, all_targets)
                print(f"[plan:{slot}:{index}] attempt {attempt+1}/{self._max_attempts} "
                      f"REJECTED: {reason}")
                continue
            # Modifications: the LLM may attach typed, template-constrained
            # directives to plan steps. Validate them against the step small
            # trees — an invalid modification rejects the WHOLE attempt (never
            # partially applied), feeding the reason back into the next prompt.
            # I3: step_index is bound to the plan the LLM wrote (pre-guard) —
            # translate it through the contract-boundary guard and the
            # dangling-goto drop before validating/grouping against `cleaned`.
            mods, mod_index_dropped = _remap_modification_step_indices(
                parsed.get("modifications"), mod_old_to_new, cleaned,
            )
            if mod_index_dropped:
                dropped = dropped + mod_index_dropped
                print(f"[plan:{slot}:{index}] attempt {attempt+1}/{self._max_attempts} "
                      f"modification step_index dropped (step removed by guard): "
                      f"{mod_index_dropped}")
            ok, reason = validate_plan_modifications(mods, cleaned)
            if not ok:
                last_reason = f"invalid modifications: {reason}"
                print(f"[plan:{slot}:{index}] attempt {attempt+1}/{self._max_attempts} "
                      f"REJECTED: {last_reason}")
                continue
            # H1 (round-2 review, sim run 003, 2026-08-29): the identical
            # check includes each failed entry's modifications too -- a
            # regenerated plan repeating the same steps but attaching a
            # genuinely new modification (e.g. a wider pan-tilt-sweep) is a
            # legitimately different strategy, not a doomed repeat.
            failed_canonicals = {
                _canonical_plan(p, m) for p, m in self._failed_plans_with_mods(slot, index)
            }
            identical = _canonical_plan(cleaned, mods) in failed_canonicals
            if identical and attempt < self._max_attempts - 1:
                last_reason = (
                    "the regenerated plan was IDENTICAL to the plan that just failed "
                    f"({identical_marker_reason or 'unknown reason'}) — you MUST change it: add, remove "
                    "or reorder steps so the failure cannot recur. "
                    + _alternatives_for_reason(identical_marker_reason or "")
                )
                print(f"[plan:{slot}:{index}] attempt {attempt+1}/{self._max_attempts} "
                      f"REJECTED: identical to failed plan")
                continue
            unrecoverable_reason = None
            if identical:
                handled, unrecoverable_reason = self._try_escape(
                    slot, index, desc, target, target_obj, target_loc,
                    identical_marker_reason, all_targets, failure_reason,
                    known_actions, known_loc_arg, prior_plan,
                )
                if handled:
                    return
            # Accepted — build the subtree on this (worker) thread, then cache.
            # H-3 (round-3 fix review): `completed_steps` must reach the
            # subtree's gates the same way `replace_target_plan` already
            # threads it -- otherwise the gate reasons only over the NEW
            # plan, so a fact an earlier (already-succeeded) step of THIS
            # target established (e.g. `held(x)` from a completed grasp) has
            # no evidence and the target can never pass its own post gate.
            try:
                subtree = self.build_target_subtree(
                    slot, index, cleaned, modifications=mods,
                    completed_steps=completed_steps,
                )
            except Exception as exc:  # noqa: BLE001 — surface for retry
                last_reason = f"subtree build failed: {exc!r}"
                print(f"[plan:{slot}:{index}] attempt {attempt+1}/{self._max_attempts} "
                      f"-> {last_reason}")
                continue
            print(f"[plan:{slot}:{index}] accepted on attempt {attempt+1}: "
                  f"{[s['action'] for s in cleaned]}"
                  + (f" mods={len(mods or [])}" if mods else "")
                  + (" (IDENTICAL — executor will not run it)" if identical else ""))
            # A final-attempt plan that still repeats the failed one is handed
            # back honestly (not the fallback) but MARKED: the executor skips
            # it instead of re-running a known dead end. E2 (runs 003/004,
            # 2026-08-29): when the escape ladder is fully exhausted
            # (unrecoverable_reason set), the marker is UNRECOVERABLE_ERROR_
            # PREFIX instead of the ordinary IDENTICAL_PLAN one — the
            # executor forces the replan budget to exhausted for that marker
            # so the target ends instead of cycling through more of these.
            if identical:
                error = (
                    f"{UNRECOVERABLE_ERROR_PREFIX}: {unrecoverable_reason}"
                    if unrecoverable_reason is not None
                    else f"{IDENTICAL_PLAN_ERROR_PREFIX}: {identical_marker_reason}"
                )
            else:
                error = None
            self._store(slot, index, desc, cleaned, subtree, error,
                        modifications=group_modifications_by_step(cleaned, mods or []))
            return
        # Every attempt failed -> guaranteed non-empty fallback plan.
        # H-3 (round-3 fix review): same completed_steps threading as the
        # accepted path above -- a fallback for a target with prior
        # completed steps must still gate over the whole (completed +
        # fallback) plan, not just the fallback acknowledgement.
        plan = _fallback_plan(desc)
        subtree = self.build_target_subtree(slot, index, plan, completed_steps=completed_steps)
        reason = f"all {self._max_attempts} attempts failed (last reason: {last_reason})"
        print(f"[plan:{slot}:{index}] {reason} -> fallback acknowledgement plan")
        self._store(slot, index, desc, plan, subtree, reason)

    def request_plan_all(
        self,
        slot: int,
        targets: List[Any],
        command: Optional[str] = None,
    ) -> None:
        """LOWER LAYER, PARALLEL. Spawn one daemon thread per target.

        Stores the full-command context (``command`` + the structured target
        list) so every worker can plan its delta without repeating earlier
        targets. Pre-seeds the cache (ready=False) so a stale plan/subtree from
        a prior use of this slot can never be swapped in while the fresh plans
        are being produced. Returns immediately; poll ``all_targets_ready``.

        ``targets`` may be either the structured dicts from ``split_command`` or
        plain strings (kept for the legacy dev-tests / mock paths) — normalised
        here.
        """
        if not isinstance(targets, list):
            raise ValueError("targets must be a list")
        for index, target in enumerate(targets):
            if isinstance(target, dict) and "depends_on" not in target:
                raise ValueError(f"target {index} depends_on is required")
            if isinstance(target, dict) and not isinstance(target.get("depends_on"), list):
                raise ValueError(f"target {index} depends_on must be a list")
            if not isinstance(target, (dict, str)):
                raise ValueError(f"target {index} must be a mapping or plain string")
        norm_targets = _normalise_targets(targets)
        ok, reason = _validate_target_contract(norm_targets)
        if not ok:
            raise ValueError(reason or "invalid target contract")
        with self._lock:
            self._facts.pop(int(slot), None)
            self._slot_context[int(slot)] = {
                "command": command or "",
                "targets": norm_targets,
            }
            for i, t in enumerate(norm_targets):
                self._cache[(slot, i)] = {
                    "desc": str(t.get("desc") or ""),
                    "plan": [],
                    "subtree": None,
                    "ready": False,
                    "error": None,
                    "failed_plans": [],
                    "tried_escapes": set(),
                }
        for i, t in enumerate(norm_targets):
            threading.Thread(
                target=self.plan_target,
                args=(
                    slot, i, str(t.get("desc") or ""),
                    command or "", str(t.get("object") or ""),
                    str(t.get("location") or ""),
                    [dict(x) for x in _dependency_ancestor_targets(norm_targets, i)],
                ),
                kwargs={"target": norm_targets[i], "all_targets": norm_targets},
                daemon=True,
            ).start()

    def replan_target(
        self, slot: int, index: int, reason: str = "",
        completed_steps: Optional[List[Dict[str, Any]]] = None,
    ) -> None:
        """Re-plan ONE target on a fresh daemon thread (lower-layer scope only).

        Invalidates the cached entry first so the OLD subtree is never re-swapped
        in while the fresh plan is in flight. Top layer and the other targets
        are untouched — a failing target re-plans only itself. The replan keeps
        the same full-command context (command / object / location / prior
        targets) so a delta re-plan does not lose the assigned location.

        ``completed_steps`` (J3, round-3 adversarial review, M8): steps of
        THIS target's own plan that already succeeded (the postcondition
        gate's partial commit before it failed) — threaded through to
        ``plan_target`` so the fresh plan does not redo them.
        """
        desc = self._get_desc(slot, index)
        if desc is None:
            return
        self._invalidate(slot, index)
        ctx = self._get_slot_context(slot)
        targets = ctx.get("targets", [])
        t = targets[index] if index < len(targets) else {}
        threading.Thread(
            target=self.plan_target,
            args=(
                slot, index, desc,
                ctx.get("command") or "", str(t.get("object") or ""),
                str(t.get("location") or ""),
                [dict(x) for x in _dependency_ancestor_targets(targets, index)],
            ),
            kwargs={
                "failure_reason": reason, "target": t, "all_targets": targets,
                "completed_steps": completed_steps,
            },
            daemon=True,
        ).start()

    def replace_target_plan(
        self,
        slot: int,
        index: int,
        plan: List[Dict[str, Any]],
        reason: str = "supervisor global replan",
        completed_steps: Optional[List[Dict[str, Any]]] = None,
    ) -> None:
        """Install a supervisor-validated remaining plan for one target.

        This is synchronous and network-free: the supervisor has already
        validated the typed plan.  Building and caching a fresh subtree here
        lets ``DynamicExecutor`` swap it at the next safe tick boundary.

        Also runs the deterministic contract-boundary guard
        (``_drop_foreign_contract_steps``): unlike ``plan_target`` this path
        never talks to an LLM, but the supervisor's replacement plan is still
        free-form JSON that could name a step belonging to a sibling target
        (e.g. a premature ``deliver``). The target/all-targets context is
        available here from this planner's own slot context — no caller
        change needed.

        Also checks (log-only, see body) that the installed plan plus
        ``completed_steps`` together cover the target's declared
        postconditions -- the same coverage rule ``plan_target`` enforces via
        ``validate_plan(..., postconditions=...)``, but not a hard rejection
        here since this synchronous path has no retry loop to hand a
        rejection reason back to.
        """
        desc = self._get_desc(slot, index) or f"target {index}"
        cleaned, _ = _clean_plan(plan)
        targets = self._get_slot_context(slot).get("targets", [])
        target = targets[index] if index < len(targets) else None
        guarded, contract_dropped = _drop_foreign_contract_steps(
            cleaned, target, targets, include_ancestors=False,
        )
        if contract_dropped:
            print(f"[replace:{slot}:{index}] contract-boundary dropped {contract_dropped}")
        if cleaned and not guarded:
            # M-1 (round-2 review): the guard stripped every step -- that IS
            # the correct outcome, not a defect to paper over. A supervisor
            # plan that only established a LATER target's fact (e.g. a bare
            # `deliver` for t0 that is really t1's premature delivery) is
            # exactly the double-execution the guard exists to remove;
            # re-installing the unfiltered plan re-creates it. Install the
            # guarded (empty) plan instead -- its postcondition gate (if the
            # target declares one) then fails cleanly on the next tick and
            # the normal replan path takes over, same as any other
            # under-specified plan. Still log the warning already emitted.
            print(f"[replace:{slot}:{index}] contract-boundary guard emptied the plan "
                  f"-- installing it anyway (the postcondition gate will fail "
                  f"cleanly and trigger a replan) instead of re-installing the "
                  f"unfiltered plan")
        cleaned = guarded
        # M2: this synchronous path is supervisor RECOVERY, never an answer
        # attempt -- GLOBAL_REPLAN explicitly allows installing an
        # announce(text=...) step (supervision/prompts.py), and unlike
        # plan_target's LLM-driven plans there is no contract distinguishing
        # "the answer" from "sorry, I could not do X" here. Tag every
        # installed announce step that carries a literal text with
        # acknowledgement=True (mirrors orchestrator._fallback_plan's own
        # tagging) so validators._action_verdict's answered-gate fallback
        # never reads a refusal/recovery announce as a spoken answer. A
        # text-less announce (reporting a prior gathering step's result) is
        # left untouched -- that is the legitimate "tell me" pattern.
        tagged: List[Dict[str, Any]] = []
        for step in cleaned:
            if (isinstance(step, dict) and step.get("action") == "announce"
                    and isinstance(step.get("params"), dict)
                    and str(step["params"].get("text") or "").strip()):
                params = dict(step["params"])
                params["acknowledgement"] = True
                step = {**step, "params": params}
            tagged.append(step)
        cleaned = tagged
        # Postcondition coverage: the supervisor's plan is the REMAINING
        # steps after `completed_steps`, so a postcondition a completed step
        # already established must not be demanded again from the remaining
        # plan. This is a warning, not a rejection -- unlike plan_target,
        # this synchronous one-shot path has no retry loop to feed a
        # rejection reason back into, so blocking here would either strand
        # the target with a stale subtree or force installing an even worse
        # fallback. Surfacing it in the log keeps the assumption checkable.
        target_postconditions = (target or {}).get("postconditions") if target else None
        if target_postconditions:
            completed_established = established_predicates(completed_steps or [])
            remaining_postconditions = [
                cond for cond in target_postconditions
                if (parse_fact(cond)[0] is None
                    or parse_fact(cond)[0].predicate not in completed_established)
            ]
            coverage_reason = uncovered_postcondition_reason(cleaned, remaining_postconditions)
            if coverage_reason is not None:
                print(f"[replace:{slot}:{index}] WARNING: {coverage_reason} "
                      "(supervisor replacement plan installed anyway -- no "
                      "retry path in replace_target_plan)")
        subtree = self.build_target_subtree(
            slot, index, cleaned, completed_steps=completed_steps,
        )
        self._store(slot, index, desc, cleaned, subtree, reason)

    # -- polling (executor thread) -------------------------------------------

    def is_target_ready(self, slot: int, index: int) -> bool:
        with self._lock:
            entry = self._cache.get((slot, index))
        return bool(entry and entry.get("ready"))

    def get_target_subtree(self, slot: int, index: int) -> Optional[py_trees.behaviour.Behaviour]:
        with self._lock:
            entry = self._cache.get((slot, index))
        if not entry or not entry.get("ready"):
            return None
        return entry.get("subtree")

    def get_action_plan(self, slot: int, index: int) -> List[Dict[str, Any]]:
        """M-8 (round-2 review): pre-existing lock-scope note — see ``_get_desc``."""
        with self._lock:
            entry = self._cache.get((slot, index))
        if not entry:
            return []
        return list(entry.get("plan") or [])

    def get_error(self, slot: int, index: int) -> Optional[str]:
        """The cache entry's ``error`` (None when absent or ready with no error).

        M-8 (round-2 review): pre-existing lock-scope note — see ``_get_desc``.
        """
        with self._lock:
            entry = self._cache.get((slot, index))
        return entry.get("error") if entry else None

    def get_modifications(self, slot: int, index: int) -> Dict[int, List[Dict[str, Any]]]:
        """The step-indexed modification groups for a cached target plan.

        Empty when the accepted plan carried no modifications.
        """
        with self._lock:
            entry = self._cache.get((slot, index))
        if not entry:
            return {}
        mods = entry.get("modifications") or {}
        return {int(k): list(v) for k, v in mods.items()}

    def all_targets_ready(self, slot: int, num: int) -> bool:
        with self._lock:
            return all(
                self._cache.get((slot, i), {}).get("ready")
                for i in range(int(num))
            )

    # -- subtree construction -------------------------------------------------

    def build_target_subtree(
        self,
        slot: int,
        index: int,
        action_plan: List[Dict[str, Any]],
        *,
        modifications: Any = None,
        completed_steps: Optional[List[Dict[str, Any]]] = None,
    ) -> py_trees.composites.Sequence:
        """Compose the per-target executing subtree for ``action_plan``.

        Returns a FRESH Sequence (built once, swapped in once). Reads its step
        plan from the BB slot ``SAVED_TARGET_PLAN_PREFIX+<slot>_<i>`` (written by
        ``BtNode_PlanAllTargets`` on the executor thread) so every MaterialiseStep
        only ever reads the Blackboard, and announces the target (CURRENT_TARGET)
        on entry.

        ``modifications`` is the plan-level modification list (validated by
        ``validate_plan_modifications`` before this is called). Each step's
        small tree is built then its modifications applied at plan-build time,
        BEFORE wrapping — the applied change is deterministic and never mutates
        a live tree.

        Structure:
            Sequence("target:<slot>:<i>")
            ├── BtNode_AnnounceFromBB(CURRENT_TARGET, prefix="Next: ")
            └── per step k: Sequence[
                    BtNode_MaterialiseStep(plan_key, k),
                    supervised ACTION_FACTORIES[act](),  # modifications applied here
                    BtNode_LogStepResult(ok=True),
                ]
        """
        seq = py_trees.composites.Sequence(f"target:{slot}:{index}", memory=True)
        ctx = self._get_slot_context(slot)
        targets = ctx.get("targets") or []
        target = targets[index] if 0 <= index < len(targets) and isinstance(targets[index], dict) else {}
        preconditions = target.get("preconditions") or []
        postconditions = target.get("postconditions") or []
        # The gates reason over the WHOLE target plan: on a supervisor
        # replacement swap the already-executed prefix (``completed_steps``)
        # still establishes facts (e.g. place(kitchen_table) -> at_robot), so
        # the precondition gate must keep deferring them rather than checking
        # them at re-entry and failing on UNKNOWN.
        full_plan = list(completed_steps or []) + list(action_plan)
        if preconditions:
            seq.add_child(BtNode_TargetPreconditionCheck(
                f"precondition gate:{slot}:{index}", preconditions, index,
                action_plan=full_plan, slot=slot,
            ))
        seq.add_child(BtNode_AnnounceFromBB(
            f"announce target:{slot}:{index}",
            bb_keys.CURRENT_TARGET,
            prefix="Next: ",
        ))
        plan_key = bb_keys.SAVED_TARGET_PLAN_PREFIX + f"{slot}_{index}"
        step_mods = group_modifications_by_step(action_plan, modifications or [])
        for k, step in enumerate(action_plan):
            action = step.get("action")
            factory = ACTION_FACTORIES.get(action)
            if factory is None:
                continue
            step_seq = py_trees.composites.Sequence(
                f"target:{slot}:{index}:step{k}", memory=True,
            )
            step_seq.add_child(BtNode_MaterialiseStep(
                f"materialise:{slot}:{index}:{k}", plan_key, k,
            ))
            small_tree = factory()
            mods = step_mods.get(k) or []
            if mods:
                small_tree, _applied = apply_modifications(
                    small_tree, action, mods,
                )
            step_seq.add_child(wrap_action_factory(
                action, lambda: small_tree, get_default_supervisor(),
            ))
            step_seq.add_child(BtNode_LogStepResult(
                f"log:{slot}:{index}:{k}", succeeded=True,
            ))
            step_seq.add_child(BtNode_SupervisorBarrier(
                f"supervisor barrier:{slot}:{index}:{k}",
            ))
            seq.add_child(step_seq)
        deferred_possible = any(self_established_facts(s) for s in full_plan)
        if postconditions or (preconditions and deferred_possible):
            seq.add_child(BtNode_TargetPostconditionCheck(
                f"postcondition gate:{slot}:{index}",
                postconditions,
                index,
                action_plan,
                target_object=str(target.get("object") or ""),
                completed_steps=completed_steps,
                target_location=str(target.get("location") or ""),
                facts_writer=lambda facts: self.record_facts(slot, facts),
                slot=slot,
            ))
        return seq
