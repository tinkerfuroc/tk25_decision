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
)
from .modifiable_nodes import (
    TEMPLATES,
    apply_modifications,
    group_modifications_by_step,
    validate_plan_modifications,
)
from .planner_validators import validate_plan, validate_dag
from .validators import apply_fact_transitions, canonical_fact, parse_fact
from .small_trees import (
    ACTION_FACTORIES,
    bb_keys,
    BtNode_AnnounceFromBB,
    get_small_tree_roles,
)
from .supervision.runtime import get_default_supervisor, wrap_action_factory
from .action_contracts import (
    ACTION_CONTRACTS,
    IDENTICAL_PLAN_ERROR_PREFIX,
    established_facts,
    render_self_satisfied_rule,
    self_established_facts,
)
from .orchestrator import (
    SYSTEM_PROMPT,
    KNOWN_LOCATIONS,
    KNOWN_OBJECT_PROMPTS,
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
       whether conditions are true. Empty condition lists are allowed.
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
    - "vlm-template" (actions count / vlm_fallback): params
      {"question_template": "<must contain the {value} placeholder>"} — swap
      the VLM question.
    - "announce-text" (actions whose tree has a literal-message announce):
      params {"text": "<spoken text>"} — change a spoken announcement.
    - "pan-tilt-sweep" (actions find_person / find_object / describe_person):
      params {"pan_deg": [floats], "tilt_deg": [floats]} — override the
      scan sweep ranges.
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
    # target's postconditions (written to the ledger when that target
    # completes). A precondition no earlier target establishes can never be
    # satisfied by the runtime gate (which runs before the target's own
    # steps), so it must be dropped here. Unparseable conditions are left
    # alone -- ``_validate_target_contract`` rejects those with a retry.
    established: set[str] = set()
    for i, target in enumerate(normalized):
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
                if canonical_fact(fact) in established:
                    kept.append(condition)
                else:
                    dropped.append(condition)
            if dropped:
                print(f"[split] target {i}: dropped unestablishable precondition(s): {dropped}")
            target["preconditions"] = kept

        postconditions = target.get("postconditions")
        if isinstance(postconditions, list):
            for condition in postconditions:
                if not isinstance(condition, str):
                    continue
                fact, _error = parse_fact(condition)
                if fact is not None:
                    established.add(canonical_fact(fact))

    return normalized


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
    return True, None


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


def _canonical_plan(plan: List[Dict[str, Any]]) -> tuple:
    """Order/param-order-insensitive-within-a-step identity for a plan.

    Used to detect a regenerated replan that is IDENTICAL to the plan that
    just failed, so `plan_target` can reject it and force the model to
    actually change something instead of burning the replan budget re-running
    the same doomed steps.
    """
    return tuple(
        (str(s.get("action")), tuple(sorted((str(k), str(v)) for k, v in (s.get("params") or {}).items())))
        for s in plan or []
    )


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


def _render_contract_block(
    contract: Optional[Dict[str, Any]],
    all_targets: Optional[List[Dict[str, Any]]],
) -> str:
    """Render the "fact contract" block for the lower-layer prompt.

    Tells the model its OWN target's declared pre/postconditions (from the
    top-layer split) plus, for every OTHER target in the same command
    (ancestor or successor), the postconditions that belong to that target
    and NOT to this one — facts it must never establish itself. See
    ``task-A-brief.md`` for the "bring me a spam" defect this closes.
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
        f"  requires (established by earlier targets, do NOT re-establish): {requires_str}",
        f"  must establish: {establish_str}",
    ]
    own_post_canon: set = set()
    for raw in postconditions:
        fact, _error = parse_fact(str(raw))
        if fact is not None:
            own_post_canon.add(canonical_fact(fact))
    owned_lines = []
    for i, other in enumerate(all_targets or []):
        if _is_same_target(other, contract):
            continue
        other_facts = []
        for raw in other.get("postconditions") or []:
            fact, _error = parse_fact(str(raw))
            if fact is None:
                continue
            canon = canonical_fact(fact)
            if canon not in own_post_canon and canon not in other_facts:
                other_facts.append(canon)
        if other_facts:
            owned_lines.append(
                f"  owned by T{i} \"{other.get('desc') or ''}\": {', '.join(other_facts)}"
            )
    if owned_lines:
        lines.append(
            "Plan ONLY the steps that establish the \"must establish\" facts from the\n"
            "\"requires\" state. Facts owned by OTHER targets of this command are listed\n"
            "below — never perform an action that establishes one of them:"
        )
        lines.extend(owned_lines)
    return "\n".join(lines) + "\n"


def _drop_foreign_contract_steps(
    plan: List[Dict[str, Any]],
    target: Optional[Dict[str, Any]],
    all_targets: Optional[List[Dict[str, Any]]],
) -> Tuple[List[Dict[str, Any]], List[str]]:
    """Deterministically drop plan steps that establish ANOTHER target's fact.

    A step is dropped iff at least one of its ``established_facts(step)`` is
    (a) NOT a canonical postcondition of ``target`` and (b) IS a canonical
    postcondition of some OTHER target in ``all_targets``. Facts are compared
    in FULL (``at_robot(kitchen)`` vs ``at_robot(balcony)`` never collide) —
    never by predicate alone. Only ``establishes`` (action_contracts.py)
    counts; ``self_establishes`` (incidental navigation, e.g. place/deliver
    reaching their own location) never triggers a drop, since it is not
    consulted here at all.

    No LLM call, no cost — this is the deterministic guard that closes the
    "bring me a spam from the laundry_desk" defect even if the prompt's
    contract block (``_render_contract_block``) is ignored by the model.
    """
    if not target or not all_targets:
        return list(plan or []), []
    own_post_canon: set = set()
    for raw in target.get("postconditions") or []:
        fact, _error = parse_fact(str(raw))
        if fact is not None:
            own_post_canon.add(canonical_fact(fact))
    foreign_post_canon: set = set()
    for other in all_targets:
        if _is_same_target(other, target):
            continue
        for raw in other.get("postconditions") or []:
            fact, _error = parse_fact(str(raw))
            if fact is not None:
                foreign_post_canon.add(canonical_fact(fact))
    kept: List[Dict[str, Any]] = []
    dropped: List[str] = []
    for step in plan or []:
        facts = established_facts(step) if isinstance(step, dict) else []
        foreign_fact = next(
            (f for f in facts if f not in own_post_canon and f in foreign_post_canon),
            None,
        )
        if foreign_fact is not None:
            dropped.append(f"contract:{step.get('action')}->{foreign_fact}")
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
    for i, other in enumerate(all_targets):
        if _is_same_target(other, target):
            continue
        for raw in other.get("postconditions") or []:
            fact, _error = parse_fact(str(raw))
            if fact is None:
                continue
            owner_by_fact.setdefault(
                canonical_fact(fact), f"T{i} \"{other.get('desc') or ''}\""
            )
    parts = []
    for entry in contract_entries:
        _, _, rest = entry.partition(":")
        action, _, fact = rest.partition("->")
        owner = owner_by_fact.get(fact, "another target")
        parts.append(f"{action} (establishes {fact}, owned by {owner})")
    if not parts:
        return ""
    return (
        " Note: the contract-boundary guard already removed "
        + "; ".join(parts)
        + " from your last plan because that fact belongs to a DIFFERENT "
        "target of this command — do not re-emit them."
    )


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
) -> str:
    """Build the lower-layer user prompt WITH full-command + prior-target context.

    Embeds the original NL command, this target's desc, the top-layer ASSIGNED
    object/location (authoritative — wins over any default), the prior targets
    (so the worker plans only its delta), the action catalogue, and the known
    arena data. ``LOWER_LAYER_SYSTEM_PROMPT`` supplies the plan JSON contract
    and the hard rules; this supplies the situation.
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
    contract_block = _render_contract_block(contract, all_targets)
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
        return openai.OpenAI(
            api_key=OPENAI_API_KEY,
            base_url="https://openrouter.ai/api/v1",
        )

    # -- cache ---------------------------------------------------------------

    def _store(self, slot, index, desc, plan, subtree, error, modifications=None):
        with self._lock:
            prev = self._cache.get((slot, index))
            failed_plans = list(prev.get("failed_plans") or []) if prev else []
            self._cache[(slot, index)] = {
                "desc": desc,
                "plan": list(plan),
                "subtree": subtree,
                "ready": True,
                "error": error,
                "modifications": modifications,
                "failed_plans": failed_plans,
            }

    def _get_desc(self, slot, index) -> Optional[str]:
        with self._lock:
            entry = self._cache.get((slot, index))
        return entry.get("desc") if entry else None

    def _invalidate(self, slot, index) -> None:
        """Mark (slot, index) not-ready so an in-flight replan's OLD subtree is
        never re-swapped in while the fresh plan is still being produced, and
        remember the plan that just failed so the replan can refuse to repeat it."""
        with self._lock:
            entry = self._cache.get((slot, index))
            if entry:
                entry["ready"] = False
                # A stale marker (e.g. IDENTICAL_PLAN_ERROR_PREFIX) must not
                # outlive the plan it described; _store rebuilds it anyway.
                entry["error"] = None
                failed_plan = list(entry.get("plan") or [])
                entry["failed_plan"] = failed_plan
                failed_plans = entry.setdefault("failed_plans", [])
                if failed_plan and _canonical_plan(failed_plan) not in {
                    _canonical_plan(p) for p in failed_plans
                }:
                    failed_plans.append(failed_plan)

    def _failed_plan(self, slot, index) -> Optional[List[Dict[str, Any]]]:
        with self._lock:
            entry = self._cache.get((slot, index))
        return list(entry["failed_plan"]) if entry and entry.get("failed_plan") else None

    def _failed_plans(self, slot, index) -> List[List[Dict[str, Any]]]:
        with self._lock:
            entry = self._cache.get((slot, index))
        return [list(p) for p in (entry.get("failed_plans") or [])] if entry else []

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

    def split_command(self, command: str) -> List[Dict[str, Any]]:
        """TOP LAYER. Blocking split of a command into structured targets.

        Returns a list of target dicts ``{desc, object, location, depends_on}``
        (see ``TOP_LAYER_SYSTEM_PROMPT``). Run inside a worker thread owned by
        ``BtNode_SplitCommand`` (offline mock returns instantly, no thread
        needed). On total LLM failure falls back to the deterministic split so
        the pipeline always proceeds.
        """
        if self._offline_mock:
            return _offline_mock_targets(command)
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
            ok, reason = _validate_target_contract(targets)
            if not ok:
                last_reason = f"your target graph/conditions are invalid: {reason}"
                print(f"[split] attempt {attempt+1}/{self._max_attempts} REJECTED: {last_reason}")
                continue
            print(f"[split] accepted on attempt {attempt+1}: "
                  f"{[t['desc'] for t in targets]}")
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
        """
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
            )
            parsed, err = _call_llm(
                client, LOWER_LAYER_SYSTEM_PROMPT, user_prompt, temperature,
            )
            if err is not None:
                last_reason = err
                print(f"[plan:{slot}:{index}] attempt {attempt+1}/{self._max_attempts} "
                      f"-> {err}")
                continue
            cleaned, dropped = _clean_plan(parsed.get("plan", []))
            cleaned, contract_dropped = _drop_foreign_contract_steps(cleaned, target, all_targets)
            dropped = list(dropped) + contract_dropped
            raw_actions = [
                s.get("action") if isinstance(s, dict) else f"<{type(s).__name__}>"
                for s in (parsed.get("plan", []) or [])
            ]
            print(f"[plan:{slot}:{index}] attempt {attempt+1}/{self._max_attempts}: "
                  f"raw {raw_actions} | kept {[s['action'] for s in cleaned]} "
                  f"| dropped {dropped}")
            if not cleaned:
                last_reason = (
                    "you returned an EMPTY plan (or only unknown actions). You "
                    "MUST return a NON-EMPTY plan of the known actions — never "
                    "refuse. If part of the target is impossible, still emit the "
                    "doable steps and finish with announce(text=...) explaining "
                    "what you could not do."
                )
                continue
            prior_plan = _flatten_prior_plans(self, slot, index)
            ok, reason = validate_plan(
                cleaned, desc or "", known_actions,
                known_locations=known_loc_arg,
                prior_plan=prior_plan,
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
            mods = parsed.get("modifications")
            ok, reason = validate_plan_modifications(mods, cleaned)
            if not ok:
                last_reason = f"invalid modifications: {reason}"
                print(f"[plan:{slot}:{index}] attempt {attempt+1}/{self._max_attempts} "
                      f"REJECTED: {last_reason}")
                continue
            failed_canonicals = {_canonical_plan(p) for p in self._failed_plans(slot, index)}
            identical = _canonical_plan(cleaned) in failed_canonicals
            if identical and attempt < self._max_attempts - 1:
                last_reason = (
                    "the regenerated plan was IDENTICAL to the plan that just failed "
                    f"({failure_reason or 'unknown reason'}) — you MUST change it: add, remove "
                    "or reorder steps so the failure cannot recur. "
                    + _alternatives_for_reason(failure_reason or "")
                )
                print(f"[plan:{slot}:{index}] attempt {attempt+1}/{self._max_attempts} "
                      f"REJECTED: identical to failed plan")
                continue
            # Accepted — build the subtree on this (worker) thread, then cache.
            try:
                subtree = self.build_target_subtree(slot, index, cleaned, modifications=mods)
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
            # it instead of re-running a known dead end.
            self._store(slot, index, desc, cleaned, subtree,
                        (f"{IDENTICAL_PLAN_ERROR_PREFIX}: {failure_reason}" if identical else None),
                        modifications=group_modifications_by_step(cleaned, mods or []))
            return
        # Every attempt failed -> guaranteed non-empty fallback plan.
        plan = _fallback_plan(desc)
        subtree = self.build_target_subtree(slot, index, plan)
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

    def replan_target(self, slot: int, index: int, reason: str = "") -> None:
        """Re-plan ONE target on a fresh daemon thread (lower-layer scope only).

        Invalidates the cached entry first so the OLD subtree is never re-swapped
        in while the fresh plan is in flight. Top layer and the other targets
        are untouched — a failing target re-plans only itself. The replan keeps
        the same full-command context (command / object / location / prior
        targets) so a delta re-plan does not lose the assigned location.
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
            kwargs={"failure_reason": reason, "target": t, "all_targets": targets},
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
        """
        desc = self._get_desc(slot, index) or f"target {index}"
        cleaned, _ = _clean_plan(plan)
        targets = self._get_slot_context(slot).get("targets", [])
        target = targets[index] if index < len(targets) else None
        cleaned, contract_dropped = _drop_foreign_contract_steps(cleaned, target, targets)
        if contract_dropped:
            print(f"[replace:{slot}:{index}] contract-boundary dropped {contract_dropped}")
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
        with self._lock:
            entry = self._cache.get((slot, index))
        if not entry:
            return []
        return list(entry.get("plan") or [])

    def get_error(self, slot: int, index: int) -> Optional[str]:
        """The cache entry's ``error`` (None when absent or ready with no error)."""
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
                action_plan=full_plan,
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
            ))
        return seq
