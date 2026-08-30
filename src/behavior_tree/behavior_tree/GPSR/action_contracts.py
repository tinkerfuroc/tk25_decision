"""Action-level contracts: what each planner action requires, establishes, records.

Single source of truth consumed by the split prompt, the plan validator, the
parameter materialiser and the runtime fact verifier. Keyed by the planner's
action vocabulary (``small_trees.ACTION_FACTORIES``), NOT by BT node class —
that is ``supervision/contracts.py``'s job.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping

from .validators import _normalize

# Cache-entry ``error`` prefix the planner stamps on a replan that came back
# IDENTICAL to the plan that just failed (final attempt only). The executor
# refuses to swap in / execute such a plan. Lives here (not planner.py) because
# planner imports orchestrator — importing it back would form a cycle.
IDENTICAL_PLAN_ERROR_PREFIX = "identical to failed plan"

# E2 (round-2 review, runs 003/004): cache-entry ``error`` prefix the planner
# stamps instead of IDENTICAL_PLAN_ERROR_PREFIX when the final-attempt
# identical plan's deterministic escape (planner._try_escape) came back empty
# BECAUSE no registry action can establish ANY of the target's own
# postconditions without repeating one already tried — the escape ladder is
# fully exhausted, not merely "try again". The executor treats this marker
# like the identical one (skip execution, target.failed) but ALSO forces the
# target's replan budget to exhausted, so the target ends honestly instead of
# burning the rest of the budget on more IDENTICAL_PLAN_SKIPPED markers for a
# plan the LLM cannot vary.
UNRECOVERABLE_ERROR_PREFIX = "unrecoverable"


@dataclass(frozen=True)
class ActionContract:
    action: str
    requires: tuple[str, ...] = ()
    establishes: tuple[str, ...] = ()
    self_establishes: Mapping[str, str] = field(default_factory=lambda: MappingProxyType({}))
    records: tuple[str, ...] = ()
    self_navigating: bool = False


def _c(action: str, **kw: Any) -> ActionContract:
    se = kw.pop("self_establishes", None)
    return ActionContract(
        action=action,
        self_establishes=MappingProxyType(dict(se or {})),
        **kw,
    )


ACTION_CONTRACTS: dict[str, ActionContract] = {
    c.action: c
    for c in (
        # Order matters for _ESTABLISHER_FOR_PREDICATE (planner.py): the FIRST
        # action establishing a predicate is its canonical establisher
        # (at_robot -> goto, object_seen -> find_object).
        _c("goto", establishes=("at_robot(location)",),
           self_establishes={"at_robot": "location"},
           records=("last_nav_location",), self_navigating=True),
        _c("find_object", establishes=("object_seen(object)",),
           records=("object_detection", "target_object_name")),
        _c("search_object", establishes=("object_seen(object)",),
           self_establishes={"at_robot": "location"},
           records=("last_nav_location", "object_detection"), self_navigating=True),
        _c("place", requires=("held(object)",), establishes=("placed(object,location)",),
           self_establishes={"at_robot": "location"},
           records=("last_nav_location",), self_navigating=True),
        _c("deliver", requires=("held(object)",), establishes=("delivered(object,recipient)",),
           self_establishes={"at_robot": "recipient_location"},
           records=("last_nav_location",), self_navigating=True),
        _c("grasp", requires=("object_seen(object)",), establishes=("held(object)",)),
        _c("find_person", establishes=("person_found(person)",),
           records=("person_detection", "target_person_pose")),
        _c("ask_person", establishes=("answered(question)",), records=("person_answer",)),
        _c("answer_question", establishes=("answered(question)",), records=("qa_answer",)),
        # `count`/`announce`/`vlm_fallback`/`llm_fallback` below ALL also
        # establish `answered(question)`, but every one of them is registered
        # AFTER ask_person/answer_question so _ESTABLISHER_FOR_PREDICATE
        # (planner.py) still resolves "answered" -> ask_person (the FIRST
        # registry entry wins — `setdefault`). This widens what the
        # postcondition-coverage check (planner_validators) and the
        # answered-gate action-verdict fallback (validators._action_verdict)
        # accept as a valid "answered" establisher without changing which
        # action is canonical for _deterministic_target_intent.
        #
        # `count` keeps `counted(object)` FIRST in its own tuple — that
        # ordering is what a caller iterating `establishes` for "the
        # predicate this step is really about" would see first; it has no
        # effect on `_ESTABLISHER_FOR_PREDICATE`, which is keyed by
        # REGISTRY order, not per-contract tuple order. `count` has no
        # `question` param, so `established_facts()` for it stays `[]`
        # (only `counted(object)` can ever resolve) — same reasoning as
        # `announce` below.
        _c("count", establishes=("counted(object)", "answered(question)"),
           records=("count_value", "count_target")),
        _c("approach_person"),
        _c("describe_person"),
        _c("follow"),
        # M-4 (round-2 review): guide navigates to `location` itself
        # (small_trees.create_guide, "retry guide goto") but previously
        # declared no self_establishes -- a guide target's at_robot(<dest>)
        # postcondition then demanded an EXTRA goto step from the model,
        # which could be placed BEFORE guide (the robot walks off alone,
        # leaving the person behind). Declaring this makes the
        # goto-before-self-nav rule and the at_robot action-verdict fallback
        # apply to guide too, same as goto/place/deliver/search_object.
        # Param name "location" per ACTION_CATALOGUE_DESCRIPTION
        # (orchestrator.py) and materialise_params.
        _c("guide", self_establishes={"at_robot": "location"}, self_navigating=True),
        _c("open"),
        # `announce` has no `question` param (only `text`), so
        # established_facts() for it stays [] — the establishes template can
        # never resolve, which is intentional: this widens what the
        # postcondition-coverage check and the answered-gate fallback accept
        # without ever letting announce silently "establish" a fact for a
        # SIBLING target via the contract-boundary guard.
        _c("announce", establishes=("answered(question)",)),
        _c("record_position"),
        # vlm_fallback/llm_fallback DO carry a `question` param, so unlike
        # announce/count, established_facts() for these two CAN resolve to a
        # real `answered(<question>)` fact when `question` is set — that is
        # deliberate (H1): a `[goto, vlm_fallback(question=...)]` /
        # `[llm_fallback(question=...)]` plan genuinely establishes the
        # question it was asked, for its own target.
        _c("vlm_fallback", establishes=("answered(question)",), records=("vlm_answer",)),
        _c("llm_fallback", establishes=("answered(question)",), records=("llm_answer",)),
    )
}


def contract_for(action: str) -> ActionContract:
    return ACTION_CONTRACTS[str(action)]


def self_established_facts(step: Mapping[str, Any]) -> list[str]:
    """Canonical facts ``step`` satisfies by itself (param absent -> nothing)."""
    contract = ACTION_CONTRACTS.get(str(step.get("action")))
    if contract is None:
        return []
    params = step.get("params") or {}
    facts: list[str] = []
    for predicate, param in contract.self_establishes.items():
        value = params.get(param)
        if value is None or not str(value).strip():
            continue
        facts.append(f"{predicate}({_normalize(str(value))})")
    return facts


def established_facts(step: Mapping[str, Any], target_object: str = "") -> list[str]:
    """Canonical facts ``step``'s action establishes FOR OTHER TARGETS.

    Mirrors ``self_established_facts`` but walks ``contract.establishes``
    templates (``"delivered(object,recipient)"`` etc.) instead of
    ``self_establishes``. Each template is parsed as ``pred(p1,p2,...)``; every
    param name is looked up in ``step["params"]``. If ANY named param is
    missing or blank, that whole template is skipped (never guess a partial
    fact) -- EXCEPT ``object`` (J10, round-3 adversarial review, tier0 #2):
    when the step's own params omit it, ``target_object`` (the TARGET's
    declared object) is used instead, the SAME fallback
    ``validators._action_verdict`` already applies. ``place``/``deliver``
    plans routinely omit ``object`` (the held object is implicit), so
    without this fallback the contract-boundary guard was blind to exactly
    the duplicate ``place,place``/``deliver,deliver`` it exists to catch.
    Used by the contract-boundary guard to tell whether a step establishes a
    fact owned by a *different* target of the same command.

    Facts are compared EXACTLY (canonicalised argument text only) — e.g. the
    top layer's ``delivered(spam,me)`` vs. a lower-layer step that emits
    ``recipient: "operator"``/``"the user"`` canonicalise to different facts
    and will NOT be recognised as the same. Normalising recipient aliases
    before comparison is intentionally left to the prompt (the top layer's
    "must establish"/"owned by" text), not this function.
    """
    contract = ACTION_CONTRACTS.get(str(step.get("action")))
    if contract is None:
        return []
    params = step.get("params") or {}
    facts: list[str] = []
    for template in contract.establishes:
        predicate, _, arglist = template.partition("(")
        arg_names = [a.strip() for a in arglist.rstrip(")").split(",") if a.strip()]
        if not arg_names:
            # A future zero-arity establishes template has no params to
            # resolve, so it can never yield a well-formed fact — never emit
            # the unparsable f"{predicate}()".
            continue
        values: list[str] = []
        skip = False
        for name in arg_names:
            value = params.get(name)
            if (value is None or not str(value).strip()) and name == "object":
                value = target_object
            if value is None or not str(value).strip():
                skip = True
                break
            values.append(_normalize(str(value)))
        if skip:
            continue
        facts.append(f"{predicate}({','.join(values)})")
    return facts


def self_navigating_destinations() -> dict[str, str]:
    return {
        name: c.self_establishes["at_robot"]
        for name, c in sorted(ACTION_CONTRACTS.items())
        if c.self_navigating and name != "goto" and "at_robot" in c.self_establishes
    }


def render_self_satisfied_rule() -> str:
    parts = [
        f"{name} (via its {c.self_establishes['at_robot']})"
        for name, c in sorted(ACTION_CONTRACTS.items())
        if "at_robot" in c.self_establishes
    ]
    return (
        "A target must NOT list as a precondition a fact that its own action "
        "establishes by itself. at_robot is self-satisfied by: " + ", ".join(parts) + "."
    )


__all__ = [
    "ActionContract", "ACTION_CONTRACTS", "IDENTICAL_PLAN_ERROR_PREFIX",
    "UNRECOVERABLE_ERROR_PREFIX", "contract_for",
    "self_established_facts", "established_facts", "self_navigating_destinations",
    "render_self_satisfied_rule",
]
