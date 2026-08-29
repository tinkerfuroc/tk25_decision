"""Pure fact parsing and deterministic verification for GPSR runtime gates."""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Mapping, Optional


class Verdict(str, Enum):
    VALID = "VALID"
    INVALID = "INVALID"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class Fact:
    predicate: str
    args: tuple[str, ...]
    raw: str


@dataclass(frozen=True)
class VerificationResult:
    verdict: Verdict
    confidence: float = 1.0
    evidence: str = ""


@dataclass(frozen=True)
class VerificationContext:
    phase: str
    established_facts: frozenset[str] = frozenset()
    completed_steps: tuple[Mapping[str, Any], ...] = ()
    target_object: str = ""
    target_location: str = ""


Tier2Hook = Callable[[Fact, Mapping[str, Any], VerificationContext], Optional[VerificationResult]]

_VOCABULARY = {
    "at_robot": 1,
    "object_seen": 1,
    "person_found": 1,
    "held": 1,
    "placed": 2,
    "delivered": 2,
    "counted": 1,
    "answered": 1,
}
_TIER2_HOOKS: dict[str, Tier2Hook] = {}
_FACT_RE = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)\s*\((.*)\)$", re.DOTALL)

# I-3 (round-2 review): matches an LLM-authored refusal/apology announce
# ("I could not count the drinks", "Sorry, I couldn't find the object") so
# _action_verdict's answered() fallback (below) never counts it as a real
# answer -- test-pinned, keep the alternation and wording in sync with
# test_gpsr_answered_gate.py::test_refusal_regex_matches_every_pinned_apology_phrasing.
_REFUSAL_RE = re.compile(
    r"^\s*(sorry[,.]?\s*)?i\s+(could\s*not|couldn't|cannot|can't|was\s+unable|"
    r"am\s+unable|did\s+not|didn't|failed)\b",
    re.IGNORECASE,
)


def _normalize(value: str) -> str:
    return re.sub(r"\s+", "_", value.strip().lower())


def parse_fact(text: str) -> tuple[Optional[Fact], Optional[str]]:
    """Parse one closed-vocabulary fact, returning a useful error on rejection."""
    raw = str(text)
    match = _FACT_RE.fullmatch(raw.strip())
    if match is None:
        return None, "fact must match predicate(arg,...) with no trailing text"

    predicate = _normalize(match.group(1))
    if predicate not in _VOCABULARY:
        return None, f"unknown predicate: {predicate}"

    body = match.group(2).strip()
    if not body:
        return None, "fact argument list cannot be empty"
    if "(" in body or ")" in body:
        return None, "nested predicates are not supported"

    raw_args = body.split(",")
    if any(not arg.strip() for arg in raw_args):
        return None, "fact arguments cannot be empty"
    args = tuple(_normalize(arg) for arg in raw_args)
    expected = _VOCABULARY[predicate]
    if len(args) != expected:
        return None, f"{predicate} expects {expected} argument(s), got {len(args)}"
    return Fact(predicate, args, raw), None


def canonical_fact(fact: Fact) -> str:
    return f"{_normalize(fact.predicate)}({','.join(_normalize(arg) for arg in fact.args)})"


def _fact_parts(value: Any) -> tuple[Optional[str], Optional[Fact]]:
    text = str(value).strip()
    fact, _ = parse_fact(text)
    if fact is None:
        return (text or None), None
    return canonical_fact(fact), fact


def apply_fact_transitions(existing: Any, additions: Any) -> list[str]:
    """Apply validated facts to the append-only ledger's current-state view.

    Valid facts are canonicalized and deduplicated. Malformed legacy strings are
    retained once, unchanged apart from surrounding whitespace. The returned
    order keeps unaffected existing facts first and appends newly effective facts
    in batch order.
    """
    result: list[str] = []
    seen: set[str] = set()
    for value in existing or []:
        normalized, fact = _fact_parts(value)
        if normalized is not None and normalized not in seen:
            seen.add(normalized)
            result.append(normalized)

    for value in additions or []:
        normalized, fact = _fact_parts(value)
        if normalized is None:
            continue
        if fact is not None and fact.predicate == "at_robot":
            result = [item for item in result
                      if (parsed := parse_fact(item)[0]) is None or parsed.predicate != "at_robot"]
            seen = set(result)
        elif fact is not None and fact.predicate in {"placed", "delivered"}:
            obj = fact.args[0]
            kept = []
            for item in result:
                parsed, _ = parse_fact(item)
                if parsed is not None and parsed.predicate == "held" and parsed.args[0] == obj:
                    seen.discard(item)
                    continue
                kept.append(item)
            result = kept
        elif fact is not None and fact.predicate == "held":
            obj = fact.args[0]
            kept = []
            for item in result:
                parsed, _ = parse_fact(item)
                if parsed is not None and parsed.predicate in {"placed", "delivered"} and parsed.args[0] == obj:
                    seen.discard(item)
                    continue
                kept.append(item)
            result = kept
        if normalized not in seen:
            seen.add(normalized)
            result.append(normalized)
    return result


def register_tier2_hook(predicate: str, hook: Optional[Tier2Hook]) -> None:
    """Install or remove the optional stronger verifier for a predicate."""
    name = _normalize(predicate)
    if hook is None:
        _TIER2_HOOKS.pop(name, None)
    else:
        _TIER2_HOOKS[name] = hook


def _result(verdict: Verdict, evidence: str, confidence: float = 1.0) -> VerificationResult:
    return VerificationResult(verdict, confidence, evidence)


def _objects_from_detection(response: Any) -> Optional[Any]:
    if response is None:
        return None
    if isinstance(response, Mapping):
        return response.get("objects")
    if hasattr(response, "objects"):
        return getattr(response, "objects")
    return response


def _nonempty_detection(response: Any) -> bool:
    objects = _objects_from_detection(response)
    if objects is None:
        return False
    try:
        return len(objects) > 0
    except TypeError:
        return bool(objects)


_LABEL_FIELDS = ("cls", "class", "name", "label", "descriptor", "person")


def _detection_items(response: Any) -> list[Any]:
    objects = _objects_from_detection(response)
    if objects is None:
        return []
    if isinstance(objects, (str, bytes)):
        return [objects]
    if isinstance(objects, Mapping):
        return [objects]
    try:
        return list(objects)
    except TypeError:
        return [objects]


def _item_label(item: Any) -> Optional[str]:
    if isinstance(item, str):
        text = item.strip()
        return _normalize(text) if text else None
    if isinstance(item, Mapping):
        for field in _LABEL_FIELDS:
            value = item.get(field)
            if isinstance(value, (str, int, float)) and not isinstance(value, bool):
                text = str(value).strip()
                if text:
                    return _normalize(text)
        return None
    for field in _LABEL_FIELDS:
        value = getattr(item, field, None)
        if isinstance(value, (str, int, float)) and not isinstance(value, bool):
            text = str(value).strip()
            if text:
                return _normalize(text)
    return None


def _detection_labels(response: Any) -> tuple[set[str], bool]:
    """Return normalized recognizable labels and whether the artifact is nonempty."""
    items = _detection_items(response)
    labels = {label for item in items if (label := _item_label(item))}
    return labels, bool(items)


def _norm_match(left: Any, right: str) -> bool:
    return _normalize(str(left)) == right


def _label_tokens(label: str) -> list[str]:
    return re.split(r"[_.]", label)


def _label_matches(label: str, requested: str) -> bool:
    """label/requested already normalised.

    Detector labels follow two grammars: `<class>.<instance>` (category-prompted
    object detection, e.g. "kitchen_item.round_white_table") and `<class> <name>`
    (person specialist, e.g. "person_liam" after normalization). The two
    grammars are matched differently (M1/M3, round-2 review):

    - `.`-label (category-prompted object detection): whole-label equality,
      whole-CLASS-segment equality ("drink" == "drink.coke"'s class), or
      (M1) whole-INSTANCE-segment equality ("pudding_box" ==
      "food.pudding_box"'s instance) -- and nothing else. Neither segment is
      ever split into sub-tokens (M3): "kitchen" must not match
      "kitchen_item.trash_can" (a class-segment fragment) and "table" must
      not match "kitchen_item.round_white_table" (an instance-segment
      fragment) -- gate false positives are the expensive direction here (a
      wrong object_seen lets a grasp run on the wrong thing).
    - `.`-less label (person specialist grammar, e.g. "person_liam" /
      "red_jacket"): whole-label equality, or `requested` as one whole
      '_'-separated token of the label ("liam" in "person_liam"). The token
      rule is deliberately conservative: `requested` must be a WHOLE token
      -- never a substring -- so "red" matches "red_jacket" but not
      "bred_jacket". A multi-word `requested` (itself containing '_') only
      matches by equality; its own tokens are never split against the
      label's tokens.
    """
    if label == requested:
        return True
    class_prefix, sep, instance = label.partition(".")
    if sep:
        return class_prefix == requested or instance == requested
    if "_" not in requested and requested in _label_tokens(label):
        return True
    return False


def _step_action(step: Mapping[str, Any]) -> str:
    action = step.get("action", step.get("name", step.get("type", "")))
    return _normalize(str(action))


def _step_params(step: Mapping[str, Any]) -> Mapping[str, Any]:
    params = step.get("params", step.get("parameters", {}))
    return params if isinstance(params, Mapping) else {}


def _step_succeeded(step: Mapping[str, Any]) -> bool:
    status = step.get("status", step.get("verdict", step.get("result", None)))
    if status is None:
        return True
    if isinstance(status, bool):
        return status
    return _normalize(str(status)) in {"success", "succeeded", "successful", "completed", "ok", "valid"}


# --- sim-mode identity relaxation (GPSR_SIM_IDENTITY_RELAXED=1) ----------
#
# Sim persons carry no name identity: the detector always labels them
# "person" regardless of who the scenario says is standing there. Without
# this flag, person_found(<Name>) rejects a correct sim run purely because
# the requested name is never among the detection labels. See _verify's
# person_found branch below for where this is consulted -- it only
# replaces the final INVALID return of the labelled-mismatch path, never
# the earlier established-fact / waving-specialist / unset-evidence paths.
_SIM_PERSON_CLASS_LABELS = {"person", "persons", "people", "human"}
_SIM_PERSON_DESCRIPTORS = {"waving_person", "waving_persons"}


def _sim_identity_relaxed_enabled() -> bool:
    # Read fresh every call (not cached at import time) so tests can
    # monkeypatch os.environ per-test without reloading the module.
    return os.environ.get("GPSR_SIM_IDENTITY_RELAXED") == "1"


def _is_person_name_arg(arg: str) -> bool:
    """True when a person_found() argument names a person rather than a
    descriptor. `arg` is already normalized (lowercase, whitespace -> "_")
    by the time _verify sees fact.args, so "waving person" and
    "waving_person" are indistinguishable here -- both are excluded.
    """
    if arg in _SIM_PERSON_DESCRIPTORS:
        return False
    if "_" in arg:
        return False
    if "person" in arg or "persons" in arg:
        return False
    return True


def _action_verdict(fact: Fact, context: VerificationContext) -> Optional[VerificationResult]:
    if context.phase != "postcondition":
        return None
    target = fact.args
    for step in context.completed_steps:
        if not isinstance(step, Mapping) or not _step_succeeded(step):
            continue
        action = _step_action(step)
        params = _step_params(step)
        if fact.predicate == "held" and action == "grasp" and _norm_match(params.get("object", ""), target[0]):
            return _result(Verdict.VALID, "action-verdict fallback: stronger verifier not installed; successful grasp action", 0.5)
        if fact.predicate == "placed" and action == "place":
            explicit_object = params.get("object", "")
            object_matches = (
                _norm_match(explicit_object, target[0])
                if str(explicit_object).strip()
                else _norm_match(context.target_object, target[0])
            )
            if _norm_match(params.get("location", ""), target[1]) and object_matches:
                return _result(Verdict.VALID, "action-verdict fallback: stronger verifier not installed; successful place action", 0.5)
        if fact.predicate == "delivered" and action == "deliver":
            if (_norm_match(params.get("object", ""), target[0]) and
                    _norm_match(params.get("recipient", ""), target[1])):
                return _result(Verdict.VALID, "action-verdict fallback: stronger verifier not installed; successful deliver action", 0.5)
        if fact.predicate == "at_robot":
            from .action_contracts import ACTION_CONTRACTS  # lazy: action_contracts imports this module
            contract = ACTION_CONTRACTS.get(str(action))
            nav_param = contract.self_establishes.get("at_robot") if contract else None
            if nav_param and _norm_match(params.get(nav_param, ""), target[0]):
                return _result(
                    Verdict.VALID,
                    f"action-verdict fallback: stronger verifier not installed; successful {action} action",
                    0.5,
                )
        if fact.predicate == "answered" and not params.get("acknowledgement"):
            from .action_contracts import ACTION_CONTRACTS  # lazy: action_contracts imports this module
            contract = ACTION_CONTRACTS.get(action)
            establishes_answered = contract is not None and any(
                t.split("(", 1)[0] == "answered" for t in contract.establishes
            )
            # M1: no longer requires a non-empty params["text"] -- a
            # text-less announce() reporting a prior count/describe_person/
            # ask_person/vlm_fallback result (the prompt's canonical "tell
            # ME" pattern) has nothing IN ITS OWN params to check; the
            # earlier successful step already qualifies via ITS OWN contract
            # (count/ask_person/... establish "answered" too, see H1), so
            # the plan is covered even before this text-less announce is
            # reached. Any step whose contract establishes "answered",
            # succeeded, and is not flagged acknowledgement qualifies.
            # I-3 (round-2 review): an LLM-authored refusal/apology announce
            # is never flagged acknowledgement (only orchestrator._fallback_plan
            # and planner.replace_target_plan tag that -- a plan the LOWER
            # LAYER LLM produced on its own has no such tag), so without this
            # check a replan that gave up ("I could not count the drinks")
            # would read back as a postcondition PASS the same as a real
            # answer. Belt-and-braces alongside the "acknowledgement": true
            # instruction in LOWER_LAYER_SYSTEM_PROMPT (planner.py) that asks
            # the model to tag its own refusals.
            if (action == "announce"
                    and _REFUSAL_RE.match(str(params.get("text") or ""))):
                establishes_answered = False
            if establishes_answered:
                reason = (
                    "action-verdict fallback: spoken announce stands as the answer; "
                    "question identity unavailable"
                    if action == "announce" else
                    f"action-verdict fallback: successful {action} action establishes "
                    "answered(); question identity unavailable"
                )
                return _result(Verdict.VALID, reason, 0.5)
    return None


def _verify(fact: Fact, evidence: Mapping[str, Any], context: VerificationContext) -> VerificationResult:
    hook = _TIER2_HOOKS.get(fact.predicate)
    if hook is not None:
        hooked = hook(fact, evidence, context)
        if hooked is not None:
            return hooked

    canonical = canonical_fact(fact)
    established = {_normalize_established(item) for item in context.established_facts}
    if fact.predicate == "at_robot":
        if "last_nav_location" in evidence and evidence["last_nav_location"] is not None:
            if not _norm_match(evidence["last_nav_location"], fact.args[0]):
                return _result(Verdict.INVALID, "navigation location mismatch")
        if canonical in established:
            return _result(Verdict.VALID, f"established fact: {canonical}")

    if canonical in established:
        return _result(Verdict.VALID, f"established fact: {canonical}")

    if fact.predicate in {"object_seen", "person_found"}:
        detection_key = "object_detection" if fact.predicate == "object_seen" else "person_detection"
        response = evidence.get(detection_key)
        if response is None and fact.predicate == "person_found":
            response = evidence.get("waving_persons")
        labels, nonempty = _detection_labels(response)
        # Specialist waving output is a typed descriptor, not a person's name.
        if (
            fact.predicate == "person_found"
            and _normalize(fact.args[0]) in {"waving_person", "waving_persons"}
            and evidence.get("person_provenance") == "waving_specialist"
            and nonempty
        ):
            return _result(Verdict.VALID, "waving-specialist person artifact matches descriptor provenance")
        if labels:
            requested_label = _normalize(fact.args[0])
            if any(_label_matches(label, requested_label) for label in labels):
                return _result(Verdict.VALID, f"{detection_key} label matches requested target")
            if (
                fact.predicate == "person_found"
                and _sim_identity_relaxed_enabled()
                and _is_person_name_arg(fact.args[0])
                and any(
                    token in _SIM_PERSON_CLASS_LABELS
                    for label in labels
                    for token in _label_tokens(label)
                )
            ):
                return _result(
                    Verdict.VALID,
                    "sim mode: person detected; name identity is not modelled in sim",
                    0.6,
                )
            return _result(Verdict.INVALID, f"{detection_key} labels do not match requested target")
        if nonempty:
            if fact.predicate == "person_found" and evidence.get("person_provenance") == "waving_specialist":
                if _normalize(fact.args[0]) in {"waving_person", "waving_persons"}:
                    return _result(Verdict.VALID, "waving-specialist person artifact has identity unavailable")
                return _result(Verdict.UNKNOWN, "waving-specialist artifact has no named-person identity")
            return _result(Verdict.VALID, f"{detection_key} identity unavailable; nonempty target-scoped artifact")
    elif fact.predicate == "counted":
        if "count_value" in evidence:
            provenance = evidence.get("count_target") or evidence.get("count_query")
            if provenance is not None and str(provenance).strip():
                if not _norm_match(provenance, fact.args[0]):
                    return _result(Verdict.INVALID, "count artifact target provenance mismatch")
                return _result(Verdict.VALID, "count artifact target provenance matches")
            return _result(Verdict.VALID, "count artifact contains count_value; target identity unavailable")
    elif fact.predicate == "answered":
        answer_keys = ("qa_answer", "person_answer", "llm_answer", "vlm_answer")
        # count also establishes answered(question) now (H1: "how many ..."
        # is a spoken-answer target too) -- mirror the `counted` branch above
        # and accept a bare count_value artifact, same as any other answer key.
        #
        # M-7 (round-2 review): `evidence` (the Blackboard FACTS mirror) is
        # cleared only on target ADVANCE (orchestrator.py ~2095-2099), not
        # per-step -- so a target that counted, then asked an unrelated
        # question whose ask_person failed, then replanned to a bare
        # announce, could still see this old count_value here and verdict
        # VALID via a stale artifact instead of the failed ask. Negligible in
        # practice (a target rarely mixes count + ask_person for two
        # DIFFERENT `answered(...)` facts), not fixed here.
        has_answer_artifact = (
            any(isinstance(evidence.get(key), str) and evidence[key].strip() for key in answer_keys)
            or "count_value" in evidence
        )
        if has_answer_artifact:
            provenance = next((evidence.get(key) for key in
                               ("qa_question", "ask_question", "question_provenance", "vlm_question", "llm_question")
                               if isinstance(evidence.get(key), str) and evidence[key].strip()), None)
            if provenance is not None:
                provenance_norm = _normalize(re.sub(r"[^a-zA-Z0-9_\s]", "", str(provenance)))
                requested_norm = _normalize(fact.args[0])
                if not (provenance_norm == requested_norm or requested_norm in provenance_norm.split("_")):
                    return _result(Verdict.INVALID, "answer artifact question provenance mismatch")
            if provenance is not None:
                return _result(Verdict.VALID, "answer artifact question provenance matches")
            return _result(Verdict.VALID, "answer artifact contains a nonempty answer; question identity unavailable")
    elif fact.predicate == "at_robot":
        if "last_nav_location" in evidence and evidence["last_nav_location"] is not None:
            if not _norm_match(evidence["last_nav_location"], fact.args[0]):
                return _result(Verdict.INVALID, "navigation location mismatch")
            if context.phase == "postcondition":
                return _result(Verdict.VALID, "intent/action evidence: matching last navigation location", 0.5)

    action_result = _action_verdict(fact, context)
    if action_result is not None:
        return action_result
    return _result(Verdict.UNKNOWN, "UNKNOWN: stronger verifier not installed and no sufficient artifact")


def _normalize_established(value: Any) -> str:
    text = str(value).strip()
    fact, error = parse_fact(text)
    return canonical_fact(fact) if fact is not None else _normalize(text)


def check_all(
    fact_texts: list[str], evidence: Mapping[str, Any], context: VerificationContext
) -> tuple[list[VerificationResult], list[Fact]]:
    """Check every supplied fact; malformed facts yield INVALID results."""
    results: list[VerificationResult] = []
    facts: list[Fact] = []
    for text in fact_texts:
        fact, error = parse_fact(text)
        if fact is None:
            results.append(_result(Verdict.INVALID, f"invalid fact: {error}"))
            continue
        facts.append(fact)
        results.append(_verify(fact, evidence, context))
    return results, facts
