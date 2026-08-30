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
    # J2 (round-3 adversarial review, M3): canonical facts the CURRENT
    # target itself declares as postconditions. In the postcondition phase
    # only, the established-fact ledger shortcut below must not apply to
    # these -- a target's own postcondition must be verified from evidence/
    # action-verdict produced DURING this target, never from a fact an
    # earlier (or even this same) target already wrote to the ledger.
    own_postconditions: frozenset[str] = frozenset()


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

# I1 (round-3 adversarial review, H1): the top layer writes an answered()
# fact from the COMMAND's own wording ("day of the month") while the lower
# layer's ask_person/llm_fallback/vlm question is phrased in its own words
# ("What is the day of the month today? Say that it is 29."). Exact-string
# provenance rejected every correct answer. Provenance is now a SOFT signal:
# bag-of-content-words overlap between the fact argument and the recorded
# question, ignoring a fixed stopword list and punctuation.
_PROVENANCE_STOPWORDS = {
    "what", "is", "the", "a", "an", "of", "to", "me", "you", "your", "please",
    "say", "tell", "today", "it", "that", "this", "in", "on", "at",
}

# M-2 (round-3 fix review): the top layer's split-prompt advertises the
# predicate as literally `answered(question)` (planner.py), and plans like
# `answered(question of the person)`/`answered(information)` are plausible,
# generic paraphrases of "the question was answered" — not a claim about a
# SPECIFIC different question. Words here carry no discriminating content on
# their own: a fact whose content words (after _PROVENANCE_STOPWORDS) are
# ENTIRELY made of these must never INVALID on zero overlap (there is
# nothing specific to disagree about), only fall through to the weak
# partial-VALID branch below.
_NON_DISCRIMINATING_FACT_WORDS = {
    "question", "answer", "information", "person", "people", "it", "thing", "things",
}


def _content_words(text: str) -> set[str]:
    cleaned = re.sub(r"[^a-zA-Z0-9_\s]", " ", str(text))
    tokens = re.split(r"[_\s]+", cleaned.lower())
    return {token for token in tokens if token and token not in _PROVENANCE_STOPWORDS}


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


# I5 (round-3 adversarial review, M2): "me"/"the operator"/"command point"/
# ... are all the SAME destination as the canonical "start_position" the
# lower layer actually navigates to (see planner_validators.is_start_alias,
# the single source of truth shared with uncovered_postcondition_reason).
# Lazy-imported below (validators.py cannot import planner_validators at
# module level -- planner_validators already imports validators).
def _at_robot_match(nav_location: Any, requested: str) -> bool:
    if _norm_match(nav_location, requested):
        return True
    from .planner_validators import is_start_alias  # lazy: avoid import cycle
    return is_start_alias(nav_location) and is_start_alias(requested)


def _label_tokens(label: str) -> list[str]:
    return re.split(r"[_.]", label)


def _known_object_names() -> frozenset[str]:
    """Normalised arena object names, loaded via the SAME knowledge loader
    the orchestrator uses (``constants.json``'s ``possible_objects``, see
    ``orchestrator.load_knowledge_from_constants`` /
    ``orchestrator.KNOWN_OBJECT_NAMES``).

    Lazy import (validators.py must not hard-depend on orchestrator.py for
    its own network-free unit tests): when orchestrator hasn't been
    imported/loaded yet, or nothing was loaded (offline/unit tests), this is
    empty -- callers keep today's permissive behaviour in that case.
    """
    try:
        from .orchestrator import KNOWN_OBJECT_NAMES  # lazy: orchestrator imports this module
    except Exception:  # noqa: BLE001 -- never let a knowledge lookup crash the gate
        return frozenset()
    return frozenset(KNOWN_OBJECT_NAMES)


def _known_object_prompt_token_sets() -> dict[str, frozenset[str]]:
    """Map each normalised known object name to the token set of its
    free-text VLM description (``orchestrator.KNOWN_OBJECT_PROMPTS`` --
    the SAME ``possible_objects`` source as ``_known_object_names``, but its
    VALUES, e.g. ``"mug" -> "red ceramic mug"`` -> ``{"red","ceramic","mug"}``).
    """
    try:
        from .orchestrator import KNOWN_OBJECT_PROMPTS  # lazy: orchestrator imports this module
    except Exception:  # noqa: BLE001 -- never let a knowledge lookup crash the gate
        return {}
    return {
        _normalize(str(key)): frozenset(_label_tokens(_normalize(str(value))))
        for key, value in KNOWN_OBJECT_PROMPTS.items()
    }


def _category_instance_known(instance: str, requested: str) -> bool:
    """J5 (round-3 adversarial review, testing session 3a): is a class-prefix
    match's INSTANCE segment (``kitchen_item.refrigerator`` -> ``refrigerator``)
    a real arena object?

    True when: no ``possible_objects`` are loaded at all (unit tests /
    offline -- keep today's permissive behaviour); OR ``requested`` is
    ITSELF a known object name (an exact-object request, e.g. "bowl", not a
    category one -- the class-prefix rule wasn't the reason it matched);
    OR a single-token known object's name matches ONE of ``instance``'s
    tokens (plural/underscore tolerant, same as grasp/place object
    identity); OR EVERY token of a multi-token known object's own name
    (``sugar_box``, ``cheez_it``, ``pudding_box``) is present among
    ``instance``'s tokens; OR EVERY token of a known object's VLM prompt
    description is present among ``instance``'s tokens.

    H-4 (round-3 fix review): the real detector's class-prefix labels are
    ``"<category>.<free-text VLM description>"`` (e.g.
    ``"kitchen item.red ceramic mug"``, not ``"kitchen item.mug"``) --
    exact-or-plural ``_same_object`` on the WHOLE instance segment never
    matched a ``possible_objects`` key, so every real detection failed this
    gate. Token containment fixes it. Every containment check here requires
    the FULL name (or the full prompt) as a token SUBSET of the instance,
    never a single shared word in isolation -- a lone descriptive word (a
    color like "white", a fragment like "it" of "cheez_it") appears across
    many unrelated objects and is not, by itself, strong evidence.
    """
    known = _known_object_names()
    if not known:
        return True
    if requested in known:
        return True
    from .planner_validators import _same_object  # lazy: planner_validators imports this module
    instance_tokens = _label_tokens(instance)
    instance_token_set = set(instance_tokens)
    single_token_names = {name for name in known if len(_label_tokens(name)) == 1}
    if any(_same_object(tok, name) for tok in instance_tokens for name in single_token_names):
        return True
    prompt_token_sets = _known_object_prompt_token_sets()
    for name in known:
        name_tokens = _label_tokens(name)
        if len(name_tokens) > 1 and set(name_tokens) <= instance_token_set:
            return True
        prompt_tokens = prompt_token_sets.get(name)
        if prompt_tokens and prompt_tokens <= instance_token_set:
            return True
    return False


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

    J5 (round-3 adversarial review, testing session 3a): whole-CLASS-segment
    equality (the category-membership rule) is VALID only when the
    INSTANCE segment is itself a known arena object -- see
    ``_category_instance_known``. Without this, ANY class-prefix match
    ("kitchen_item.refrigerator" for a requested category "kitchen_item")
    passed even when the arena doesn't model that instance as a graspable
    object at all.
    """
    if label == requested:
        return True
    class_prefix, sep, instance = label.partition(".")
    if sep:
        if instance == requested:
            return True
        return class_prefix == requested and _category_instance_known(instance, requested)
    if "_" not in requested and requested in _label_tokens(label):
        return True
    return False


# I2 (round-3 adversarial review, H2): `count_target` is the count step's
# literal `object` param (the LOWER layer's own wording); the counted() fact
# argument comes from the TOP layer's split (the command's wording). Exact
# `_norm_match` rejected "drinks"/"drink", "persons"/"person" and "kitchen
# items"/"kitchen item" -- all correct counts. Reuse the same plural
# tolerance `planner_validators._same_object` already applies to grasp/place
# object identity, plus `_label_matches`' class/instance rule, and only then
# fall back to a per-token plural-tolerant comparison for multi-word
# descriptors ("persons pointing to the left" / "person pointing to the
# left") whose plural lives on the head noun, not the last token.
def _count_provenance_matches(requested: str, provenance: Any) -> bool:
    from .planner_validators import _same_object  # lazy: planner_validators imports this module

    requested_norm = _normalize(requested)
    provenance_norm = _normalize(str(provenance))
    if requested_norm == provenance_norm:
        return True
    if _same_object(requested_norm, provenance_norm):
        return True
    if _label_matches(provenance_norm, requested_norm) or _label_matches(requested_norm, provenance_norm):
        return True
    req_tokens = requested_norm.split("_")
    prov_tokens = provenance_norm.split("_")
    if len(req_tokens) == len(prov_tokens) and len(req_tokens) > 1 and req_tokens[1:] == prov_tokens[1:]:
        if _same_object(req_tokens[0], prov_tokens[0]):
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
    """True when a person_found() argument is eligible for the generic
    sim-relaxation degrade below (see its call site).

    `arg` is already normalized (lowercase, whitespace -> "_") by the time
    _verify sees fact.args, so "waving person" and "waving_person" are
    indistinguishable here.

    Only the SPECIALIST waving descriptor (``_SIM_PERSON_DESCRIPTORS``) is
    excluded -- it has its own provenance-gated VALID/UNKNOWN branch above
    (a generic "person" label with no ``waving_specialist`` provenance is
    not itself evidence someone is waving) and must never fall through to
    the generic degrade.

    J13 (round-3 adversarial review, tier0 #4): this USED TO also exclude
    any arg containing "_" or "person"/"persons" -- rejecting every OTHER
    descriptor too (gesture/pose/clothing, e.g. "person raising their left
    arm" -> "person_raising_their_left_arm") even though the sim models
    none of those either. The sim's detector carries no more identity for
    a descriptor than it does for a name, so the same degrade now applies.
    """
    return arg not in _SIM_PERSON_DESCRIPTORS


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
    # J2: the ledger shortcut is only for facts OTHER targets established --
    # never for a fact this target itself is being asked to newly verify.
    is_own_postcondition = context.phase == "postcondition" and canonical in context.own_postconditions
    if fact.predicate == "at_robot":
        if "last_nav_location" in evidence and evidence["last_nav_location"] is not None:
            if not _at_robot_match(evidence["last_nav_location"], fact.args[0]):
                return _result(Verdict.INVALID, "navigation location mismatch")
        if canonical in established and not is_own_postcondition:
            return _result(Verdict.VALID, f"established fact: {canonical}")

    if canonical in established and not is_own_postcondition:
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
                # J13: unified reason for any relaxed argument -- name or
                # descriptor, the sim models neither.
                return _result(
                    Verdict.VALID,
                    f"sim mode: descriptor {fact.args[0]} not modelled",
                    0.6,
                )
            # J5: give the specific reason when a label's class prefix
            # matched the requested category but its instance is not a
            # known arena object (rejected by _label_matches/
            # _category_instance_known) -- more actionable than the generic
            # "labels do not match" for a category request.
            for label in labels:
                class_prefix, sep, instance = label.partition(".")
                if (
                    sep and class_prefix == requested_label
                    and not _category_instance_known(instance, requested_label)
                ):
                    return _result(
                        Verdict.INVALID,
                        f"detected {label} is not a known arena object for "
                        f"category {requested_label}",
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
                if not _count_provenance_matches(fact.args[0], provenance):
                    return _result(Verdict.INVALID, "count artifact target provenance mismatch")
                return _result(Verdict.VALID, "count artifact target provenance matches")
            return _result(Verdict.VALID, "count artifact contains count_value; target identity unavailable")
    elif fact.predicate == "answered":
        # X2 (round-3 fix review): describe_person's ONLY answer artifact is
        # REPORT_INFO (it speaks + buffers a description, never writing
        # qa_answer/llm_answer/...) -- without this key, coverage accepting
        # describe_person as an answered() establisher (action_contracts.py)
        # still had no runtime artifact to VALID against.
        answer_keys = ("qa_answer", "person_answer", "llm_answer", "vlm_answer", "report_info")
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
                fact_words = _content_words(fact.args[0])
                provenance_words = _content_words(provenance)
                if not fact_words or fact_words.issubset(provenance_words):
                    return _result(Verdict.VALID, "answer artifact question provenance matches")
                # M-2 (round-3 fix review): only a DISCRIMINATING fact word
                # (one that says something specific, not just "question" /
                # "person" / ...) can justify rejecting on zero overlap.
                discriminating = fact_words - _NON_DISCRIMINATING_FACT_WORDS
                if (
                    discriminating
                    and not (fact_words & provenance_words)
                    and len(provenance_words) >= 2
                ):
                    return _result(Verdict.INVALID, "answer artifact question provenance mismatch")
                return _result(
                    Verdict.VALID, "answer artifact question provenance partially matches", 0.6
                )
            return _result(Verdict.VALID, "answer artifact contains a nonempty answer; question identity unavailable")
    elif fact.predicate == "at_robot":
        if "last_nav_location" in evidence and evidence["last_nav_location"] is not None:
            if not _at_robot_match(evidence["last_nav_location"], fact.args[0]):
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
