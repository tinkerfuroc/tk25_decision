"""Post-validators for plans returned by the LLM planner.

Pure functions, no ROS / py_trees / openai dependencies — shared between
``orchestrator.BtNode_PlanActions`` (runtime) and ``cmd_understanding_test``
(offline test) so both apply the same acceptance criteria.

The validators catch failure modes the prompt asks the LLM to avoid but that
it still emits some of the time:

1. Angle-bracket placeholder leakage in ``announce`` ``text`` fields
   (``"Today is the <day>."``). The downstream announce action would
   read this verbatim.
2. ``guide(location)`` used as a goto substitute — i.e. emitted without any
   preceding ``find_person`` step.
3. Category-to-instance collapse: command says "a drink" / "a fruit" /
   "a snack" but the LLM substituted a concrete object (``coke``, ``apple``,
   …). The robot would search for only that one instance and miss siblings.
4. Dropped tail-clauses: the command names a destination after a follow
   ("follow them to the bedroom") but no ``goto`` for that destination is
   emitted.
5. Implicit navigation: ``find_object(location=X)`` without a preceding
   ``goto(location=X)``. At runtime the robot would scan from wherever it
   happens to be standing.
6. Spurious ``find_object`` before ``count`` of the same object: ``count``
   performs its own scan; the extra find_object picks one closest instance
   for no reason and can fail the whole plan.
7. Unknown locations (only when ``known_locations`` is provided): a
   location param that has no pose in constants.json would leave the
   previous step's nav goal on the blackboard — silent wrong navigation.

Use ``validate_plan(plan, command, known_actions, category_words,
known_locations)`` from the planner; on rejection it returns
``(False, reason)`` and the planner can set ``LAST_FAILURE`` and return
FAILURE so the self-correction subtree re-prompts.
"""

from __future__ import annotations

import re
from typing import Any, Dict, Iterable, List, Optional, Tuple

from .action_contracts import ACTION_CONTRACTS, self_established_facts, self_navigating_destinations
from .validators import canonical_fact, parse_fact

__all__ = [
    "validate_dag", "validate_plan", "uncovered_postcondition_reason",
    "established_predicates", "is_start_alias",
]


# Match anything that looks like an unresolved template token: <day>, <country>,
# <name>, <foo bar>, etc. Whitelist a few legitimate uses such as <unknown> if
# you ever want them — none right now.
PLACEHOLDER_RE = re.compile(r"<[A-Za-z_][\w \-]*>")

# Location names that mean "where the robot received the command" (≈ the
# operator). Kept in sync with orchestrator.START_LOCATION_ALIASES.
# I5 (round-3 adversarial review, M2): "me"/"the user"/"the operator"/
# "command point" are common split-layer wordings for the SAME destination
# ("bring ME the spam", "report to the operator") that were previously
# missing -- every entry here is already normalised (lowercase, spaces ->
# underscores, matching `_norm_loc`/`parse_fact`'s own normalisation).
START_LOCATION_WORDS = frozenset({
    "start_position", "instruction_point", "start", "operator",
    "me", "the_user", "the_operator", "command_point",
})


def is_start_alias(value: Any) -> bool:
    """True when ``value`` names "wherever the robot received the command".

    I5: the split layer's own postcondition can address the operator by any
    of several synonyms (``me``, ``the user``, ``the operator``, ``command
    point``, ...) while the lower layer's plan always navigates to the
    canonical ``start_position`` -- every alias in ``START_LOCATION_WORDS``
    must be treated as ONE destination, both by
    ``uncovered_postcondition_reason`` (below) and by the runtime gate's
    ``at_robot`` provenance check (``validators._verify``, lazy-imported
    from there to avoid the validators<->planner_validators import cycle).
    Single source of truth so the two can never disagree about which words
    mean "the operator".
    """
    return _norm_loc(value) in START_LOCATION_WORDS

# Descriptor words that can follow "follow ... to the" in a person's pointing
# gesture ("pointing to the left/right") but are NOT destinations — the
# dropped-follow-tail rule must not demand a goto() to them.
FOLLOW_DIRECTION_WORDS = frozenset({"left", "right"})


def _norm_loc(name: str) -> str:
    """Normalise a location name for matching: lowercase, spaces -> underscores.

    The generator / operator / LLM say "living room", while the map waypoint
    (constants.json possible_poses) is ``living_room``. Both spellings must
    match. Mirrors ``orchestrator._norm_loc`` (kept local to avoid a circular
    import).
    """
    return str(name).lower().replace(" ", "_").strip()

# Command clauses that address a result back to the operator ("tell me",
# "bring me", "deliver it to me", ...).
OPERATOR_REPORT_RE = re.compile(
    r"\b(tell|show|give|bring|hand|say to|report to)\s+me\b|\bto me\b"
)

# Default set of category words the GPSR command generator may emit. Keep this
# in sync with ``object_categories_singular`` / ``object_categories_plural``
# fed to the generator. Both singular and plural forms are checked.
DEFAULT_CATEGORY_WORDS = frozenset({
    "drink", "drinks",
    "fruit", "fruits",
    "snack", "snacks",
    "food", "foods",
    "cleaner", "cleaners",
    "toy", "toys",
    "dish", "dishes",
    "cutlery",
})

# L1a (round-4 battery fix, run 016): object-param words that are never
# themselves an "invented attribute" -- generic placeholders an LLM may
# legitimately fall back to when it genuinely has no concrete noun.
#
# W-1 (round-4 review fix, HIGH): these words are ONLY a whole-param bypass
# (the param IS just "it"/"item"/...) below -- they must NEVER enter the
# known-object token-subset match, or a legitimate free-text object
# containing one of these as an ordinary WORD (e.g. "Cheez-It" tokenizes to
# ["cheez", "it"]) gets falsely flagged as if "it" were a matched
# known-object subset, with the nonsensical suggested fix ``use "it"``.
_GENERIC_OBJECT_WORDS = frozenset({"object", "it", "item", "thing"})


def _known_object_match(
    tokens: List[str], known_objects: Iterable[str],
) -> Optional[Tuple[List[str], set]]:
    """Find the known object name (from ``known_objects``) most
    specifically matched within ``tokens``.

    W-2 (round-4 review fix, MEDIUM): honors multi-word known names stored
    underscore-joined (``orchestrator.KNOWN_OBJECT_NAMES``, e.g.
    "white_shirt" for constants.json's "white shirt") by requiring ALL of
    the name's own ``split("_")`` tokens to be present among ``tokens`` --
    name-tokens SUBSET-OF query-tokens, the same shape as the existing
    ``planner._is_physical_object_arg`` prior art (``known.split("_")``),
    reused rather than reinvented. Prefers the known name with the MOST
    tokens when several match, so a genuine multi-word hit
    ("white_shirt") always wins over an incidental single-token one
    ("shirt", from some unrelated known name that happens to share it).

    Shared by ``_object_attribute_reason`` (L1a) and
    ``orchestrator.reduce_unknown_object_query`` (L1b) so the two sites
    can't drift into different matching schemes.

    Returns ``(name_tokens, covered_query_tokens)`` for the best match, or
    None when no known name's tokens are all covered.
    """
    best: Optional[Tuple[List[str], set]] = None
    for name in known_objects:
        name_tokens = [t for t in str(name).strip().lower().split("_") if t]
        if not name_tokens:
            continue
        covered: set = set()
        matched_all = True
        for nt in name_tokens:
            hit = next((ot for ot in tokens if ot == nt), None)
            if hit is None:
                matched_all = False
                break
            covered.add(hit)
        if matched_all and (best is None or len(name_tokens) > len(best[0])):
            best = (name_tokens, covered)
    return best


def _object_attribute_reason(
    obj: str, command_tokens: set, known_objects: Iterable[str],
    category_words: Iterable[str],
) -> Optional[str]:
    """Rejection reason for an invented-attribute ``object`` param, or None.

    Sim battery run 016: the LLM planned ``object: "red bowl"`` for a command
    that only ever said "bowl" (the spawned YCB bowl is white) -- four VLM
    scans for "red bowl" never matched anything in the scene. Reject only
    when ALL of: (i) the param is not itself a known object/category/generic
    word, (ii) it contains a known-object token subset (W-2: honoring
    multi-word names like "white_shirt"), and (iii) it also carries a token
    the command never used (the invented attribute). A param like "red mug"
    is accepted when the command itself says "red mug" -- the LLM only
    copied the command.
    """
    obj_norm = str(obj).strip().lower()
    if not obj_norm:
        return None
    # W-1: known_set feeds the token-subset match below and must NEVER
    # contain _GENERIC_OBJECT_WORDS -- the whole-param bypass just below is
    # the ONLY place those words apply.
    known_set = (
        {str(w).lower() for w in known_objects}
        | {str(w).lower() for w in category_words}
    )
    # W-2: the whole-param bypass must recognise a multi-word known name
    # spelled either way -- "white shirt" (LLM wording) or "white_shirt"
    # (KNOWN_OBJECT_NAMES' own storage form).
    if (obj_norm in known_set or obj_norm.replace(" ", "_") in known_set
            or obj_norm in _GENERIC_OBJECT_WORDS):
        return None
    obj_tokens = _tokenize(obj)
    if not obj_tokens:
        return None
    match = _known_object_match(obj_tokens, known_set)
    if match is None:
        return None  # no known-object subset -- not this rule's business
    matched, covered = match
    extra = [t for t in obj_tokens if t not in covered]
    if not extra:
        return None
    if all(t in command_tokens for t in extra):
        return None  # every extra token is right there in the command
    return (
        f'object "{obj}" adds attributes not in the command; '
        f'use "{" ".join(matched)}"'
    )


def validate_dag(targets: list[dict[str, Any]]) -> tuple[bool, Optional[str]]:
    """Validate the canonical ordered-DAG representation of GPSR targets.

    Dependencies are intentionally required to point to an earlier target in
    the supplied order.  This keeps execution index-sequential while allowing
    a target to depend on multiple completed targets.
    """
    from collections.abc import Mapping

    if not isinstance(targets, list):
        return False, f"targets is not a list: {type(targets).__name__}"

    target_ids: list[str] = []
    id_indices: dict[str, int] = {}
    for index, target in enumerate(targets):
        if not isinstance(target, Mapping):
            return False, f"target {index} is not a mapping: {target!r}"
        target_id = target.get("id")
        if not isinstance(target_id, str) or not target_id.strip():
            return False, f"target {index}: id must be a non-empty string"
        if target_id in id_indices:
            return False, (
                f"target {index} {target_id!r}: duplicate target id; "
                f"first seen at target {id_indices[target_id]}"
            )
        id_indices[target_id] = index
        target_ids.append(target_id)

    dependencies: list[list[str]] = []
    for index, target in enumerate(targets):
        target_id = target_ids[index]
        if "depends_on" not in target:
            return False, f"target {index} {target_id!r}: depends_on is required"
        depends_on = target["depends_on"]
        if not isinstance(depends_on, list):
            return False, (
                f"target {index} {target_id!r}: depends_on must be a list"
            )

        seen: set[str] = set()
        target_dependencies: list[str] = []
        for dependency_index, dependency in enumerate(depends_on):
            if not isinstance(dependency, str) or not dependency.strip():
                return False, (
                    f"target {index} {target_id!r}: dependency {dependency_index} "
                    "in depends_on must be a non-empty string target ID"
                )
            if dependency in seen:
                return False, (
                    f"target {index} {target_id!r}: duplicate dependency ID "
                    f"{dependency!r}"
                )
            seen.add(dependency)
            if dependency not in id_indices:
                return False, (
                    f"target {index} {target_id!r}: unknown dependency id "
                    f"{dependency!r}"
                )
            dependency_target_index = id_indices[dependency]
            if dependency == target_id:
                return False, (
                    f"target {index} {target_id!r}: self-dependency on "
                    f"{dependency!r} is not allowed"
                )
            if dependency_target_index >= index:
                return False, (
                    f"target {index} {target_id!r}: dependency {dependency!r} "
                    "must reference a strictly earlier target"
                )
            target_dependencies.append(dependency)
        dependencies.append(target_dependencies)

    # Defensive topological check.  Earlier-only dependencies make a cycle
    # impossible, but keep this guard explicit for future representation changes.
    indegree = [len(deps) for deps in dependencies]
    dependents: list[list[int]] = [[] for _ in targets]
    for target_index, deps in enumerate(dependencies):
        for dependency in deps:
            dependents[id_indices[dependency]].append(target_index)
    ready = [index for index, degree in enumerate(indegree) if degree == 0]
    processed = 0
    while ready:
        index = ready.pop(0)
        processed += 1
        for dependent in dependents[index]:
            indegree[dependent] -= 1
            if indegree[dependent] == 0:
                ready.append(dependent)
    if processed != len(targets):
        cycle_index = next(
            index for index, degree in enumerate(indegree) if degree > 0
        )
        return False, (
            f"target {cycle_index} {target_ids[cycle_index]!r}: dependency cycle detected"
        )

    return True, None


def _tokenize(text: str) -> List[str]:
    return re.findall(r"[a-zA-Z]+", text.lower())


def _detect_command_categories(command: str, category_words: Iterable[str]) -> List[str]:
    """Return any category nouns found in the original command."""
    toks = set(_tokenize(command))
    return sorted({w for w in category_words if w in toks})


def _detect_follow_destinations(command: str) -> List[str]:
    """Return location nouns trailing a `follow ... to the X` clause.

    Returns the bare destination noun(s). Heuristic only — meant as a tail-drop
    signal, not as a parser.
    """
    matches = re.findall(
        r"follow(?:\s+\w+){1,4}\s+to\s+the\s+((?:[a-z]+_)*[a-z]+(?:\s+room)?)",
        command.lower(),
    )
    # Normalize to the canonical underscore form used by known locations
    # (living room -> living_room), since lower-layer plans use _norm_loc.
    return [re.sub(r"\s+", "_", m.strip()) for m in matches]


# L-2 (round-3 fix review): the +s/+es rule below misses common irregular
# plurals an operator (or the split layer) may say -- "counted(people)" vs
# "count(object='person')" -- rejected outright with no matching rule.
_IRREGULAR_PLURALS = {
    "people": "person",
    "children": "child",
    "knives": "knife",
    "shelves": "shelf",
    "boxes": "box",
    "dishes": "dish",
    "glasses": "glass",
}


def _same_object(a: str, b: str) -> bool:
    """Case-insensitive object-name match tolerant of singular/plural."""
    a, b = a.lower(), b.lower()
    if a == b:
        return True
    for x, y in ((a, b), (b, a)):
        if y in (x + "s", x + "es"):
            return True
        if _IRREGULAR_PLURALS.get(x) == y:
            return True
    return False


def established_predicates(steps: Iterable[Dict[str, Any]]) -> set:
    """Predicate names any step in ``steps`` establishes, per its contract.

    Shared by ``uncovered_postcondition_reason`` (below) and
    ``planner.replace_target_plan`` (which subtracts predicates already
    established by ``completed_steps`` before re-checking coverage on the
    REMAINING plan) — both need "what does this list of steps establish"
    computed identically.
    """
    predicates: set = set()
    for step in steps:
        if not isinstance(step, dict):
            continue
        contract = ACTION_CONTRACTS.get(step.get("action"))
        if contract is not None:
            predicates.update(t.split("(", 1)[0] for t in contract.establishes)
    return predicates


def uncovered_postcondition_reason(
    plan: List[Dict[str, Any]], postconditions: Iterable[str],
) -> Optional[str]:
    """Rejection message for the first of ``postconditions`` the plan leaves uncovered.

    Predicate-level match only: a step's action counts as an establisher of a
    postcondition's predicate if that predicate appears among
    ``ACTION_CONTRACTS[action].establishes`` — the actual argument identity
    (e.g. ``person_found(sarah)`` vs. a step scanning for someone else) is
    left to the runtime gate, which has execution evidence this static check
    does not.

    ``at_robot`` is special-cased: a step that self-navigates to a location
    (``goto``/``search_object``/``place``/``deliver`` — anything with
    ``at_robot`` in ``self_establishes``, per ``self_established_facts``)
    satisfies ``at_robot(X)`` ONLY when its own destination matches ``X``
    (normalised — M-3, round-2 review: a guard-reduced
    ``[goto(laundry_desk)]`` must NOT satisfy ``at_robot(kitchen)`` just
    because SOME goto exists; that plan passes this static check and then
    fails the RUNTIME gate instead, on a mismatched
    ``last_nav_location``). ``goto`` already appears in ``established``
    (predicate-level) via its own ``establishes=("at_robot(location)",))``,
    but that generic branch is skipped entirely for ``at_robot`` (below) so
    it can never re-introduce the any-destination bug for ``goto`` either.

    Returns ``None`` when every parsable postcondition is covered (or there
    are none); otherwise the FIRST uncovered postcondition's rejection
    message (checked in ``postconditions`` order — it does not wait to
    collect every miss), naming every registry establisher for its predicate.
    """
    established = established_predicates(plan)
    self_nav_destinations = {
        nav_fact.args[0]
        for step in plan
        if isinstance(step, dict)
        for f in self_established_facts(step)
        for nav_fact, _err in (parse_fact(f),)
        if nav_fact is not None and nav_fact.predicate == "at_robot" and nav_fact.args
    }

    for condition in postconditions or []:
        fact, _error = parse_fact(condition)
        if fact is None:
            continue
        if fact.predicate == "at_robot":
            # Destination match ONLY -- never falls through to the generic
            # `established` check below, which would re-introduce the
            # any-destination bug via goto's own
            # establishes=("at_robot(location)",).
            # I5: an operator-alias postcondition (at_robot(me)/at_robot(the
            # operator)/...) is covered by a goto to ANY operator alias
            # (typically start_position, per the split prompt) -- not just
            # an exact string match.
            if fact.args and (
                fact.args[0] in self_nav_destinations
                or (is_start_alias(fact.args[0])
                    and any(is_start_alias(dest) for dest in self_nav_destinations))
            ):
                continue
        elif fact.predicate in established:
            continue
        establishers = [
            name for name, c in ACTION_CONTRACTS.items()
            if any(t.split("(", 1)[0] == fact.predicate for t in c.establishes)
        ]
        return (
            f"plan establishes nothing for postcondition {canonical_fact(fact)}; "
            "add one of: " + ", ".join(establishers)
        )
    return None


def validate_plan(
    plan: List[Dict[str, Any]],
    command: str,
    known_actions: Iterable[str],
    category_words: Iterable[str] = DEFAULT_CATEGORY_WORDS,
    known_locations: Optional[Iterable[str]] = None,
    prior_plan: Optional[Iterable[Dict[str, Any]]] = None,
    postconditions: Optional[Iterable[str]] = None,
    known_objects: Optional[Iterable[str]] = None,
    raw_command: Optional[str] = None,
) -> Tuple[bool, Optional[str]]:
    """Apply post-checks to a planner-returned plan.

    ``prior_plan`` (optional) is an already-accepted action plan from an EARLIER
    target of the same command (two-layer planner). Its actions seed the
    cross-target state so a later target is not re-rejected for legitimate
    handoff: a ``guide()`` after an earlier ``find_person``, a ``find_object``
    at a location an earlier ``goto`` already reached, or a ``goto`` to a label
    an earlier ``record_position`` fixed. Only the prior STEPS are seeded —
    the current ``plan`` is validated on its own.

    ``postconditions`` (optional) is the target's own declared postcondition
    list (``target["postconditions"]``). When given, every parsable
    postcondition must be named by at least one step's contract
    (``ACTION_CONTRACTS[action].establishes``) — see
    ``uncovered_postcondition_reason``. This is the LAST check applied, so
    it never pre-empts an earlier, more specific rejection.

    ``known_objects`` (optional, opt-in like ``known_locations``) enables the
    L1a invented-attribute-object check (see ``_object_attribute_reason``):
    a ``find_object``/``grasp`` object param that adds a descriptive token
    the command never used, on top of a known-object token subset, is
    rejected. ``raw_command`` overrides ``command`` as the text that check
    compares against (the two-layer planner has the full multi-clause
    command available beyond this call's per-target ``command``); defaults
    to ``command`` when not given.

    Returns (True, None) if the plan passes, (False, reason) otherwise.
    Empty plans are accepted here — callers decide whether emptiness is a
    failure in their context (an impossible command should legitimately
    return an empty plan with a reasoning).
    """
    if not isinstance(plan, list):
        return False, f"plan is not a list: {type(plan).__name__}"

    known_actions = set(known_actions)
    known_loc_set = (
        {_norm_loc(l) for l in known_locations} if known_locations else None
    )
    saw_find_person = False
    saw_ask_person = False
    saw_vlm_fallback = False
    saw_llm_fallback = False
    saw_count = False
    saw_describe_person = False
    saw_goto_destinations: set = set()
    recorded_labels: set = set()  # labels fixed by an earlier record_position

    # Seed cross-target state from an earlier target's accepted plan.
    if prior_plan:
        for step in prior_plan:
            if not isinstance(step, dict):
                continue
            action = step.get("action")
            params = step.get("params") or {}
            if action == "find_person":
                saw_find_person = True
            elif action == "ask_person":
                saw_ask_person = True
            elif action == "vlm_fallback":
                saw_vlm_fallback = True
            elif action == "llm_fallback":
                saw_llm_fallback = True
            elif action == "count":
                saw_count = True
            elif action == "describe_person":
                saw_describe_person = True
            elif action == "record_position":
                lab = params.get("label")
                if isinstance(lab, str) and lab.strip():
                    recorded_labels.add(_norm_loc(lab))
            elif action == "goto":
                loc = params.get("location")
                if isinstance(loc, str):
                    saw_goto_destinations.add(_norm_loc(loc))

    for i, step in enumerate(plan):
        if not isinstance(step, dict):
            return False, f"step {i} is not a dict: {step!r}"
        action = step.get("action")
        params = step.get("params") or {}
        if not isinstance(params, dict):
            return False, f"step {i}: params is not a dict"
        if action not in known_actions:
            return False, f"step {i}: unknown action {action!r}"

        if action == "find_person":
            saw_find_person = True
        if action == "ask_person":
            saw_ask_person = True
        if action == "vlm_fallback":
            saw_vlm_fallback = True
        if action == "llm_fallback":
            saw_llm_fallback = True
        if action == "count":
            saw_count = True
        if action == "describe_person":
            saw_describe_person = True
        # Rule: a text-less ``announce`` reports the latest gathered result from
        # the REPORT_INFO buffer, so it needs a prior result-producing action
        # (count / describe_person / ask_person / vlm_fallback / llm_fallback).
        # A bare announce with nothing gathered would speak an empty/stale buffer.
        if action == "announce" and not (params.get("text") or params.get("message")):
            if not (saw_count or saw_describe_person or saw_ask_person
                    or saw_vlm_fallback or saw_llm_fallback):
                return False, (
                    f"step {i}: announce() with no text reports the last gathered "
                    "result, but no count/describe_person/ask_person/vlm_fallback/"
                    "llm_fallback ran before it. Either give announce a literal "
                    "text=..., or do the gathering action first (then goto "
                    "start_position, then the text-less announce)."
                )
        if action == "record_position":
            lab = params.get("label")
            if isinstance(lab, str) and lab.strip():
                recorded_labels.add(_norm_loc(lab))
        if action == "goto":
            loc = params.get("location")
            if isinstance(loc, str):
                saw_goto_destinations.add(_norm_loc(loc))

        # Rule: every location param must be a known location, a recorded
        # dynamic label, or a start alias. An unknown name has no pose, so the
        # robot would silently keep the previous goal. Recorded labels must
        # have been fixed by a record_position step EARLIER in the plan —
        # because we iterate in order, recorded_labels only holds labels from
        # prior steps here, which also enforces record-before-goto.
        if known_loc_set is not None:
            for loc_key in ("location", "recipient_location"):
                loc = params.get(loc_key)
                if not isinstance(loc, str):
                    continue
                low = _norm_loc(loc)
                # known locations are also normalised ("living room" == living_room)
                if low in known_loc_set or low in recorded_labels:
                    continue
                return False, (
                    f"step {i}: {loc_key}={loc!r} is not a known location and "
                    "was not fixed by an earlier record_position(label=...). "
                    "Use a known location, or record_position it first."
                )

        # Rule: guide must follow a find_person earlier in the plan.
        if action == "guide" and not saw_find_person:
            return False, (
                f"step {i}: guide() used without a prior find_person() — "
                "guide is for leading a person, not for moving the robot. "
                "Use goto() to move the robot."
            )

        # Rule: navigation is never implicit. find_object with a location
        # param must come after an explicit goto to that location.
        if action == "find_object":
            loc = params.get("location")
            if isinstance(loc, str) and _norm_loc(loc) not in saw_goto_destinations:
                return False, (
                    f"step {i}: find_object(location={loc!r}) without a "
                    f"preceding goto(location={loc!r}). Emit the goto step "
                    "explicitly before searching."
                )

        # Rule (L1a, round-4 battery fix, run 016): a find_object/grasp
        # object param must not invent a descriptive attribute the command
        # never named on top of a known-object noun ("red bowl" when the
        # command only ever said "bowl").
        if known_objects is not None and action in ("find_object", "grasp"):
            obj = params.get("object")
            if isinstance(obj, str):
                attr_reason = _object_attribute_reason(
                    obj, set(_tokenize(raw_command if raw_command else command)),
                    known_objects, category_words,
                )
                if attr_reason is not None:
                    return False, attr_reason

        # Rule: no angle-bracket placeholders in say/tell_info text.
        for k, v in params.items():
            if isinstance(v, str) and PLACEHOLDER_RE.search(v):
                m = PLACEHOLDER_RE.search(v).group(0)
                return False, (
                    f"step {i}: angle-bracket placeholder {m!r} in "
                    f"params[{k!r}]={v!r}. Resolve the literal value or "
                    "refuse the step — never emit unresolved templates."
                )

    # Rule: category-to-instance collapse.
    # If the command mentions a category and the plan has a find_object whose
    # object is a concrete known item (not ANY category word), reject. We
    # accept both singular and plural forms — drinks/drink, snacks/snack, etc.
    cats = _detect_command_categories(command, category_words)
    if cats:
        cat_set = {w.lower() for w in category_words}
        for i, step in enumerate(plan):
            if step.get("action") not in ("find_object", "search_object"):
                continue
            obj = (step.get("params") or {}).get("object")
            if not isinstance(obj, str):
                continue
            if obj.lower() in cat_set:
                continue  # planner used a category noun (any form) — good
            return False, (
                f"step {i}: command refers to category {cats!r} but "
                f"{step.get('action')} was emitted with concrete object {obj!r}. "
                "Pass the category noun through as the object parameter so "
                "the vision module searches for any matching item."
            )

    # Rule: count is self-contained. A find_object for the same object before
    # a count step adds a pointless closest-instance pick that can fail the
    # plan and adds nothing — count scans the category prompt itself.
    count_objects = [
        ((step.get("params") or {}).get("object"), i)
        for i, step in enumerate(plan) if step.get("action") == "count"
    ]
    for count_obj, count_idx in count_objects:
        if not isinstance(count_obj, str):
            continue
        for i, step in enumerate(plan):
            if step.get("action") != "find_object":
                continue
            obj = (step.get("params") or {}).get("object")
            if isinstance(obj, str) and _same_object(obj, count_obj):
                return False, (
                    f"step {i}: find_object({obj!r}) is redundant before "
                    f"count({count_obj!r}) at step {count_idx} — count() "
                    "performs its own scan. Remove the find_object step."
                )

    # Rule: no goto immediately before a self-navigating action. ``deliver``
    # and ``place`` always drive to their own destination, so ANY goto right
    # before them double-navigates — even when the LLM tries to dodge by
    # moving the location out of the action into a preceding goto (deliver
    # then re-navigates to the stale pose that goto just set). The destination
    # must live on the action itself: deliver -> recipient_location,
    # place -> location. No goto before them.
    _SELF_NAV_DEST = self_navigating_destinations()
    for i in range(len(plan) - 1):
        if plan[i].get("action") != "goto":
            continue
        nxt = plan[i + 1]
        dest_key = _SELF_NAV_DEST.get(nxt.get("action"))
        if dest_key is None:
            continue
        goto_loc = (plan[i].get("params") or {}).get("location")
        return False, (
            f"step {i}: goto(location={goto_loc!r}) right before "
            f"{nxt.get('action')} is redundant — {nxt.get('action')} "
            f"navigates to its destination itself. Drop the goto and put the "
            f"destination on the {nxt.get('action')} step "
            f"({dest_key}={goto_loc!r})."
        )

    # Rule: report addressed to the operator must be delivered back at the
    # start. If the command says "tell/bring/give ME ..." AND the robot left
    # the start area to do the task, it must return to ``start_position``
    # before the final report — otherwise it reports to an empty spot.
    if plan and OPERATOR_REPORT_RE.search(command.lower()):
        left_start = any(
            s.get("action") == "goto"
            and isinstance((s.get("params") or {}).get("location"), str)
            and (s["params"]["location"].lower() not in START_LOCATION_WORDS)
            for s in plan
        )
        if left_start:
            last = plan[-1]
            last_action = last.get("action")
            last_params = last.get("params") or {}
            if last_action == "announce":
                prev = plan[-2] if len(plan) >= 2 else {}
                prev_loc = str((prev.get("params") or {}).get("location", "")).lower()
                if not (prev.get("action") == "goto"
                        and prev_loc in START_LOCATION_WORDS):
                    return False, (
                        "command reports to ME but the final announce is not "
                        "preceded by goto(location=start_position) — the robot "
                        "left the start area, so it must return before "
                        "reporting. Insert goto(location=start_position) before "
                        "the final announce."
                    )
            elif last_action == "deliver":
                recip = str(last_params.get("recipient_location", "")).lower()
                if recip not in START_LOCATION_WORDS:
                    return False, (
                        "command delivers an object to ME but the deliver step "
                        "has no recipient_location=start_position — set it so "
                        "the robot brings the object back to the operator."
                    )

    # Rule: dropped follow-tail. If the command says "follow them to the
    # bedroom" but the plan never goes to the bedroom, that's a silent drop.
    # Skip bare direction descriptors ("pointing to the left/right") — those
    # are person descriptors, not destinations.
    for dest in _detect_follow_destinations(command):
        if dest in FOLLOW_DIRECTION_WORDS:
            continue
        if dest not in saw_goto_destinations:
            return False, (
                f"command mentions 'follow ... to the {dest}' but no "
                f"goto(location={dest!r}) step was emitted. Append a goto "
                "after the follow so the robot can reach the destination."
            )

    # Rule: the plan must cover its target's declared postconditions. Run
    # LAST so every earlier, more specific rejection keeps precedence over
    # this coarser one.
    if postconditions:
        reason = uncovered_postcondition_reason(plan, postconditions)
        if reason is not None:
            return False, reason

    return True, None
