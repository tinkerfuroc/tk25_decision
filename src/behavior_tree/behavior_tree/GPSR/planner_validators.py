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


# Match anything that looks like an unresolved template token: <day>, <country>,
# <name>, <foo bar>, etc. Whitelist a few legitimate uses such as <unknown> if
# you ever want them — none right now.
PLACEHOLDER_RE = re.compile(r"<[A-Za-z_][\w \-]*>")

# Location names that mean "where the robot received the command" (≈ the
# operator). Kept in sync with orchestrator.START_LOCATION_ALIASES.
START_LOCATION_WORDS = frozenset({
    "start_position", "instruction_point", "start", "operator",
})

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
        r"follow(?:\s+\w+){1,4}\s+to\s+the\s+([a-z_][a-z_]*)",
        command.lower(),
    )
    return matches


def _same_object(a: str, b: str) -> bool:
    """Case-insensitive object-name match tolerant of singular/plural."""
    a, b = a.lower(), b.lower()
    if a == b:
        return True
    for x, y in ((a, b), (b, a)):
        if y in (x + "s", x + "es"):
            return True
    return False


def validate_plan(
    plan: List[Dict[str, Any]],
    command: str,
    known_actions: Iterable[str],
    category_words: Iterable[str] = DEFAULT_CATEGORY_WORDS,
    known_locations: Optional[Iterable[str]] = None,
) -> Tuple[bool, Optional[str]]:
    """Apply post-checks to a planner-returned plan.

    Returns (True, None) if the plan passes, (False, reason) otherwise.
    Empty plans are accepted here — callers decide whether emptiness is a
    failure in their context (an impossible command should legitimately
    return an empty plan with a reasoning).
    """
    if not isinstance(plan, list):
        return False, f"plan is not a list: {type(plan).__name__}"

    known_actions = set(known_actions)
    known_loc_set = (
        {str(l).lower() for l in known_locations} if known_locations else None
    )
    saw_find_person = False
    saw_ask_person = False
    saw_vlm_fallback = False
    saw_goto_destinations: set = set()
    recorded_labels: set = set()  # labels fixed by an earlier record_position

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
        # Rule: report_answer needs a prior ask_person — there is no captured
        # answer to report otherwise (and "tell me X" likely meant ask_person).
        if action == "report_answer" and not saw_ask_person:
            return False, (
                f"step {i}: report_answer() without a prior ask_person() — "
                "there is no captured answer to report. To tell the operator a "
                "spoken fact, ask_person(question=...) first, then return to "
                "start_position, then report_answer()."
            )
        # Rule: report_view needs a prior vlm_fallback — nothing was looked at
        # to report otherwise.
        if action == "report_view" and not saw_vlm_fallback:
            return False, (
                f"step {i}: report_view() without a prior vlm_fallback() — "
                "there is no observation to report. To tell the operator what "
                "the robot saw elsewhere, vlm_fallback(question=...) first, then "
                "return to start_position, then report_view()."
            )
        if action == "record_position":
            lab = params.get("label")
            if isinstance(lab, str) and lab.strip():
                recorded_labels.add(lab.strip().lower())
        if action == "goto":
            loc = params.get("location")
            if isinstance(loc, str):
                saw_goto_destinations.add(loc.lower())

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
                low = loc.lower()
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
            if isinstance(loc, str) and loc.lower() not in saw_goto_destinations:
                return False, (
                    f"step {i}: find_object(location={loc!r}) without a "
                    f"preceding goto(location={loc!r}). Emit the goto step "
                    "explicitly before searching."
                )

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
            if step.get("action") != "find_object":
                continue
            obj = (step.get("params") or {}).get("object")
            if not isinstance(obj, str):
                continue
            if obj.lower() in cat_set:
                continue  # planner used a category noun (any form) — good
            return False, (
                f"step {i}: command refers to category {cats!r} but "
                f"find_object was emitted with concrete object {obj!r}. "
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
    _SELF_NAV_DEST = {"deliver": "recipient_location", "place": "location"}
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
    for dest in _detect_follow_destinations(command):
        if dest not in saw_goto_destinations:
            return False, (
                f"command mentions 'follow ... to the {dest}' but no "
                f"goto(location={dest!r}) step was emitted. Append a goto "
                "after the follow so the robot can reach the destination."
            )

    return True, None
