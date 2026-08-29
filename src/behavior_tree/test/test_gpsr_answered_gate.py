"""C1 — announce() satisfies answered(question) for its OWN target.

Regression coverage for the "tell me what day it is" defect: llm_fallback
failed, the replan fell back to announce(text="Today is Saturday.") which
SUCCEEDED at runtime, but the postcondition gate reported
answered(what day is today) UNKNOWN because only qa_answer/person_answer/
llm_answer/vlm_answer evidence satisfied the answered() branch. A spoken
announce carries no such artifact.

Two things had to change together:

1. action_contracts.ACTION_CONTRACTS["announce"] now declares
   establishes=("answered(question)",) -- registered AFTER ask_person /
   answer_question so the FIRST-registered establisher
   (planner._ESTABLISHER_FOR_PREDICATE) stays ask_person. count/vlm_fallback/
   llm_fallback also declare it now (round-2 H1 fix), all likewise
   registered after ask_person/answer_question.
2. validators._action_verdict gains a generic "answered" fallback: a
   completed, successful step whose contract establishes "answered" counts
   as an answer, UNLESS the step is flagged params["acknowledgement"] = True
   -- the planner's guaranteed fallback plan (orchestrator._fallback_plan)
   sets that flag on its apology announce so a planning FAILURE can never
   read back as a postcondition PASS. (round-2 M1 fix: this no longer also
   requires a non-empty params["text"] -- a text-less announce reporting a
   prior count/ask_person/vlm_fallback/llm_fallback result now qualifies via
   THAT predecessor step's own contract, without inspecting the announce's
   own params at all. The static bare-announce rule in
   planner_validators.validate_plan is what actually enforces that a
   text-less announce only follows a real gathering step.)
"""
from __future__ import annotations

import pytest

from behavior_tree.GPSR import action_contracts as ac
from behavior_tree.GPSR.orchestrator import _fallback_plan
from behavior_tree.GPSR.action_contracts import IDENTICAL_PLAN_ERROR_PREFIX
from behavior_tree.GPSR.validators import (
    Verdict,
    VerificationContext,
    _action_verdict,
    _verify,
    parse_fact,
)


# ---------------------------------------------------------------------------
# Registry order: answered's canonical establisher stays ask_person.
# ---------------------------------------------------------------------------

def test_announce_establishes_answered_but_registered_after_ask_person():
    contract = ac.contract_for("announce")
    assert any(t.split("(", 1)[0] == "answered" for t in contract.establishes)

    from behavior_tree.GPSR.planner import _ESTABLISHER_FOR_PREDICATE
    assert _ESTABLISHER_FOR_PREDICATE["answered"] == "ask_person"


def test_all_answered_establishers_registered_after_ask_person_and_answer_question():
    # H1: count/announce/vlm_fallback/llm_fallback all declare
    # establishes=(..., "answered(question)") now, but every one of them is
    # registered AFTER ask_person/answer_question in ACTION_CONTRACTS, so
    # the canonical establisher (_ESTABLISHER_FOR_PREDICATE, first-registered
    # wins via setdefault) never changes.
    order = list(ac.ACTION_CONTRACTS)
    ask_index = order.index("ask_person")
    answer_index = order.index("answer_question")
    for action in ("count", "announce", "vlm_fallback", "llm_fallback"):
        contract = ac.contract_for(action)
        assert any(t.split("(", 1)[0] == "answered" for t in contract.establishes), action
        assert order.index(action) > ask_index, action
        assert order.index(action) > answer_index, action

    from behavior_tree.GPSR.planner import _ESTABLISHER_FOR_PREDICATE
    assert _ESTABLISHER_FOR_PREDICATE["answered"] == "ask_person"
    assert _ESTABLISHER_FOR_PREDICATE["counted"] == "count"


def test_announce_established_facts_stays_empty():
    # announce has a `text` param, not `question` -- the establishes
    # template can never resolve, so established_facts() (used by the
    # contract-boundary guard) must never claim announce drops a fact for a
    # sibling target.
    assert ac.established_facts({"action": "announce", "params": {"text": "hi"}}) == []
    assert ac.established_facts({"action": "announce", "params": {}}) == []


# ---------------------------------------------------------------------------
# _action_verdict: the generic answered() fallback.
# ---------------------------------------------------------------------------

def test_action_verdict_announce_success_is_valid_with_generic_reason():
    fact, _ = parse_fact("answered(what_day_is_today)")
    ctx = VerificationContext(
        phase="postcondition",
        completed_steps=(
            {"action": "announce", "params": {"text": "Today is Saturday."}},
        ),
    )
    result = _action_verdict(fact, ctx)
    assert result is not None
    assert result.verdict is Verdict.VALID
    assert result.confidence == 0.5
    assert result.evidence == (
        "action-verdict fallback: spoken announce stands as the answer; "
        "question identity unavailable"
    )


def test_action_verdict_announce_empty_text_still_valid_since_M1():
    # M1 (round-2 fix): the verdict no longer inspects params["text"] at all
    # -- a text-less announce's OWN contract already establishes "answered"
    # (H1), so it qualifies on its own, same as ask_person/answer_question/
    # count/vlm_fallback/llm_fallback would. What actually stops a
    # groundless text-less announce from ever reaching here is the STATIC
    # bare-announce rule in planner_validators.validate_plan (it requires a
    # prior count/describe_person/ask_person/vlm_fallback/llm_fallback
    # before a text-less announce is even accepted into a plan) -- this
    # runtime fallback trusts that gate already ran.
    fact, _ = parse_fact("answered(what_day_is_today)")
    ctx = VerificationContext(
        phase="postcondition",
        completed_steps=({"action": "announce", "params": {"text": ""}},),
    )
    result = _action_verdict(fact, ctx)
    assert result is not None
    assert result.verdict is Verdict.VALID
    assert result.confidence == 0.5
    # Full path agrees.
    result = _verify(fact, {}, ctx)
    assert result.verdict is Verdict.VALID


def test_action_verdict_textless_announce_after_count_is_valid():
    # The t1-42 sim shape: [goto, count, goto(start_position), announce()]
    # for an answered(how many ...) target. count itself now establishes
    # "answered" too (H1), so this qualifies even before the text-less
    # announce step is reached.
    fact, _ = parse_fact("answered(how_many_drinks_there_are_on_the_kitchen_table)")
    ctx = VerificationContext(
        phase="postcondition",
        completed_steps=(
            {"action": "goto", "params": {"location": "kitchen_table"}},
            {"action": "count", "params": {"object": "drinks", "location": "kitchen_table"}},
            {"action": "goto", "params": {"location": "start_position"}},
            {"action": "announce", "params": {}},
        ),
    )
    result = _action_verdict(fact, ctx)
    assert result is not None
    assert result.verdict is Verdict.VALID
    assert result.confidence == 0.5


def test_verify_answered_accepts_bare_count_value_artifact():
    # H1: validators._verify's answered branch now mirrors the `counted`
    # branch and accepts a bare count_value artifact as sufficient runtime
    # evidence for answered(...), same as qa_answer/llm_answer/vlm_answer.
    fact, _ = parse_fact("answered(how_many_drinks)")
    ctx = VerificationContext(phase="postcondition", completed_steps=())
    result = _verify(fact, {"count_value": 3}, ctx)
    assert result.verdict is Verdict.VALID
    assert "count_value" not in result.evidence  # generic identity-unavailable wording
    assert "nonempty answer" in result.evidence


def test_action_verdict_unrelated_action_yields_no_verdict():
    fact, _ = parse_fact("answered(what_day_is_today)")
    ctx = VerificationContext(
        phase="postcondition",
        completed_steps=({"action": "follow", "params": {"person": "sarah"}},),
    )
    assert _action_verdict(fact, ctx) is None


def test_action_verdict_precondition_phase_never_verdicts():
    fact, _ = parse_fact("answered(what_day_is_today)")
    ctx = VerificationContext(
        phase="precondition",
        completed_steps=(
            {"action": "announce", "params": {"text": "Today is Saturday."}},
        ),
    )
    assert _action_verdict(fact, ctx) is None


def test_action_verdict_ignores_acknowledgement_flagged_announce():
    # The planner's guaranteed fallback plan (orchestrator._fallback_plan)
    # marks its apology announce acknowledgement=True -- it must NEVER stand
    # in for a real answer.
    fact, _ = parse_fact("answered(what_day_is_today)")
    ctx = VerificationContext(
        phase="postcondition",
        completed_steps=(
            {"action": "announce", "params": {
                "text": "I heard your command but could not work out a plan.",
                "acknowledgement": True,
            }},
        ),
    )
    assert _action_verdict(fact, ctx) is None


# ---------------------------------------------------------------------------
# I-3 (round-2 review) -- an LLM-authored refusal/apology announce (NOT
# flagged acknowledgement, since the LLM wrote it, not _fallback_plan /
# replace_target_plan) must never stand in for a real answer either.
# ---------------------------------------------------------------------------

def test_action_verdict_ignores_refusal_worded_announce_without_acknowledgement_flag():
    # Scenario: count/vlm_fallback failed, the replan fell back to
    # [goto start_position, announce(text="I could not count the drinks")].
    # Coverage accepts it (announce establishes answered()) and the announce
    # itself succeeds -- but the text is a refusal, not an answer, and this
    # replan plan never tagged it acknowledgement=True (only
    # _fallback_plan/replace_target_plan do that). _action_verdict must
    # still refuse to call this VALID.
    fact, _ = parse_fact("answered(how_many_drinks)")
    ctx = VerificationContext(
        phase="postcondition",
        completed_steps=(
            {"action": "announce", "params": {"text": "I could not count the drinks"}},
        ),
    )
    assert _action_verdict(fact, ctx) is None


def test_action_verdict_accepts_real_answer_announce_without_acknowledgement_flag():
    fact, _ = parse_fact("answered(how_many_drinks)")
    ctx = VerificationContext(
        phase="postcondition",
        completed_steps=(
            {"action": "announce", "params": {"text": "There are three drinks."}},
        ),
    )
    result = _action_verdict(fact, ctx)
    assert result is not None and result.verdict is Verdict.VALID


@pytest.mark.parametrize("text", [
    "I could not count the drinks",
    "Sorry, I couldn't find the object",
    "sorry, I cannot answer that",
    "I can't complete that task",
    "I was unable to locate the person",
    "I am unable to see the object",
    "I did not find anything",
    "I didn't understand the question",
    "I failed to grasp the object",
])
def test_refusal_regex_matches_every_pinned_apology_phrasing(text):
    from behavior_tree.GPSR.validators import _REFUSAL_RE
    assert _REFUSAL_RE.match(text) is not None


def test_refusal_regex_does_not_match_a_real_answer():
    from behavior_tree.GPSR.validators import _REFUSAL_RE
    assert _REFUSAL_RE.match("There are three drinks.") is None
    assert _REFUSAL_RE.match("Today is Saturday.") is None


# ---------------------------------------------------------------------------
# I-3(a) -- the lower-layer prompt (system + retry text) instructs the model
# to tag its own refusal/apology announce acknowledgement=True.
# ---------------------------------------------------------------------------

def test_lower_layer_system_prompt_requires_acknowledgement_on_refusal_announce():
    from behavior_tree.GPSR.planner import LOWER_LAYER_SYSTEM_PROMPT
    assert '"acknowledgement": true' in LOWER_LAYER_SYSTEM_PROMPT
    assert "REFUSAL" in LOWER_LAYER_SYSTEM_PROMPT
    assert "never an answer" in LOWER_LAYER_SYSTEM_PROMPT.lower() or \
        "not an answer" in LOWER_LAYER_SYSTEM_PROMPT.lower()


def test_empty_plan_retry_reason_requires_acknowledgement_on_refusal_announce(monkeypatch):
    import py_trees
    from behavior_tree.GPSR import planner as planner_mod

    p = planner_mod.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "_new_client", lambda: object())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    prompts = []

    def fake_call(client, system, user, temperature):
        prompts.append(user)
        return {"plan": []}, None

    monkeypatch.setattr(planner_mod, "_call_llm", fake_call)

    p.plan_target(0, 0, "tell me what day it is", command="tell me what day it is")

    assert len(prompts) == 2
    assert "acknowledgement" in prompts[1]


def test_artifact_branch_still_wins_when_present():
    # A step with an actual qa_answer/llm_answer artifact must win over the
    # action-verdict fallback -- the fallback only fires when _verify's
    # earlier, more specific branches found nothing.
    fact, _ = parse_fact("answered(what_day_is_today)")
    ctx = VerificationContext(
        phase="postcondition",
        completed_steps=(
            {"action": "announce", "params": {"text": "Today is Saturday."}},
        ),
    )
    result = _verify(fact, {"llm_answer": "Saturday"}, ctx)
    assert result.verdict is Verdict.VALID
    assert result.evidence == (
        "answer artifact contains a nonempty answer; question identity unavailable"
    )


def test_action_verdict_widening_covers_ask_person_and_answer_question():
    # Deliberate widening: ANY action whose contract establishes answered()
    # qualifies the same way, not just announce -- proves the check is
    # generic (contract-driven), not a hardcoded "announce" string compare.
    # L1 (round-2 fix): a REALISTIC step shape -- ask_person/answer_question
    # carry `question`, never `text` (orchestrator.materialise_params reads
    # `question` for ask_person, see orchestrator.py ~1208-1211). Per M1 the
    # verdict no longer inspects any param at all, so this also proves the
    # rule holds for the actual param shape these actions use, not a
    # fabricated `text` key that ask_person never has.
    for action in ("ask_person", "answer_question"):
        fact, _ = parse_fact("answered(what_day_is_today)")
        ctx = VerificationContext(
            phase="postcondition",
            completed_steps=({"action": action, "params": {"question": "What day is today?"}},),
        )
        result = _action_verdict(fact, ctx)
        assert result is not None and result.verdict is Verdict.VALID, action


# ---------------------------------------------------------------------------
# C1.3 — the fallback-plan acknowledgement announce IS executed (not
# skipped) by the executor, so it must carry the acknowledgement flag.
# ---------------------------------------------------------------------------

def test_fallback_plan_marks_its_announce_as_acknowledgement():
    plan = _fallback_plan("some command")
    assert len(plan) == 1
    assert plan[0]["action"] == "announce"
    assert plan[0]["params"]["acknowledgement"] is True
    assert plan[0]["params"]["text"].strip()


def test_fallback_plan_error_marker_does_not_match_identical_plan_prefix():
    # Pins the C1.3 assumption: the cached entry planner.plan_target stores
    # for an exhausted-attempts fallback ("all N attempts failed (last
    # reason: ...)") does NOT start with IDENTICAL_PLAN_ERROR_PREFIX, so
    # orchestrator.DynamicExecutor's REQUESTING branch
    # (error.startswith(IDENTICAL_PLAN_ERROR_PREFIX)) does not skip it -- it
    # gets swapped in and executed like any other plan. Without the
    # acknowledgement flag above, validators._action_verdict's new answered()
    # fallback would then turn this planning FAILURE into a postcondition
    # gate PASS.
    reason = f"all 4 attempts failed (last reason: some LLM error)"
    assert not reason.startswith(IDENTICAL_PLAN_ERROR_PREFIX)


def test_plan_target_exhaustion_stores_ready_non_identical_acknowledgement(monkeypatch):
    # End-to-end pin of C1.3: when every LLM attempt fails, plan_target's
    # exhausted-attempts fallback is stored READY (not skipped) with an error
    # that does not match IDENTICAL_PLAN_ERROR_PREFIX, and its sole step
    # carries acknowledgement=True.
    import py_trees
    from behavior_tree.GPSR import planner as planner_mod

    p = planner_mod.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "_new_client", lambda: object())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    monkeypatch.setattr(planner_mod, "_call_llm", lambda *a, **k: (None, "model error 400"))

    p.plan_target(0, 0, "tell me what day it is",
                  command="tell me what day it is")

    assert p.is_target_ready(0, 0) is True
    error = p.get_error(0, 0)
    assert error is not None and "all 2 attempts failed" in error
    assert not error.startswith(IDENTICAL_PLAN_ERROR_PREFIX)
    plan = p.get_action_plan(0, 0)
    assert plan[0]["action"] == "announce"
    assert plan[0]["params"]["acknowledgement"] is True


# ---------------------------------------------------------------------------
# M2 (round-2 fix) — replace_target_plan tags installed announce+text steps
# acknowledgement=True (supervisor recovery is never an answer), and the flag
# must survive materialise_params / _canonical_plan / group_modifications_by_step.
# ---------------------------------------------------------------------------

def test_replace_target_plan_tags_installed_announce_with_text_as_acknowledgement(monkeypatch):
    import py_trees
    from behavior_tree.GPSR import planner as planner_mod

    p = planner_mod.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    target = {
        "id": "t0", "desc": "tell me what day it is", "object": "", "location": "",
        "depends_on": [], "preconditions": [], "postconditions": ["answered(what day is today)"],
    }
    p._slot_context[0] = {"command": "tell me what day it is", "targets": [target]}
    plan = [{"action": "announce", "params": {"text": "I could not find the answer"}}]

    p.replace_target_plan(0, 0, plan, reason="supervisor global replan")

    installed = p.get_action_plan(0, 0)
    assert installed[0]["params"]["acknowledgement"] is True
    # The caller's own plan list/dict must not be mutated in place.
    assert "acknowledgement" not in plan[0]["params"]


def test_replace_target_plan_leaves_textless_announce_untagged(monkeypatch):
    import py_trees
    from behavior_tree.GPSR import planner as planner_mod

    p = planner_mod.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    target = {
        "id": "t0", "desc": "tell me how many drinks", "object": "", "location": "",
        "depends_on": [], "preconditions": [], "postconditions": ["answered(how many drinks)"],
    }
    p._slot_context[0] = {"command": "tell me how many drinks", "targets": [target]}
    plan = [
        {"action": "count", "params": {"object": "drinks"}},
        {"action": "announce", "params": {}},
    ]

    p.replace_target_plan(0, 0, plan, reason="supervisor global replan")

    installed = p.get_action_plan(0, 0)
    assert "acknowledgement" not in installed[1]["params"]


def test_acknowledgement_flag_survives_materialise_params():
    # materialise_params only reads text/message for announce -- the extra
    # acknowledgement key must not raise or change ANNOUNCE_TEXT resolution.
    import py_trees
    from py_trees.common import Access
    from behavior_tree.GPSR.orchestrator import materialise_params
    from behavior_tree.GPSR.small_trees import bb_keys

    py_trees.blackboard.Blackboard.clear()
    bb = py_trees.blackboard.Client(name="t")
    bb.register_key(bb_keys.ANNOUNCE_TEXT, access=Access.WRITE)
    bb.register_key(bb_keys.ANNOUNCE_TEXT, access=Access.READ)
    bb.register_key(bb_keys.REPORT_INFO, access=Access.READ)

    materialise_params(bb, "announce", {"text": "sorry about that", "acknowledgement": True})
    assert bb.get(bb_keys.ANNOUNCE_TEXT) == "sorry about that"
    py_trees.blackboard.Blackboard.clear()


def test_acknowledgement_flag_included_in_canonical_plan_identity():
    # _canonical_plan (planner.py) is only used for identical-plan detection
    # -- the flag being part of identity is fine (and expected): two plans
    # differing ONLY by the flag must not be considered identical.
    from behavior_tree.GPSR.planner import _canonical_plan

    a = [{"action": "announce", "params": {"text": "hi"}}]
    b = [{"action": "announce", "params": {"text": "hi", "acknowledgement": True}}]
    assert _canonical_plan(a) != _canonical_plan(b)


def test_acknowledgement_flag_does_not_break_modification_grouping():
    # group_modifications_by_step matches by step_index (or unique action
    # name) only -- it never inspects params -- so the flag must not stop a
    # modification from being assigned to its step.
    from behavior_tree.GPSR.modifiable_nodes import group_modifications_by_step

    plan = [{"action": "announce", "params": {"text": "sorry", "acknowledgement": True}}]
    mods = [{"step_index": 0, "field": "text", "value": "sorry again", "reason": "clarify"}]
    grouped = group_modifications_by_step(plan, mods)
    assert 0 in grouped
    assert grouped[0][0]["value"] == "sorry again"
