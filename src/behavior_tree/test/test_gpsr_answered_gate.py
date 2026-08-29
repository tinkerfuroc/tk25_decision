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
