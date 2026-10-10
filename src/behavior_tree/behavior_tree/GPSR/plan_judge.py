"""Offline LLM-judge for GPSR plans — SEMANTIC correctness check.

``planner_validators.validate_plan`` checks a plan's STRUCTURAL validity (known
actions/locations, ordering rules, no placeholders). It cannot tell whether the
plan actually *means* what the command asked — a plan can pass every rule yet
grasp the wrong object, drive to a valid-but-wrong location, pick the wrong
action (describe_person for a name), or drop a clause. This module closes that
gap: a SECOND model scores each generated plan against its command.

OFFLINE / TEST ONLY. This is NOT wired into the robot runtime — it adds a second
model's latency and nondeterminism, which we never want on the floor. Import
``judge_plan`` / ``judge_batch`` from a test or dev script.

Best practice: judge with a DIFFERENT model than the planner (a model judging its
own output shares its blind spots). The planner default is DeepSeek; the judge
default is ``openai/gpt-4.1``. Override with ``GPSR_JUDGE_MODEL``.
"""

from __future__ import annotations

import json
import re
from typing import Any, Dict, Iterable, List, Optional, Tuple

import openai

from .config import OPENAI_API_KEY, _resolve_model

# Judge model — a different, strong model from the planner. Override via env.
JUDGE_MODEL = _resolve_model(("GPSR_JUDGE_MODEL",), "openai/gpt-4.1")

_JUDGE_SYSTEM = """
You are a STRICT evaluator for a household service robot's task planner
(RoboCup@Home GPSR). Given a natural-language COMMAND and the PLAN the planner
produced (an ordered list of atomic actions with parameters), decide whether the
plan CORRECTLY and COMPLETELY accomplishes the command.

You are given the robot's ACTION CATALOGUE (what each action does) and the known
LOCATIONS / OBJECTS for context. Judge only against the command's intent — not
your own preferred phrasing.

Judge on these seven criteria:
1. COVERAGE — every requirement/clause in the command maps to a step; nothing is
   silently dropped. A command that reports something to the operator
   ("tell me", "let me know", "report to me") MUST end by bringing that result
   back to the operator's spot (``start_position``) and announcing it. A command
   that hands an object to the operator ("bring/give me X") must deliver with
   ``recipient_location=start_position``.
2. OBJECTS — object parameters match what was asked. A CATEGORY word ("a drink",
   "a fruit", "a snack") must stay a category, not be replaced by one instance.
3. LOCATIONS — location parameters match the places named in the command.
4. ACTION CHOICE — the right action per sub-task: ``ask_person`` for facts only
   obtainable by ASKING (name, age, favourite drink, where they are from);
   ``describe_person`` for VISIBLE traits (clothing, pose, what they hold);
   ``count`` for "how many"; ``vlm_fallback`` for "look at X and tell me";
   ``grasp`` for picking up; ``guide`` for leading a person somewhere; ``follow``
   for following a person; ``deliver`` to hand a held object to someone.
5. ORDER & DEPENDENCIES — prerequisites first: ``find_person`` (then
   ``approach_person``) before ``describe_person`` / ``ask_person`` / ``guide`` /
   handover; ``goto`` a place before grasping/searching there.
6. PARAMETERS — recipient, recipient_location, question, label, etc. are correct
   and internally consistent with the command.
7. NO HARMFUL EXTRAS — no redundant or contradictory steps that would break the
   task.

Do NOT penalise (these are CORRECT, not defects):
- A location not in the known list handled by ``record_position(label=...)`` then
  ``goto(label)`` — that is the intended pattern for runtime-named places.
- Closed appliances / no-grasp furniture (refrigerator, fridge, washing machine,
  dishwasher, cabinet, shelf, coat rack) handled by an ``open`` step and/or a
  ``grasp`` with ``ask_referee``/``from_*`` — the robot legitimately cannot open
  or reach into these, so asking a referee is correct.
- Minor wording of ``question`` / ``announce`` text, or harmless extra announces.

Respond with JSON ONLY, no prose:
{
  "verdict": "PASS" | "FAIL",
  "confidence": <float 0.0-1.0>,
  "issues": [
    {"severity": "critical" | "major" | "minor",
     "criterion": "coverage|objects|locations|action_choice|order|parameters|extras",
     "detail": "<what is wrong and why>"}
  ],
  "summary": "<one sentence>"
}
Rule: verdict = FAIL if there is ANY critical or major issue; minor issues alone
stay PASS. An empty issues list means a clean PASS.
""".strip()


def _extract_json(text: str) -> Optional[Dict[str, Any]]:
    """Best-effort parse of a JSON object from a model reply."""
    if not text:
        return None
    try:
        obj = json.loads(text)
        return obj if isinstance(obj, dict) else None
    except Exception:
        pass
    m = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", text, re.DOTALL)
    if m:
        try:
            return json.loads(m.group(1))
        except Exception:
            pass
    m = re.search(r"\{.*\}", text, re.DOTALL)
    if m:
        try:
            return json.loads(m.group(0))
        except Exception:
            pass
    return None


def _client() -> "openai.OpenAI":
    return openai.OpenAI(api_key=OPENAI_API_KEY, base_url="https://openrouter.ai/api/v1")


def _build_user_prompt(
    command: str,
    plan: List[Dict[str, Any]],
    action_catalogue: str,
    known_locations: Optional[Iterable[str]],
    known_objects: Optional[Iterable[str]],
) -> str:
    parts = []
    if action_catalogue:
        parts.append("ACTION CATALOGUE (what each action does):\n" + action_catalogue)
    if known_locations:
        parts.append("KNOWN LOCATIONS: " + ", ".join(sorted(str(l) for l in known_locations)))
    if known_objects:
        parts.append("KNOWN OBJECTS (arena hint; objects are open-vocab): "
                     + ", ".join(sorted(str(o) for o in known_objects)))
    parts.append("COMMAND:\n" + command)
    parts.append("PLAN (ordered):\n" + json.dumps(plan, ensure_ascii=False, indent=2))
    parts.append("Evaluate the plan against the command. Return the JSON verdict now.")
    return "\n\n".join(parts)


def judge_plan(
    command: str,
    plan: List[Dict[str, Any]],
    *,
    action_catalogue: str = "",
    known_locations: Optional[Iterable[str]] = None,
    known_objects: Optional[Iterable[str]] = None,
    client: Optional["openai.OpenAI"] = None,
    model: str = JUDGE_MODEL,
    temperature: float = 0.0,
    max_tokens: int = 1200,
) -> Dict[str, Any]:
    """Score one ``plan`` against its ``command`` with the judge model.

    Returns a dict: ``{verdict, confidence, issues, summary, model}``. On any LLM
    or parse error, returns ``{verdict: "ERROR", ...}`` (never raises) so a batch
    run continues.
    """
    client = client or _client()
    user_prompt = _build_user_prompt(
        command, plan, action_catalogue, known_locations, known_objects
    )
    try:
        kwargs = dict(
            model=model,
            messages=[
                {"role": "system", "content": _JUDGE_SYSTEM},
                {"role": "user", "content": user_prompt},
            ],
            temperature=temperature,
            max_tokens=max_tokens,
            response_format={"type": "json_object"},
        )
        try:
            resp = client.chat.completions.create(**kwargs)
        except Exception as exc:  # model that rejects response_format?
            if "response_format" not in repr(exc).lower():
                raise
            kwargs.pop("response_format", None)
            resp = client.chat.completions.create(**kwargs)
        msg = resp.choices[0].message
        raw = (getattr(msg, "content", None) or "").strip()
        if not raw:
            raw = (getattr(msg, "reasoning", None) or "").strip()
        parsed = _extract_json(raw)
        if parsed is None:
            return {"verdict": "ERROR", "confidence": 0.0, "issues": [],
                    "summary": f"unparseable judge reply: {raw[:200]!r}", "model": model}
        parsed.setdefault("verdict", "ERROR")
        parsed.setdefault("issues", [])
        parsed.setdefault("confidence", 0.0)
        parsed.setdefault("summary", "")
        parsed["model"] = model
        # Normalise verdict casing.
        parsed["verdict"] = str(parsed["verdict"]).upper()
        return parsed
    except Exception as exc:  # noqa: BLE001 — never break the batch
        return {"verdict": "ERROR", "confidence": 0.0, "issues": [],
                "summary": f"judge call failed: {exc!r}", "model": model}


def judge_batch(
    items: List[Tuple[str, List[Dict[str, Any]]]],
    *,
    action_catalogue: str = "",
    known_locations: Optional[Iterable[str]] = None,
    known_objects: Optional[Iterable[str]] = None,
    model: str = JUDGE_MODEL,
) -> List[Dict[str, Any]]:
    """Judge a list of ``(command, plan)`` pairs. Returns one verdict dict each."""
    client = _client()
    out = []
    for command, plan in items:
        out.append(judge_plan(
            command, plan, action_catalogue=action_catalogue,
            known_locations=known_locations, known_objects=known_objects,
            client=client, model=model,
        ))
    return out
