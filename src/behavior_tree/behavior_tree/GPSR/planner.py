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

import json
import random
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
from ..config import is_full_mock_mode
from .planner_validators import validate_plan
from .small_trees import ACTION_FACTORIES, bb_keys, BtNode_AnnounceFromBB
from .supervision.runtime import get_default_supervisor, wrap_action_factory
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
    _build_planner_user_prompt,
    _clean_plan,
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
          "desc": "<standalone NL instruction>",
          "object": "<concrete object noun if this clause names one, else \"\">",
          "location": "<assigned location if this clause names one, else \"\">",
          "depends_on": <index of the target that must finish first, or -1>
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
       set object=Fanta, location=office, and depends_on to the index of the
       earlier target that establishes it. This is how the assigned location
       survives from the command down to the lower layer.
    4. Set "depends_on" to the index of the target that must complete BEFORE
       this one because it establishes something this target needs (the
       object's assigned location, the person being found, the object being
       picked up). Use -1 when the target is independent.
    5. Keep each target as close to the original wording as possible; do not
       invent actions, locations, or details not present in the command.
    6. Never return an empty list. If the command has one clause, return a
       single-element list containing the command itself.
    7. If a clause cannot be resolved into concrete work, keep it as a target
       anyway — the lower layer will plan an announcement for it.
""").strip()


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
    - If a prior target will have already delivered the result to the operator,
      still emit any remaining steps so the plan is complete — but skip the
      repeated fetch/grasp.
    - A room named in the command (e.g. "the living room", "the kitchen") is
      legitimate even if it has no hardcoded waypoint in the known-location
      list: plan ``record_position(label=<room>)`` then ``goto(location=<room>)``
      so the pose is captured at runtime. NEVER refuse a step with
      "cannot find a known location" when the command names a real room or
      place — record it instead.
""").strip() + "\n\n" + SYSTEM_PROMPT


# ---------------------------------------------------------------------------
# Offline mock (full-mock preset): deterministic, network-free planning.
# ---------------------------------------------------------------------------

def _normalise_targets(raw: List[Any]) -> List[Dict[str, Any]]:
    """Coerce the LLM top-layer output into the canonical structured-target shape.

    Accepts either dicts (desc/object/location/depends_on — the expected form)
    or plain strings (kept for backwards tolerance). Guarantees a ``desc`` and
    fills ``object``/``location``/``depends_on`` with safe defaults.
    """
    targets: List[Dict[str, Any]] = []
    for i, t in enumerate(raw):
        if isinstance(t, dict):
            desc = str(t.get("desc") or "").strip()
            if not desc:
                continue
            targets.append({
                "desc": desc,
                "object": str(t.get("object") or "").strip(),
                "location": str(t.get("location") or "").strip(),
                "depends_on": _norm_depends_on(t.get("depends_on"), i),
            })
        elif isinstance(t, str) and t.strip():
            targets.append({
                "desc": t.strip(),
                "object": "",
                "location": "",
                "depends_on": i - 1 if i > 0 else -1,
            })
    return targets


def _norm_depends_on(value: Any, index: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return index - 1 if index > 0 else -1


def _flatten_prior_plans(planner, slot: int, index: int) -> List[Dict[str, Any]]:
    """Concatenate the already-ACCEPTED action-plans of targets < ``index``.

    Seeded into ``validate_plan(prior_plan=...)`` so a later target is not
    re-rejected for legitimate handoff (e.g. a ``guide()`` after an earlier
    target already ran ``find_person``). Only targets whose plans have landed in
    the cache count — a parallel worker that finished before its predecessor
    simply seeds nothing, which is the safe (conservative) choice.
    """
    flat: List[Dict[str, Any]] = []
    for i in range(max(0, index)):
        entry = planner.get_action_plan(slot, i)
        if entry:
            flat.extend(entry)
    return flat


def _build_lower_layer_user_prompt(
    command: str,
    desc: str,
    target_obj: str,
    target_loc: str,
    prior_targets: List[Dict[str, Any]],
    state_log: List[str],
    failure_msg: Optional[str] = None,
    nonce: Optional[str] = None,
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
    body += "\nReturn the JSON plan now."
    return body


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
            "desc": part,
            "object": "",
            "location": "",
            "depends_on": i - 1 if i > 0 else -1,
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
        self._offline_mock = is_full_mock_mode()
        # (slot, index) -> {"desc": str, "plan": list[dict], "subtree": Behaviour|None,
        #                   "ready": bool, "error": str|None}
        self._cache: Dict[Tuple[int, int], Dict[str, Any]] = {}
        # slot -> {"command": str, "targets": list[dict]} — full-command context
        # for the lower layer, set once per command by request_plan_all.
        self._slot_context: Dict[int, Dict[str, Any]] = {}
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

    def _store(self, slot, index, desc, plan, subtree, error):
        with self._lock:
            self._cache[(slot, index)] = {
                "desc": desc,
                "plan": list(plan),
                "subtree": subtree,
                "ready": True,
                "error": error,
            }

    def _get_desc(self, slot, index) -> Optional[str]:
        with self._lock:
            entry = self._cache.get((slot, index))
        return entry.get("desc") if entry else None

    def _invalidate(self, slot, index) -> None:
        """Mark (slot, index) not-ready so an in-flight replan's OLD subtree is
        never re-swapped in while the fresh plan is still being produced."""
        with self._lock:
            entry = self._cache.get((slot, index))
            if entry:
                entry["ready"] = False

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

    def reset(self) -> None:
        """Drop every cached plan/subtree (new command, test teardown)."""
        with self._lock:
            self._cache.clear()
            self._slot_context.clear()

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
            targets = _normalise_targets(parsed.get("targets") or [])
            if not targets:
                last_reason = (
                    "you returned an EMPTY targets list. You MUST return a "
                    "NON-EMPTY list of structured targets."
                )
                print(f"[split] attempt {attempt+1}/{self._max_attempts} REJECTED: "
                      f"{last_reason}")
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
    ) -> None:
        """LOWER LAYER. Blocking: plan ONE target and pre-build its subtree.

        ``command`` / ``target_obj`` / ``target_loc`` / ``prior_targets`` are the
        full-command context the TOP layer produced (see request_plan_all): the
        original NL command, the object / assigned location of THIS target, and
        the earlier targets of the same command. The lower layer plans against
        that context — the assigned location wins over any predefined default —
        and validates against prior targets so it does not repeat their work.

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
                last_reason = reason
                print(f"[plan:{slot}:{index}] attempt {attempt+1}/{self._max_attempts} "
                      f"REJECTED: {reason}")
                continue
            # Accepted — build the subtree on this (worker) thread, then cache.
            try:
                subtree = self.build_target_subtree(slot, index, cleaned)
            except Exception as exc:  # noqa: BLE001 — surface for retry
                last_reason = f"subtree build failed: {exc!r}"
                print(f"[plan:{slot}:{index}] attempt {attempt+1}/{self._max_attempts} "
                      f"-> {last_reason}")
                continue
            print(f"[plan:{slot}:{index}] accepted on attempt {attempt+1}: "
                  f"{[s['action'] for s in cleaned]}")
            self._store(slot, index, desc, cleaned, subtree, None)
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
        norm_targets = [
            t if isinstance(t, dict) else {"desc": str(t), "object": "",
                                           "location": "", "depends_on": -1}
            for t in targets
        ]
        with self._lock:
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
                }
        for i, t in enumerate(norm_targets):
            threading.Thread(
                target=self.plan_target,
                args=(
                    slot, i, str(t.get("desc") or ""),
                    command or "", str(t.get("object") or ""),
                    str(t.get("location") or ""),
                    [dict(x) for x in norm_targets[:i]],
                ),
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
                [dict(x) for x in targets[:index]],
            ),
            kwargs={"failure_reason": reason}, daemon=True,
        ).start()

    def replace_target_plan(
        self,
        slot: int,
        index: int,
        plan: List[Dict[str, Any]],
        reason: str = "supervisor global replan",
    ) -> None:
        """Install a supervisor-validated remaining plan for one target.

        This is synchronous and network-free: the supervisor has already
        validated the typed plan.  Building and caching a fresh subtree here
        lets ``DynamicExecutor`` swap it at the next safe tick boundary.
        """
        desc = self._get_desc(slot, index) or f"target {index}"
        cleaned, _ = _clean_plan(plan)
        subtree = self.build_target_subtree(slot, index, cleaned)
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
    ) -> py_trees.composites.Sequence:
        """Compose the per-target executing subtree for ``action_plan``.

        Returns a FRESH Sequence (built once, swapped in once). Reads its step
        plan from the BB slot ``SAVED_TARGET_PLAN_PREFIX+<slot>_<i>`` (written by
        ``BtNode_PlanAllTargets`` on the executor thread) so every MaterialiseStep
        only ever reads the Blackboard, and announces the target (CURRENT_TARGET)
        on entry.

        Structure:
            Sequence("target:<slot>:<i>")
            ├── BtNode_AnnounceFromBB(CURRENT_TARGET, prefix="Next: ")
            └── per step k: Sequence[
                    BtNode_MaterialiseStep(plan_key, k),
                    supervised ACTION_FACTORIES[act](),
                    BtNode_LogStepResult(ok=True),
                ]
        """
        seq = py_trees.composites.Sequence(f"target:{slot}:{index}", memory=True)
        seq.add_child(BtNode_AnnounceFromBB(
            f"announce target:{slot}:{index}",
            bb_keys.CURRENT_TARGET,
            prefix="Next: ",
        ))
        plan_key = bb_keys.SAVED_TARGET_PLAN_PREFIX + f"{slot}_{index}"
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
            step_seq.add_child(wrap_action_factory(
                action, factory, get_default_supervisor(),
            ))
            step_seq.add_child(BtNode_LogStepResult(
                f"log:{slot}:{index}:{k}", succeeded=True,
            ))
            step_seq.add_child(BtNode_SupervisorBarrier(
                f"supervisor barrier:{slot}:{index}:{k}",
            ))
            seq.add_child(step_seq)
        return seq
