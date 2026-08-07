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
from .orchestrator import (
    SYSTEM_PROMPT,
    KNOWN_LOCATIONS,
    START_LOCATION_ALIASES,
    BtNode_LogStepResult,
    BtNode_MaterialiseStep,
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
    once. Split it into an ordered list of SELF-CONTAINED targets — one per
    clause that the robot can work on independently — resolving cross-clause
    references so each target reads like a standalone instruction.

    Respond with JSON only, in this exact shape:
    {
      "reasoning": "<short explanation of the split>",
      "targets": ["<target 1>", "<target 2>", ...]
    }

    Split rules:
    1. Keep each target self-contained: resolve pronouns and cross-references
       into concrete nouns. Example: "fetch Susan a coke, she is in the living
       room" -> ["grab a coke", "find Susan in the living room", "deliver the
       coke to Susan"]. The object "coke" is named in every target it touches.
    2. Preserve the order in which the command gives the clauses — do not
       reorder the work.
    3. Keep each target as close to the original wording as possible; do not
       invent actions, locations, or details not present in the command.
    4. Never return an empty list. If the command has one clause, return a
       single-element list containing the command itself.
    5. If a clause cannot be resolved into concrete work, keep it as a target
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
    single self-contained goal; plan only the steps needed to achieve it.
""").strip() + "\n\n" + SYSTEM_PROMPT


# ---------------------------------------------------------------------------
# Offline mock (full-mock preset): deterministic, network-free planning.
# ---------------------------------------------------------------------------

def _offline_mock_targets(command: str) -> List[str]:
    """Deterministic top-layer split for the all-mock preset (network-free).

    Batch debug commands use ``|`` as a command separator; reuse that convention
    here so an offline integration test can drive several targets at once. A
    command with no ``|`` is a single target (the whole command).
    """
    return [p.strip() for p in str(command or "").split("|") if p.strip()]


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

    def reset(self) -> None:
        """Drop every cached plan/subtree (new command, test teardown)."""
        with self._lock:
            self._cache.clear()

    # -- top layer ----------------------------------------------------------

    def split_command(self, command: str) -> List[str]:
        """TOP LAYER. Blocking split of a command into self-contained targets.

        Run inside a worker thread owned by ``BtNode_SplitCommand`` (offline
        mock returns instantly, no thread needed). On total LLM failure falls
        back to the deterministic split so the pipeline always proceeds.
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
            targets = [
                t for t in (parsed.get("targets") or [])
                if isinstance(t, str) and t.strip()
            ]
            if not targets:
                last_reason = (
                    "you returned an EMPTY targets list. You MUST return a "
                    "NON-EMPTY list of self-contained targets."
                )
                print(f"[split] attempt {attempt+1}/{self._max_attempts} REJECTED: "
                      f"{last_reason}")
                continue
            print(f"[split] accepted on attempt {attempt+1}: {targets}")
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
        failure_reason: Optional[str] = None,
    ) -> None:
        """LOWER LAYER. Blocking: plan ONE target and pre-build its subtree.

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
            user_prompt = _build_planner_user_prompt(
                desc, state_log, last_reason, nonce=nonce,
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
            ok, reason = validate_plan(
                cleaned, desc or "", known_actions,
                known_locations=known_loc_arg,
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

    def request_plan_all(self, slot: int, targets: List[str]) -> None:
        """LOWER LAYER, PARALLEL. Spawn one daemon thread per target.

        Pre-seeds the cache (ready=False) so a stale plan/subtree from a prior
        use of this slot can never be swapped in while the fresh plans are being
        produced. Returns immediately; poll ``all_targets_ready``.
        """
        with self._lock:
            for i, desc in enumerate(targets):
                self._cache[(slot, i)] = {
                    "desc": desc,
                    "plan": [],
                    "subtree": None,
                    "ready": False,
                    "error": None,
                }
        for i, desc in enumerate(targets):
            threading.Thread(
                target=self.plan_target, args=(slot, i, desc), daemon=True,
            ).start()

    def replan_target(self, slot: int, index: int, reason: str = "") -> None:
        """Re-plan ONE target on a fresh daemon thread (lower-layer scope only).

        Invalidates the cached entry first so the OLD subtree is never re-swapped
        in while the fresh plan is in flight. Top layer and the other targets
        are untouched — a failing target re-plans only itself.
        """
        desc = self._get_desc(slot, index)
        if desc is None:
            return
        self._invalidate(slot, index)
        threading.Thread(
            target=self.plan_target, args=(slot, index, desc),
            kwargs={"failure_reason": reason}, daemon=True,
        ).start()

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
                    ACTION_FACTORIES[act](),     # fresh node instance
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
            step_seq.add_child(factory())
            step_seq.add_child(BtNode_LogStepResult(
                f"log:{slot}:{index}:{k}", succeeded=True,
            ))
            seq.add_child(step_seq)
        return seq
