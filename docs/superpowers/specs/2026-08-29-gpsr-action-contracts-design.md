# GPSR orchestrator: action contracts, deferred preconditions, replan hygiene

Date: 2026-08-29
Base: branch `gpsr-sim-battery` @ `8320ce1` (all file:line refs below are against that commit)
Status: draft, awaiting user review

## Problem

The t2-2026 sim battery (Aug 27–28) ranks the orchestrator-owned failures as:

1. After a grasp failure the replanner emits `place(kitchen_table)` with no preceding `goto`; the target's `at_robot(kitchen_table)` precondition verifies INVALID; the target replans 3–4 times, burns its budget, is SKIPPED, and every dependent target is BLOCKED. Runs end as executor FAILURE after 300–430 s.
2. Replans can produce duplicated steps (`s2026-003` materialized `place(kitchen_table), place(kitchen_table)`).
3. `report.json` records `"plan": []` for every two-layer run because the two-layer flow never emits `step.finished` telemetry (`bench/events.py` only appends steps on that event).

Failure 1 is not LLM flakiness. Four layers hold contradictory copies of "who is responsible for the robot being at a location":

| Layer | Says | Where |
|---|---|---|
| Split prompt | `place` carries precondition `at_robot(dest)` | `planner.py:139-147` (rule 7) |
| Plan validator | forbids `goto` immediately before `place`/`deliver` — "they self-navigate" | `planner_validators.py:426-448` |
| Runtime verifier | `at_robot(X)` VALID only if `last_nav_location == X` or the fact is already established | `validators.py:323-328`; action fallback accepts only `goto` at `:309` |
| Materializer | writes `LAST_NAV_LOCATION` only for `goto` and `search_object` | `orchestrator.py:1051-1052`, `:1068` |

`create_place` (`small_trees.py:1475-1489`) and `create_deliver` really do navigate internally, so the validator rule is correct about the tree — but the materializer never records that navigation, and preconditions are checked at the **target boundary** (`BtNode_TargetPreconditionCheck`, `orchestrator.py:1532`) before the target's first step runs. A place-target with precondition `at_robot(dest)` therefore fails at entry, before `place` can navigate. Every replan hits the same wall; the per-target budget (`orchestrator.py:2115-2133`) only delays the SKIP.

## Goals

- Make the at_robot contradiction impossible to reintroduce: one declaration per action, every consumer derives from it.
- A precondition that the target's own plan establishes must not fail the target at entry, but must still be verified.
- Replans stop spending budget on plans identical to the one that just failed, and never carry duplicated steps.
- The bench records the materialized plan for two-layer runs.

Non-goals (tracked separately): sim-time vs wall-time budgeting; splitting `orchestrator.py` (3164 lines) beyond the seams this change touches; sim head-camera aim; sim person-identity modelling.

## Design

### 1. `ActionContract` registry

New module `src/behavior_tree/behavior_tree/GPSR/action_contracts.py`.

```python
@dataclass(frozen=True)
class ActionContract:
    action: str
    requires: tuple[str, ...] = ()            # fact templates needed at entry, NOT self-satisfied
    establishes: tuple[str, ...] = ()         # fact templates asserted on success
    self_establishes: Mapping[str, str] = {}  # predicate -> param name the action satisfies by itself
    records: tuple[str, ...] = ()             # evidence names (keys of _TARGET_GATE_EVIDENCE_KEYS) it writes
    self_navigating: bool = False

ACTION_CONTRACTS: Mapping[str, ActionContract]
def contract_for(action: str) -> ActionContract   # KeyError for unknown action
```

Initial entries (all others declare exactly what they do today; no behaviour change):

| action | requires | establishes | self_establishes | records | self_navigating |
|---|---|---|---|---|---|
| goto | — | at_robot(location) | at_robot→location | last_nav_location | yes |
| search_object | — | object_seen(object) | at_robot→location | last_nav_location, object_detection | yes |
| place | held(object) | placed(object,location) | at_robot→location | last_nav_location | yes |
| deliver | held(object) | delivered(object,recipient) | at_robot→recipient_location | last_nav_location | yes |
| grasp | object_seen(object) | held(object) | — | — | no |
| find_object | — | object_seen(object) | — | object_detection, target_object_name | no |
| find_person | — | person_found(person) | — | person_detection, target_person_pose | no |
| count | — | counted(object) | — | count_value, count_target | no |
| ask_person / answer_question | — | answered(question) | — | person_answer / qa_answer | no |
| others (approach_person, describe_person, follow, guide, open, announce, record_position, vlm_fallback, llm_fallback) | — | — | — | as today | no |

Fact templates use the closed vocabulary already in the split prompt (`planner.py:127-129`); parameter names in parentheses are resolved against the step's `params` at check time.

This registry is deliberately **not** merged into `supervision/contracts.py`. That registry is keyed by BT node class and carries supervisor safety semantics (`EffectRisk`, evidence modalities). Merging would make the network-free plan validator import the supervision package and would force class↔action translation everywhere.

### 2. Consumers derive from the registry

| Consumer | Today | After |
|---|---|---|
| `materialise_params` (`orchestrator.py:1049-1052`, `:1066-1068`) | `LAST_NAV_LOCATION` set iff `action == "goto"` (and in the `search_object` branch) | set for any action with `"last_nav_location" in contract.records`, from `params[contract.self_establishes["at_robot"]]` |
| `planner_validators._SELF_NAV_DEST` (`:433`) | hardcoded `{"deliver": "recipient_location", "place": "location"}` | `{a: c.self_establishes["at_robot"] for a, c in ACTION_CONTRACTS.items() if c.self_navigating and a != "goto"}` — rule text unchanged |
| `validators._action_verdict` (`:309`) | `at_robot` accepted only when `action == "goto"` | accepted when `"at_robot" in contract_for(action).self_establishes` and the named param matches |
| Split prompt rule 7 (`planner.py:139-147`) | teaches `place` with precondition `at_robot(balcony)` | rewritten: a target must not list as a precondition a fact its own action establishes; the self-satisfied list is rendered from the registry (`goto/place/deliver/search_object → at_robot`) so prose cannot drift from code |
| `_deterministic_target_intent` (`planner.py:397-421`) | hand-written predicate→action map | derived: first action whose `establishes` predicate matches |

### 3. Deferred preconditions

`BtNode_TargetPreconditionCheck` gains an `action_plan` argument (the target's cleaned plan; the builder at `planner.py` already has it when it constructs the gate).

At `update()`:
1. Partition preconditions into `deferred` and `checked`: a precondition is deferred when its predicate is in `contract_for(step.action).self_establishes` for some step in `action_plan`.
2. Check only `checked` at entry, as today.
3. Write `deferred` to a new BB key `bb_keys.DEFERRED_PRECONDITIONS` (`"gpsr/deferred_preconditions"`, list[str], target-scoped) and log `precondition deferred: at_robot(kitchen_table) (satisfied by place)`.

`BtNode_TargetPostconditionCheck` reads `DEFERRED_PRECONDITIONS` in `update()` and verifies them together with the postconditions, using the same evidence path. Because `materialise_params` now records `last_nav_location` for `place`/`deliver`, and `_action_verdict` accepts them, a deferred `at_robot(dest)` verifies VALID after a successful `place` and INVALID after a navigation failure inside it. The fact is committed to the fact store exactly as a postcondition would be.

`_swap_in` (`orchestrator.py:2016-2019`) clears `DEFERRED_PRECONDITIONS` when advancing to a new target (it is in the target-scoped evidence group, not the nav-persistent one).

### 4. Replan hygiene

All in `DynamicExecutor._on_target_failure` (`orchestrator.py:2075-2143`) and `GPSRPlanner` (`planner.py`).

- **Real reason.** The failing gate/step's `feedback_message` (e.g. `precondition unmet: at_robot(kitchen_table) (invalid)`) is captured by `DynamicExecutor` from the child's tip and passed as `reason` to `replan_target`, replacing the generic child-FAILURE string. `plan_target` already prefixes `previous attempt failed: {reason}` into the prompt (`planner.py:824-825`).
- **Dedupe.** `_clean_plan` drops a step whose `(action, params)` equals the immediately preceding step's. Logged as `dropped duplicate step i`.
- **Short-circuit.** `replan_target` stores the failed plan's canonical form on the cache entry. When the fresh plan's canonical form equals it, the planner does not mark the entry ready; it retries the LLM (within `max_attempts`) with the reason amplified: `previous attempt failed: {reason}; the regenerated plan was identical — change it`. Only if all attempts are identical does it hand the plan back, so the executor's budget is spent on a genuinely new plan or on the honest fallback. Budget semantics in `_on_target_failure` are unchanged.

### 5. Telemetry

The two-layer flow (`_create_execute_slot_new` / `DynamicExecutor`) emits through the existing `GpsrTelemetry`:
- `plan.materialized` with `{slot, target_index, steps}` from `_swap_in`, once per swap (initial and replan).
- `step.finished` with `{slot, target_index, step_index, action, params, status, feedback}` from `BtNode_MaterialiseStep`'s terminal status, matching the event shape the legacy flow already emits so `bench/events.py` needs no change.

`report.json.results[].plan` is then populated for two-layer runs, PASS or FAIL. This is also how the fix is proven end-to-end.

## Error handling

- Unknown action at contract lookup raises `KeyError` inside plan validation (`validate_plan`), never at runtime, consistent with `NodeContractRegistry.require`.
- A `self_establishes` param missing from a step's `params` is a plan-validation error (`step i: place declares at_robot via 'location' but has no location`).
- Deferral never hides a failure: a deferred fact that does not verify at the post-check fails the target with the same `postcondition unmet` message and enters the normal replan path — now with the precise reason.

## Testing

TDD, network-free, under the existing `run-dec-tests.sh` suite:

1. `test_action_contracts.py`: `set(ACTION_CONTRACTS) == set(ACTION_FACTORIES)`; every `records` name is a key in `_TARGET_GATE_EVIDENCE_KEYS`; every `self_establishes` param appears in the action's known params.
2. `materialise_params` writes `LAST_NAV_LOCATION` for `place(location=X)` and `deliver(recipient_location=X)`; still does for `goto`/`search_object`; not for `grasp`.
3. `planner_validators`: the goto-before-self-nav rule rejects `goto, place` and `goto, deliver`, derived from the registry (monkeypatch a fake self-navigating action and assert the rule picks it up).
4. `validators._action_verdict`: `at_robot(X)` VALID after `place(location=X)`.
5. Precondition gate: `at_robot(X)` on a target whose plan contains `place(location=X)` is deferred (SUCCESS at entry, key populated); the post-check picks it up and passes with nav evidence, fails without.
6. `_clean_plan` dedupes consecutive identical steps and keeps non-consecutive repeats.
7. Short-circuit: with a stub LLM returning the same plan twice, the entry is not marked ready until a different plan (or attempts exhausted); the executor budget is not decremented in between.
8. Telemetry: a scripted two-layer run emits `plan.materialized` and one `step.finished` per step; `bench.events.parse_events(...).task.steps` is non-empty.

Integration (batched to the `gpsr command testing robustness` session, sim stack on GPU1): re-run `bringMeObjFromPlcmt` and `findObjInRoom` (the two storm reproducers) before/after, comparing milestone failure counts, replan count per target, wall time, and the now-populated `plan` field. Success: no `at_robot` INVALID at target entry; at most one replan per grasp failure; `plan` non-empty in `report.json`.

## Rollout

Implement on a branch from `gpsr-sim-battery`, one PR, commits in the order of §1→§5 so each is reviewable alone. §1–§3 are the fix; §4–§5 are independent and can land separately if review stalls.
