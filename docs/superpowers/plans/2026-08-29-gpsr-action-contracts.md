# GPSR Action Contracts Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate the deterministic `at_robot` replan storm by making one `ActionContract` registry the single source of truth for what each planner action requires/establishes/records, deferring self-satisfied target preconditions, deduplicating and short-circuiting replans, and emitting two-layer plan telemetry so the bench records it.

**Architecture:** A new frozen-dataclass registry `action_contracts.py` sits beside `small_trees.ACTION_FACTORIES`. Four existing hardcoded tables (`materialise_params`, `planner_validators._SELF_NAV_DEST`, `validators._action_verdict`, the split prompt's rule 7) become lookups into it. `BtNode_TargetPreconditionCheck` learns the target's plan and defers any precondition the plan self-establishes to `BtNode_TargetPostconditionCheck` via a new blackboard key. `GPSRPlanner.plan_target` rejects a regenerated plan identical to the one that just failed; `_clean_plan` drops consecutive duplicate steps. `DynamicExecutor._swap_in` emits `plan.materialized` and sets `TASK_ID`; failed steps emit `step.finished`.

**Tech Stack:** Python 3.10, py_trees / py_trees_ros (ROS 2 Humble), pytest (bare, `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`).

**Spec:** `docs/superpowers/specs/2026-08-29-gpsr-action-contracts-design.md`

## Global Constraints

- Base branch: `gpsr-sim-battery` @ `8320ce1`. Work on a branch from it (this worktree branch `worktree-gpsr-action-contracts-spec` already is).
- All code under `src/behavior_tree/behavior_tree/GPSR/`; all tests under `src/behavior_tree/test/`.
- Tests are network-free. Never call the LLM in a test.
- Run tests with (from the repo root):
  ```bash
  source /opt/ros/humble/setup.bash && source /home/tinker/tk25_ws/install/setup.bash
  export PYTHONPATH="$PWD/src/behavior_tree:$PYTHONPATH:/home/tinker/tk25_ws/src/tk25_decision/.venv_decision/lib/python3.10/site-packages"
  cd src/behavior_tree && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/<file> -q
  ```
  (Below, "Run:" lines assume this environment is already set and cwd is `src/behavior_tree`.)
- Fact vocabulary is closed (`validators._VOCABULARY`): `at_robot/1 object_seen/1 person_found/1 held/1 placed/2 delivered/2 counted/1 answered/1`.
- Blackboard keys are class attributes on `small_trees.bb_keys`, string values `"gpsr/<snake_name>"`.
- Commit after every task with a `feat(gpsr):` / `test(gpsr):` / `fix(gpsr):` prefix. Do not push to `gpsr-sim-battery` or `main`.

## File map

| File | Role |
|---|---|
| Create `GPSR/action_contracts.py` | `ActionContract` dataclass, `ACTION_CONTRACTS`, `contract_for`, `self_established_facts`, `self_navigating_destinations`, `render_self_satisfied_rule` |
| Modify `GPSR/orchestrator.py` | `_clean_plan` dedupe (`:604`); `materialise_params` nav recording (`:1049-1052`); `_TARGET_GATE_EVIDENCE_KEYS` + `BtNode_TargetPreconditionCheck` deferral (`:1482`, `:1532`); `BtNode_TargetPostconditionCheck` picks up deferred (`:1573`); `DynamicExecutor._swap_in` telemetry + TASK_ID (`:2005`); `DynamicExecutor.tick` failed-step telemetry (`:2265`) |
| Modify `GPSR/planner.py` | split prompt rule 7 (`:139-147`); `_deterministic_target_intent` (`:397`); `build_target_subtree` passes plan to precondition gate (`:1103`); `_invalidate` stores failed plan; `plan_target` short-circuit (`:875-884`) |
| Modify `GPSR/planner_validators.py` | `_SELF_NAV_DEST` derived (`:433`) |
| Modify `GPSR/validators.py` | `_action_verdict` `at_robot` via registry (`:309`) |
| Modify `GPSR/small_trees.py` | `bb_keys.DEFERRED_PRECONDITIONS` (`:179`) |
| Create `test/test_gpsr_action_contracts.py`, `test/test_gpsr_deferred_preconditions.py`, `test/test_gpsr_replan_hygiene.py`, `test/test_gpsr_two_layer_telemetry.py` | tests per task |

---

### Task 1: `ActionContract` registry

**Files:**
- Create: `src/behavior_tree/behavior_tree/GPSR/action_contracts.py`
- Test: `src/behavior_tree/test/test_gpsr_action_contracts.py`

**Interfaces:**
- Consumes: `behavior_tree.GPSR.small_trees.ACTION_FACTORIES` (dict[str, callable]); `behavior_tree.GPSR.orchestrator._TARGET_GATE_EVIDENCE_KEYS` (tuple of `(evidence_name, bb_key)`).
- Produces:
  - `ActionContract(action, requires, establishes, self_establishes, records, self_navigating)` frozen dataclass.
  - `ACTION_CONTRACTS: dict[str, ActionContract]`
  - `contract_for(action: str) -> ActionContract` (raises `KeyError`).
  - `self_established_facts(step: Mapping) -> list[str]` — canonical fact strings a step satisfies by itself, e.g. `["at_robot(kitchen_table)"]`; empty when the param is absent.
  - `self_navigating_destinations() -> dict[str, str]` — `{action: param_name}` for self-navigating non-`goto` actions.
  - `render_self_satisfied_rule() -> str` — prose line listing self-satisfied predicates per action.

- [ ] **Step 1: Write the failing tests**

```python
# src/behavior_tree/test/test_gpsr_action_contracts.py
from __future__ import annotations

import pytest

from behavior_tree.GPSR import action_contracts as ac
from behavior_tree.GPSR.small_trees import ACTION_FACTORIES
from behavior_tree.GPSR.orchestrator import _TARGET_GATE_EVIDENCE_KEYS


def test_every_factory_has_a_contract_and_vice_versa():
    assert set(ac.ACTION_CONTRACTS) == set(ACTION_FACTORIES)


def test_records_are_known_evidence_names():
    known = {name for name, _ in _TARGET_GATE_EVIDENCE_KEYS}
    for contract in ac.ACTION_CONTRACTS.values():
        assert set(contract.records) <= known, contract.action


def test_contract_for_unknown_action_raises():
    with pytest.raises(KeyError):
        ac.contract_for("teleport")


def test_place_and_deliver_self_establish_at_robot():
    assert ac.contract_for("place").self_establishes == {"at_robot": "location"}
    assert ac.contract_for("deliver").self_establishes == {"at_robot": "recipient_location"}
    assert ac.contract_for("goto").self_establishes == {"at_robot": "location"}
    assert ac.contract_for("grasp").self_establishes == {}


def test_self_established_facts_from_step():
    assert ac.self_established_facts({"action": "place", "params": {"location": "kitchen table"}}) == ["at_robot(kitchen_table)"]
    assert ac.self_established_facts({"action": "deliver", "params": {"object": "coke", "recipient": "Susan"}}) == []
    assert ac.self_established_facts({"action": "grasp", "params": {"object": "coke"}}) == []


def test_self_navigating_destinations_excludes_goto():
    assert ac.self_navigating_destinations() == {
        "deliver": "recipient_location",
        "place": "location",
        "search_object": "location",
    }


def test_render_self_satisfied_rule_mentions_each_self_navigator():
    text = ac.render_self_satisfied_rule()
    for action in ("goto", "place", "deliver", "search_object"):
        assert action in text
    assert "at_robot" in text
```

- [ ] **Step 2: Run to verify failure**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_gpsr_action_contracts.py -q`
Expected: `ModuleNotFoundError: No module named 'behavior_tree.GPSR.action_contracts'`

- [ ] **Step 3: Implement the module**

```python
# src/behavior_tree/behavior_tree/GPSR/action_contracts.py
"""Action-level contracts: what each planner action requires, establishes, records.

Single source of truth consumed by the split prompt, the plan validator, the
parameter materialiser and the runtime fact verifier. Keyed by the planner's
action vocabulary (``small_trees.ACTION_FACTORIES``), NOT by BT node class —
that is ``supervision/contracts.py``'s job.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping

from .validators import _normalize


@dataclass(frozen=True)
class ActionContract:
    action: str
    requires: tuple[str, ...] = ()
    establishes: tuple[str, ...] = ()
    self_establishes: Mapping[str, str] = field(default_factory=lambda: MappingProxyType({}))
    records: tuple[str, ...] = ()
    self_navigating: bool = False


def _c(action: str, **kw: Any) -> ActionContract:
    se = kw.pop("self_establishes", None)
    return ActionContract(
        action=action,
        self_establishes=MappingProxyType(dict(se or {})),
        **kw,
    )


ACTION_CONTRACTS: dict[str, ActionContract] = {
    c.action: c
    for c in (
        # Order matters for _ESTABLISHER_FOR_PREDICATE (planner.py): the FIRST
        # action establishing a predicate is its canonical establisher
        # (at_robot -> goto, object_seen -> find_object).
        _c("goto", establishes=("at_robot(location)",),
           self_establishes={"at_robot": "location"},
           records=("last_nav_location",), self_navigating=True),
        _c("find_object", establishes=("object_seen(object)",),
           records=("object_detection", "target_object_name")),
        _c("search_object", establishes=("object_seen(object)",),
           self_establishes={"at_robot": "location"},
           records=("last_nav_location", "object_detection"), self_navigating=True),
        _c("place", requires=("held(object)",), establishes=("placed(object,location)",),
           self_establishes={"at_robot": "location"},
           records=("last_nav_location",), self_navigating=True),
        _c("deliver", requires=("held(object)",), establishes=("delivered(object,recipient)",),
           self_establishes={"at_robot": "recipient_location"},
           records=("last_nav_location",), self_navigating=True),
        _c("grasp", requires=("object_seen(object)",), establishes=("held(object)",)),
        _c("find_person", establishes=("person_found(person)",),
           records=("person_detection", "target_person_pose")),
        _c("count", establishes=("counted(object)",), records=("count_value", "count_target")),
        _c("ask_person", establishes=("answered(question)",), records=("person_answer",)),
        _c("answer_question", establishes=("answered(question)",), records=("qa_answer",)),
        _c("approach_person"),
        _c("describe_person"),
        _c("follow"),
        _c("guide"),
        _c("open"),
        _c("announce"),
        _c("record_position"),
        _c("vlm_fallback", records=("vlm_answer",)),
        _c("llm_fallback", records=("llm_answer",)),
    )
}


def contract_for(action: str) -> ActionContract:
    return ACTION_CONTRACTS[str(action)]


def self_established_facts(step: Mapping[str, Any]) -> list[str]:
    """Canonical facts ``step`` satisfies by itself (param absent -> nothing)."""
    contract = ACTION_CONTRACTS.get(str(step.get("action")))
    if contract is None:
        return []
    params = step.get("params") or {}
    facts: list[str] = []
    for predicate, param in contract.self_establishes.items():
        value = params.get(param)
        if value is None or not str(value).strip():
            continue
        facts.append(f"{predicate}({_normalize(str(value))})")
    return facts


def self_navigating_destinations() -> dict[str, str]:
    return {
        name: c.self_establishes["at_robot"]
        for name, c in sorted(ACTION_CONTRACTS.items())
        if c.self_navigating and name != "goto" and "at_robot" in c.self_establishes
    }


def render_self_satisfied_rule() -> str:
    parts = [
        f"{name} (via its {c.self_establishes['at_robot']})"
        for name, c in sorted(ACTION_CONTRACTS.items())
        if "at_robot" in c.self_establishes
    ]
    return (
        "A target must NOT list as a precondition a fact that its own action "
        "establishes by itself. at_robot is self-satisfied by: " + ", ".join(parts) + "."
    )


__all__ = [
    "ActionContract", "ACTION_CONTRACTS", "contract_for",
    "self_established_facts", "self_navigating_destinations", "render_self_satisfied_rule",
]
```

- [ ] **Step 4: Run to verify pass**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_gpsr_action_contracts.py -q`
Expected: 7 passed. If `test_records_are_known_evidence_names` fails on `target_person_pose`/`vlm_answer`/`llm_answer`, check `_TARGET_GATE_EVIDENCE_KEYS` (`orchestrator.py:1482-1502`) — those names are present there; fix the contract's `records`, not the test.

- [ ] **Step 5: Commit**

```bash
git add src/behavior_tree/behavior_tree/GPSR/action_contracts.py src/behavior_tree/test/test_gpsr_action_contracts.py
git commit -m "feat(gpsr): ActionContract registry — single source of truth per planner action"
```

---

### Task 2: Materialiser records navigation for every self-navigating action

**Files:**
- Modify: `src/behavior_tree/behavior_tree/GPSR/orchestrator.py:1049-1052`
- Test: `src/behavior_tree/test/test_gpsr_action_contracts.py` (append)

**Interfaces:**
- Consumes: `action_contracts.contract_for`.
- Produces: `materialise_params` writes `bb_keys.LAST_NAV_LOCATION` for any action whose contract `records` contains `"last_nav_location"`, taking the value from `params[contract.self_establishes["at_robot"]]`.

- [ ] **Step 1: Write the failing test**

Append to `test/test_gpsr_action_contracts.py`:

```python
import py_trees
from py_trees.common import Access

from behavior_tree.GPSR.orchestrator import materialise_params
from behavior_tree.GPSR.small_trees import bb_keys


def _bb():
    py_trees.blackboard.Blackboard.clear()
    bb = py_trees.blackboard.Client(name="t")
    for key in (bb_keys.LAST_NAV_LOCATION, bb_keys.TARGET_LOCATION, bb_keys.TARGET_POSE,
                bb_keys.GRASP_ASK_REFEREE, bb_keys.GRASP_REFEREE_LOCATION,
                bb_keys.GRASP_REFEREE_POSE, bb_keys.GRASP_REFEREE_IS_APPLIANCE,
                bb_keys.TARGET_OBJECT_NAME, bb_keys.TARGET_OBJECT_PROMPT,
                bb_keys.TARGET_PERSON_PROMPT, bb_keys.ANNOUNCE_TEXT, bb_keys.ASK_QUESTION,
                bb_keys.VLM_QUESTION, bb_keys.LLM_QUESTION, bb_keys.CURRENT_DYNLABEL):
        bb.register_key(key, access=Access.WRITE)
        bb.register_key(key, access=Access.READ)
    for key in (bb_keys.REPORT_INFO, bb_keys.START_POSE, bb_keys.DYNAMIC_LOCATIONS):
        bb.register_key(key, access=Access.READ)
    from behavior_tree.GPSR.orchestrator import SEARCH_POSE_KEYS
    for key in SEARCH_POSE_KEYS:
        bb.register_key(key, access=Access.WRITE)
    bb.set(bb_keys.LAST_NAV_LOCATION, "", overwrite=True)
    return bb


@pytest.mark.parametrize("action,params,expected", [
    ("goto", {"location": "kitchen_table"}, "kitchen_table"),
    ("place", {"location": "kitchen_table"}, "kitchen_table"),
    ("deliver", {"object": "coke", "recipient": "Susan", "recipient_location": "living_room"}, "living_room"),
    ("search_object", {"object": "coke", "location": "kitchen"}, "kitchen"),
])
def test_materialise_records_last_nav_for_self_navigators(action, params, expected):
    bb = _bb()
    materialise_params(bb, action, params)
    assert bb.get(bb_keys.LAST_NAV_LOCATION) == expected


def test_materialise_does_not_touch_last_nav_for_grasp():
    bb = _bb()
    bb.set(bb_keys.LAST_NAV_LOCATION, "shelf", overwrite=True)
    materialise_params(bb, "grasp", {"object": "coke"})
    assert bb.get(bb_keys.LAST_NAV_LOCATION) == "shelf"


def test_materialise_deliver_without_location_leaves_last_nav():
    bb = _bb()
    bb.set(bb_keys.LAST_NAV_LOCATION, "office", overwrite=True)
    materialise_params(bb, "deliver", {"object": "coke", "recipient": "Susan"})
    assert bb.get(bb_keys.LAST_NAV_LOCATION) == "office"
```

- [ ] **Step 2: Run to verify failure**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_gpsr_action_contracts.py -q -k materialise`
Expected: `place` and `deliver` cases FAIL (`assert '' == 'kitchen_table'`); goto/search_object/grasp pass. If `_bb()` raises on an unregistered key, add that key to the register loop — `materialise_params` may touch keys beyond this list; do not weaken the assertions.

- [ ] **Step 3: Implement**

In `orchestrator.py` add near the other GPSR imports at the top of the file (find the line `from .small_trees import` and add after it):

```python
from .action_contracts import contract_for as _contract_for
```

Replace lines 1049-1052:

```python
    # Track where the robot navigates so a following grasp can tell it is a
    # shelf grasp (the fetch flow is goto(location) -> grasp(object)).
    if action == "goto":
        bb_client.set(bb_keys.LAST_NAV_LOCATION, str(params.get("location") or ""), overwrite=True)
```

with:

```python
    # Track where the robot navigates so a following grasp can tell it is a
    # shelf grasp, and so at_robot(<dest>) can verify after ANY action that
    # navigates itself (goto / place / deliver / search_object). Which actions
    # and which param is the action contract's business, not this function's.
    try:
        contract = _contract_for(action)
    except KeyError:
        contract = None
    if contract is not None and "last_nav_location" in contract.records:
        nav_param = contract.self_establishes.get("at_robot")
        nav_value = params.get(nav_param) if nav_param else None
        if nav_value is not None and str(nav_value).strip():
            bb_client.set(bb_keys.LAST_NAV_LOCATION, str(nav_value), overwrite=True)
```

Then in the `search_object` branch (line ~1066-1068) delete the now-redundant write:

```python
        # Remember where we searched so a following grasp can tell it is a
        # shelf grasp even if the planner forgot the from_shelf flag.
        bb_client.set(bb_keys.LAST_NAV_LOCATION, str(loc or ""), overwrite=True)
```

but keep the `DEFAULT_OBJECT_LOCATIONS` fallback for `loc` by moving it before the contract block: if `action == "search_object"` and `not params.get("location")` and `params.get("object")`, the location is `DEFAULT_OBJECT_LOCATIONS.get(str(obj).lower())`. Simplest: compute `params = dict(params)` at the top of the search_object handling and set `params["location"] = loc` when the fallback resolves, then let the contract block run *after* the search_object branch. To do that, move the contract block to just after the `search_object` block (before the `grasp` block at `:1080`). Final order: location→pose lookup, search_object branch (which may fill `params["location"]`), contract nav-record block, grasp block.

- [ ] **Step 4: Run all GPSR tests that touch materialisation**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_gpsr_action_contracts.py test/test_gpsr_resolve_pose_rooms.py test/test_scan_stores_no_match_response.py -q`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add src/behavior_tree/behavior_tree/GPSR/orchestrator.py src/behavior_tree/test/test_gpsr_action_contracts.py
git commit -m "fix(gpsr): materialise_params records LAST_NAV_LOCATION for every self-navigating action"
```

---

### Task 3: Plan validator and runtime verifier derive from the registry

**Files:**
- Modify: `src/behavior_tree/behavior_tree/GPSR/planner_validators.py:433`
- Modify: `src/behavior_tree/behavior_tree/GPSR/validators.py:309-310`
- Modify: `src/behavior_tree/behavior_tree/GPSR/planner.py:397-421`
- Test: `src/behavior_tree/test/test_gpsr_action_contracts.py` (append)

**Interfaces:**
- Consumes: `action_contracts.self_navigating_destinations`, `action_contracts.contract_for`, `action_contracts.ACTION_CONTRACTS`.
- Produces: no new names. `validate_plan` rejects `goto` before any self-navigating action; `_action_verdict` returns VALID for `at_robot(X)` after a succeeded step of any action whose `self_establishes["at_robot"]` param matches `X`.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_gpsr_action_contracts.py`:

```python
from behavior_tree.GPSR.planner_validators import validate_plan
from behavior_tree.GPSR.validators import (
    Verdict, VerificationContext, _action_verdict, parse_fact,
)


def test_validate_plan_rejects_goto_before_each_self_navigator(monkeypatch):
    fake = ac.ActionContract("teleport", self_establishes={"at_robot": "location"},
                             records=("last_nav_location",), self_navigating=True)
    monkeypatch.setitem(ac.ACTION_CONTRACTS, "teleport", fake)
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "teleport", "params": {"location": "kitchen"}},
    ]
    ok, reason = validate_plan(plan, "teleport to the kitchen", {"goto", "teleport"})
    assert not ok and "teleport" in reason and "redundant" in reason


def test_action_verdict_accepts_at_robot_after_place_and_deliver():
    fact, _ = parse_fact("at_robot(kitchen_table)")
    ctx = VerificationContext(
        phase="postcondition", established_facts=frozenset(),
        completed_steps=(
            {"action": "place", "params": {"location": "kitchen_table"}, "succeeded": True},
        ),
        target_object="", target_location="",
    )
    result = _action_verdict(fact, ctx)
    assert result is not None and result.verdict is Verdict.VALID

    fact, _ = parse_fact("at_robot(living_room)")
    ctx = VerificationContext(
        phase="postcondition", established_facts=frozenset(),
        completed_steps=(
            {"action": "deliver", "params": {"object": "coke", "recipient": "Susan",
                                             "recipient_location": "living_room"}, "succeeded": True},
        ),
        target_object="", target_location="",
    )
    result = _action_verdict(fact, ctx)
    assert result is not None and result.verdict is Verdict.VALID


def test_action_verdict_rejects_at_robot_after_grasp():
    fact, _ = parse_fact("at_robot(kitchen_table)")
    ctx = VerificationContext(
        phase="postcondition", established_facts=frozenset(),
        completed_steps=({"action": "grasp", "params": {"object": "coke"}, "succeeded": True},),
        target_object="", target_location="",
    )
    assert _action_verdict(fact, ctx) is None


def test_deterministic_intent_derives_from_registry():
    from behavior_tree.GPSR.planner import _deterministic_target_intent
    intents = _deterministic_target_intent({"desc": "x", "postconditions": ["placed(plant,balcony)", "at_robot(balcony)"]})
    assert [i["action"] for i in intents] == ["place", "goto"]
    assert intents[1]["params"] == {"location": "balcony"}
```

`_step_succeeded` (`validators.py:242-248`) treats a step with no `status`/`verdict`/`result` key as succeeded, so the `"succeeded": True` keys above are inert and may be dropped.

- [ ] **Step 2: Run to verify failure**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_gpsr_action_contracts.py -q -k "self_navigator or action_verdict or deterministic"`
Expected: `rejects_goto_before_each_self_navigator` FAILS (teleport not in hardcoded dict); `accepts_at_robot_after_place_and_deliver` FAILS (`result is None`); `rejects_at_robot_after_grasp` passes; `deterministic_intent` passes or fails depending on ordering — it must pass after Step 3.

- [ ] **Step 3: Implement**

`planner_validators.py` — add import at top (`from .action_contracts import self_navigating_destinations`) and replace line 433:

```python
    _SELF_NAV_DEST = {"deliver": "recipient_location", "place": "location"}
```
with
```python
    _SELF_NAV_DEST = self_navigating_destinations()
```
Leave the loop and message unchanged.

`validators.py` — this module is imported by `action_contracts.py` (for `_normalize`), so import lazily to avoid a cycle. Replace lines 309-310:

```python
        if fact.predicate == "at_robot" and action == "goto" and _norm_match(params.get("location", ""), target[0]):
            return _result(Verdict.VALID, "action-verdict fallback: stronger verifier not installed; successful goto action", 0.5)
```
with
```python
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
```

`planner.py` — replace the hand-written map in `_deterministic_target_intent` (lines 406-418):

```python
        params = {"location": fact.args[0]} if fact.predicate == "at_robot" and fact.args else {}
        action = {
            "at_robot": "goto",
            ...
        }.get(fact.predicate)
        if action:
            intents.append({"action": action, "params": params})
```
with
```python
        params = {"location": fact.args[0]} if fact.predicate == "at_robot" and fact.args else {}
        action = _ESTABLISHER_FOR_PREDICATE.get(fact.predicate)
        if action:
            intents.append({"action": action, "params": params})
```
and add the module-level table just above the function (after the imports; `action_contracts` import goes with the other `from .` imports at `planner.py:60`):

```python
from .action_contracts import ACTION_CONTRACTS, render_self_satisfied_rule, self_established_facts

# First action (in registry order) whose ``establishes`` names each predicate.
# goto is listed before search_object in the registry so at_robot -> goto.
_ESTABLISHER_FOR_PREDICATE: Dict[str, str] = {}
for _name, _contract in ACTION_CONTRACTS.items():
    for _tmpl in _contract.establishes:
        _pred = _tmpl.split("(", 1)[0]
        _ESTABLISHER_FOR_PREDICATE.setdefault(_pred, _name)
```
The registry (Task 1) lists `find_object` before `search_object` precisely so this map reproduces the old one (`at_robot→goto`, `object_seen→find_object`, `person_found→find_person`, `counted→count`, `answered→ask_person`, `held→grasp`, `placed→place`, `delivered→deliver`). Add this assertion to `test_deterministic_intent_derives_from_registry`:

```python
    assert _deterministic_target_intent({"desc": "x", "postconditions": ["object_seen(coke)"]})[0]["action"] == "find_object"
    assert _deterministic_target_intent({"desc": "x", "postconditions": ["answered(name)"]})[0]["action"] == "ask_person"
```

- [ ] **Step 4: Run**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_gpsr_action_contracts.py test/test_gpsr_fact_validators.py test/test_gpsr_target_planner.py test/test_gpsr_target_dag.py -q`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add src/behavior_tree/behavior_tree/GPSR/planner_validators.py src/behavior_tree/behavior_tree/GPSR/validators.py src/behavior_tree/behavior_tree/GPSR/planner.py src/behavior_tree/behavior_tree/GPSR/action_contracts.py src/behavior_tree/test/test_gpsr_action_contracts.py
git commit -m "refactor(gpsr): plan validator, fact verifier and intent map derive from ActionContract"
```

---

### Task 4: Split prompt rule 7 rendered from the registry

**Files:**
- Modify: `src/behavior_tree/behavior_tree/GPSR/planner.py:139-147, :158`
- Test: `src/behavior_tree/test/test_gpsr_action_contracts.py` (append)

**Interfaces:**
- Consumes: `action_contracts.render_self_satisfied_rule`.
- Produces: `planner.TOP_LAYER_SYSTEM_PROMPT` (defined at `planner.py:84`, terminated at `:158`) contains the rendered rule and no longer instructs a `place` target to carry `at_robot` as a precondition.

- [ ] **Step 1: Write the failing test**

```python
def test_split_prompt_rule_7_is_rendered_from_registry():
    from behavior_tree.GPSR import planner
    prompt = planner.TOP_LAYER_SYSTEM_PROMPT
    assert ac.render_self_satisfied_rule() in prompt
    assert "preconditions held(plant), at_robot(balcony)" not in prompt
    assert "precondition held(plant) and\n       postcondition placed(plant,balcony)" in prompt or "precondition held(plant)" in prompt
```

- [ ] **Step 2: Run to verify failure**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_gpsr_action_contracts.py -q -k rule_7`
Expected: FAIL on the first assertion.

- [ ] **Step 3: Implement**

Replace rule 7 (`planner.py:139-147`) with:

```
    7. Split executable state transitions even within one relocation job.
       "Move the plant from the kitchen to the balcony" MUST produce three
       targets: acquire/grasp with postcondition held(plant); transport/goto
       balcony depending on that target with precondition held(plant) and
       postcondition at_robot(balcony); place depending on transport with
       precondition held(plant) and postcondition placed(plant,balcony).
       __SELF_SATISFIED_RULE__ Preconditions list only facts an EARLIER
       target must have established. A "then"/"and" separates targets when
       it joins distinct work: "grab a coke from the kitchen, then take it to
       Susan in the living room" has two end-states.
```

and change the constant's terminator at `:158` from `""").strip()` to
`""").strip().replace("__SELF_SATISFIED_RULE__", render_self_satisfied_rule())`.
The prompt contains JSON braces, which is why this uses `.replace` and not `.format`.

- [ ] **Step 4: Run**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_gpsr_action_contracts.py test/test_gpsr_target_planner.py -q`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add src/behavior_tree/behavior_tree/GPSR/planner.py src/behavior_tree/test/test_gpsr_action_contracts.py
git commit -m "feat(gpsr): split prompt rule 7 renders self-satisfied preconditions from the registry"
```

---

### Task 5: Deferred preconditions

**Files:**
- Modify: `src/behavior_tree/behavior_tree/GPSR/small_trees.py:179` (add key)
- Modify: `src/behavior_tree/behavior_tree/GPSR/orchestrator.py:1532-1571` (`BtNode_TargetPreconditionCheck`), `:1573-1660` (`BtNode_TargetPostconditionCheck`), `:2016-2019` (`_swap_in` clearing)
- Modify: `src/behavior_tree/behavior_tree/GPSR/planner.py:1102-1105, 1140-1150` (`build_target_subtree`)
- Test: `src/behavior_tree/test/test_gpsr_deferred_preconditions.py`

**Interfaces:**
- Consumes: `action_contracts.self_established_facts(step) -> list[str]`.
- Produces:
  - `bb_keys.DEFERRED_PRECONDITIONS = "gpsr/deferred_preconditions"` (list[str] of canonical facts).
  - `BtNode_TargetPreconditionCheck(name, preconditions, target_index, action_plan=())` — new keyword `action_plan: Sequence[Mapping]`.
  - `BtNode_TargetPostconditionCheck` unchanged signature; additionally verifies whatever is in `DEFERRED_PRECONDITIONS`, and runs even when `postconditions` is empty if deferred facts exist. **Note:** `build_target_subtree` only adds the postcondition gate `if postconditions:` — Task 5 changes that to `if postconditions or any(self_established_facts(s) for s in action_plan)`.

- [ ] **Step 1: Write the failing tests**

```python
# src/behavior_tree/test/test_gpsr_deferred_preconditions.py
from __future__ import annotations

import py_trees
from py_trees.common import Access, Status

from behavior_tree.GPSR.orchestrator import (
    BtNode_TargetPreconditionCheck, BtNode_TargetPostconditionCheck, _TARGET_GATE_EVIDENCE_KEYS,
)
from behavior_tree.GPSR.small_trees import bb_keys


def _writer():
    py_trees.blackboard.Blackboard.clear()
    w = py_trees.blackboard.Client(name="w")
    for key in (bb_keys.FACTS, bb_keys.DEFERRED_PRECONDITIONS, bb_keys.LAST_NAV_LOCATION):
        w.register_key(key, access=Access.WRITE)
        w.register_key(key, access=Access.READ)
    for _, key in _TARGET_GATE_EVIDENCE_KEYS:
        w.register_key(key, access=Access.WRITE)
    w.set(bb_keys.FACTS, [], overwrite=True)
    w.set(bb_keys.DEFERRED_PRECONDITIONS, [], overwrite=True)
    return w


PLACE_PLAN = [{"action": "place", "params": {"location": "kitchen_table"}}]


def _tick(node):
    node.setup()
    node.initialise()
    return node.update()


def test_self_satisfied_precondition_is_deferred_not_failed():
    w = _writer()
    gate = BtNode_TargetPreconditionCheck("pre", ["at_robot(kitchen_table)"], 0, action_plan=PLACE_PLAN)
    assert _tick(gate) is Status.SUCCESS
    assert w.get(bb_keys.DEFERRED_PRECONDITIONS) == ["at_robot(kitchen_table)"]
    assert "deferred" in gate.feedback_message


def test_non_self_satisfied_precondition_still_checked():
    w = _writer()
    gate = BtNode_TargetPreconditionCheck("pre", ["held(coke)"], 0, action_plan=PLACE_PLAN)
    assert _tick(gate) is Status.FAILURE
    assert "precondition unmet: held(coke)" in gate.feedback_message
    assert w.get(bb_keys.DEFERRED_PRECONDITIONS) == []


def test_mixed_preconditions_defer_only_self_satisfied():
    w = _writer()
    w.set(bb_keys.FACTS, ["held(coke)"], overwrite=True)
    gate = BtNode_TargetPreconditionCheck("pre", ["held(coke)", "at_robot(kitchen_table)"], 0, action_plan=PLACE_PLAN)
    assert _tick(gate) is Status.SUCCESS
    assert w.get(bb_keys.DEFERRED_PRECONDITIONS) == ["at_robot(kitchen_table)"]


def test_deferred_precondition_at_a_different_location_is_not_deferred():
    w = _writer()
    gate = BtNode_TargetPreconditionCheck("pre", ["at_robot(office)"], 0, action_plan=PLACE_PLAN)
    assert _tick(gate) is Status.FAILURE


def _post(completed):
    return BtNode_TargetPostconditionCheck(
        "post", [], 0, PLACE_PLAN, target_object="coke", completed_steps=completed,
    )


def test_postcondition_gate_verifies_deferred_with_nav_evidence():
    w = _writer()
    w.set(bb_keys.DEFERRED_PRECONDITIONS, ["at_robot(kitchen_table)"], overwrite=True)
    w.set(bb_keys.LAST_NAV_LOCATION, "kitchen_table", overwrite=True)
    gate = _post([])
    assert _tick(gate) is Status.SUCCESS
    assert "at_robot(kitchen_table)" in w.get(bb_keys.FACTS)


def test_postcondition_gate_fails_deferred_without_nav_evidence():
    w = _writer()
    w.set(bb_keys.DEFERRED_PRECONDITIONS, ["at_robot(kitchen_table)"], overwrite=True)
    w.set(bb_keys.LAST_NAV_LOCATION, "office", overwrite=True)
    gate = _post([])
    assert _tick(gate) is Status.FAILURE
    assert "at_robot(kitchen_table)" in gate.feedback_message
```

- [ ] **Step 2: Run to verify failure**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_gpsr_deferred_preconditions.py -q`
Expected: `AttributeError: type object 'bb_keys' has no attribute 'DEFERRED_PRECONDITIONS'`.

- [ ] **Step 3: Implement**

`small_trees.py` after line 179:
```python
    DEFERRED_PRECONDITIONS = "gpsr/deferred_preconditions"  # list[str] — canonical preconditions the active target's own plan self-establishes; verified by the postcondition gate instead of at entry
```

`orchestrator.py` — `BtNode_TargetPreconditionCheck`:

```python
class BtNode_TargetPreconditionCheck(Behaviour):
    """Verify a target's preconditions at its execution boundary.

    A precondition the target's OWN plan establishes by itself (e.g.
    ``at_robot(kitchen_table)`` when the plan contains ``place(location=
    kitchen_table)``) is DEFERRED: not checked here, written to
    ``DEFERRED_PRECONDITIONS`` and verified by the postcondition gate once the
    establishing step has run. Checking it at entry would fail every
    self-navigating action before it could navigate.
    """

    def __init__(self, name: str, preconditions: List[str], target_index: int,
                 action_plan: Sequence[Mapping[str, Any]] = ()):
        super().__init__(name)
        self._preconditions = list(preconditions or [])
        self._target_index = int(target_index)
        self._self_established = {
            fact for step in (action_plan or []) for fact in _self_established_facts(step)
        }
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.FACTS, access=Access.READ)
        self._bb.register_key(bb_keys.DEFERRED_PRECONDITIONS, access=Access.WRITE)
        for _, key in _TARGET_GATE_EVIDENCE_KEYS:
            self._bb.register_key(key, access=Access.READ)

    def update(self):
        deferred: List[str] = []
        checked: List[str] = []
        for source in self._preconditions:
            fact, _err = parse_fact(source)
            canonical = canonical_fact(fact) if fact is not None else None
            if canonical is not None and canonical in self._self_established:
                deferred.append(canonical)
            else:
                checked.append(source)
        self._bb.set(bb_keys.DEFERRED_PRECONDITIONS, deferred, overwrite=True)
        if deferred:
            self.feedback_message = "precondition deferred: " + ", ".join(deferred)
        if not checked:
            return Status.SUCCESS
        context = VerificationContext(
            phase="precondition",
            established_facts=frozenset(_target_gate_facts(self._bb)),
            target_object="",
            target_location="",
        )
        evidence = _target_gate_evidence(self._bb)
        for source in checked:
            try:
                results, _ = check_all([source], evidence, context)
                result = results[0]
            except Exception:
                result = None
            verdict = Verdict.INVALID if result is None else result.verdict
            if verdict is not Verdict.VALID:
                self.feedback_message = f"precondition unmet: {source} ({verdict.value})"
                return Status.FAILURE
        return Status.SUCCESS
```
Add `from .action_contracts import self_established_facts as _self_established_facts` next to the `_contract_for` import from Task 2, and make sure `parse_fact`, `canonical_fact`, `Sequence`, `Mapping` are imported (`validators` already exports both fact helpers; `typing` for the rest).

`BtNode_TargetPostconditionCheck`:
- `setup`: register `bb_keys.DEFERRED_PRECONDITIONS` with `Access.READ` and `Access.WRITE`.
- `update`: replace the first two lines
  ```python
        if not self._postconditions:
            return Status.SUCCESS
  ```
  with
  ```python
        try:
            deferred = list(self._bb.get(bb_keys.DEFERRED_PRECONDITIONS) or [])
        except KeyError:
            deferred = []
        sources = list(self._postconditions) + [d for d in deferred if d not in self._postconditions]
        if not sources:
            return Status.SUCCESS
  ```
  and change `for source in self._postconditions:` to `for source in sources:`. After the FACTS write succeeds (just before the final `return Status.SUCCESS`), add `self._bb.set(bb_keys.DEFERRED_PRECONDITIONS, [], overwrite=True)`.

  Note: for a deferred `at_robot(X)`, `_verify` (`validators.py:323-328`) returns VALID when `last_nav_location` matches, INVALID when it mismatches. With Task 2, `place`/`deliver` now write `last_nav_location`, so this is the honest check.

`_swap_in` (`orchestrator.py:2016-2019`): inside the `if self._active_target_index != index:` block add `self._bb.set(bb_keys.DEFERRED_PRECONDITIONS, [], overwrite=True)` and register the key with WRITE in `DynamicExecutor.setup` (find where it registers `TARGET_REPLAN_COUNT` and add alongside).

`planner.py build_target_subtree`:
```python
        if preconditions:
            seq.add_child(BtNode_TargetPreconditionCheck(
                f"precondition gate:{slot}:{index}", preconditions, index,
                action_plan=action_plan,
            ))
```
and
```python
        deferred_possible = any(self_established_facts(s) for s in action_plan)
        if postconditions or (preconditions and deferred_possible):
            seq.add_child(BtNode_TargetPostconditionCheck(
```

- [ ] **Step 4: Run**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_gpsr_deferred_preconditions.py test/test_gpsr_target_gates.py test/test_gpsr_two_layer_supervision.py test/test_gpsr_action_contracts.py -q`
Expected: all pass. If `test_gpsr_target_gates.py` constructs `BtNode_TargetPreconditionCheck` positionally with three args it still works (new kwarg defaults to `()`).

- [ ] **Step 5: Commit**

```bash
git add src/behavior_tree/behavior_tree/GPSR/small_trees.py src/behavior_tree/behavior_tree/GPSR/orchestrator.py src/behavior_tree/behavior_tree/GPSR/planner.py src/behavior_tree/test/test_gpsr_deferred_preconditions.py
git commit -m "feat(gpsr): defer self-satisfied target preconditions to the postcondition gate"
```

---

### Task 6: Replan hygiene — dedupe and short-circuit

**Files:**
- Modify: `src/behavior_tree/behavior_tree/GPSR/orchestrator.py:604-620` (`_clean_plan`)
- Modify: `src/behavior_tree/behavior_tree/GPSR/planner.py:657-679` (`_store`/`_invalidate`), `:875-884` (`plan_target`)
- Test: `src/behavior_tree/test/test_gpsr_replan_hygiene.py`

**Interfaces:**
- Produces:
  - `_clean_plan` drops a step equal to the immediately preceding kept step; the dropped entry is reported in the second return value as `"duplicate:<action>"`.
  - `GPSRPlanner._invalidate(slot, index)` also stores `entry["failed_plan"] = entry["plan"]`.
  - `GPSRPlanner._failed_plan(slot, index) -> list[dict] | None`.
  - `_canonical_plan(plan) -> tuple` module-level helper in `planner.py`.
  - In `plan_target`, a validated plan whose `_canonical_plan` equals the failed plan is rejected with `last_reason = "the regenerated plan was IDENTICAL to the plan that just failed (<reason>) — you MUST change it"` unless it is the final attempt.
  - Existing behaviour retained: `_last_child_feedback` already threads the failing leaf's `feedback_message` into `replan_target(reason)` — covered by a test here, no code change.

- [ ] **Step 1: Write the failing tests**

```python
# src/behavior_tree/test/test_gpsr_replan_hygiene.py
from __future__ import annotations

import py_trees
from py_trees.common import Status

from behavior_tree.GPSR.orchestrator import DynamicExecutor, _clean_plan
from behavior_tree.GPSR import planner as planner_mod


def test_clean_plan_drops_consecutive_duplicate_steps():
    raw = [
        {"action": "goto", "params": {"location": "living_room"}},
        {"action": "place", "params": {"location": "kitchen_table"}},
        {"action": "place", "params": {"location": "kitchen_table"}},
        {"action": "announce", "params": {"text": "done"}},
    ]
    cleaned, dropped = _clean_plan(raw)
    assert [s["action"] for s in cleaned] == ["goto", "place", "announce"]
    assert dropped == ["duplicate:place"]


def test_clean_plan_keeps_non_consecutive_repeats():
    raw = [
        {"action": "goto", "params": {"location": "a"}},
        {"action": "announce", "params": {"text": "x"}},
        {"action": "goto", "params": {"location": "a"}},
    ]
    cleaned, _ = _clean_plan(raw)
    assert len(cleaned) == 3


def test_canonical_plan_ignores_param_order():
    a = [{"action": "deliver", "params": {"object": "coke", "recipient": "Susan"}}]
    b = [{"action": "deliver", "params": {"recipient": "Susan", "object": "coke"}}]
    assert planner_mod._canonical_plan(a) == planner_mod._canonical_plan(b)


class _StubClient:  # never used; plan_target is driven via monkeypatched _call_llm
    pass


def test_plan_target_rejects_plan_identical_to_failed_one(monkeypatch):
    p = planner_mod.GPSRPlanner(max_attempts=3)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    p._slot_context[0] = {"command": "put the coke on the kitchen table",
                          "targets": [{"id": "t0", "desc": "place the coke on the kitchen table",
                                       "object": "coke", "location": "kitchen_table", "depends_on": []}]}
    same = {"plan": [{"action": "place", "params": {"location": "kitchen_table"}}]}
    different = {"plan": [{"action": "goto", "params": {"location": "kitchen_table"}},
                          {"action": "announce", "params": {"text": "here"}}]}
    responses = iter([same, same, different])
    reasons = []

    def fake_call(client, system, user, temperature):
        reasons.append(user)
        return next(responses), None

    monkeypatch.setattr(planner_mod, "_call_llm", fake_call)
    monkeypatch.setattr(planner_mod, "validate_plan", lambda *a, **k: (True, ""))
    monkeypatch.setattr(planner_mod, "validate_plan_modifications", lambda *a, **k: (True, ""))

    # Seed the cache as if `same` had been executed and failed.
    p._store(0, 0, "place the coke on the kitchen table", same["plan"], py_trees.behaviours.Success("old"), None)
    p._invalidate(0, 0)
    assert p._failed_plan(0, 0) == same["plan"]

    p.plan_target(0, 0, "place the coke on the kitchen table", command="put the coke on the kitchen table",
                  target_obj="coke", target_loc="kitchen_table", failure_reason="precondition unmet: at_robot(kitchen_table) (invalid)")
    assert p.get_action_plan(0, 0) == different["plan"]
    assert any("IDENTICAL" in r for r in reasons[1:])


def test_plan_target_accepts_identical_plan_on_final_attempt(monkeypatch):
    p = planner_mod.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    p._slot_context[0] = {"command": "c", "targets": [{"id": "t0", "desc": "d", "object": "", "location": "", "depends_on": []}]}
    same = {"plan": [{"action": "announce", "params": {"text": "x"}}]}
    monkeypatch.setattr(planner_mod, "_call_llm", lambda *a, **k: (same, None))
    monkeypatch.setattr(planner_mod, "validate_plan", lambda *a, **k: (True, ""))
    monkeypatch.setattr(planner_mod, "validate_plan_modifications", lambda *a, **k: (True, ""))
    p._store(0, 0, "d", same["plan"], py_trees.behaviours.Success("old"), None)
    p._invalidate(0, 0)
    p.plan_target(0, 0, "d", command="c", failure_reason="x")
    assert p.get_action_plan(0, 0) == same["plan"]  # honest hand-back, not the fallback


def test_executor_passes_leaf_feedback_as_replan_reason():
    leaf = py_trees.behaviours.Failure("gate")
    leaf.feedback_message = "precondition unmet: at_robot(kitchen_table) (invalid)"
    seq = py_trees.composites.Sequence("target", memory=True)
    seq.add_child(leaf)
    for _ in seq.tick():
        pass
    assert DynamicExecutor._last_child_feedback(seq) == leaf.feedback_message
```

`GPSRPlanner._slot_context` is the plain dict `request_plan_all` fills (`planner.py:641`, `:950`); the tests seed it directly.

- [ ] **Step 2: Run to verify failure**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_gpsr_replan_hygiene.py -q`
Expected: dedupe test FAILS (three actions kept); `_canonical_plan` AttributeError; `_failed_plan` AttributeError; the leaf-feedback test passes.

- [ ] **Step 3: Implement**

`orchestrator.py` `_clean_plan`:
```python
def _clean_plan(plan_raw: Any) -> Tuple[List[Dict[str, Any]], List[str]]:
    """Keep only well-formed {action, params} steps using known actions.

    Consecutive identical steps (same action AND params) are collapsed to one:
    a replan that emits ``place, place`` would execute the second against a
    now-empty gripper and fail for a reason that has nothing to do with the
    command.
    """
    cleaned: List[Dict[str, Any]] = []
    dropped: List[str] = []
    if not isinstance(plan_raw, list):
        return cleaned, [f"<{type(plan_raw).__name__}>"]
    for step in plan_raw:
        if not isinstance(step, dict):
            dropped.append(f"<{type(step).__name__}>")
            continue
        action = step.get("action")
        params = step.get("params", {}) or {}
        if action not in ACTION_FACTORIES:
            dropped.append(str(action))
            continue
        if cleaned and cleaned[-1]["action"] == action and cleaned[-1]["params"] == params:
            dropped.append(f"duplicate:{action}")
            continue
        cleaned.append({"action": action, "params": params})
    return cleaned, dropped
```

`planner.py` — module-level helper (near `_deterministic_target_intent`):
```python
def _canonical_plan(plan: List[Dict[str, Any]]) -> tuple:
    return tuple(
        (str(s.get("action")), tuple(sorted((str(k), str(v)) for k, v in (s.get("params") or {}).items())))
        for s in plan or []
    )
```

`_invalidate`:
```python
    def _invalidate(self, slot, index) -> None:
        """Mark (slot, index) not-ready so an in-flight replan's OLD subtree is
        never re-swapped in while the fresh plan is still being produced, and
        remember the plan that just failed so the replan can refuse to repeat it."""
        with self._lock:
            entry = self._cache.get((slot, index))
            if entry:
                entry["ready"] = False
                entry["failed_plan"] = list(entry.get("plan") or [])

    def _failed_plan(self, slot, index) -> Optional[List[Dict[str, Any]]]:
        with self._lock:
            entry = self._cache.get((slot, index))
        return list(entry["failed_plan"]) if entry and entry.get("failed_plan") else None
```

`plan_target` — after the `validate_plan_modifications` block (`:895`) and before `# Accepted — build the subtree`, insert:
```python
            failed = self._failed_plan(slot, index)
            if (failed is not None and _canonical_plan(cleaned) == _canonical_plan(failed)
                    and attempt < self._max_attempts - 1):
                last_reason = (
                    "the regenerated plan was IDENTICAL to the plan that just failed "
                    f"({failure_reason or 'unknown reason'}) — you MUST change it: add, remove "
                    "or reorder steps so the failure cannot recur"
                )
                print(f"[plan:{slot}:{index}] attempt {attempt+1}/{self._max_attempts} "
                      f"REJECTED: identical to failed plan")
                continue
```
`_store` should clear `failed_plan` (the new entry dict has no such key, which is already the case since `_store` rebuilds the dict — verify and leave).

- [ ] **Step 4: Run**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_gpsr_replan_hygiene.py test/test_gpsr_target_planner.py -q`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add src/behavior_tree/behavior_tree/GPSR/orchestrator.py src/behavior_tree/behavior_tree/GPSR/planner.py src/behavior_tree/test/test_gpsr_replan_hygiene.py
git commit -m "fix(gpsr): dedupe consecutive plan steps; reject a replan identical to the failed plan"
```

---

### Task 7: Two-layer plan telemetry the bench can read

**Files:**
- Modify: `src/behavior_tree/behavior_tree/GPSR/orchestrator.py:2005-2040` (`_swap_in`), `:2259-2265` (`tick` failure branch)
- Test: `src/behavior_tree/test/test_gpsr_two_layer_telemetry.py`

**Interfaces:**
- Consumes: `telemetry.get_default_telemetry()`, `GpsrTelemetry.emit(event_type, payload, *, task_id, phase)`, `GpsrTelemetry.task_id(slot)`, `bench.events.parse_events`.
- Produces:
  - In `_swap_in`: if `bb_keys.TASK_ID` is unset/None, set it to `telemetry.task_id(self._slot + 1)`; emit `plan.materialized` with `{"slot", "target_index", "revision", "steps": [{"action","params"}...]}`, `task_id=<task id>`, `phase="planning"`.
  - In `tick` when the child returns FAILURE: emit `step.finished` with `{"step_index": PLAN_INDEX-1, "action": CURRENT_ACTION, "params": CURRENT_PARAMS, "outcome": "failed", "feedback": reason}`, `task_id`, `phase="execution"`.

- [ ] **Step 1: Background (no action)**

`BtNode_LogStepResult` (`orchestrator.py:1428-1446`) already emits `step.finished` on success with `task_id=self._bb.get(bb_keys.TASK_ID)`, but **nothing in `orchestrator.py` ever writes `bb_keys.TASK_ID`** (verified: `grep "bb_keys.TASK_ID" orchestrator.py` shows only reads), so the event carries `task_id=None` and `bench/events.py:128-130` drops it. Failed steps never reach `LogStepResult` at all. Setting `TASK_ID` in `_swap_in` fixes the first; `_emit_failed_step` fixes the second. The executor is named `executor task {slot+1}` (`orchestrator.py:3065`) and the bench maps that N to `/task-N`, so the task id must use `slot + 1`.

- [ ] **Step 2: Write the failing test**

```python
# src/behavior_tree/test/test_gpsr_two_layer_telemetry.py
from __future__ import annotations

import json
from pathlib import Path

import py_trees
from py_trees.common import Access

from behavior_tree.GPSR import telemetry as telemetry_mod
from behavior_tree.GPSR.bench.events import parse_events
from behavior_tree.GPSR.orchestrator import DynamicExecutor
from behavior_tree.GPSR.small_trees import bb_keys


class _Planner:
    def __init__(self, plan):
        self._plan = plan
    def is_target_ready(self, slot, index): return True
    def get_target_subtree(self, slot, index):
        seq = py_trees.composites.Sequence("target", memory=True)
        seq.add_child(py_trees.behaviours.Failure("gate"))
        seq.children[0].feedback_message = "precondition unmet: at_robot(kitchen_table) (invalid)"
        return seq
    def get_action_plan(self, slot, index): return list(self._plan)
    def get_facts(self, slot): return []
    def replan_target(self, slot, index, reason): pass


def _events(tmp_path, plan):
    tele = telemetry_mod.GpsrTelemetry(tmp_path, enabled=True, trajectory_id="traj")
    telemetry_mod.set_default_telemetry(tele)
    path = tele.directory / "events.jsonl"
    py_trees.blackboard.Blackboard.clear()
    w = py_trees.blackboard.Client(name="w")
    for key in (bb_keys.TASK_ID, bb_keys.CURRENT_ACTION, bb_keys.CURRENT_PARAMS, bb_keys.PLAN_INDEX,
                bb_keys.TARGET_INDEX, bb_keys.TARGET_REPLAN_COUNT, bb_keys.REPLAN_REQUEST,
                bb_keys.STATE_LOG, bb_keys.FACTS, bb_keys.DEFERRED_PRECONDITIONS,
                bb_keys.SUPERVISOR_STEP_DISPOSITION, bb_keys.CURRENT_TARGET,
                bb_keys.SAVED_TARGETS_PREFIX + "0"):
        w.register_key(key, access=Access.WRITE)
        w.register_key(key, access=Access.READ)
    w.set(bb_keys.TASK_ID, None, overwrite=True)
    w.set(bb_keys.SAVED_TARGETS_PREFIX + "0",
          [{"id": "t0", "desc": "place the coke", "depends_on": []}], overwrite=True)
    w.set(bb_keys.CURRENT_ACTION, "place", overwrite=True)
    w.set(bb_keys.CURRENT_PARAMS, {"location": "kitchen_table"}, overwrite=True)
    w.set(bb_keys.PLAN_INDEX, 1, overwrite=True)
    w.set(bb_keys.TARGET_INDEX, 0, overwrite=True)
    w.set(bb_keys.TARGET_REPLAN_COUNT, 0, overwrite=True)
    w.set(bb_keys.REPLAN_REQUEST, {}, overwrite=True)
    w.set(bb_keys.STATE_LOG, [], overwrite=True)
    w.set(bb_keys.FACTS, [], overwrite=True)

    ex = DynamicExecutor("executor task 1", 0, _Planner(plan), max_replans_per_target=1)
    tree = py_trees.trees.BehaviourTree(ex)
    ex.setup(gpsr_tree=tree, node=object())  # DynamicExecutor reads SAVED_TARGETS_PREFIX+slot on its first tick (orchestrator.py:2204)
    for _ in range(3):
        tree.tick()
    tele.close(status="done")
    telemetry_mod.set_default_telemetry(None)
    return path, w


def test_swap_in_emits_plan_materialized_and_sets_task_id(tmp_path):
    plan = [{"action": "place", "params": {"location": "kitchen_table"}}]
    path, w = _events(tmp_path, plan)
    lines = [json.loads(l) for l in path.read_text().splitlines() if l.strip()]
    mat = [e for e in lines if e["event_type"] == "plan.materialized"]
    assert mat and mat[0]["payload"]["steps"] == plan
    assert str(mat[0]["task_id"]).endswith("/task-1")
    assert w.get(bb_keys.TASK_ID) == mat[0]["task_id"]


def test_failed_step_emits_step_finished_the_bench_can_parse(tmp_path):
    plan = [{"action": "place", "params": {"location": "kitchen_table"}}]
    path, _ = _events(tmp_path, plan)
    results = parse_events(Path(path))
    assert results[1].steps == [("place", "failed")]
```

`DynamicExecutor.setup` needs a `node` kwarg but only stores it; `object()` is enough for a network-free tick. If `setup` raises on a missing ROS handle attribute during `_swap_in` (`py_trees.trees.setup` on the new subtree), wrap the stub subtree so it has no ROS behaviours — the `Failure` leaf above has none. Do not change production signatures to fit the test.

- [ ] **Step 3: Run to verify failure**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_gpsr_two_layer_telemetry.py -q`
Expected: both FAIL — no `plan.materialized` event; `results[1].steps == []`.

- [ ] **Step 4: Implement**

In `_swap_in` right after `self._active_target_index = index` (`:2020`):

```python
        telemetry = get_default_telemetry()
        if telemetry is not None:
            try:
                task_id = self._bb.get(bb_keys.TASK_ID)
            except KeyError:
                task_id = None
            if not task_id:
                # "executor task N" is 1-based (create_batch_command_flow_new) and
                # bench/events.py joins it with the /task-N suffix, so N = slot + 1.
                task_id = telemetry.task_id(self._slot + 1)
                self._bb.set(bb_keys.TASK_ID, task_id, overwrite=True)
            try:
                revision = int(self._bb.get(bb_keys.TARGET_REPLAN_COUNT) or 0) + 1
            except KeyError:
                revision = 1
            get_action_plan = getattr(self._planner, "get_action_plan", None)
            steps = list(get_action_plan(self._slot, index)) if callable(get_action_plan) else []
            try:
                telemetry.emit(
                    "plan.materialized",
                    {"slot": self._slot, "target_index": index, "revision": revision,
                     "steps": [{"action": s.get("action"), "params": s.get("params") or {}} for s in steps]},
                    task_id=task_id, phase="planning",
                )
            except Exception:
                pass
```
Register `bb_keys.TASK_ID` (READ+WRITE), `CURRENT_ACTION`, `CURRENT_PARAMS`, `PLAN_INDEX` (READ) in `DynamicExecutor.setup` alongside the existing registrations.

In `tick`, replace
```python
                    else:
                        self._on_target_failure(self._last_child_feedback(node))
```
with
```python
                    else:
                        reason = self._last_child_feedback(node)
                        self._emit_failed_step(reason)
                        self._on_target_failure(reason)
```
and add the method:
```python
    def _emit_failed_step(self, reason: str) -> None:
        telemetry = get_default_telemetry()
        if telemetry is None:
            return
        try:
            action = self._bb.get(bb_keys.CURRENT_ACTION)
            params = self._bb.get(bb_keys.CURRENT_PARAMS) or {}
            step_index = int(self._bb.get(bb_keys.PLAN_INDEX) or 1) - 1
            task_id = self._bb.get(bb_keys.TASK_ID)
        except KeyError:
            return
        try:
            telemetry.emit(
                "step.finished",
                {"step_index": step_index, "action": action, "params": params,
                 "outcome": "failed", "feedback": reason},
                task_id=task_id, phase="execution",
            )
        except Exception:
            pass
```
If Step 1 showed `TASK_ID` was never written in the two-layer flow, the `_swap_in` change above is also what makes the existing success-path `step.finished` (from `BtNode_LogStepResult`) parseable.

- [ ] **Step 5: Run**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_gpsr_two_layer_telemetry.py test/test_gpsr_telemetry.py test/test_gpsr_bench_events.py test/test_gpsr_two_layer_supervision.py -q`
Expected: all pass.

- [ ] **Step 6: Commit**

```bash
git add src/behavior_tree/behavior_tree/GPSR/orchestrator.py src/behavior_tree/test/test_gpsr_two_layer_telemetry.py
git commit -m "feat(gpsr): two-layer executor emits plan.materialized and failed step.finished; sets TASK_ID"
```

---

### Task 8: Full suite + sim battery request

**Files:** none new.

- [ ] **Step 1: Run the whole GPSR test set**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_gpsr_*.py test/test_sim_identity_relaxation.py test/test_tier2_spawn_hooks.py test/test_scan_stores_no_match_response.py test/test_parse_count_from_answer.py -q`
Expected: all pass. Fix regressions in the task that introduced them; do not skip tests.

- [ ] **Step 2: Tier-0 smoke without the LLM**

Run from `src/behavior_tree` with the env set:
```bash
python3 -m behavior_tree.GPSR.gpsr_bench tier0 --offline-planner \
  --corpus behavior_tree/GPSR/gpsr_runs/bench/t2-2026/corpus.jsonl \
  --out /home/tinker/.claude/jobs/5b873e5d/tmp/t0-check
```
(flags per `gpsr_bench.py:158-166`: `--corpus`, `--out`, `--offline-planner`, optional `--only-class`, `--timeout`).
Expected: `SUMMARY.md` written under `--out`, no ERROR verdicts. The offline mock planner exercises `_clean_plan`, the validators and `build_target_subtree` end-to-end without an LLM.

- [ ] **Step 3: Request the sim battery**

Send to the `gpsr command testing robustness` session (via SendMessage) — branch name + HEAD sha, and ask for: `bringMeObjFromPlcmt` (s2026-000) and `findObjInRoom` (s2026-003) with `--timeout 1500`, reporting per run: milestone failures, `TARGET_REPLAN_COUNT` peaks / number of `tree regenerated` lines, wall seconds, and whether `report.json.results[].plan` is non-empty. Success criteria: no `precondition unmet: at_robot` at target entry; ≤1 replan per grasp failure; `plan` populated.

- [ ] **Step 4: Commit any fixups and stop**

Do not merge. Report the branch and battery outcome to the user.
