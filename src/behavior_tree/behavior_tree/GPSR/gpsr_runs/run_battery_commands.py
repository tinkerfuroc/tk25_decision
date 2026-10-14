"""Drive a hand-written battery of tricky NL commands through the real GPSRPlanner.

Top-layer split bias + modifiable-tree acceptance test. Each command exercises a
specific planner behaviour; the log captures the TOP-layer split targets (with
object/location/depends_on), each target's LOWER-layer action plan, any applied
modifications, and a scripted checker verdict.

Run with the decision venv (real LLM, API key/model resolved from workspace .env):
  .../.venv_decision/bin/python .../gpsr_runs/run_battery_commands.py
"""

import datetime
import json
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent                      # .../GPSR/gpsr_runs
PKG = HERE.parent.parent.parent                              # .../behavior_tree
sys.path.insert(0, str(PKG))

from behavior_tree.GPSR.planner import GPSRPlanner          # noqa: E402
from behavior_tree.GPSR.orchestrator import (               # noqa: E402
    KNOWN_LOCATIONS, START_LOCATION_ALIASES,
    BtNode_TargetPostconditionCheck, BtNode_TargetPreconditionCheck,
    load_knowledge_from_constants,
)
from behavior_tree.GPSR.gpsr_full import CONSTANTS_PATH     # noqa: E402
from behavior_tree.GPSR.planner_validators import (                     # noqa: E402
    validate_dag, validate_plan,
)
from behavior_tree.GPSR.validators import parse_fact                         # noqa: E402
from behavior_tree.GPSR.small_trees import ACTION_FACTORIES              # noqa: E402
from behavior_tree.GPSR.modifiable_nodes import (                        # noqa: E402
    apply_modifications, diff_trees,
)
from behavior_tree.GPSR.plan_checker import (                            # noqa: E402
    PlanCheck, ScriptedPlanChecker, build_check_context,
)
from behavior_tree.GPSR.tree_serialization import serialize_tree         # noqa: E402

# --- the battery -------------------------------------------------------------
COMMANDS = [
    # A: split-bias stress — multi-action, weak/no connectives
    "grab a coke, some chips and a lemonade from the kitchen",
    "bring Susan the red jacket she left in the office",
    "move the plant from the kitchen to the balcony",
    "serve breakfast to the guests in the dining room",
    # B: modifiable-tree triggers
    "say hello to the person wearing a red jacket in the office",
    "follow the man in the blue shirt to the kitchen",
    "tell me where the person raising their left arm is",
    "count how many cups are on the table",
    # C: dependency / pronoun stress
    "grab a coke from the kitchen, then take it to Susan in the living room",
    "find the red cup, wash it, and put it back on the shelf",
]

#: Expected top-layer split shape, for a quick pass/fail note in the summary.
EXPECT = {
    0: ("enumeration -> 3 fetch targets, each location=kitchen", 3),
    1: ("no-connective 3 jobs -> find jacket / find Susan / deliver", 3),
    2: ("relocation -> acquire / transport / place validator/state transition targets", 3),
    3: ("ambiguous; must commit to a defensible split", "any"),
    4: ("attribute-person-specialist trigger", "any"),
    5: ("attribute on follow find_person", "any"),
    6: ("gesture descriptor -> specialist (acceptance yardstick)", "any"),
    7: ("count -> vlm-template candidate", "any"),
    8: ("'then' + it-resolution + depends_on=['t0']", 2),
    9: ("two connectives, it resolved twice, 3-chain", 3),
}

RUNS_LOG_DIR = HERE / "logs"


def validate_target_contract(targets: list[dict], planner: GPSRPlanner, slot: int) -> list[str]:
    """Return human-readable target schema, fact, and gate violations.

    This deliberately inspects only already-built planner subtrees and uses the
    pure fact/DAG validators, so it is safe to call from offline battery tests.
    Malformed planner output is reported as violations rather than raised.
    """
    from collections.abc import Mapping

    violations: list[str] = []
    try:
        dag_ok, dag_reason = validate_dag(targets)
    except Exception as exc:  # noqa: BLE001
        dag_ok, dag_reason = False, f"validator raised {exc!r}"
    if not dag_ok:
        violations.append(f"DAG invalid: {dag_reason}")

    for index, target in enumerate(targets if isinstance(targets, list) else []):
        target_id = target.get("id", f"index-{index}") if isinstance(target, Mapping) else f"index-{index}"
        if not isinstance(target, Mapping):
            violations.append(f"target {index} {target_id!r} is not a mapping")
            continue

        condition_values = {}
        for phase in ("preconditions", "postconditions"):
            conditions = target.get(phase)
            if not isinstance(conditions, list):
                violations.append(
                    f"target {index} {target_id!r}: {phase} must be a list"
                )
                condition_values[phase] = []
                continue
            condition_values[phase] = conditions
            for source in conditions:
                if not isinstance(source, str):
                    violations.append(
                        f"target {index} {target_id!r}: {phase} source {source!r} "
                        "must be a string fact"
                    )
                    continue
                try:
                    _fact, reason = parse_fact(source)
                except Exception as exc:  # noqa: BLE001
                    reason = f"fact parser raised {exc!r}"
                if reason:
                    violations.append(
                        f"target {index} {target_id!r} {phase} source {source!r}: {reason}"
                    )

        try:
            subtree = planner.get_target_subtree(slot, index)
        except Exception as exc:  # noqa: BLE001
            subtree = None
            violations.append(
                f"target {index} {target_id!r}: subtree lookup failed: {exc!r}"
            )
        if subtree is None:
            violations.append(f"target {index} {target_id!r}: subtree is missing")
            continue

        counts = {"preconditions": 0, "postconditions": 0}

        def walk(node):
            if isinstance(node, BtNode_TargetPreconditionCheck):
                counts["preconditions"] += 1
            if isinstance(node, BtNode_TargetPostconditionCheck):
                counts["postconditions"] += 1
            for child in getattr(node, "children", ()) or ():
                walk(child)

        try:
            walk(subtree)
        except Exception as exc:  # noqa: BLE001
            violations.append(
                f"target {index} {target_id!r}: subtree inspection failed: {exc!r}"
            )
            continue
        for phase, label in (("preconditions", "precondition gate"),
                             ("postconditions", "postcondition gate")):
            expected = 1 if condition_values[phase] else 0
            actual = counts[phase]
            if actual != expected:
                qualifier = "exactly one" if expected else "zero"
                violations.append(
                    f"target {index} {target_id!r}: {label} requires {qualifier} "
                    f"when {phase} are {'present' if expected else 'empty'}; found {actual}"
                )
    return violations


def _slug(text: str, n: int = 40) -> str:
    return "".join(c if c.isalnum() else "_" for c in text).strip("_")[:n] or "run"


def _short(v):
    if v is None:
        return "None"
    s = str(v)
    return s if len(s) <= 60 else s[:57] + "..."


def report_step_modifications(planner, slot, target_index, step_index, action):
    """Same as run_ten_cmd.report_step_modifications: print + return applied tuples."""
    grouped = planner.get_modifications(slot, target_index)
    mods = grouped.get(step_index, [])
    if not mods:
        print("          MODIFICATIONS: (none)")
        return []
    factory = ACTION_FACTORIES.get(action)
    if factory is None:
        print(f"          MODIFICATIONS: (action {action} unknown — none)")
        return []
    before = factory()
    after, applied = apply_modifications(before, action, mods)
    print(f"          MODIFICATIONS: {len(applied)} applied")
    for template, node_id, reason in applied:
        print(f"            - {template} @ {node_id}: {reason}")
    diff = diff_trees(before, after, f"small/{action}")
    added = [r for r in diff if r["reason"] == "added"]
    changed = [r for r in diff if r["reason"] == "changed"]
    removed = [r for r in diff if r["reason"] == "removed"]
    if diff:
        print(f"          TREE DIFF: +{len(added)} ~{len(changed)} -{len(removed)}")
        for record in diff[:10]:
            print(f"            {record['reason']:>8} {record['node_id']}  "
                  f"({record['type']}: {record.get('name','')})")
    else:
        print("          TREE DIFF: (structure unchanged — param-only change)")
    return applied


def main():
    RUNS_LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    model = os.environ.get("GPSR_LLM_MODEL", "?")
    log_path = RUNS_LOG_DIR / f"battery_{stamp}_{_slug(model)}_{len(COMMANDS)}cmds.log"
    fh = open(log_path, "w", buffering=1)
    import contextlib

    class _Tee:
        def __init__(self, fh_):
            self._fh = fh_
            self._real = sys.stdout

        def write(self, data):
            self._real.write(data)
            self._fh.write(data)
            self._fh.flush()

        def flush(self):
            self._real.flush()
            self._fh.flush()

    old = sys.stdout
    sys.stdout = _Tee(fh)
    try:
        load_knowledge_from_constants(CONSTANTS_PATH)
        planner = GPSRPlanner()
        checker = ScriptedPlanChecker(default=lambda: PlanCheck.approved(
            "plan-check", summary="scripted approval"))
        known_loc = (set(KNOWN_LOCATIONS.keys()) | START_LOCATION_ALIASES) or None

        print(f"# battery run model={model} commands={len(COMMANDS)}")
        print(f"# log -> {log_path}\n")

        validated = 0
        total = 0
        split_notes = []
        contract_violations_total = 0
        for slot, cmd in enumerate(COMMANDS):
            print("=" * 78)
            print(f"COMMAND {slot + 1}: {cmd}")
            print("=" * 78)
            try:
                targets = planner.split_command(cmd)
            except Exception as exc:  # noqa: BLE001
                print(f"  !! split failed: {exc!r}")
                continue
            print(f"\n[TOP-LAYER] split -> {len(targets)} target(s):")
            for i, t in enumerate(targets):
                if not isinstance(t, dict):
                    print(f"  T{i}: {t}")
                    continue
                print(f"  T{i}: id={t.get('id')!r} desc={t.get('desc')}"
                      f" object={t.get('object','')!r}"
                      f" location={t.get('location','')!r}"
                      f" depends_on={t.get('depends_on','')!r}"
                      f" preconditions={t.get('preconditions','')!r}"
                      f" postconditions={t.get('postconditions','')!r}")
            exp = EXPECT.get(slot, ("", "any"))
            expected_note, expected_n = exp
            got_n = len(targets)
            if expected_n != "any" and got_n != expected_n:
                split_notes.append(
                    f"  cmd {slot+1}: got {got_n} target(s), expected {expected_n} "
                    f"({expected_note})")
                print(f"  !! SPLIT NOTE: {split_notes[-1]}")
            else:
                print(f"  split shape OK — {expected_note}")

            try:
                planner.request_plan_all(slot, targets, command=cmd)
                deadline = time.time() + 120.0
                while time.time() < deadline:
                    if planner.all_targets_ready(slot, len(targets)):
                        break
                    time.sleep(0.05)
            except Exception as exc:  # noqa: BLE001
                print(f"  !! planning failed: {exc!r}")
                continue

            contract_violations = validate_target_contract(targets, planner, slot)
            contract_violations_total += len(contract_violations)
            for violation in contract_violations:
                print(f"  TARGET CONTRACT VIOLATION: {violation}")

            prior_steps = []
            for i in range(len(targets)):
                desc = targets[i].get("desc") if isinstance(targets[i], dict) else targets[i]
                plan = planner.get_action_plan(slot, i)
                ok, reason = validate_plan(plan, desc, set(ACTION_FACTORIES.keys()),
                                           known_locations=known_loc,
                                           prior_plan=prior_steps)
                prior_steps.extend(plan)
                total += 1
                if ok:
                    validated += 1
                print(f"\n  TARGET T{i}: {desc}")
                print(f"    plan ({len(plan)} steps, validate={ok}"
                      + (f": {reason}" if reason else "") + "):")
                for k, step in enumerate(plan):
                    act = step.get("action")
                    params = step.get("params", {})
                    print(f"      step{k} [{act}] params={params}")
                    report_step_modifications(planner, slot, i, k, act)
                sub = planner.get_target_subtree(slot, i)
                if sub is not None:
                    try:
                        import py_trees
                        tree_str = py_trees.display.unicode_tree(sub, show_status=False)
                        print("      decision tree:")
                        for line in tree_str.splitlines():
                            print(f"        {line}")
                    except Exception as exc:  # noqa: BLE001
                        print(f"      (tree render failed: {exc!r})")
            print()

        print("=" * 78)
        print(f"SUMMARY: {validated}/{total} plans validated")
        print(f"SUMMARY: {contract_violations_total} target contract violation(s)")
        if split_notes:
            print("SPLIT SHAPE NOTES:")
            for n in split_notes:
                print(n)
        else:
            print("All split shapes matched expectations.")
        print(f"[done] log -> {log_path}")
    finally:
        sys.stdout = old
        fh.close()


if __name__ == "__main__":
    main()
