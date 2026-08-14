from types import SimpleNamespace

import py_trees
from py_trees.common import Access, Status

from behavior_tree.GPSR.orchestrator import (
    BtNode_TargetPostconditionCheck,
    BtNode_TargetPreconditionCheck,
    DynamicExecutor,
    materialise_params,
    _target_gate_evidence,
)
from behavior_tree.GPSR.validators import register_tier2_hook
from behavior_tree.GPSR.small_trees import bb_keys


def _setup(node):
    py_trees.trees.setup(root=node)
    return node


def _bb(name="target_gates"):
    bb = py_trees.blackboard.Client(name=name)
    for key in (
        bb_keys.FACTS,
        bb_keys.LAST_NAV_LOCATION,
        bb_keys.TARGET_OBJECT_DETECTION,
        bb_keys.TARGET_PERSON_DETECTION,
        bb_keys.ALL_WAVING_PERSONS,
        bb_keys.TARGET_PERSON_POSE,
        bb_keys.TARGET_OBJECT_PROMPT,
        bb_keys.TARGET_PERSON_PROMPT,
        bb_keys.QA_QUESTION,
        bb_keys.ASK_QUESTION,
        bb_keys.VLM_QUESTION,
        bb_keys.LLM_QUESTION,
        bb_keys.COUNT_VALUE,
        bb_keys.QA_ANSWER,
        bb_keys.PERSON_ANSWER,
        bb_keys.LLM_ANSWER,
        bb_keys.VLM_ANSWER,
        bb_keys.GRASP_ASK_REFEREE,
        bb_keys.DYNAMIC_LOCATIONS,
        bb_keys.GRASP_REFEREE_LOCATION,
        bb_keys.GRASP_REFEREE_POSE,
        bb_keys.GRASP_REFEREE_IS_APPLIANCE,
        bb_keys.TARGET_OBJECT_NAME,
        bb_keys.TARGET_LOCATION,
        bb_keys.TARGET_POSE,
    ):
        bb.register_key(key, access=Access.WRITE)
    return bb


def test_pre_empty_conditions_succeed():
    _bb().set(bb_keys.FACTS, [], overwrite=True)
    node = _setup(BtNode_TargetPreconditionCheck("pre", [], 0))
    assert node.update() is Status.SUCCESS


def test_pre_established_held_succeeds():
    bb = _bb("pre-held")
    bb.set(bb_keys.FACTS, ["held(plant pot)"], overwrite=True)
    node = _setup(BtNode_TargetPreconditionCheck("pre", ["held(plant pot)"], 0))
    assert node.update() is Status.SUCCESS


def test_pre_missing_held_fails_with_unknown_feedback():
    bb = _bb("pre-missing")
    bb.set(bb_keys.FACTS, [], overwrite=True)
    node = _setup(BtNode_TargetPreconditionCheck("pre", ["held(plant pot)"], 0))
    assert node.update() is Status.FAILURE
    assert node.feedback_message == "precondition unmet: held(plant pot) (UNKNOWN)"


def test_pre_verifier_exception_blames_exact_later_source():
    bb = _bb("pre-hook-error")
    bb.set(bb_keys.FACTS, ["at_robot(kitchen)"], overwrite=True)

    def raising_hook(_fact, _evidence, _context):
        raise RuntimeError("broken verifier")

    register_tier2_hook("held", raising_hook)
    try:
        node = _setup(BtNode_TargetPreconditionCheck(
            "pre", ["at_robot(kitchen)", "held(cup)"], 0
        ))
        assert node.update() is Status.FAILURE
        assert node.feedback_message == "precondition unmet: held(cup) (INVALID)"
    finally:
        register_tier2_hook("held", None)


def test_pre_malformed_fails_with_original_source_text():
    _bb("pre-malformed").set(bb_keys.FACTS, [], overwrite=True)
    text = "held(plant pot) trailing"
    node = _setup(BtNode_TargetPreconditionCheck("pre", [text], 0))
    assert node.update() is Status.FAILURE
    assert node.feedback_message == f"precondition unmet: {text} (INVALID)"


def test_target_gate_evidence_omits_missing_and_none_but_preserves_zero():
    bb = _bb("evidence-absence")
    bb.set(bb_keys.COUNT_VALUE, 0, overwrite=True)
    assert _target_gate_evidence(bb) == {"count_value": 0}


def test_target_gate_evidence_includes_specialist_and_provenance_inputs():
    bb = _bb("evidence-provenance")
    bb.set(bb_keys.ALL_WAVING_PERSONS, [{"pose": 1}], overwrite=True)
    bb.set(bb_keys.TARGET_PERSON_POSE, "pose", overwrite=True)
    bb.set(bb_keys.TARGET_OBJECT_PROMPT, "cups", overwrite=True)
    bb.set(bb_keys.QA_QUESTION, "What color?", overwrite=True)
    bb.set(bb_keys.ASK_QUESTION, "What color?", overwrite=True)
    evidence = _target_gate_evidence(bb)
    assert evidence["waving_persons"] == [{"pose": 1}]
    assert evidence["target_person_pose"] == "pose"
    assert evidence["count_target"] == "cups"
    assert evidence["qa_question"] == "What color?"
    assert evidence["ask_question"] == "What color?"
    assert evidence["person_provenance"] == "waving_specialist"


def test_pre_evidence_mapping_preserves_zero_and_artifacts_and_answers():
    bb = _bb("pre-evidence")
    bb.set(bb_keys.FACTS, [], overwrite=True)
    bb.set(bb_keys.COUNT_VALUE, 0, overwrite=True)
    bb.set(bb_keys.TARGET_OBJECT_DETECTION, {"objects": ["cup"]}, overwrite=True)
    bb.set(bb_keys.TARGET_PERSON_DETECTION, {"objects": ["Alex"]}, overwrite=True)
    bb.set(bb_keys.QA_ANSWER, "blue", overwrite=True)
    node = _setup(BtNode_TargetPreconditionCheck(
        "pre", ["counted(cups)", "object_seen(cup)", "person_found(alex)", "answered(color)"], 0
    ))
    assert node.update() is Status.SUCCESS


def test_post_matching_grasp_writes_and_merges_fact():
    bb = _bb("post-grasp")
    bb.set(bb_keys.FACTS, ["at_robot(kitchen)"], overwrite=True)
    written = []
    node = _setup(BtNode_TargetPostconditionCheck(
        "post", ["held(plant pot)"], 0,
        [{"action": "grasp", "params": {"object": "plant pot"}}],
        facts_writer=written.append,
    ))
    assert node.update() is Status.SUCCESS
    assert written == [["held(plant_pot)"]]
    assert bb.get(bb_keys.FACTS) == ["at_robot(kitchen)", "held(plant_pot)"]


def test_postcondition_mirror_applies_same_transitions_as_planner_store():
    bb = _bb("post-transition")
    bb.set(bb_keys.FACTS, ["at_robot(kitchen)", "held(plant pot)"], overwrite=True)
    node = _setup(BtNode_TargetPostconditionCheck(
        "post", ["placed(plant pot,table)", "at_robot(balcony)"], 0,
        [{"action": "place", "params": {"object": "plant pot", "location": "table"}},
         {"action": "goto", "params": {"location": "balcony"}}],
    ))
    assert node.update() is Status.SUCCESS
    assert bb.get(bb_keys.FACTS) == ["placed(plant_pot,table)", "at_robot(balcony)"]


def test_post_supervisor_replacement_uses_preserved_prefix_for_gate_only():
    bb = _bb("post-supervisor-prefix")
    bb.set(bb_keys.FACTS, [], overwrite=True)
    bb.set(bb_keys.LAST_NAV_LOCATION, "kitchen", overwrite=True)
    node = _setup(BtNode_TargetPostconditionCheck(
        "post", ["at_robot(kitchen)", "held(cup)"], 0,
        [{"action": "grasp", "params": {"object": "cup"}}],
        completed_steps=[{"action": "goto", "params": {"location": "kitchen"}}],
    ))
    assert node.update() is Status.SUCCESS
    assert bb.get(bb_keys.FACTS) == ["at_robot(kitchen)", "held(cup)"]


def test_post_matching_goto_establishes_at_robot():
    bb = _bb("post-goto")
    bb.set(bb_keys.FACTS, [], overwrite=True)
    node = _setup(BtNode_TargetPostconditionCheck(
        "post", ["at_robot(kitchen)"], 0,
        [{"action": "goto", "params": {"location": "kitchen"}}],
    ))
    assert node.update() is Status.SUCCESS
    assert bb.get(bb_keys.FACTS) == ["at_robot(kitchen)"]


def test_post_mismatching_action_is_unknown_failure():
    bb = _bb("post-mismatch")
    bb.set(bb_keys.FACTS, [], overwrite=True)
    node = _setup(BtNode_TargetPostconditionCheck(
        "post", ["held(plant pot)"], 0,
        [{"action": "grasp", "params": {"object": "mug"}}],
    ))
    assert node.update() is Status.FAILURE
    assert node.feedback_message == "postcondition unmet: held(plant pot) (UNKNOWN)"


def test_post_all_or_nothing_on_second_failure():
    bb = _bb("post-aon")
    bb.set(bb_keys.FACTS, ["at_robot(kitchen)"], overwrite=True)
    written = []
    node = _setup(BtNode_TargetPostconditionCheck(
        "post", ["held(plant pot)", "placed(plant pot,table)"], 0,
        [{"action": "grasp", "params": {"object": "plant pot"}}],
        facts_writer=written.append,
    ))
    assert node.update() is Status.FAILURE
    assert written == []
    assert bb.get(bb_keys.FACTS) == ["at_robot(kitchen)"]


def test_post_canonical_dedup_preserves_prior_order():
    bb = _bb("post-dedup")
    bb.set(bb_keys.FACTS, ["at_robot(kitchen)", "held(plant pot)"], overwrite=True)
    written = []
    node = _setup(BtNode_TargetPostconditionCheck(
        "post", ["held(plant pot)"], 0,
        [{"action": "grasp", "params": {"object": "plant pot"}}],
        facts_writer=written.append,
    ))
    assert node.update() is Status.SUCCESS
    assert written == [["held(plant_pot)"]]
    assert bb.get(bb_keys.FACTS) == ["at_robot(kitchen)", "held(plant_pot)"]


def test_post_writer_exception_does_not_merge():
    bb = _bb("post-writer-error")
    bb.set(bb_keys.FACTS, [], overwrite=True)

    def fail(_facts):
        raise RuntimeError("disk full")

    node = _setup(BtNode_TargetPostconditionCheck(
        "post", ["held(plant pot)"], 0,
        [{"action": "grasp", "params": {"object": "plant pot"}}],
        facts_writer=fail,
    ))
    assert node.update() is Status.FAILURE
    assert node.feedback_message.startswith("postcondition fact write failed:")
    assert bb.get(bb_keys.FACTS) == []


def test_post_verifier_exception_blames_exact_later_source():
    bb = _bb("post-hook-error")
    bb.set(bb_keys.FACTS, ["at_robot(kitchen)"], overwrite=True)

    def raising_hook(_fact, _evidence, _context):
        raise RuntimeError("broken verifier")

    register_tier2_hook("held", raising_hook)
    try:
        node = _setup(BtNode_TargetPostconditionCheck(
            "post", ["at_robot(kitchen)", "held(cup)"], 0,
            [{"action": "goto", "params": {"location": "kitchen"}}],
        ))
        assert node.update() is Status.FAILURE
        assert node.feedback_message == "postcondition unmet: held(cup) (INVALID)"
    finally:
        register_tier2_hook("held", None)


def test_post_malformed_middle_maps_feedback_to_original_source():
    bb = _bb("post-malformed-middle")
    bb.set(bb_keys.FACTS, [], overwrite=True)
    malformed = "not a fact"
    node = _setup(BtNode_TargetPostconditionCheck(
        "post", ["held(plant pot)", malformed, "at_robot(kitchen)"], 0,
        [{"action": "grasp", "params": {"object": "plant pot"}}],
    ))
    assert node.update() is Status.FAILURE
    assert node.feedback_message == f"postcondition unmet: {malformed} (INVALID)"


def test_dynamic_executor_last_child_feedback_uses_leaf_tip():
    class _Leaf(py_trees.behaviour.Behaviour):
        def update(self):
            self.feedback_message = "leaf feedback"
            return Status.FAILURE

    leaf = _Leaf("leaf")
    root = py_trees.composites.Sequence("root", memory=True)
    root.add_child(leaf)
    list(root.tick())
    assert DynamicExecutor._last_child_feedback(root) == "leaf feedback"


class _PlannerWithFacts:
    def get_target_subtree(self, slot, index):
        return py_trees.behaviours.Success("ready")

    def get_action_plan(self, slot, index):
        return []

    def get_facts(self, slot):
        self.calls = getattr(self, "calls", []) + [(slot,)]
        return ["held(cup)"]


class _PlannerWithoutFacts:
    def get_target_subtree(self, slot, index):
        return py_trees.behaviours.Success("ready")

    def get_action_plan(self, slot, index):
        return []

    def _get_desc(self, slot, index):
        return "target"


def _executor(planner):
    root = py_trees.composites.Sequence("root", memory=True)
    executor = DynamicExecutor("executor", 0, planner)
    root.add_child(executor)
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(node=SimpleNamespace(get_name=lambda: "stub"), gpsr_tree=tree)
    if hasattr(planner, "targets"):
        executor._bb.register_key(
            bb_keys.SAVED_TARGETS_PREFIX + "0", access=Access.WRITE
        )
        executor._bb.set(
            bb_keys.SAVED_TARGETS_PREFIX + "0",
            list(planner.targets),
            overwrite=True,
        )
    return executor, tree


def test_swap_in_clears_target_local_evidence_and_preserves_facts_budget():
    bb = _bb("swap-in")
    bb.register_key(bb_keys.REPLAN_REQUEST, access=Access.WRITE)
    bb.register_key(bb_keys.SUPERVISOR_STEP_DISPOSITION, access=Access.WRITE)
    bb.register_key(bb_keys.TARGET_REPLAN_COUNT, access=Access.WRITE)
    bb.set(bb_keys.FACTS, ["at_robot(old)"], overwrite=True)
    bb.set(bb_keys.REPLAN_REQUEST, {"level": "target"}, overwrite=True)
    bb.set(bb_keys.SUPERVISOR_STEP_DISPOSITION, {"decision": "x"}, overwrite=True)
    bb.set(bb_keys.TARGET_REPLAN_COUNT, 2, overwrite=True)
    local_values = {
        bb_keys.LAST_NAV_LOCATION: "old",
        bb_keys.TARGET_OBJECT_DETECTION: {"objects": ["cup"]},
        bb_keys.TARGET_PERSON_DETECTION: {"objects": ["Alex"]},
        bb_keys.ALL_WAVING_PERSONS: [{"pose": 1}],
        bb_keys.TARGET_PERSON_POSE: "pose",
        bb_keys.TARGET_OBJECT_PROMPT: "cups",
        bb_keys.TARGET_PERSON_PROMPT: "Alex",
        bb_keys.QA_QUESTION: "old question",
        bb_keys.ASK_QUESTION: "old ask question",
        bb_keys.VLM_QUESTION: "old vlm question",
        bb_keys.LLM_QUESTION: "old llm question",
        bb_keys.COUNT_VALUE: 0,
        bb_keys.QA_ANSWER: "old qa",
        bb_keys.PERSON_ANSWER: "old person",
        bb_keys.LLM_ANSWER: "old llm",
        bb_keys.VLM_ANSWER: "old vlm",
    }
    for key, value in local_values.items():
        bb.set(key, value, overwrite=True)
    executor, _tree = _executor(_PlannerWithFacts())
    executor._swap_in(0)
    assert bb.get(bb_keys.REPLAN_REQUEST) == {}
    assert bb.get(bb_keys.SUPERVISOR_STEP_DISPOSITION) is None
    assert bb.get(bb_keys.FACTS) == ["held(cup)"]
    assert executor._planner.calls == [(0,)]
    assert bb.get(bb_keys.TARGET_REPLAN_COUNT) == 2
    assert bb.get(bb_keys.LAST_NAV_LOCATION) == "old"
    for key in local_values:
        if key != bb_keys.LAST_NAV_LOCATION:
            assert bb.get(key) is None


def test_same_target_swap_preserves_target_evidence_and_last_nav_for_grasp_materialisation():
    bb = _bb("same-target-evidence")
    bb.register_key(bb_keys.REPLAN_REQUEST, access=Access.WRITE)
    bb.register_key(bb_keys.SUPERVISOR_STEP_DISPOSITION, access=Access.WRITE)
    bb.register_key(bb_keys.TARGET_REPLAN_COUNT, access=Access.WRITE)
    values = {
        bb_keys.LAST_NAV_LOCATION: "kitchen_shelf",
        bb_keys.TARGET_OBJECT_DETECTION: {"objects": ["cup"]},
        bb_keys.TARGET_PERSON_DETECTION: {"objects": ["Alex"]},
        bb_keys.ALL_WAVING_PERSONS: [{"pose": 1}],
        bb_keys.TARGET_PERSON_POSE: "pose",
        bb_keys.TARGET_OBJECT_PROMPT: "cups",
        bb_keys.TARGET_PERSON_PROMPT: "Alex",
        bb_keys.QA_QUESTION: "old question",
        bb_keys.ASK_QUESTION: "old ask question",
        bb_keys.VLM_QUESTION: "old vlm question",
        bb_keys.LLM_QUESTION: "old llm question",
        bb_keys.COUNT_VALUE: 2,
        bb_keys.QA_ANSWER: "blue",
        bb_keys.PERSON_ANSWER: "Alex",
        bb_keys.LLM_ANSWER: "answer",
        bb_keys.VLM_ANSWER: "vision",
    }
    for key, value in values.items():
        bb.set(key, value, overwrite=True)
    executor, _tree = _executor(_PlannerWithoutFacts())
    executor.status = Status.RUNNING
    executor._state = "REQUESTING"
    executor._index = 0
    executor._active_target_index = 0
    executor._swap_in(0)
    for key, value in values.items():
        assert bb.get(key) == value
    # materialise_params reads this retained navigation state when deciding
    # whether a subsequent grasp needs the no-grasp referee branch.
    assert bb.get(bb_keys.LAST_NAV_LOCATION) == "kitchen_shelf"


def test_new_target_swap_clears_target_evidence_but_retains_last_nav():
    bb = _bb("new-target-evidence")
    bb.register_key(bb_keys.REPLAN_REQUEST, access=Access.WRITE)
    bb.register_key(bb_keys.SUPERVISOR_STEP_DISPOSITION, access=Access.WRITE)
    bb.register_key(bb_keys.TARGET_REPLAN_COUNT, access=Access.WRITE)
    values = {
        bb_keys.LAST_NAV_LOCATION: "kitchen_shelf",
        bb_keys.TARGET_OBJECT_DETECTION: {"objects": ["cup"]},
        bb_keys.TARGET_PERSON_DETECTION: {"objects": ["Alex"]},
        bb_keys.ALL_WAVING_PERSONS: [{"pose": 1}],
        bb_keys.TARGET_PERSON_POSE: "pose",
        bb_keys.TARGET_OBJECT_PROMPT: "cups",
        bb_keys.TARGET_PERSON_PROMPT: "Alex",
        bb_keys.QA_QUESTION: "old question",
        bb_keys.ASK_QUESTION: "old ask question",
        bb_keys.VLM_QUESTION: "old vlm question",
        bb_keys.LLM_QUESTION: "old llm question",
        bb_keys.COUNT_VALUE: 2,
        bb_keys.QA_ANSWER: "blue",
        bb_keys.PERSON_ANSWER: "Alex",
        bb_keys.LLM_ANSWER: "answer",
        bb_keys.VLM_ANSWER: "vision",
    }
    for key, value in values.items():
        bb.set(key, value, overwrite=True)
    executor, _tree = _executor(_PlannerWithoutFacts())
    executor.status = Status.RUNNING
    executor._state = "REQUESTING"
    executor._index = 1
    executor._swap_in(1)
    assert bb.get(bb_keys.LAST_NAV_LOCATION) == "kitchen_shelf"
    for key in values:
        if key != bb_keys.LAST_NAV_LOCATION:
            assert bb.get(key) is None


def test_swap_in_without_get_facts_does_not_crash():
    bb = _bb("swap-no-facts")
    executor, _tree = _executor(_PlannerWithoutFacts())
    executor._swap_in(0)


def test_zero_target_fresh_activation_without_get_facts_clears_stale_facts():
    bb = _bb("zero-target-no-facts")
    bb.register_key(bb_keys.SAVED_TARGETS_PREFIX + "0", access=Access.WRITE)
    bb.set(bb_keys.FACTS, ["held(stale)"], overwrite=True)
    bb.set(bb_keys.SAVED_TARGETS_PREFIX + "0", [], overwrite=True)
    executor, _tree = _executor(_PlannerWithoutFacts())
    list(executor.tick())
    assert executor.status is Status.SUCCESS
    assert bb.get(bb_keys.FACTS) == []


def test_fresh_activation_without_get_facts_clears_stale_facts():
    bb = _bb("fresh-no-facts")
    bb.register_key(bb_keys.SAVED_TARGETS_PREFIX + "0", access=Access.WRITE)
    bb.set(bb_keys.FACTS, ["held(stale)"], overwrite=True)
    bb.set(bb_keys.SAVED_TARGETS_PREFIX + "0", ["target"], overwrite=True)
    executor, _tree = _executor(_PlannerWithoutFacts())
    list(executor.tick())
    assert bb.get(bb_keys.FACTS) == []


def test_without_get_facts_preserves_facts_across_target_swap_while_running():
    bb = _bb("running-no-facts")
    bb.set(bb_keys.FACTS, ["held(cup)"], overwrite=True)
    executor, _tree = _executor(_PlannerWithoutFacts())
    executor.status = Status.RUNNING
    executor._state = "REQUESTING"
    executor._index = 1
    executor._swap_in(1)
    assert bb.get(bb_keys.FACTS) == ["held(cup)"]


def test_without_get_facts_preserves_facts_during_same_target_replan():
    bb = _bb("replan-no-facts")
    bb.set(bb_keys.FACTS, ["held(cup)"], overwrite=True)
    executor, _tree = _executor(_PlannerWithoutFacts())
    executor.status = Status.RUNNING
    executor._state = "REQUESTING"
    executor._index = 0
    executor._swap_in(0)
    assert bb.get(bb_keys.FACTS) == ["held(cup)"]


class _DependencyPlanner:
    def __init__(self, targets, outcomes):
        self.targets = targets
        self.outcomes = list(outcomes)
        self.requested = []
        self.ticked = []

    def get_targets(self, slot):
        return [dict(target, depends_on=list(target.get("depends_on", []))) for target in self.targets]

    def get_target_subtree(self, slot, index):
        self.requested.append(index)
        outcome = self.outcomes[index]
        planner = self

        class _Target(py_trees.behaviour.Behaviour):
            def update(self):
                planner.ticked.append(index)
                return outcome

        return _Target(f"target-{index}")

    def get_action_plan(self, slot, index):
        return []

    def get_facts(self, slot):
        return []

    def _get_desc(self, slot, index):
        return self.targets[index]["desc"]

    def replan_target(self, slot, index, reason):
        pass


def _run_executor_until_terminal(executor, max_ticks=20):
    for _ in range(max_ticks):
        list(executor.tick())
        if executor.status in (Status.SUCCESS, Status.FAILURE):
            return
    raise AssertionError("executor did not reach a terminal status")


def test_skipped_prerequisite_blocks_dependent_without_requesting_or_ticking_it():
    planner = _DependencyPlanner([
        {"id": "t0", "desc": "first", "depends_on": []},
        {"id": "t1", "desc": "second", "depends_on": ["t0"], "preconditions": []},
    ], [Status.FAILURE, Status.SUCCESS])
    executor, _tree = _executor(planner)
    executor._max_replans = 1

    _run_executor_until_terminal(executor)

    assert planner.requested == [0, 0]
    assert planner.ticked == [0, 0]
    assert executor.status is Status.FAILURE


def test_transitive_dependency_blocks_downstream_target():
    planner = _DependencyPlanner([
        {"id": "t0", "desc": "first", "depends_on": []},
        {"id": "t1", "desc": "second", "depends_on": ["t0"]},
        {"id": "t2", "desc": "third", "depends_on": ["t1"]},
    ], [Status.FAILURE, Status.SUCCESS, Status.SUCCESS])
    executor, _tree = _executor(planner)
    executor._max_replans = 1

    _run_executor_until_terminal(executor)

    assert planner.requested == [0, 0]
    assert planner.ticked == [0, 0]
    assert executor.status is Status.FAILURE


def test_independent_later_target_runs_after_skip_but_executor_fails_partially():
    planner = _DependencyPlanner([
        {"id": "t0", "desc": "first", "depends_on": []},
        {"id": "t1", "desc": "independent", "depends_on": []},
    ], [Status.FAILURE, Status.SUCCESS])
    executor, _tree = _executor(planner)
    executor._max_replans = 1

    _run_executor_until_terminal(executor)

    assert planner.requested == [0, 0, 1]
    assert planner.ticked == [0, 0, 1]
    assert executor.status is Status.FAILURE


def test_all_success_dependency_chain_ends_success():
    planner = _DependencyPlanner([
        {"id": "t0", "desc": "first", "depends_on": []},
        {"id": "t1", "desc": "second", "depends_on": ["t0"]},
    ], [Status.SUCCESS, Status.SUCCESS])
    executor, _tree = _executor(planner)

    _run_executor_until_terminal(executor)

    assert planner.requested == [0, 1]
    assert planner.ticked == [0, 1]
    assert executor.status is Status.SUCCESS
