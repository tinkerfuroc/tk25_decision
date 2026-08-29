from __future__ import annotations

import threading

import pytest

from behavior_tree.GPSR import planner as planner_module
from behavior_tree.GPSR.planner import (
    GPSRPlanner,
    TOP_LAYER_SYSTEM_PROMPT,
    _build_lower_layer_user_prompt,
    _normalise_targets,
)
from behavior_tree.GPSR.planner_validators import validate_dag


def test_normalise_targets_emits_canonical_defaults_and_stable_ids():
    targets = _normalise_targets(["  find cup  ", {"desc": "place", "object": "cup"}])
    assert targets == [
        {"id": "t0", "desc": "find cup", "object": "", "location": "", "depends_on": [], "preconditions": [], "postconditions": []},
        {"id": "t1", "desc": "place", "object": "cup", "location": "", "depends_on": [], "preconditions": [], "postconditions": []},
    ]


def test_normalise_targets_maps_legacy_positional_dependencies_to_ids():
    targets = _normalise_targets([
        {"id": "acquire", "desc": "acquire"},
        {"id": "move", "desc": "move", "depends_on": 0},
        {"desc": "place", "depends_on": [0, " move ", 0]},
    ])
    assert [t["id"] for t in targets] == ["acquire", "move", "t2"]
    assert targets[1]["depends_on"] == ["acquire"]
    assert targets[2]["depends_on"] == ["acquire", "move"]


def test_normalise_targets_preserves_duplicate_explicit_ids_for_dag_rejection():
    targets = _normalise_targets([
        {"id": "same", "desc": "one"},
        {"id": "same", "desc": "two"},
    ])
    assert [t["id"] for t in targets] == ["same", "same"]
    assert validate_dag(targets)[0] is False


def test_normalise_targets_invalid_condition_shape_is_rejected_by_contract():
    target = _normalise_targets([{"desc": "one", "postconditions": "held(cup)"}])
    assert target[0]["postconditions"] == "held(cup)"
    assert planner_module._validate_target_contract(target)[0] is False


def test_normalise_targets_invalid_dependency_remains_rejected_by_dag():
    targets = _normalise_targets([
        {"desc": "one"},
        {"desc": "two", "depends_on": 99},
    ])
    assert validate_dag(targets)[0] is False


@pytest.mark.parametrize("raw", [123, "target", {"desc": "target"}, ("target",)])
def test_normalise_targets_rejects_non_list_collections(raw):
    assert _normalise_targets(raw) == []


def test_normalise_targets_preserves_malformed_dependencies_and_conditions():
    target = _normalise_targets([{
        "desc": "target",
        "depends_on": [" ", "-1", " ", 7],
        "preconditions": ["  " , 42],
        "postconditions": ["\t", None],
    }])[0]
    assert target["depends_on"] == ["", "-1", 7]
    assert target["preconditions"] == ["", 42]
    assert target["postconditions"] == ["", None]
    assert planner_module._validate_target_contract([target])[0] is False


def test_normalise_targets_strips_explicit_ids_before_dag_validation():
    targets = _normalise_targets([
        {"id": " same ", "desc": "one"},
        {"id": "same", "desc": "two"},
    ])
    assert [target["id"] for target in targets] == ["same", "same"]
    assert "duplicate" in (validate_dag(targets)[1] or "")


def test_normalise_targets_drops_preconditions_no_earlier_target_establishes():
    targets = _normalise_targets([
        {
            "desc": "Get a spam from the laundry_desk",
            "preconditions": ["at_robot(laundry_desk)"],
            "postconditions": ["held(spam)"],
        },
        {
            "desc": "Deliver the spam to me",
            "preconditions": ["held(spam)", "at_robot(start_position)"],
            "postconditions": ["delivered(spam,me)"],
        },
    ])
    assert targets[0]["preconditions"] == []
    assert targets[1]["preconditions"] == ["held(spam)"]


def test_fact_store_applies_transitions_and_returns_defensive_copies():
    planner = GPSRPlanner()
    planner.record_facts(2, ["held(cup)", "at_robot(kitchen)", "held(cup)"])
    planner.record_facts(2, ["placed(cup,table)", "at_robot(balcony)"])
    assert planner.get_facts(2) == ["placed(cup,table)", "at_robot(balcony)"]
    facts = planner.get_facts(2)
    facts.append("held(other)")
    assert planner.get_facts(2) == ["placed(cup,table)", "at_robot(balcony)"]
    planner.reset()
    assert planner.get_facts(2) == []


def test_fact_store_concurrent_writers_do_not_lose_facts():
    planner = GPSRPlanner()
    barrier = threading.Barrier(2)

    def write(fact):
        barrier.wait()
        planner.record_facts(0, [fact])

    threads = [threading.Thread(target=write, args=(fact,)) for fact in ("held(cup)", "at_robot(kitchen)")]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert set(planner.get_facts(0)) == {"held(cup)", "at_robot(kitchen)"}


def test_top_prompt_describes_schema_vocabulary_and_relocation_transitions():
    for field in ("id", "depends_on", "preconditions", "postconditions"):
        assert field in TOP_LAYER_SYSTEM_PROMPT
    rule_three = TOP_LAYER_SYSTEM_PROMPT.split("3.", 1)[1].split("4.", 1)[0]
    assert "depends_on to the index" not in rule_three
    assert "earlier target's ID" in rule_three or "earlier target ID" in rule_three
    for fact in ("at_robot", "object_seen", "person_found", "held", "placed", "delivered", "counted", "answered"):
        assert fact in TOP_LAYER_SYSTEM_PROMPT
    assert "three" in TOP_LAYER_SYSTEM_PROMPT
    assert "postcondition held(plant)" in TOP_LAYER_SYSTEM_PROMPT
    assert "postcondition at_robot(balcony)" in TOP_LAYER_SYSTEM_PROMPT


def test_lower_prompt_includes_verified_facts_or_none():
    prompt = _build_lower_layer_user_prompt("cmd", "target", "plant", "balcony", [], [], verified_facts=["held(plant)"])
    assert "Verified world-state facts" in prompt
    assert "  - held(plant)" in prompt
    assert "deterministic target gates" in prompt
    assert "(none yet)" in _build_lower_layer_user_prompt("cmd", "target", "", "", [], [])


@pytest.mark.parametrize("bad_targets", [123, "target", {"desc": "target"}, ("target",)])
def test_split_retries_non_list_targets_then_accepts_valid(monkeypatch, bad_targets):
    planner = GPSRPlanner(max_attempts=2)
    planner._offline_mock = False
    responses = [
        {"targets": bad_targets},
        {"targets": [{"id": "a", "desc": "good", "depends_on": [], "preconditions": [], "postconditions": []}]},
    ]
    monkeypatch.setattr(planner, "_new_client", lambda: object())
    monkeypatch.setattr(planner_module, "_call_llm", lambda *args: (responses.pop(0), None))
    assert planner.split_command("get a cup")[0]["desc"] == "good"


def test_split_all_non_list_attempts_fall_back_without_exception(monkeypatch):
    planner = GPSRPlanner(max_attempts=2)
    planner._offline_mock = False
    responses = [{"targets": 1}, {"targets": {"desc": "bad"}}]
    monkeypatch.setattr(planner, "_new_client", lambda: object())
    monkeypatch.setattr(planner_module, "_call_llm", lambda *args: (responses.pop(0), None))
    assert planner.split_command("first | second") == planner_module._offline_mock_targets("first | second")


def test_split_retries_invalid_graph_or_condition_then_accepts_valid(monkeypatch):
    planner = GPSRPlanner(max_attempts=2)
    planner._offline_mock = False
    responses = [
        {"targets": [{"id": "a", "desc": "bad", "depends_on": ["missing"], "postconditions": ["unknown(x)"]}]},
        {"targets": [{"id": "a", "desc": "good", "depends_on": [], "preconditions": [], "postconditions": ["held(cup)"]}]},
    ]
    reasons = []
    monkeypatch.setattr(planner, "_new_client", lambda: object())
    monkeypatch.setattr(planner_module, "_call_llm", lambda *args: (responses.pop(0), None))
    result = planner.split_command("get a cup")
    assert result[0]["desc"] == "good"


def test_plan_target_refreshes_verified_facts_inside_each_retry(monkeypatch):
    planner = GPSRPlanner(max_attempts=2)
    planner._offline_mock = False
    prompts = []
    planner.record_facts(0, ["held(cup)"])
    monkeypatch.setattr(planner, "_new_client", lambda: object())
    original_prompt = planner_module._build_lower_layer_user_prompt

    def capture_prompt(*args, **kwargs):
        prompts.append(kwargs.get("verified_facts"))
        return original_prompt(*args, **kwargs)

    monkeypatch.setattr(planner_module, "_build_lower_layer_user_prompt", capture_prompt)
    monkeypatch.setattr(planner_module, "_call_llm", lambda *args: ({
        "plan": [{"action": "announce", "params": {"text": "ok"}}]
    }, None))
    validations = iter([(False, "retry"), (True, None)])

    def validate(*args, **kwargs):
        result = next(validations)
        if result[0] is False:
            planner.record_facts(0, ["at_robot(kitchen)"])
        return result

    monkeypatch.setattr(planner_module, "validate_plan", validate)
    planner.plan_target(0, 0, "say ok")
    assert prompts == [["held(cup)"], ["held(cup)", "at_robot(kitchen)"]]


def test_build_target_subtree_post_gate_includes_preserved_prefix_without_executing_it():
    planner = GPSRPlanner()
    with planner._lock:
        planner._slot_context[0] = {"command": "cmd", "targets": [{
            "id": "t0", "desc": "finish cup", "object": "cup", "location": "kitchen",
            "depends_on": [], "preconditions": [],
            "postconditions": ["at_robot(kitchen)", "held(cup)"],
        }]}
    prefix = [{"action": "goto", "params": {"location": "kitchen"}}]
    remainder = [{"action": "grasp", "params": {"object": "cup"}}]
    tree = planner.build_target_subtree(0, 0, remainder, completed_steps=prefix)
    gate = tree.children[-1]
    assert gate._action_plan == prefix + remainder
    assert sum("step0" in child.name for child in tree.children) == 1


def test_build_target_subtree_pre_gate_sees_self_establishers_in_preserved_prefix():
    # Supervisor replacement: place(kitchen_table) already ran, only [announce]
    # remains. The precondition gate must still treat at_robot(kitchen_table)
    # as self-established (deferred), not check it at entry and fail.
    planner = GPSRPlanner()
    with planner._lock:
        planner._slot_context[0] = {"command": "cmd", "targets": [{
            "id": "t0", "desc": "place coke", "object": "coke", "location": "kitchen_table",
            "depends_on": [], "preconditions": ["at_robot(kitchen_table)"],
            "postconditions": [],
        }]}
    prefix = [{"action": "place", "params": {"location": "kitchen_table"}}]
    remainder = [{"action": "announce", "params": {"text": "done"}}]
    tree = planner.build_target_subtree(0, 0, remainder, completed_steps=prefix)
    pre = tree.children[0]
    assert type(pre).__name__ == "BtNode_TargetPreconditionCheck"
    assert "at_robot(kitchen_table)" in pre._self_established
    post = tree.children[-1]
    assert type(post).__name__ == "BtNode_TargetPostconditionCheck"
    assert post._action_plan == prefix + remainder


def test_build_target_subtree_defensively_copies_preserved_prefix():
    planner = GPSRPlanner()
    with planner._lock:
        planner._slot_context[0] = {"command": "cmd", "targets": [{
            "id": "t0", "desc": "finish", "object": "", "location": "",
            "depends_on": [], "preconditions": [], "postconditions": ["held(cup)"],
        }]}
    prefix = [{"action": "goto", "params": {"location": "kitchen"}}]
    tree = planner.build_target_subtree(0, 0, [], completed_steps=prefix)
    prefix[0]["params"]["location"] = "mutated"
    assert tree.children[-1]._action_plan[0]["params"]["location"] == "kitchen"


def test_build_target_subtree_places_pre_and_post_gates_around_announcement(monkeypatch):
    planner = GPSRPlanner()
    with planner._lock:
        planner._slot_context[0] = {"command": "cmd", "targets": [{
            "id": "t0", "desc": "go", "object": "plant", "location": "balcony",
            "depends_on": [], "preconditions": ["held(plant)"],
            "postconditions": ["at_robot(balcony)"],
        }]}
    tree = planner.build_target_subtree(0, 0, [])
    assert type(tree.children[0]).__name__ == "BtNode_TargetPreconditionCheck"
    assert type(tree.children[1]).__name__ == "BtNode_AnnounceFromBB"
    assert type(tree.children[-1]).__name__ == "BtNode_TargetPostconditionCheck"


def test_build_target_subtree_without_conditions_preserves_prior_shape():
    planner = GPSRPlanner()
    with planner._lock:
        planner._slot_context[0] = {"command": "cmd", "targets": [{
            "id": "t0", "desc": "say", "object": "", "location": "",
            "depends_on": [], "preconditions": [], "postconditions": [],
        }]}
    tree = planner.build_target_subtree(0, 0, [])
    assert [type(child).__name__ for child in tree.children] == ["BtNode_AnnounceFromBB"]


def test_request_plan_all_canonicalizes_plain_strings(monkeypatch):
    planner = GPSRPlanner()
    started = []
    monkeypatch.setattr(planner, "plan_target", lambda *args, **kwargs: started.append(args))
    planner.request_plan_all(4, ["one", {"desc": "two", "id": "second", "depends_on": []}], command="cmd")
    assert planner._get_slot_context(4)["targets"] == [
        {"id": "t0", "desc": "one", "object": "", "location": "", "depends_on": [], "preconditions": [], "postconditions": []},
        {"id": "second", "desc": "two", "object": "", "location": "", "depends_on": [], "preconditions": [], "postconditions": []},
    ]


def test_request_plan_all_clears_facts_for_new_command(monkeypatch):
    planner = GPSRPlanner()
    planner.record_facts(4, ["held(old)"])
    monkeypatch.setattr(planner, "plan_target", lambda *args, **kwargs: None)
    planner.request_plan_all(4, ["new target"], command="new command")
    assert planner.get_facts(4) == []


def test_request_plan_all_same_command_replan_preserves_facts(monkeypatch):
    planner = GPSRPlanner()
    planner.record_facts(4, ["held(cup)"])
    monkeypatch.setattr(planner, "plan_target", lambda *args, **kwargs: None)
    planner.request_plan_all(4, ["target"], command="command")
    planner.record_facts(4, ["at_robot(kitchen)"])
    planner.replan_target(4, 0, "retry")
    assert planner.get_facts(4) == ["at_robot(kitchen)"]


def test_request_plan_all_rejects_invalid_contract_before_mutating_or_spawning(monkeypatch):
    planner = GPSRPlanner()
    planner.record_facts(4, ["held(old)"])
    planner._slot_context[4] = {"command": "old", "targets": [{"id": "old"}]}
    planner._cache[(4, 0)] = {"ready": True, "plan": [{"action": "announce"}]}
    started = []
    monkeypatch.setattr(planner, "plan_target", lambda *args, **kwargs: started.append(args))

    with pytest.raises(ValueError, match="unknown dependency"):
        planner.request_plan_all(4, [{"id": "a", "desc": "a", "depends_on": ["missing"]}])

    assert started == []
    assert planner.get_facts(4) == ["held(old)"]
    assert planner._get_slot_context(4)["command"] == "old"
    assert planner._cache[(4, 0)]["ready"] is True


def test_request_plan_all_rejects_dict_missing_depends_on_but_accepts_plain_strings(monkeypatch):
    planner = GPSRPlanner()
    monkeypatch.setattr(planner, "plan_target", lambda *args, **kwargs: None)

    with pytest.raises(ValueError, match="depends_on is required"):
        planner.request_plan_all(0, [{"id": "a", "desc": "a"}])

    planner.request_plan_all(1, ["legacy target"])
    assert planner.get_targets(1)[0]["depends_on"] == []


@pytest.mark.parametrize("bad_target", [
    {"id": "a", "desc": "a", "depends_on": None},
    {"id": "a", "desc": "a", "depends_on": [], "postconditions": ["not-a-fact"]},
])
def test_request_plan_all_rejects_malformed_contract_without_mutation(monkeypatch, bad_target):
    planner = GPSRPlanner()
    planner.record_facts(2, ["held(old)"])
    planner._slot_context[2] = {"command": "old", "targets": []}
    started = []
    monkeypatch.setattr(planner, "plan_target", lambda *args, **kwargs: started.append(args))

    with pytest.raises(ValueError):
        planner.request_plan_all(2, [bad_target])

    assert started == []
    assert planner.get_facts(2) == ["held(old)"]
    assert planner._get_slot_context(2)["command"] == "old"


def test_dependency_ancestor_targets_include_only_transitive_declared_dependencies():
    targets = _normalise_targets([
        {"id": "independent", "desc": "independent", "depends_on": []},
        {"id": "root", "desc": "root", "depends_on": []},
        {"id": "middle", "desc": "middle", "depends_on": ["root"]},
        {"id": "leaf", "desc": "leaf", "depends_on": ["middle"]},
    ])
    assert [t["id"] for t in planner_module._dependency_ancestor_targets(targets, 3)] == ["root", "middle"]
    assert planner_module._dependency_ancestor_targets(targets, 1) == []


def test_request_plan_all_passes_only_dependency_ancestors_to_workers(monkeypatch):
    planner = GPSRPlanner()
    started = []
    monkeypatch.setattr(planner, "plan_target", lambda *args, **kwargs: started.append(args))
    targets = [
        {"id": "independent", "desc": "independent", "depends_on": []},
        {"id": "root", "desc": "root", "depends_on": []},
        {"id": "middle", "desc": "middle", "depends_on": ["root"]},
        {"id": "leaf", "desc": "leaf", "depends_on": ["middle"]},
    ]
    planner.request_plan_all(0, targets, command="cmd")
    assert {args[1]: [t["id"] for t in args[6]] for args in started} == {
        0: [], 1: [], 2: ["root"], 3: ["root", "middle"],
    }


def test_prior_validation_context_is_independent_of_cache_readiness(monkeypatch):
    planner = GPSRPlanner()
    targets = _normalise_targets([
        {"id": "find", "desc": "find person", "depends_on": [], "postconditions": ["person_found(Alex)"]},
        {"id": "guide", "desc": "guide Alex", "depends_on": ["find"]},
    ])
    with planner._lock:
        planner._slot_context[0] = {"command": "cmd", "targets": targets}
    expected = planner_module._deterministic_prior_plan(targets, 1)
    planner._cache[(0, 0)] = {"plan": [{"action": "find_person", "params": {"descriptor": "Alex"}}]}
    assert planner_module._flatten_prior_plans(planner, 0, 1) == expected
    planner._cache.clear()
    assert planner_module._flatten_prior_plans(planner, 0, 1) == expected


def test_get_targets_returns_defensive_snapshot_of_slot_context(monkeypatch):
    planner = GPSRPlanner()
    monkeypatch.setattr(planner, "plan_target", lambda *args, **kwargs: None)
    planner.request_plan_all(5, [{
        "id": "acquire",
        "desc": "acquire cup",
        "depends_on": [],
    }, {
        "id": "place",
        "desc": "place cup",
        "depends_on": ["acquire"],
    }], command="command")

    snapshot = planner.get_targets(5)
    snapshot[0]["desc"] = "mutated"
    snapshot[1]["depends_on"].append("unexpected")
    snapshot.append({"id": "extra"})

    assert planner.get_targets(5) == [
        {"id": "acquire", "desc": "acquire cup", "object": "", "location": "", "depends_on": [], "preconditions": [], "postconditions": []},
        {"id": "place", "desc": "place cup", "object": "", "location": "", "depends_on": ["acquire"], "preconditions": [], "postconditions": []},
    ]

def test_two_layer_planner_honours_offline_planner_override(monkeypatch):
    import behavior_tree.GPSR.orchestrator as orch
    import behavior_tree.GPSR.planner as planner_mod

    monkeypatch.setattr(orch, "is_full_mock_mode", lambda: True)

    monkeypatch.delenv("GPSR_OFFLINE_PLANNER", raising=False)
    assert planner_mod.GPSRPlanner()._offline_mock is True

    monkeypatch.setenv("GPSR_OFFLINE_PLANNER", "0")
    assert planner_mod.GPSRPlanner()._offline_mock is False

    monkeypatch.setenv("GPSR_OFFLINE_PLANNER", "1")
    assert planner_mod.GPSRPlanner()._offline_mock is True
