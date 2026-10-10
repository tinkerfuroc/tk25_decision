"""Control-flow telemetry contracts for planned and actually ticked GPSR trees."""
from __future__ import annotations

import json

import py_trees

from behavior_tree.GPSR.plan_viz import planned_tree_document
from behavior_tree.GPSR.telemetry import GpsrTelemetry
from behavior_tree.GPSR.tree_serialization import serialize_tree


def test_planned_serializer_includes_semantics_blackboard_shape_and_action_context() -> None:
    def goto_tree():
        root = py_trees.composites.Sequence(name="go now", memory=True)
        root.add_child(py_trees.behaviours.Success(name="navigate leaf"))
        return root

    document = planned_tree_document(
        [{"action": "goto", "params": {"location": "kitchen"}}],
        {"goto": goto_tree},
        "planned-test",
    )
    action_node = next(node for node in document["nodes"] if node["name"].startswith("go now"))
    assert document["tree_document_version"] == 2
    assert action_node["node_class"] == "composite"
    assert action_node["type"] == "Sequence"
    assert action_node["semantics"]["category"] == "composite"
    assert action_node["semantics"]["kind"] == "sequence"
    assert action_node["blackboard_access"] == {"read": [], "write": [], "exclusive": []}
    assert action_node["action_context"] == {
        "action": "goto",
        "params": {"location": "kitchen"},
        "step_index": 0,
        "boundary": True,
    }
    descendant = next(node for node in document["nodes"] if node["name"] == "navigate leaf")
    assert descendant["action_context"] == {
        "action": "goto",
        "params": {"location": "kitchen"},
        "step_index": 0,
        "boundary": False,
    }
    assert descendant["node_class"] == "leaf"
    assert descendant["type"] == "Success"
    root = next(node for node in document["nodes"] if node["id"] == "planned/root")
    assert root["action_context"] == {}


def test_parallel_policy_and_stock_decorator_semantics_are_explicit() -> None:
    first = py_trees.behaviours.Success(name="first")
    second = py_trees.behaviours.Success(name="second")
    third = py_trees.behaviours.Success(name="third")
    selected = py_trees.composites.Parallel(
        name="selected parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected([first, third], synchronise=True),
        children=[first, second, third],
    )
    parallel = serialize_tree(selected, kind="executor")
    semantics = parallel["nodes"][0]["semantics"]
    assert parallel["tree_document_version"] == 2
    assert parallel["nodes"][0]["node_class"] == "composite"
    assert semantics["kind"] == "parallel"
    assert semantics["parallel_policy"] == "selected"
    assert semantics["synchronise"] is True
    assert semantics["selected_child_ids"] == ["executor/root/0", "executor/root/2"]

    all_policy = serialize_tree(
        py_trees.composites.Parallel(
            "all parallel",
            py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=True),
            [py_trees.behaviours.Success("child")],
        ),
        kind="executor",
    )["nodes"][0]["semantics"]
    one_policy = serialize_tree(
        py_trees.composites.Parallel(
            "one parallel",
            py_trees.common.ParallelPolicy.SuccessOnOne(),
            [py_trees.behaviours.Success("child")],
        ),
        kind="executor",
    )["nodes"][0]["semantics"]
    assert (all_policy["parallel_policy"], all_policy["synchronise"]) == ("all", True)
    assert (one_policy["parallel_policy"], one_policy["synchronise"]) == ("one", False)

    failure_is_success = serialize_tree(
        py_trees.decorators.FailureIsSuccess("optimistic", py_trees.behaviours.Failure("child")),
        kind="executor",
    )["nodes"][0]
    assert failure_is_success["node_class"] == "decorator"
    assert failure_is_success["semantics"]["kind"] == "failure_is_success"
    assert failure_is_success["semantics"]["status_mapping"]["FAILURE"] == "SUCCESS"

    timeout = serialize_tree(
        py_trees.decorators.Timeout("short timeout", py_trees.behaviours.Running("child"), duration=2.5),
        kind="executor",
    )["nodes"][0]["semantics"]
    assert timeout["timeout"] == {"duration_s": 2.5, "on_timeout": "FAILURE"}

    condition = serialize_tree(
        py_trees.decorators.Condition("wait success", py_trees.behaviours.Running("child"), py_trees.common.Status.SUCCESS),
        kind="executor",
    )["nodes"][0]["semantics"]
    assert condition["condition"] == {"succeed_status": "SUCCESS", "otherwise": "RUNNING"}

    one_shot = serialize_tree(
        py_trees.decorators.OneShot(
            "only once",
            py_trees.behaviours.Success("child"),
            py_trees.common.OneShotPolicy.ON_COMPLETION,
        ),
        kind="executor",
    )["nodes"][0]["semantics"]
    assert one_shot["oneshot"]["policy"] == "on_completion"

    foreach = serialize_tree(
        py_trees.decorators.ForEach("for each", py_trees.behaviours.Success("child"), "/items", "/item"),
        kind="executor",
    )["nodes"][0]["semantics"]
    assert foreach["foreach"] == {"source_key": "/items", "target_key": "/item"}


def test_snapshot_visitor_emits_only_visited_nodes_and_retry_deltas(tmp_path) -> None:
    retry = py_trees.decorators.Retry(
        name="retry once more",
        child=py_trees.behaviours.Failure(name="always fails"),
        num_failures=3,
    )
    root = py_trees.composites.Sequence(name="root", memory=True)
    root.add_children([retry, py_trees.behaviours.Success(name="must not be visited")])
    tree = py_trees.trees.BehaviourTree(root)
    telemetry = GpsrTelemetry(tmp_path, trajectory_id="tree-runtime", enabled=True)
    telemetry.attach_tick_visitor(tree)
    observe = telemetry.post_tick_handler()

    tree.tick()
    observe(tree)
    tree.tick()
    observe(tree)
    telemetry.close()

    records = [
        json.loads(line)
        for line in (tmp_path / "debug" / "tree-runtime" / "events.jsonl").read_text().splitlines()
    ]
    generated = next(record for record in records if record["event_type"] == "tree.generated")
    retry_topology = next(node for node in generated["payload"]["nodes"] if node["name"] == "retry once more")
    skipped_topology = next(node for node in generated["payload"]["nodes"] if node["name"] == "must not be visited")
    assert retry_topology["semantics"]["control_flow"] == "retry"
    assert retry_topology["semantics"]["counter"] == {"kind": "retry", "counter": "failures", "limit": 3}

    ticks = [record["payload"] for record in records if record["event_type"] == "tree.tick_observed"]
    assert len(ticks) == 2
    first_ids = {item["id"] for item in ticks[0]["visited_nodes"]}
    assert skipped_topology["id"] not in first_ids
    assert len(first_ids) == len(ticks[0]["visit_order"])
    assert [item["visit_order"] for item in ticks[0]["visited_nodes"]] == list(range(len(first_ids)))
    assert ticks[1]["retry_repeat_deltas"] == [
        {
            "node_id": retry_topology["id"],
            "kind": "retry",
            "counter": "failures",
            "previous": 1,
            "current": 2,
            "delta": 1,
            "limit": 3,
        }
    ]


def test_repeat_counter_delta_uses_the_same_tick_contract(tmp_path) -> None:
    root = py_trees.decorators.Repeat(
        name="repeat successful probe",
        child=py_trees.behaviours.Success(name="always succeeds"),
        num_success=3,
    )
    tree = py_trees.trees.BehaviourTree(root)
    telemetry = GpsrTelemetry(tmp_path, trajectory_id="repeat-runtime", enabled=True)
    telemetry.attach_tick_visitor(tree)
    observe = telemetry.post_tick_handler()

    tree.tick()
    observe(tree)
    tree.tick()
    observe(tree)
    telemetry.close()

    records = [
        json.loads(line)
        for line in (tmp_path / "debug" / "repeat-runtime" / "events.jsonl").read_text().splitlines()
    ]
    second = [record["payload"] for record in records if record["event_type"] == "tree.tick_observed"][1]
    assert second["counter_deltas"][0]["kind"] == "repeat"
    assert second["counter_deltas"][0]["counter"] == "successes"
    assert second["counter_deltas"][0]["previous"] == 1
    assert second["counter_deltas"][0]["current"] == 2


def test_tick_telemetry_reads_absolute_py_trees_blackboard_keys(tmp_path, monkeypatch) -> None:
    from py_trees.blackboard import Blackboard

    monkeypatch.setitem(Blackboard.storage, "/gpsr/task_id", "task-live")
    monkeypatch.setitem(Blackboard.storage, "/gpsr/current_action", "goto")
    monkeypatch.setitem(Blackboard.storage, "/gpsr/current_params", {"location": "kitchen"})
    monkeypatch.setitem(Blackboard.storage, "/gpsr/plan_revision", 4)
    monkeypatch.setitem(Blackboard.storage, "/gpsr/plan_index", 2)

    root = py_trees.behaviours.Running(name="navigate")
    tree = py_trees.trees.BehaviourTree(root)
    telemetry = GpsrTelemetry(tmp_path, trajectory_id="blackboard-runtime", enabled=True)
    telemetry.attach_tick_visitor(tree)
    observe = telemetry.post_tick_handler()

    tree.tick()
    observe(tree)
    telemetry.close()

    records = [
        json.loads(line)
        for line in (tmp_path / "debug" / "blackboard-runtime" / "events.jsonl").read_text().splitlines()
    ]
    tick = next(record for record in records if record["event_type"] == "tree.tick_observed")
    assert tick["task_id"] == "task-live"
    assert tick["payload"]["active_action_context"] == {
        "action": "goto",
        "params": {"location": "kitchen"},
        "plan_revision": 4,
        "plan_index": 2,
    }
