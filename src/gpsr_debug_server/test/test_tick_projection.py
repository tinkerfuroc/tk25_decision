"""Projection contract for full per-tick control-flow runtime telemetry."""
from __future__ import annotations

from gpsr_debug_server.store import DebugStore


def _event(sequence: int, event_id: str, event_type: str, payload: dict) -> dict:
    return {
        "trajectory_id": "tick-trace",
        "sequence": sequence,
        "event_id": event_id,
        "type": event_type,
        "occurred_at": f"2026-08-04T00:00:{sequence:02d}Z",
        "payload": payload,
    }


def test_tick_observed_keeps_current_previous_runtime_and_counter_deltas() -> None:
    with DebugStore(":memory:", checkpoint_interval=1) as store:
        assert store.append_event(
            _event(
                1,
                "tree",
                "tree.generated",
                {
                    "kind": "executor",
                    "tree_revision": 0,
                    "nodes": [
                        {
                            "id": "executor/root/0",
                            "name": "retry navigation",
                            "node_class": "py_trees.decorators.Retry",
                            "semantics": {"category": "decorator", "control_flow": "retry"},
                            "blackboard_access": {"read": ["/gpsr/target_location"], "write": [], "exclusive": []},
                            "action_context": {"action": "goto", "params": {"location": "kitchen"}},
                        }
                    ],
                },
            )
        )
        assert store.append_event(
            _event(
                2,
                "tick-1",
                "tree.tick_observed",
                {
                    "tree_revision": 0,
                    "tick": 1,
                    "visited_nodes": [
                        {
                            "id": "executor/root/0",
                            "visit_order": 0,
                            "status": "RUNNING",
                            "counters": {"kind": "retry", "counter": "failures", "value": 1, "limit": 3},
                        }
                    ],
                    "visit_order": ["executor/root/0"],
                    "counter_deltas": [],
                    "active_action_context": {"action": "goto", "params": {"location": "kitchen"}},
                },
            )
        )
        assert store.append_event(
            _event(
                3,
                "tick-2",
                "tree.tick_observed",
                {
                    "tree_revision": 0,
                    "tick": 2,
                    "visited_nodes": [
                        {
                            "id": "executor/root/0",
                            "visit_order": 0,
                            "status": "RUNNING",
                            "counters": {"kind": "retry", "counter": "failures", "value": 2, "limit": 3},
                        }
                    ],
                    "visit_order": ["executor/root/0"],
                    "retry_repeat_deltas": [
                        {
                            "node_id": "executor/root/0",
                            "kind": "retry",
                            "counter": "failures",
                            "previous": 1,
                            "current": 2,
                            "delta": 1,
                            "limit": 3,
                        }
                    ],
                },
            )
        )

        tree = store.trajectory_snapshot("tick-trace")["trees"]["revisions"]["0"]
        runtime = tree["runtime"]
        assert runtime["current_tick"]["tick"] == 2
        assert runtime["previous_tick"]["tick"] == 1
        assert runtime["current_tick"]["nodes"]["executor/root/0"]["counters"]["value"] == 2
        assert runtime["current_tick"]["counter_deltas"][0]["delta"] == 1
        # Existing topology consumers keep their semantic metadata and latest
        # node state on the original ``nodes`` map.
        assert tree["nodes"]["executor/root/0"]["semantics"]["control_flow"] == "retry"
        assert tree["nodes"]["executor/root/0"]["status"] == "RUNNING"
