#!/usr/bin/env python3
"""Seed the GPSR debugger with a hardware-free supervisor validation replay."""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timedelta, timezone
from pathlib import Path
import shutil
from typing import Any, Mapping

from behavior_tree.GPSR.supervision.context import (
    FixtureContextProvider,
    gpsr_arm_pose_orbbec_look,
)
from behavior_tree.GPSR.supervision.models import CaptureRequest
from gpsr_debug_server.store import DebugStore


TRAJECTORY_ID = "gpsr-supervisor-hardware-free-validation"
BASE_TIME = datetime(2026, 8, 5, 7, 0, tzinfo=timezone.utc)


ARM_POSE_ORBBEC_LOOK = gpsr_arm_pose_orbbec_look()


def _tree(active: str, next_node: str | None) -> dict[str, Any]:
    nodes = [
        {
            "id": "sequence",
            "name": "pick coke safely",
            "type": "Sequence",
            "node_class": "sequence",
            "children": ["goto-table", "scan", "grasp", "return-host"],
            "status": "RUNNING",
        },
        {"id": "goto-table", "name": "go to dining table", "type": "BtNode_GotoAction", "status": "SUCCESS"},
        {"id": "scan", "name": "find coke", "type": "BtNode_ScanForGeneralist", "status": "INVALID"},
        {"id": "grasp", "name": "grasp coke", "type": "BtNode_Grasp", "status": "INVALID"},
        {"id": "return-host", "name": "return to host", "type": "BtNode_GotoAction", "status": "INVALID"},
    ]
    for node in nodes:
        if node["id"] == active:
            node["status"] = "FAILURE" if active in {"scan", "grasp"} else "SUCCESS"
        if next_node and node["id"] == next_node:
            node["next_to_tick"] = True
    return {
        "tree_document_version": 2,
        "root": "sequence",
        "nodes": nodes,
        "edges": [
            {"source": "sequence", "target": child}
            for child in ("goto-table", "scan", "grasp", "return-host")
        ],
    }


def _request(
    *,
    checkpoint_id: str,
    subtask_id: str,
    goal: str,
    node_id: str,
    node_name: str,
    class_name: str,
    status: str,
    effect: str,
    risk: str,
    next_node: Mapping[str, Any] | None,
    blackboard: Mapping[str, Any],
    recovery_ledger: tuple[Mapping[str, Any], ...] = (),
    pose: tuple[float, float, float] = (1.0, 1.0, 0.0),
    joints: tuple[float, ...] = ARM_POSE_ORBBEC_LOOK,
) -> CaptureRequest:
    return CaptureRequest(
        checkpoint_id=checkpoint_id,
        task_id="hardware-free-gpsr",
        subtask_id=subtask_id,
        tree_revision="executor-r1",
        plan_revision=1,
        original_instruction="Pick up the coke from the dining table and bring it back to me.",
        subtask_goal=goal,
        terminal_node={
            "node_id": node_id,
            "name": node_name,
            "class_name": class_name,
            "reported_status": status,
            "effect": effect,
            "risk": risk,
        },
        next_node=dict(next_node) if next_node else None,
        subtask_tree=_tree(node_id, next_node.get("node_id") if next_node else None),
        blackboard={
            **dict(blackboard),
            "gpsr/arm_pose_name": "arm_pos_orbbec_look",
        },
        execution_history=(
            {"node": node_name, "status": status},
        ),
        recovery_ledger=recovery_ledger,
        robot_pose=pose,
        arm_joints=joints,
    )


class Replay:
    def __init__(self, state_dir: Path) -> None:
        self.state_dir = state_dir
        self.artifact_dir = state_dir / "artifacts"
        self.artifact_dir.mkdir(parents=True, exist_ok=True)
        self.store = DebugStore(state_dir / "events.sqlite3")
        self.provider = FixtureContextProvider(
            output_dir=self.artifact_dir,
            clock=lambda: BASE_TIME,
        )
        self.sequence = 0
        self.previous_event_id: str | None = None

    def close(self) -> None:
        self.store.close()

    def emit(self, event_type: str, payload: Mapping[str, Any]) -> str:
        self.sequence += 1
        event_id = f"supervisor-demo-{self.sequence:04d}-{event_type.replace('.', '-')}"
        event: dict[str, Any] = {
            "schema": "tinker.gpsr.telemetry",
            "schema_version": 1,
            "trajectory_id": TRAJECTORY_ID,
            "sequence": self.sequence,
            "event_id": event_id,
            "type": event_type,
            "phase": "supervision" if event_type.startswith("supervisor.") else "execution",
            "occurred_at": (BASE_TIME + timedelta(seconds=self.sequence * 2)).isoformat().replace("+00:00", "Z"),
            "payload": dict(payload),
        }
        if self.previous_event_id:
            event["causation_id"] = self.previous_event_id
        self.store.append_event(event)
        self.previous_event_id = event_id
        return event_id

    def checkpoint(self, request: CaptureRequest, *, test_case: str, risk: str) -> None:
        snapshot = self.provider.capture(request)
        artifact_records: list[dict[str, Any]] = []
        prefix = _safe_name(request.checkpoint_id)
        for artifact in snapshot.artifacts:
            record = asdict(artifact)
            if artifact.path and not artifact.missing:
                source = Path(artifact.path)
                target = self.artifact_dir / f"{prefix}-{artifact.role}{source.suffix.lower()}"
                if source.resolve() != target.resolve():
                    shutil.copyfile(source, target)
                record["path"] = None
                record["url"] = f"/api/v1/artifacts/{target.name}"
            artifact_records.append(record)
        payload = snapshot.to_prompt_dict()
        payload.update(
            {
                "test_case": test_case,
                "node": dict(request.terminal_node),
                "reported_status": request.terminal_node["reported_status"],
                "risk": risk,
                "artifacts": artifact_records,
            }
        )
        self.emit("supervisor.checkpoint.created", payload)


def _safe_name(value: str) -> str:
    return "".join(character if character.isalnum() or character in "-_" else "_" for character in value)[:80]


def seed(state_dir: Path) -> str:
    state_dir.mkdir(parents=True, exist_ok=True)
    database = state_dir / "events.sqlite3"
    if database.exists():
        existing = DebugStore(database)
        try:
            if any(
                item["trajectory_id"] == TRAJECTORY_ID
                for item in existing.list_trajectories(limit=500)
            ):
                return TRAJECTORY_ID
        finally:
            existing.close()
    replay = Replay(state_dir)
    try:
        replay.emit(
            "trajectory.started",
            {
                "name": "GPSR supervisor · hardware-free validation",
                "pinned": True,
                "status": "running",
                "source": (
                    "camera fixtures + rendered navigation map + xArm7 URDF/STL"
                ),
                "mode": "hardware-free replay",
                "metadata": {
                    "command": "Pick up the coke from the dining table and bring it back to me.",
                    "validation_date": "2026-08-05",
                },
            },
        )
        replay.emit(
            "task.command_received",
            {
                "task_id": "hardware-free-gpsr",
                "command": "Pick up the coke from the dining table and bring it back to me.",
                "status": "running",
            },
        )
        replay.emit(
            "supervisor.test.summary",
            {
                "title": "Validated without a robot · deterministic suite plus live OpenRouter calls",
                "status": "passed",
                "tests": [
                    {
                        "role": "Multimodal verifier",
                        "status": "passed",
                        "coverage": (
                            "Front + upward wrist cameras, rendered map and "
                            "Tinker-base/xArm7 pose entered one strict-schema "
                            "query."
                        ),
                        "model": "openai/gpt-5.6-luna",
                        "effort": "medium",
                        "mode": "live OpenRouter",
                    },
                    {
                        "role": "Cross-camera negative control",
                        "status": "passed",
                        "coverage": (
                            "A real AprilTag calibration frame from another "
                            "scene was rejected as sensor_context_mismatch."
                        ),
                        "model": "openai/gpt-5.6-luna",
                        "effort": "medium",
                        "mode": "live OpenRouter",
                    },
                    {
                        "role": "Local recovery planner",
                        "status": "passed",
                        "coverage": "A missing coke produced a typed, allow-listed recovery macro instead of executable model output.",
                        "model": "openai/gpt-5.6-luna",
                        "effort": "high",
                        "mode": "live OpenRouter",
                    },
                    {
                        "role": "Global planner",
                        "status": "passed",
                        "coverage": "Destructive change and exhausted recovery budget produced replan/abort decisions.",
                        "model": "openai/gpt-5.6-luna",
                        "effort": "high",
                        "mode": "live OpenRouter",
                    },
                    {
                        "role": "Runtime + dashboard",
                        "status": "passed",
                        "coverage": "BT barriers, idempotency, bounded retries, telemetry projection and offline replay.",
                        "model": "pytest + node:test",
                        "mode": "hardware-free",
                    },
                ],
            },
        )

        clear = _request(
            checkpoint_id="cp-01-goto-table",
            subtask_id="step-01-goto-table",
            goal="reach the dining table",
            node_id="goto-table",
            node_name="go to dining table",
            class_name="BtNode_GotoAction",
            status="SUCCESS",
            effect="navigation",
            risk="reversible_motion",
            next_node={"node_id": "scan", "name": "find coke", "class_name": "BtNode_ScanForGeneralist"},
            blackboard={
                "gpsr/target_location": "dining_table",
                "gpsr/current_action": "goto",
                "gpsr/plan_index": 1,
                "gpsr/navigation_result": "reached",
            },
            pose=(1.25, 1.1, 0.12),
        )
        replay.checkpoint(clear, test_case="All clear · continue execution", risk="reversible_motion")
        replay.emit(
            "supervisor.query.completed",
            {
                "checkpoint_id": clear.checkpoint_id,
                "role": "verify",
                "model": "openai/gpt-5.6-luna",
                "reasoning_effort": "medium",
                "prompt_version": "gpsr-supervisor-v2",
                "latency_ms": 4821,
                "attempt": 1,
                "usage": {"total_tokens": 1876},
            },
        )
        replay.emit(
            "supervisor.verdict.received",
            {
                "checkpoint_id": clear.checkpoint_id,
                "verdict": "all_clear",
                "bt_assessment": "agree",
                "subtask_status": "achieved",
                "world_change": "none",
                "escalation": "none",
                "failure_category": "",
                "evidence": [
                    "The map overlay places the robot at the dining-table waypoint.",
                    "The navigation action and blackboard both report the destination reached.",
                ],
                "rationale": "The reported BT success agrees with the map and state evidence; the scan node may proceed.",
                "confidence": 0.96,
            },
        )

        recoverable = _request(
            checkpoint_id="cp-02-find-coke",
            subtask_id="step-02-find-coke",
            goal="find the coke before grasping it",
            node_id="scan",
            node_name="find coke",
            class_name="BtNode_ScanForGeneralist",
            status="FAILURE",
            effect="perception",
            risk="observation",
            next_node={"node_id": "grasp", "name": "grasp coke", "class_name": "BtNode_Grasp"},
            blackboard={
                "gpsr/target_object_name": "coke",
                "gpsr/target_location": "dining_table",
                "gpsr/current_action": "find",
                "gpsr/object_matches": [],
            },
            pose=(1.32, 1.08, 0.08),
        )
        replay.checkpoint(recoverable, test_case="Recoverable · insert a scan subtree", risk="observation")
        replay.emit(
            "supervisor.query.completed",
            {
                "checkpoint_id": recoverable.checkpoint_id,
                "role": "verify",
                "model": "openai/gpt-5.6-luna",
                "reasoning_effort": "medium",
                "latency_ms": 5310,
                "attempt": 1,
                "usage": {"total_tokens": 2034},
            },
        )
        replay.emit(
            "supervisor.verdict.received",
            {
                "checkpoint_id": recoverable.checkpoint_id,
                "verdict": "recoverable",
                "bt_assessment": "agree",
                "subtask_status": "not_achieved",
                "world_change": "non_destructive",
                "escalation": "local_recovery",
                "failure_category": "target_not_detected",
                "evidence": [
                    "The scan result contains no coke detection.",
                    "No spill, breakage, collision, or other destructive change is visible.",
                ],
                "rationale": "The camera covers only one viewpoint; a bounded pan/tilt scan can gather new evidence safely.",
                "confidence": 0.88,
            },
        )
        replay.emit(
            "supervisor.query.completed",
            {
                "checkpoint_id": recoverable.checkpoint_id,
                "role": "local_recovery",
                "model": "openai/gpt-5.6-luna",
                "reasoning_effort": "high",
                "latency_ms": 6244,
                "attempt": 1,
                "usage": {"total_tokens": 1512},
            },
        )
        replay.emit(
            "supervisor.recovery.proposed",
            {
                "checkpoint_id": recoverable.checkpoint_id,
                "issue_id": "issue-target-not-detected",
                "strategy_id": "scan-wide-views",
                "kind": "scan_views",
                "arguments": {
                    "angles": [[-45, 5], [0, 12], [45, 5]],
                    "perception_action": "find_object",
                    "target": "coke",
                },
                "rationale": "Three distinct viewpoints cover occlusions while keeping the base and arm stationary.",
                "expected_evidence": ["coke bounding box", "3D target pose"],
                "stop_conditions": ["target acquired", "all three views exhausted"],
            },
        )
        replay.emit(
            "supervisor.recovery.started",
            {
                "checkpoint_id": recoverable.checkpoint_id,
                "issue_id": "issue-target-not-detected",
                "strategy_id": "scan-wide-views",
                "kind": "scan_views",
            },
        )
        replay.emit(
            "supervisor.recovery.finished",
            {
                "checkpoint_id": recoverable.checkpoint_id,
                "issue_id": "issue-target-not-detected",
                "strategy_id": "scan-wide-views",
                "succeeded": True,
                "failed_distinct": 0,
            },
        )

        exhausted = _request(
            checkpoint_id="cp-03-recovery-budget",
            subtask_id="step-02-find-coke",
            goal="find a coke suitable for grasping",
            node_id="scan",
            node_name="find coke after recovery",
            class_name="BtNode_ScanForGeneralist",
            status="FAILURE",
            effect="perception",
            risk="observation",
            next_node={"node_id": "grasp", "name": "grasp coke", "class_name": "BtNode_Grasp"},
            blackboard={
                "gpsr/target_object_name": "coke",
                "gpsr/target_location": "dining_table",
                "gpsr/object_matches": [],
            },
            recovery_ledger=(
                {"strategy_id": "scan-left-right", "succeeded": False},
                {"strategy_id": "move-closer-and-rescan", "succeeded": False},
                {"strategy_id": "relocalize-table", "succeeded": False},
            ),
            pose=(1.28, 1.03, -0.08),
        )
        replay.checkpoint(exhausted, test_case="Three strategies failed · replan minimally", risk="observation")
        replay.emit(
            "supervisor.verdict.received",
            {
                "checkpoint_id": exhausted.checkpoint_id,
                "verdict": "unrecoverable",
                "bt_assessment": "agree",
                "subtask_status": "not_achieved",
                "world_change": "non_destructive",
                "escalation": "global_replan",
                "failure_category": "recovery_budget_exhausted",
                "evidence": [
                    "Three distinct recovery strategies ended without a coke detection.",
                    "The current table remains safe, but the local subtask has no remaining retry budget.",
                ],
                "rationale": "Escalate to the original instruction and preserve completed navigation while changing only the search location.",
                "confidence": 0.94,
            },
        )
        replay.emit(
            "supervisor.query.completed",
            {
                "checkpoint_id": exhausted.checkpoint_id,
                "role": "global_replan",
                "model": "openai/gpt-5.6-luna",
                "reasoning_effort": "high",
                "latency_ms": 7119,
                "attempt": 1,
                "usage": {"total_tokens": 2290},
            },
        )
        replay.emit(
            "supervisor.global.proposed",
            {
                "checkpoint_id": exhausted.checkpoint_id,
                "action": "replan_remaining",
                "replacement_plan": [
                    {"action": "goto", "params": {"location": "kitchen_counter"}},
                    {"action": "find", "params": {"object": "coke"}},
                    {"action": "grasp", "params": {"object": "coke"}},
                    {"action": "goto", "params": {"location": "host"}},
                ],
                "preserved_completed_steps": 1,
                "relaxed_constraints": ["source location may be kitchen instead of dining table"],
                "rationale": "The requested object is absent from the table after bounded recovery; search the kitchen while preserving completed work.",
                "operator_message": "",
            },
        )

        broken = _request(
            checkpoint_id="cp-04-destructive-change",
            subtask_id="step-03-grasp-coke",
            goal="grasp and retain the coke safely",
            node_id="grasp",
            node_name="grasp coke",
            class_name="BtNode_Grasp",
            status="FAILURE",
            effect="manipulation",
            risk="irreversible",
            next_node={"node_id": "return-host", "name": "return to host", "class_name": "BtNode_GotoAction"},
            blackboard={
                "gpsr/target_object_name": "coke",
                "gpsr/grasp_result": "object_dropped",
                "gpsr/safety_state": "stop",
            },
            pose=(1.45, 1.15, 0.0),
        )
        replay.checkpoint(broken, test_case="Destructive change · stop and explain", risk="irreversible")
        replay.emit(
            "supervisor.verdict.received",
            {
                "checkpoint_id": broken.checkpoint_id,
                "verdict": "unrecoverable",
                "bt_assessment": "agree",
                "subtask_status": "not_achieved",
                "world_change": "destructive",
                "escalation": "global_replan",
                "failure_category": "object_broken",
                "evidence": [
                    "The manipulation result reports that the object was dropped.",
                    "Continuing the grasp sequence could worsen an unsafe scene.",
                ],
                "rationale": "The requested transfer can no longer be completed safely; stop robot effects and report the concrete reason.",
                "confidence": 0.98,
            },
        )
        replay.emit(
            "supervisor.query.completed",
            {
                "checkpoint_id": broken.checkpoint_id,
                "role": "global_replan",
                "model": "openai/gpt-5.6-luna",
                "reasoning_effort": "high",
                "latency_ms": 6832,
                "attempt": 1,
                "usage": {"total_tokens": 1944},
            },
        )
        replay.emit(
            "supervisor.global.proposed",
            {
                "checkpoint_id": broken.checkpoint_id,
                "action": "abort_and_report",
                "replacement_plan": [
                    {"action": "goto", "params": {"location": "host"}},
                    {"action": "announce", "params": {"message": "task could not be completed safely"}},
                ],
                "preserved_completed_steps": 2,
                "relaxed_constraints": [],
                "rationale": "The object was dropped and is treated as damaged; do not retry irreversible manipulation.",
                "operator_message": "I'm sorry, the coke was dropped during the grasp and may be damaged, so I stopped rather than risk making the situation worse.",
            },
        )
        replay.emit(
            "trajectory.completed",
            {
                "status": "succeeded",
                "completed": True,
                "message": "Hardware-free supervisor validation replay completed.",
            },
        )
    finally:
        replay.close()
    return TRAJECTORY_ID


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--state-dir",
        type=Path,
        default=Path("/tmp/gpsr-supervisor-dashboard"),
        help="Debugger state directory to populate.",
    )
    args = parser.parse_args()
    trajectory_id = seed(args.state_dir)
    print(f"Seeded {trajectory_id} in {args.state_dir}")


if __name__ == "__main__":
    main()
