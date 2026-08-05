from __future__ import annotations

import json

import pytest

from behavior_tree.GPSR.adaptive import DebugControlGateway, TreeRevisionCoordinator, plan_to_tree_ir


def _proposal(tree, *, proposal_id: str = "p-1"):
    from gpsr_trace.metadata import AgentMetadata, ProposalMetadata
    from gpsr_trace.patch import AddNode, BehaviorTreeProposal, TreePatch
    from gpsr_trace.ir import NodeSpec, stable_node_id

    child = NodeSpec(
        node_id=stable_node_id("test", proposal_id),
        node_type="gpsr.action",
        params={"action": "announce", "params": {"text": "updated"}},
    )
    return BehaviorTreeProposal(
        metadata=ProposalMetadata.create(
            agent=AgentMetadata(agent_id="agent-a", role="planner"),
            proposal_id=proposal_id,
            base_tree_version=tree.version,
            summary="add a declarative step",
        ),
        patch=TreePatch((AddNode(child, tree.root_id),), base_version=tree.version),
    )


def test_plan_ir_is_stable_and_revision_coordinator_is_atomic() -> None:
    pytest.importorskip("gpsr_trace")
    tree = plan_to_tree_ir([{"action": "goto", "params": {"location": "kitchen"}}], namespace="task")
    same = plan_to_tree_ir([{"action": "goto", "params": {"location": "kitchen"}}], namespace="task")
    assert tree.to_dict() == same.to_dict()
    coordinator = TreeRevisionCoordinator(tree)
    proposal = _proposal(tree)
    assert coordinator.submit_proposal(proposal).status == "queued"
    result = coordinator.apply_pending()[0]
    assert result.status == "committed"
    assert coordinator.version == tree.version + 1
    assert len(coordinator.tree.nodes) == len(tree.nodes) + 1


def test_gateway_accepts_only_typed_proposals_and_checks_revision() -> None:
    pytest.importorskip("gpsr_trace")
    tree = plan_to_tree_ir([{"action": "announce", "params": {"text": "hello"}}])
    gateway = DebugControlGateway(TreeRevisionCoordinator(tree))
    assert gateway.handle({"command": "apply_patch", "payload": {"patch": {}}})["status"] == "rejected"
    proposal = _proposal(tree).to_dict()
    command = {"command": "activate_proposal", "expected_revision": tree.version, "payload": {"proposal": proposal}}
    result = gateway.handle(command)
    assert result["status"] == "applied"
    assert result["results"][0]["status"] == "committed"
    assert gateway.handle({"command": "pause"})["paused"] is True
