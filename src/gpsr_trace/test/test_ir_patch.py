import pytest

from gpsr_trace import (
    AddNode,
    AgentMetadata,
    BehaviorTree,
    BehaviorTreeProposal,
    MoveNode,
    NodeSpec,
    NodeTypeRegistry,
    NodeTypeSchema,
    ParamSpec,
    ProposalMetadata,
    TreePatch,
    UpdateNode,
    ValidationError,
    apply_patch,
    stable_node_id,
)


def _tree_and_registry() -> tuple[BehaviorTree, NodeTypeRegistry]:
    root = NodeSpec("root", "sequence", {}, ("say-a", "branch"))
    say_a = NodeSpec("say-a", "say", {"text": "Hello"})
    branch = NodeSpec("branch", "sequence", {}, ("say-b",))
    say_b = NodeSpec("say-b", "say", {"text": "Welcome"})
    tree = BehaviorTree("root", {node.id: node for node in (root, say_a, branch, say_b)})
    registry = NodeTypeRegistry(
        {
            "sequence": NodeTypeSchema("sequence"),
            "say": NodeTypeSchema("say", {"text": ParamSpec(str, required=True)}, max_children=0),
        }
    )
    return tree.validate(registry), registry


def test_stable_ids_and_immutable_tree_schema_validation() -> None:
    assert stable_node_id("gpsr", "root", "say") == stable_node_id("gpsr", "root", "say")
    tree, registry = _tree_and_registry()
    assert tree.nodes["say-a"].params["text"] == "Hello"
    with pytest.raises(TypeError):
        tree.nodes["say-a"].params["text"] = "changed"  # type: ignore[index]
    with pytest.raises(ValidationError, match="unregistered"):
        BehaviorTree("root", {"root": NodeSpec("root", "unknown")}).validate(registry)


def test_patch_is_atomic_and_validates_structure_and_schema() -> None:
    tree, registry = _tree_and_registry()
    patch = TreePatch(
        (
            MoveNode("say-a", "branch", index=1),
            UpdateNode("say-b", params={"text": "Please follow me"}),
            AddNode(NodeSpec("say-c", "say", {"text": "Done"}), "root"),
        ),
        base_version=1,
    )

    changed = apply_patch(tree, patch, registry)

    assert changed.version == 2
    assert changed.nodes["root"].children == ("branch", "say-c")
    assert changed.nodes["branch"].children == ("say-b", "say-a")
    assert changed.nodes["say-b"].params["text"] == "Please follow me"
    assert tree.nodes["root"].children == ("say-a", "branch")

    invalid = TreePatch((AddNode(NodeSpec("bad", "unknown"), "root"),), base_version=1)
    with pytest.raises(ValidationError, match="unregistered"):
        apply_patch(tree, invalid, registry)
    assert "bad" not in tree.nodes


def test_structural_validation_and_proposal_metadata() -> None:
    with pytest.raises(ValidationError, match="missing child"):
        BehaviorTree("root", {"root": NodeSpec("root", "sequence", children=("missing",))})

    agent = AgentMetadata("planner", "planning-agent", capabilities=("tree-patch",))
    proposal = BehaviorTreeProposal(
        ProposalMetadata.create(agent=agent, base_tree_version=1, summary="Clarify greeting"),
        TreePatch((UpdateNode("say-a", params={"text": "Hi"}),), base_version=1),
    )
    assert proposal.to_dict()["metadata"]["agent"]["agent_id"] == "planner"
