"""Lightweight GPSR trace transport and declarative behavior-tree tooling."""

from .client import BoundedSpool, TraceClient, TraceClientStats
from .events import (
    TRACE_EVENT_VERSION,
    ArtifactReference,
    ContentPolicy,
    RedactionPolicy,
    TraceEvent,
    ValidationError,
)
from .ir import BehaviorTree, NodeSpec, NodeTypeRegistry, NodeTypeSchema, ParamSpec, stable_node_id
from .metadata import AgentMetadata, ProposalMetadata
from .patch import (
    AddNode,
    AddNodeOp,
    BehaviorTreeProposal,
    MoveNode,
    MoveNodeOp,
    RemoveNode,
    RemoveNodeOp,
    ReplaceNode,
    ReplaceNodeOp,
    TreePatch,
    UpdateNode,
    UpdateNodeOp,
    apply_patch,
    operation_from_dict,
)

__all__ = [
    "TRACE_EVENT_VERSION",
    "AddNode",
    "AddNodeOp",
    "AgentMetadata",
    "ArtifactReference",
    "BehaviorTree",
    "BehaviorTreeProposal",
    "BoundedSpool",
    "ContentPolicy",
    "MoveNode",
    "MoveNodeOp",
    "NodeSpec",
    "NodeTypeRegistry",
    "NodeTypeSchema",
    "ParamSpec",
    "ProposalMetadata",
    "RedactionPolicy",
    "RemoveNode",
    "RemoveNodeOp",
    "ReplaceNode",
    "ReplaceNodeOp",
    "TraceClient",
    "TraceClientStats",
    "TraceEvent",
    "TreePatch",
    "UpdateNode",
    "UpdateNodeOp",
    "ValidationError",
    "apply_patch",
    "operation_from_dict",
    "stable_node_id",
]
