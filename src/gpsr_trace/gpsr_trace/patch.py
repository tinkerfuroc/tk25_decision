"""Typed, atomic patch operations for :mod:`gpsr_trace.ir` behavior trees."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

from .events import JsonValue, ValidationError, _require_identifier
from .ir import BehaviorTree, NodeSpec, NodeTypeRegistry, _freeze_json, thaw_json
from .metadata import ProposalMetadata


def _validate_index(index: int | None) -> None:
    if index is not None and (not isinstance(index, int) or index < 0):
        raise ValidationError("patch index must be a non-negative integer or None")


@dataclass(frozen=True)
class AddNode:
    """Add ``node`` as a child of ``parent_id`` at an optional child index."""

    node: NodeSpec
    parent_id: str | None
    index: int | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.node, NodeSpec):
            raise ValidationError("AddNode.node must be NodeSpec")
        if self.parent_id is not None:
            _require_identifier("parent_id", self.parent_id)
        _validate_index(self.index)

    def to_dict(self) -> dict[str, JsonValue]:
        return {"op": "add", "node": self.node.to_dict(), "parent_id": self.parent_id, "index": self.index}


@dataclass(frozen=True)
class RemoveNode:
    """Remove a leaf node, or a complete subtree when ``recursive`` is true."""

    node_id: str
    recursive: bool = False

    def __post_init__(self) -> None:
        _require_identifier("node_id", self.node_id)
        if not isinstance(self.recursive, bool):
            raise ValidationError("recursive must be boolean")

    def to_dict(self) -> dict[str, JsonValue]:
        return {"op": "remove", "node_id": self.node_id, "recursive": self.recursive}


@dataclass(frozen=True)
class ReplaceNode:
    """Replace a complete node declaration while preserving its stable id."""

    node: NodeSpec

    def __post_init__(self) -> None:
        if not isinstance(self.node, NodeSpec):
            raise ValidationError("ReplaceNode.node must be NodeSpec")

    def to_dict(self) -> dict[str, JsonValue]:
        return {"op": "replace", "node": self.node.to_dict()}


@dataclass(frozen=True)
class MoveNode:
    """Move one non-root node under a different parent."""

    node_id: str
    parent_id: str
    index: int | None = None

    def __post_init__(self) -> None:
        _require_identifier("node_id", self.node_id)
        _require_identifier("parent_id", self.parent_id)
        _validate_index(self.index)

    def to_dict(self) -> dict[str, JsonValue]:
        return {"op": "move", "node_id": self.node_id, "parent_id": self.parent_id, "index": self.index}


@dataclass(frozen=True)
class UpdateNode:
    """Merge parameter changes and/or change a node type without changing its id."""

    node_id: str
    params: Mapping[str, JsonValue] | None = None
    remove_params: tuple[str, ...] = ()
    node_type: str | None = None

    def __post_init__(self) -> None:
        _require_identifier("node_id", self.node_id)
        if self.node_type is not None:
            _require_identifier("node_type", self.node_type)
        if self.params is not None:
            if not isinstance(self.params, Mapping):
                raise ValidationError("update params must be a mapping or None")
            frozen = _freeze_json(dict(self.params), field_name="update params")
            object.__setattr__(self, "params", frozen)
        remove_params = tuple(self.remove_params)
        for name in remove_params:
            _require_identifier("parameter name", name)
        if len(set(remove_params)) != len(remove_params):
            raise ValidationError("remove_params may not contain duplicate names")
        object.__setattr__(self, "remove_params", remove_params)

    def to_dict(self) -> dict[str, JsonValue]:
        return {
            "op": "update",
            "node_id": self.node_id,
            "params": thaw_json(self.params) if self.params is not None else None,
            "remove_params": list(self.remove_params),
            "node_type": self.node_type,
        }


PatchOperation = AddNode | RemoveNode | ReplaceNode | MoveNode | UpdateNode
# Explicit aliases make the operation role clear at call sites that use an
# ``*Op`` naming convention.
AddNodeOp = AddNode
RemoveNodeOp = RemoveNode
ReplaceNodeOp = ReplaceNode
MoveNodeOp = MoveNode
UpdateNodeOp = UpdateNode


def operation_from_dict(value: Mapping[str, Any]) -> PatchOperation:
    """Parse one tagged patch operation from JSON-compatible data."""

    if not isinstance(value, Mapping) or not isinstance(value.get("op"), str):
        raise ValidationError("patch operation must be a mapping with an op field")
    operation = value["op"]
    try:
        if operation == "add":
            return AddNode(NodeSpec.from_dict(value["node"]), value.get("parent_id"), value.get("index"))
        if operation == "remove":
            return RemoveNode(value["node_id"], value.get("recursive", False))
        if operation == "replace":
            return ReplaceNode(NodeSpec.from_dict(value["node"]))
        if operation == "move":
            return MoveNode(value["node_id"], value["parent_id"], value.get("index"))
        if operation == "update":
            return UpdateNode(
                node_id=value["node_id"],
                params=value.get("params"),
                remove_params=tuple(value.get("remove_params", ())),
                node_type=value.get("node_type"),
            )
    except KeyError as exc:
        raise ValidationError(f"missing field for {operation!r} patch operation: {exc.args[0]}") from exc
    raise ValidationError(f"unknown patch operation: {operation!r}")


@dataclass(frozen=True)
class TreePatch:
    """An ordered, atomic set of typed edits for one tree version."""

    operations: tuple[PatchOperation, ...]
    base_version: int | None = None

    def __post_init__(self) -> None:
        operations = tuple(self.operations)
        if not operations:
            raise ValidationError("tree patch must contain at least one operation")
        if not all(isinstance(operation, (AddNode, RemoveNode, ReplaceNode, MoveNode, UpdateNode)) for operation in operations):
            raise ValidationError("tree patch contains an unsupported operation")
        object.__setattr__(self, "operations", operations)
        if self.base_version is not None and (
            not isinstance(self.base_version, int) or self.base_version < 0
        ):
            raise ValidationError("base_version must be a non-negative integer or None")

    def to_dict(self) -> dict[str, JsonValue]:
        return {"base_version": self.base_version, "operations": [op.to_dict() for op in self.operations]}

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "TreePatch":
        if not isinstance(value, Mapping) or set(value) != {"base_version", "operations"}:
            raise ValidationError("patch fields must be exactly base_version and operations")
        operations = value["operations"]
        if not isinstance(operations, list):
            raise ValidationError("patch operations must be a list")
        return cls(tuple(operation_from_dict(item) for item in operations), value["base_version"])


@dataclass(frozen=True)
class BehaviorTreeProposal:
    """A patch with the immutable metadata needed for review and audit."""

    metadata: ProposalMetadata
    patch: TreePatch

    def __post_init__(self) -> None:
        if not isinstance(self.metadata, ProposalMetadata):
            raise ValidationError("proposal metadata must be ProposalMetadata")
        if not isinstance(self.patch, TreePatch):
            raise ValidationError("proposal patch must be TreePatch")
        if (
            self.metadata.base_tree_version is not None
            and self.patch.base_version is not None
            and self.metadata.base_tree_version != self.patch.base_version
        ):
            raise ValidationError("proposal metadata and patch base versions disagree")

    def to_dict(self) -> dict[str, JsonValue]:
        return {"metadata": self.metadata.to_dict(), "patch": self.patch.to_dict()}


def _replace_children(node: NodeSpec, children: list[str]) -> NodeSpec:
    return NodeSpec(node.node_id, node.node_type, node.params, tuple(children))


def _insert_child(node: NodeSpec, child_id: str, index: int | None) -> NodeSpec:
    children = list(node.children)
    if child_id in children:
        raise ValidationError(f"node {child_id!r} is already a child of {node.id!r}")
    if index is None:
        children.append(child_id)
    elif index <= len(children):
        children.insert(index, child_id)
    else:
        raise ValidationError(f"child index {index} is outside parent {node.id!r}")
    return _replace_children(node, children)


def _parent_ids(nodes: Mapping[str, NodeSpec], child_id: str) -> list[str]:
    return [node_id for node_id, node in nodes.items() if child_id in node.children]


def _subtree_ids(nodes: Mapping[str, NodeSpec], node_id: str) -> set[str]:
    result: set[str] = set()
    pending = [node_id]
    while pending:
        current = pending.pop()
        if current in result:
            continue
        result.add(current)
        pending.extend(nodes[current].children)
    return result


def apply_patch(
    tree: BehaviorTree,
    patch: TreePatch,
    registry: NodeTypeRegistry | None = None,
) -> BehaviorTree:
    """Apply all edits atomically and validate the resulting immutable tree.

    The input tree is never mutated.  Any invalid operation or failed final
    structural/schema validation raises :class:`ValidationError` and leaves it
    untouched.
    """

    if not isinstance(tree, BehaviorTree):
        raise ValidationError("tree must be BehaviorTree")
    if not isinstance(patch, TreePatch):
        raise ValidationError("patch must be TreePatch")
    if patch.base_version is not None and patch.base_version != tree.version:
        raise ValidationError(
            f"patch base version {patch.base_version} does not match tree version {tree.version}"
        )

    nodes = dict(tree.nodes)
    root_id = tree.root_id
    for operation in patch.operations:
        if isinstance(operation, AddNode):
            if operation.node.id in nodes:
                raise ValidationError(f"cannot add duplicate node id {operation.node.id!r}")
            if operation.parent_id is None:
                raise ValidationError("adding a second root is not supported; use ReplaceNode for the root")
            if operation.parent_id not in nodes:
                raise ValidationError(f"add parent does not exist: {operation.parent_id!r}")
            nodes[operation.node.id] = operation.node
            nodes[operation.parent_id] = _insert_child(
                nodes[operation.parent_id], operation.node.id, operation.index
            )
        elif isinstance(operation, RemoveNode):
            if operation.node_id not in nodes:
                raise ValidationError(f"cannot remove unknown node {operation.node_id!r}")
            if operation.node_id == root_id:
                raise ValidationError("removing the root node is not supported")
            children = nodes[operation.node_id].children
            if children and not operation.recursive:
                raise ValidationError("cannot remove a non-leaf node without recursive=True")
            parents = _parent_ids(nodes, operation.node_id)
            if len(parents) != 1:
                raise ValidationError(f"node {operation.node_id!r} does not have exactly one parent")
            parent = nodes[parents[0]]
            nodes[parent.id] = _replace_children(parent, [child for child in parent.children if child != operation.node_id])
            remove_ids = _subtree_ids(nodes, operation.node_id) if operation.recursive else {operation.node_id}
            for node_id in remove_ids:
                nodes.pop(node_id, None)
        elif isinstance(operation, ReplaceNode):
            if operation.node.id not in nodes:
                raise ValidationError(f"cannot replace unknown node {operation.node.id!r}")
            nodes[operation.node.id] = operation.node
        elif isinstance(operation, MoveNode):
            if operation.node_id not in nodes:
                raise ValidationError(f"cannot move unknown node {operation.node_id!r}")
            if operation.node_id == root_id:
                raise ValidationError("cannot move the root node")
            if operation.parent_id not in nodes:
                raise ValidationError(f"move parent does not exist: {operation.parent_id!r}")
            parents = _parent_ids(nodes, operation.node_id)
            if len(parents) != 1:
                raise ValidationError(f"node {operation.node_id!r} does not have exactly one parent")
            old_parent = nodes[parents[0]]
            nodes[old_parent.id] = _replace_children(
                old_parent, [child for child in old_parent.children if child != operation.node_id]
            )
            nodes[operation.parent_id] = _insert_child(
                nodes[operation.parent_id], operation.node_id, operation.index
            )
        elif isinstance(operation, UpdateNode):
            if operation.node_id not in nodes:
                raise ValidationError(f"cannot update unknown node {operation.node_id!r}")
            current = nodes[operation.node_id]
            params = thaw_json(current.params)
            for parameter in operation.remove_params:
                params.pop(parameter, None)
            if operation.params is not None:
                params.update(thaw_json(operation.params))
            nodes[operation.node_id] = NodeSpec(
                current.id,
                operation.node_type if operation.node_type is not None else current.node_type,
                params,
                current.children,
            )
        else:  # pragma: no cover - TreePatch prevents this, retained for defensive use.
            raise ValidationError("unsupported patch operation")

    result = BehaviorTree(root_id=root_id, nodes=nodes, version=tree.version + 1)
    return result.validate(registry)
