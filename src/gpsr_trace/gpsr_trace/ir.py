"""Immutable declarative behavior-tree intermediate representation (IR)."""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
from types import MappingProxyType
from typing import Any, Callable, Mapping

from .events import JsonValue, ValidationError, _copy_json, _require_identifier


def _freeze_json(value: Any, *, field_name: str) -> Any:
    """Deep-freeze a JSON value, detaching it from caller-owned containers."""

    copied = _copy_json(value, field_name=field_name)

    def freeze(item: Any) -> Any:
        if isinstance(item, dict):
            return MappingProxyType({key: freeze(child) for key, child in item.items()})
        if isinstance(item, list):
            return tuple(freeze(child) for child in item)
        return item

    return freeze(copied)


def thaw_json(value: Any) -> JsonValue:
    """Return a detached normal JSON value from an IR field."""

    def thaw(item: Any) -> JsonValue:
        if isinstance(item, Mapping):
            return {str(key): thaw(child) for key, child in item.items()}
        if isinstance(item, tuple):
            return [thaw(child) for child in item]
        return item

    return thaw(value)


def stable_node_id(namespace: str, *parts: str) -> str:
    """Create a deterministic node id from a logical, stable location.

    The caller supplies a namespace and path-like parts; the hash makes the id
    safe for transport while keeping it independent of process-local object
    identity or patch ordering.
    """

    _require_identifier("node id namespace", namespace)
    if not parts or any(not isinstance(part, str) or not part for part in parts):
        raise ValidationError("stable node id requires one or more non-empty string parts")
    encoded = json.dumps([namespace, *parts], ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    return "node_" + hashlib.sha256(encoded).hexdigest()[:24]


@dataclass(frozen=True)
class NodeSpec:
    """An immutable node declaration; edges are stored by child node id."""

    node_id: str
    node_type: str
    params: Mapping[str, JsonValue] = field(default_factory=dict)
    children: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        _require_identifier("node_id", self.node_id)
        _require_identifier("node_type", self.node_type)
        if not isinstance(self.params, Mapping):
            raise ValidationError("node params must be a mapping")
        frozen_params = _freeze_json(dict(self.params), field_name="node params")
        if not isinstance(frozen_params, Mapping):  # Defensive, ``dict`` is always an object.
            raise ValidationError("node params must be a JSON object")
        object.__setattr__(self, "params", frozen_params)
        children = tuple(self.children)
        for child in children:
            _require_identifier("child node id", child)
        if len(set(children)) != len(children):
            raise ValidationError(f"node {self.node_id!r} has duplicate children")
        object.__setattr__(self, "children", children)

    @property
    def id(self) -> str:
        """Alias convenient for serializers and behavior-tree tooling."""

        return self.node_id

    @classmethod
    def create(
        cls,
        *,
        node_type: str,
        stable_namespace: str,
        stable_path: tuple[str, ...] | list[str],
        params: Mapping[str, JsonValue] | None = None,
        children: tuple[str, ...] | list[str] = (),
    ) -> "NodeSpec":
        """Create a node whose id is deterministically derived from its path."""

        return cls(
            node_id=stable_node_id(stable_namespace, *tuple(stable_path)),
            node_type=node_type,
            params=params or {},
            children=tuple(children),
        )

    def to_dict(self) -> dict[str, JsonValue]:
        return {
            "id": self.node_id,
            "type": self.node_type,
            "params": thaw_json(self.params),
            "children": list(self.children),
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "NodeSpec":
        if not isinstance(value, Mapping):
            raise ValidationError("node specification must be a mapping")
        expected = {"id", "type", "params", "children"}
        unknown = set(value) - expected
        missing = expected - set(value)
        if unknown or missing:
            raise ValidationError("node fields must be exactly id, type, params, children")
        return cls(
            node_id=value["id"],
            node_type=value["type"],
            params=value["params"],
            children=tuple(value["children"]),
        )


ParamValidator = Callable[[JsonValue], bool]


@dataclass(frozen=True)
class ParamSpec:
    """Schema for a node parameter registered in :class:`NodeTypeRegistry`."""

    types: type | tuple[type, ...]
    required: bool = False
    validator: ParamValidator | None = None

    def __post_init__(self) -> None:
        types = self.types if isinstance(self.types, tuple) else (self.types,)
        if not types or any(not isinstance(item, type) for item in types):
            raise ValidationError("parameter types must contain Python types")
        if not isinstance(self.required, bool):
            raise ValidationError("parameter required must be boolean")
        if self.validator is not None and not callable(self.validator):
            raise ValidationError("parameter validator must be callable")
        object.__setattr__(self, "types", types)

    def accepts(self, value: JsonValue) -> bool:
        return isinstance(value, self.types) and (self.validator is None or bool(self.validator(value)))


def _normalise_param_spec(value: ParamSpec | type | tuple[type, ...]) -> ParamSpec:
    return value if isinstance(value, ParamSpec) else ParamSpec(value)


@dataclass(frozen=True)
class NodeTypeSchema:
    """Registration-time constraints for one behavior-tree node type."""

    name: str
    params: Mapping[str, ParamSpec | type | tuple[type, ...]] = field(default_factory=dict)
    allow_additional_params: bool = False
    min_children: int = 0
    max_children: int | None = None

    def __post_init__(self) -> None:
        _require_identifier("node type name", self.name)
        if not isinstance(self.params, Mapping):
            raise ValidationError("node type params must be a mapping")
        normalised: dict[str, ParamSpec] = {}
        for name, specification in self.params.items():
            _require_identifier("parameter name", name)
            normalised[name] = _normalise_param_spec(specification)
        object.__setattr__(self, "params", MappingProxyType(normalised))
        if not isinstance(self.allow_additional_params, bool):
            raise ValidationError("allow_additional_params must be boolean")
        if not isinstance(self.min_children, int) or self.min_children < 0:
            raise ValidationError("min_children must be a non-negative integer")
        if self.max_children is not None and (
            not isinstance(self.max_children, int) or self.max_children < self.min_children
        ):
            raise ValidationError("max_children must be at least min_children or None")

    def validate(self, node: NodeSpec) -> None:
        if len(node.children) < self.min_children:
            raise ValidationError(f"node {node.id!r} requires at least {self.min_children} children")
        if self.max_children is not None and len(node.children) > self.max_children:
            raise ValidationError(f"node {node.id!r} allows at most {self.max_children} children")
        unknown = set(node.params) - set(self.params)
        if unknown and not self.allow_additional_params:
            raise ValidationError(
                f"node {node.id!r} has unregistered parameters: {', '.join(sorted(unknown))}"
            )
        for name, specification in self.params.items():
            if specification.required and name not in node.params:
                raise ValidationError(f"node {node.id!r} is missing required parameter {name!r}")
            if name in node.params and not specification.accepts(node.params[name]):
                accepted = ", ".join(item.__name__ for item in specification.types)
                raise ValidationError(
                    f"node {node.id!r} parameter {name!r} must satisfy registered type(s): {accepted}"
                )


@dataclass(frozen=True)
class NodeTypeRegistry:
    """An immutable registry of behavior-tree node types and parameter schemas."""

    schemas: Mapping[str, NodeTypeSchema] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.schemas, Mapping):
            raise ValidationError("node registry schemas must be a mapping")
        normalised: dict[str, NodeTypeSchema] = {}
        for name, schema in self.schemas.items():
            if not isinstance(schema, NodeTypeSchema):
                raise ValidationError("node registry values must be NodeTypeSchema")
            if name != schema.name:
                raise ValidationError("node registry key must match NodeTypeSchema.name")
            normalised[name] = schema
        object.__setattr__(self, "schemas", MappingProxyType(normalised))

    def with_schema(self, schema: NodeTypeSchema) -> "NodeTypeRegistry":
        """Return a registry extended or replaced with *schema*."""

        if not isinstance(schema, NodeTypeSchema):
            raise ValidationError("schema must be NodeTypeSchema")
        schemas = dict(self.schemas)
        schemas[schema.name] = schema
        return NodeTypeRegistry(schemas)

    def register(self, schema: NodeTypeSchema) -> "NodeTypeRegistry":
        """Alias for :meth:`with_schema` for registry-building call sites."""

        return self.with_schema(schema)

    def schema_for(self, node_type: str) -> NodeTypeSchema:
        try:
            return self.schemas[node_type]
        except KeyError as exc:
            raise ValidationError(f"unregistered node type: {node_type!r}") from exc

    def validate_node(self, node: NodeSpec) -> None:
        self.schema_for(node.node_type).validate(node)


def _validate_structure(root_id: str, nodes: Mapping[str, NodeSpec]) -> None:
    if not nodes:
        raise ValidationError("behavior tree must contain at least its root node")
    if root_id not in nodes:
        raise ValidationError("root_id must identify a node in the tree")

    parents: dict[str, list[str]] = {node_id: [] for node_id in nodes}
    for node_id, node in nodes.items():
        if not isinstance(node, NodeSpec):
            raise ValidationError("behavior tree nodes must be NodeSpec values")
        if node_id != node.id:
            raise ValidationError("behavior tree mapping keys must match NodeSpec ids")
        for child_id in node.children:
            if child_id not in nodes:
                raise ValidationError(f"node {node_id!r} references missing child {child_id!r}")
            if child_id == node_id:
                raise ValidationError(f"node {node_id!r} cannot be its own child")
            parents[child_id].append(node_id)

    if parents[root_id]:
        raise ValidationError("root node must not have a parent")
    for node_id, node_parents in parents.items():
        if node_id != root_id and len(node_parents) != 1:
            raise ValidationError(f"non-root node {node_id!r} must have exactly one parent")

    state: dict[str, int] = {}

    def visit(node_id: str) -> None:
        color = state.get(node_id, 0)
        if color == 1:
            raise ValidationError(f"behavior tree contains a cycle at node {node_id!r}")
        if color == 2:
            return
        state[node_id] = 1
        for child_id in nodes[node_id].children:
            visit(child_id)
        state[node_id] = 2

    visit(root_id)
    unreachable = set(nodes) - set(state)
    if unreachable:
        raise ValidationError(
            "every node must be reachable from root; unreachable: " + ", ".join(sorted(unreachable))
        )


@dataclass(frozen=True)
class BehaviorTree:
    """A versioned immutable behavior-tree graph rooted at ``root_id``."""

    root_id: str
    nodes: Mapping[str, NodeSpec]
    version: int = 1

    def __post_init__(self) -> None:
        _require_identifier("root_id", self.root_id)
        if not isinstance(self.version, int) or self.version < 0:
            raise ValidationError("tree version must be a non-negative integer")
        if not isinstance(self.nodes, Mapping):
            raise ValidationError("tree nodes must be a mapping")
        copied = dict(self.nodes)
        _validate_structure(self.root_id, copied)
        object.__setattr__(self, "nodes", MappingProxyType(copied))

    def validate(self, registry: NodeTypeRegistry | None = None) -> "BehaviorTree":
        """Validate structural invariants and, if supplied, node schemas."""

        _validate_structure(self.root_id, self.nodes)
        if registry is not None:
            if not isinstance(registry, NodeTypeRegistry):
                raise ValidationError("registry must be NodeTypeRegistry or None")
            for node in self.nodes.values():
                registry.validate_node(node)
        return self

    def to_dict(self) -> dict[str, JsonValue]:
        return {
            "version": self.version,
            "root_id": self.root_id,
            "nodes": {node_id: node.to_dict() for node_id, node in self.nodes.items()},
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "BehaviorTree":
        if not isinstance(value, Mapping):
            raise ValidationError("behavior tree must be a mapping")
        if set(value) != {"version", "root_id", "nodes"}:
            raise ValidationError("tree fields must be exactly version, root_id, nodes")
        raw_nodes = value["nodes"]
        if not isinstance(raw_nodes, Mapping):
            raise ValidationError("tree nodes must be a mapping")
        return cls(
            root_id=value["root_id"],
            version=value["version"],
            nodes={node_id: NodeSpec.from_dict(node) for node_id, node in raw_nodes.items()},
        )
