"""Shared semantic GPSR behaviour-tree serialisation.

The planned tree is built only for visualisation while the executor tree is
the dispatcher that actually ticks on the robot.  They need the same stable
node identifiers and semantic metadata so debugger clients can describe either
tree without knowing GPSR implementation classes.
"""
from __future__ import annotations

from typing import Any, Mapping


def serialize_tree(root: Any, *, kind: str, label: str | None = None) -> dict[str, Any]:
    """Return a JSON-safe, path-stable topology for a ``py_trees`` root.

    This module deliberately uses introspection instead of importing
    ``py_trees``.  Planned-tree rendering and the live telemetry producer can
    therefore share it without adding an import-time runtime dependency.
    """

    nodes: list[dict[str, Any]] = []
    edges: list[dict[str, Any]] = []

    def walk(
        node: Any,
        parent_id: str | None,
        path: str,
        order: int,
        inherited_action: Mapping[str, Any] | None,
    ) -> None:
        node_id = stable_node_id(kind, path)
        action_context = _action_context(node, inherited_action)
        children = list(getattr(node, "children", ()) or ())
        child_ids = [stable_node_id(kind, f"{path}/{index}") for index, _child in enumerate(children)]
        semantics = _semantics(node, children, child_ids)
        node_record = {
            # Keep the original fields consumers already use.
            "id": node_id,
            "node_id": node_id,
            "parent_id": parent_id,
            "name": _text(getattr(node, "name", type(node).__name__)),
            "type": type(node).__name__,
            "status": _status_name(getattr(node, "status", None)),
            "children": child_ids,
            # New semantic debugger contract.
            "order": order,
            # ``node_class`` deliberately classifies behaviour-tree flow; the
            # Python implementation type stays in ``type`` for diagnostics.
            "node_class": semantics["category"],
            "semantics": semantics,
            "blackboard_access": _blackboard_access(node),
            "action_context": action_context,
        }
        nodes.append(node_record)
        for index, child in enumerate(children):
            child_id = stable_node_id(kind, f"{path}/{index}")
            edges.append({"source": node_id, "target": child_id, "order": index})
            walk(child, node_id, f"{path}/{index}", len(nodes), action_context)

    walk(root, None, "root", 0, None)
    document: dict[str, Any] = {
        "tree_document_version": 2,
        "kind": kind,
        "root_id": stable_node_id(kind, "root"),
        "nodes": nodes,
        "edges": edges,
    }
    if label is not None:
        document["label"] = _text(label)
    return document


def stable_node_id(kind: str, path: str) -> str:
    """Return the stable structural id used by both tree representations."""

    return f"{kind}/{path}"


def runtime_counters(node: Any) -> dict[str, Any]:
    """Extract Retry/Repeat counters without coupling to a py_trees version."""

    if hasattr(node, "failures") and hasattr(node, "num_failures"):
        return {
            "kind": "retry",
            "counter": "failures",
            "value": _integer(getattr(node, "failures", None)),
            "limit": _integer(getattr(node, "num_failures", None)),
        }
    if hasattr(node, "success") and hasattr(node, "num_success"):
        return {
            "kind": "repeat",
            "counter": "successes",
            "value": _integer(getattr(node, "success", None)),
            "limit": _integer(getattr(node, "num_success", None)),
        }
    return {}


def _semantics(node: Any, children: list[Any], child_ids: list[str]) -> dict[str, Any]:
    class_name = type(node).__name__.lower()
    category = "leaf"
    kind = "leaf"
    if "retry" in class_name or (hasattr(node, "failures") and hasattr(node, "num_failures")):
        category, kind = "decorator", "retry"
    elif "repeat" in class_name or (hasattr(node, "success") and hasattr(node, "num_success")):
        category, kind = "decorator", "repeat"
    elif "selector" in class_name or "choice" in class_name:
        category, kind = "composite", "selector"
    elif "sequence" in class_name:
        category, kind = "composite", "sequence"
    elif "parallel" in class_name:
        category, kind = "composite", "parallel"
    elif "decorator" in class_name or hasattr(node, "decorated"):
        category, kind = "decorator", _snake_case(type(node).__name__)
    elif children:
        category, kind = "composite", _snake_case(type(node).__name__)

    result: dict[str, Any] = {
        "category": category,
        "kind": kind,
        # Compatibility for clients written before ``kind`` was formalised.
        "control_flow": kind,
    }
    memory = getattr(node, "memory", None)
    if isinstance(memory, bool):
        result["memory"] = memory
    counters = runtime_counters(node)
    if counters:
        result["counter"] = {key: value for key, value in counters.items() if key != "value"}
    if kind == "parallel":
        result.update(_parallel_semantics(node, children, child_ids))
    if category == "decorator":
        result.update(_decorator_semantics(node, kind))
    return result


def _parallel_semantics(node: Any, children: list[Any], child_ids: list[str]) -> dict[str, Any]:
    policy = getattr(node, "policy", None)
    policy_name = type(policy).__name__.lower() if policy is not None else ""
    if "selected" in policy_name:
        success_policy = "selected"
    elif "one" in policy_name:
        success_policy = "one"
    else:
        success_policy = "all"
    result: dict[str, Any] = {
        "parallel_policy": success_policy,
        "synchronise": bool(getattr(policy, "synchronise", False)),
    }
    if success_policy == "selected":
        by_object_id = {id(child): child_ids[index] for index, child in enumerate(children)}
        result["selected_child_ids"] = [
            by_object_id[id(child)]
            for child in getattr(policy, "children", ()) or ()
            if id(child) in by_object_id
        ]
    return result


def _decorator_semantics(node: Any, kind: str) -> dict[str, Any]:
    result: dict[str, Any] = {}
    status_mapping = _status_mapping(kind)
    if status_mapping is not None:
        result["status_mapping"] = status_mapping
    if hasattr(node, "duration"):
        result["timeout"] = {
            "duration_s": getattr(node, "duration"),
            "on_timeout": "FAILURE",
        }
    if hasattr(node, "succeed_status"):
        result["condition"] = {
            "succeed_status": _status_name(getattr(node, "succeed_status")),
            "otherwise": "RUNNING",
        }
    if hasattr(node, "policy") and kind == "one_shot":
        result["oneshot"] = {
            "policy": _enum_name(getattr(node, "policy")),
            "final_status": _status_name(getattr(node, "final_status")) if getattr(node, "final_status", None) else None,
        }
    if hasattr(node, "source_key") or hasattr(node, "target_key"):
        result["foreach"] = {
            "source_key": _text(getattr(node, "source_key", "")),
            "target_key": _text(getattr(node, "target_key", "")),
        }
    return result


def _status_mapping(kind: str) -> dict[str, str] | None:
    passthrough = {"SUCCESS": "SUCCESS", "FAILURE": "FAILURE", "RUNNING": "RUNNING", "INVALID": "INVALID"}
    changed = {
        "failure_is_success": ("FAILURE", "SUCCESS"),
        "failure_is_running": ("FAILURE", "RUNNING"),
        "success_is_failure": ("SUCCESS", "FAILURE"),
        "success_is_running": ("SUCCESS", "RUNNING"),
        "running_is_failure": ("RUNNING", "FAILURE"),
        "running_is_success": ("RUNNING", "SUCCESS"),
        "inverter": ("SUCCESS", "FAILURE"),
    }
    if kind not in changed:
        return None
    result = dict(passthrough)
    source, target = changed[kind]
    result[source] = target
    if kind == "inverter":
        result["FAILURE"] = "SUCCESS"
    return result


def _blackboard_access(node: Any) -> dict[str, list[str]]:
    access = {"read": set(), "write": set(), "exclusive": set()}
    for client in getattr(node, "blackboards", ()) or ():
        for name in access:
            for key in getattr(client, name, ()) or ():
                access[name].add(_text(key))
    return {name: sorted(values) for name, values in access.items()}


def _action_context(node: Any, inherited: Mapping[str, Any] | None) -> dict[str, Any]:
    explicit = getattr(node, "_gpsr_action_context", None)
    if isinstance(explicit, Mapping):
        return {**_json_value(explicit), "boundary": True}
    name = _text(getattr(node, "name", ""))
    if name.startswith("branch:"):
        return {"action": name.partition(":")[2], "boundary": True}
    if inherited:
        return {**_json_value(inherited), "boundary": False}
    expected = getattr(node, "_expected", None)
    if isinstance(expected, str) and expected:
        return {"action": expected, "boundary": False}
    return {}


def _status_name(status: Any) -> str:
    value = getattr(status, "name", status)
    return _text(value or "INVALID")


def _integer(value: Any) -> int | None:
    return value if isinstance(value, int) and not isinstance(value, bool) else None


def _text(value: Any) -> str:
    text = str(value)
    return text if len(text) <= 1024 else text[:1021] + "..."


def _snake_case(value: str) -> str:
    result: list[str] = []
    for index, char in enumerate(value):
        if char.isupper() and index and (not value[index - 1].isupper()):
            result.append("_")
        result.append(char.lower())
    return "".join(result)


def _enum_name(value: Any) -> str:
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name.lower()
    return _snake_case(type(value).__name__)


def _json_value(value: Any, depth: int = 0) -> Any:
    if depth > 6:
        return "<max-depth>"
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Mapping):
        return {str(key): _json_value(item, depth + 1) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_json_value(item, depth + 1) for item in value]
    return _text(value)


__all__ = ["runtime_counters", "serialize_tree", "stable_node_id"]
