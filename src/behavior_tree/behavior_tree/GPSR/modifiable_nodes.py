"""Plan-time, template-constrained modifications to the GPSR small trees.

The lower-layer planner may decide the generic small tree cannot express the
behaviour a target needs (e.g. a "person in a red jacket" wants a colour
specialist branch, not the generic descriptor scan). Rather than let the LLM
emit arbitrary node structures, it returns a list of TYPED modification
directives (``validate_modifications``). Each directive names one closed
template; a TRUSTED handler in :data:`REGISTRY` applies it to the target
step's small tree at plan-build time.

Safety invariant (mirrors ``supervision``): the LLM supplies a template name,
a target node id, and parameters against a closed schema — never code. Handlers
are registered here, validate params against ``TemplateSpec.params``, and fall
back to the unmodified tree on any error. Applied once, before swap-in; never
mutates a live tree.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Mapping

from . import small_trees
from .custom_nodes import BtNode_VLMQuery
from .tree_serialization import serialize_tree

try:  # gpsr_trace is optional (allows old installs to import the GPSR module)
    from gpsr_trace.ir import ParamSpec
    from gpsr_trace.events import ValidationError
except ImportError:  # pragma: no cover
    class ParamSpec:  # type: ignore[no-redef]
        def __init__(self, types, required=False, validator=None):  # noqa: ANN001
            self.types = types
            self.required = required

    class ValidationError(ValueError):
        pass


class ModificationValidationError(ValidationError):
    """A modification directive failed validation and must be rejected."""


# ---------------------------------------------------------------------------
# Template parameter schemas (closed, JSON-safe types)
# ---------------------------------------------------------------------------

def _nonempty_str(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _vlm_template_str(value: Any) -> bool:
    """A VLM question template must be non-empty AND keep the ``{value}``
    placeholder the node fills at runtime."""
    return _nonempty_str(value) and "{value}" in value


def _str_list(value: Any) -> bool:
    return isinstance(value, list) and all(isinstance(item, (int, float)) for item in value)


#: Common float-list schemas reused by several templates.
FLOAT_LIST = ParamSpec((list, tuple))
TILT_LIST = ParamSpec((list, tuple), validator=_str_list)
PAN_LIST = ParamSpec((list, tuple), validator=_str_list)


# ---------------------------------------------------------------------------
# Template specs — the closed set the LLM may emit
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class TemplateSpec:
    """A closed modification template the lower-layer planner may apply.

    ``applies_to`` names the node roles (see ``small_trees.SMALL_TREE_ROLES``)
    this template may target; ``params`` is the closed parameter schema. A
    directive names the template, a ``target_node_id`` (a serialized node id),
    the parameters, and a free-text ``reason`` (why the modification is needed)
    — the reason becomes part of the ``(template, node_id, reason)`` audit
    tuple. Validation mirrors ``gpsr_trace.ir.NodeTypeSchema``.
    """

    name: str
    applies_to: tuple[str, ...]
    params: Mapping[str, ParamSpec] = field(default_factory=dict)
    description: str = ""

    def validate_params(self, params: Mapping[str, Any]) -> None:
        unknown = set(params) - set(self.params)
        if unknown:
            raise ModificationValidationError(
                f"template {self.name!r}: unregistered params: {', '.join(sorted(unknown))}"
            )
        for name, spec in self.params.items():
            if spec.required and name not in params:
                raise ModificationValidationError(
                    f"template {self.name!r}: missing required param {name!r}"
                )
            if name in params and not spec.accepts(params[name]):
                raise ModificationValidationError(
                    f"template {self.name!r}: param {name!r} rejected by schema"
                )


# ---------------------------------------------------------------------------
# Trusted handlers — trusted code that rebuilds the target subtree.
# ---------------------------------------------------------------------------

Handler = Callable[[Any, str, Mapping[str, Any], str], Any]
#   (subtree_root, action, params, target_node_id) -> new subtree root


def _nested_role_for(action: str, target_id: str) -> str | None:
    """Map a serialized node id of ``action``'s tree to a role, if any."""
    roles = small_trees.get_small_tree_roles()
    for role, ids in roles.items():
        if target_id in ids:
            return role
    return None


def _walk_to_node(root: Any, action: str, target_id: str) -> Any | None:
    """Resolve a serialized node id (``small/<action>/root/0/1``) to the live
    node. Works because a factory rebuild produces the SAME topology, hence the
    SAME structural ids, as the original tree."""
    prefix = f"small/{action}/root"
    if not target_id.startswith(prefix):
        return None
    rest = target_id[len(prefix):].strip("/")
    if not rest:
        return root
    node = root
    for part in rest.split("/"):
        try:
            index = int(part)
            node = node.children[index]
        except (ValueError, IndexError, AttributeError):
            return None
    return node


def _handler_attribute_person_specialist(root: Any, action: str, params: Mapping[str, Any], target_id: str) -> Any:
    if action != "find_person":
        raise ModificationValidationError("attribute-person-specialist only applies to find_person")
    gate = str(params.get("gate") or "").strip()
    prompt = str(params.get("prompt") or "").strip()
    if not gate:
        raise ModificationValidationError("attribute-person-specialist requires a non-empty 'gate'")
    return small_trees.create_find_person(
        extra_specialist={"gate": gate, "prompt": prompt or f"person {gate}"},
        pan_deg=params.get("pan_deg"),
        tilt_deg=params.get("tilt_deg"),
    )


def _handler_person_specialist(root: Any, action: str, params: Mapping[str, Any], target_id: str) -> Any:
    """Pin the person scan to a named-person prompt (bare names are not
    visually detectable, so this pins the descriptor for the generalist)."""
    if action != "find_person":
        raise ModificationValidationError("person-specialist only applies to find_person")
    name = str(params.get("name") or "").strip()
    if not name:
        raise ModificationValidationError("person-specialist requires a non-empty 'name'")
    return small_trees.create_find_person(
        extra_specialist={"gate": name, "prompt": f"person {name}"},
        pan_deg=params.get("pan_deg"),
        tilt_deg=params.get("tilt_deg"),
    )


def _handler_vlm_template(root: Any, action: str, params: Mapping[str, Any], target_id: str) -> Any:
    """Swap the question template on the TARGETED BtNode_VLMQuery.

    Rebuilds via the factory (same topology/ids), resolves ``target_id`` to the
    live query node, and pins ``_template`` on exactly that node — so the diff
    is a single changed node. The template must keep the ``{value}``
    placeholder the node fills at runtime.
    """
    template = str(params.get("question_template") or "").strip()
    if not template:
        raise ModificationValidationError("vlm-template requires a non-empty 'question_template'")
    if "{value}" not in template:
        raise ModificationValidationError(
            "vlm-template question_template must contain the {value} placeholder"
        )
    factory = small_trees.ACTION_FACTORIES.get(action)
    if factory is None:
        raise ModificationValidationError(f"vlm-template: unknown action {action!r}")
    rebuilt = factory()
    node = _walk_to_node(rebuilt, action, target_id)
    if node is None or not isinstance(node, BtNode_VLMQuery):
        raise ModificationValidationError(
            f"vlm-template: target {target_id!r} is not a VLM query node"
        )
    node._template = template
    return rebuilt


def _handler_announce_text(root: Any, action: str, params: Mapping[str, Any], target_id: str) -> Any:
    """Change the spoken message on the TARGETED literal-message announce leaf.

    Only a ``BtNode_Announce`` built with a literal ``message=`` (``given_msg``
    set, no ``bb_source``) can be pinned here — a blackboard-sourced announce
    reads its text at runtime and is untouched.
    """
    text = str(params.get("text") or "").strip()
    if not text:
        raise ModificationValidationError("announce-text requires non-empty 'text'")
    factory = small_trees.ACTION_FACTORIES.get(action)
    if factory is None:
        raise ModificationValidationError(f"announce-text: unknown action {action!r}")
    rebuilt = factory()
    node = _walk_to_node(rebuilt, action, target_id)
    if node is None or type(node).__name__ != "BtNode_Announce":
        raise ModificationValidationError(
            f"announce-text: target {target_id!r} is not an announce leaf"
        )
    if getattr(node, "bb_source", None) is not None or getattr(node, "given_msg", None) is None:
        raise ModificationValidationError(
            "announce-text: target announce reads from the blackboard; cannot pin text"
        )
    node.given_msg = text
    return rebuilt


def _handler_pan_tilt_sweep(root: Any, action: str, params: Mapping[str, Any], target_id: str) -> Any:
    """Rebuild the pan/tilt sweep ranges for a scanning tree.

    ``find_person`` (human tilt), ``find_object`` / ``describe_person``
    (object tilt). ``pan_deg`` / ``tilt_deg`` are float lists.
    """
    pan_deg = params.get("pan_deg")
    tilt_deg = params.get("tilt_deg")
    if not pan_deg and not tilt_deg:
        raise ModificationValidationError("pan-tilt-sweep requires pan_deg and/or tilt_deg")
    if action == "find_person":
        rebuilt = small_trees.create_find_person(pan_deg=pan_deg, tilt_deg=tilt_deg)
    elif action == "find_object":
        rebuilt = small_trees.create_find_object(pan_deg=pan_deg, tilt_deg=tilt_deg)
    elif action == "describe_person":
        rebuilt = small_trees.create_describe_person(pan_deg=pan_deg, tilt_deg=tilt_deg)
    else:
        raise ModificationValidationError(
            f"pan-tilt-sweep does not apply to action {action!r}"
        )
    return rebuilt


def _handler_search_spots(root: Any, action: str, params: Mapping[str, Any], target_id: str) -> Any:
    """Scale the search_object spot sweep to a target capacity."""
    if action != "search_object":
        raise ModificationValidationError("search-spots only applies to search_object")
    capacity = int(params.get("capacity") or 0)
    if capacity < 1 or capacity > 32:
        raise ModificationValidationError("search-spots capacity must be 1..32")
    rebuilt = small_trees.create_search_object(capacity=capacity)
    return rebuilt


REGISTRY: dict[str, Handler] = {
    "attribute-person-specialist": _handler_attribute_person_specialist,
    "person-specialist": _handler_person_specialist,
    "vlm-template": _handler_vlm_template,
    "announce-text": _handler_announce_text,
    "pan-tilt-sweep": _handler_pan_tilt_sweep,
    "search-spots": _handler_search_spots,
}

TEMPLATES: dict[str, TemplateSpec] = {
    "attribute-person-specialist": TemplateSpec(
        name="attribute-person-specialist",
        applies_to=("find_person_sweep",),
        params={
            "gate": ParamSpec(str, required=True, validator=_nonempty_str),
            "prompt": ParamSpec(str, required=False, validator=_nonempty_str),
            "pan_deg": PAN_LIST,
            "tilt_deg": TILT_LIST,
        },
        description="Add a descriptor-gated attribute specialist branch to find_person "
                    "(e.g. 'person wearing a red jacket').",
    ),
    "person-specialist": TemplateSpec(
        name="person-specialist",
        applies_to=("find_person_sweep",),
        params={
            "name": ParamSpec(str, required=True, validator=_nonempty_str),
            "pan_deg": PAN_LIST,
            "tilt_deg": TILT_LIST,
        },
        description="Pin find_person to a named-person prompt for the generalist scan.",
    ),
    "vlm-template": TemplateSpec(
        name="vlm-template",
        applies_to=("vlm_query", "count_vlm_branch"),
        params={"question_template": ParamSpec(str, required=True, validator=_vlm_template_str)},
        description="Swap the VLM question template on count/vlm_fallback query nodes.",
    ),
    "announce-text": TemplateSpec(
        name="announce-text",
        applies_to=("announce_leaf",),
        params={"text": ParamSpec(str, required=True, validator=_nonempty_str)},
        description="Change the spoken text of an announce leaf.",
    ),
    "pan-tilt-sweep": TemplateSpec(
        name="pan-tilt-sweep",
        applies_to=("find_person_sweep", "find_object_sweep", "describe_person_sweep"),
        params={
            "pan_deg": PAN_LIST,
            "tilt_deg": TILT_LIST,
        },
        description="Override the pan/tilt sweep ranges of a scanning tree.",
    ),
    "search-spots": TemplateSpec(
        name="search-spots",
        applies_to=("search_object_sweep",),
        params={"capacity": ParamSpec((int,), required=True)},
        description="Scale the search_object spot sweep to a target capacity.",
    ),
}


# ---------------------------------------------------------------------------
# Deep-copy mutation helpers (VLM template / announce text)
# ---------------------------------------------------------------------------

def _swap_vlm_templates(root: Any, template: str) -> Any:
    for node in root.iterate():
        if isinstance(node, BtNode_VLMQuery) and getattr(node, "_template", None) is not None:
            node._template = template
    return root


def _replace_announce_text(root: Any, text: str) -> Any:
    for node in root.iterate():
        if type(node).__name__ == "BtNode_Announce" and getattr(node, "given_msg", None) is not None:
            node.given_msg = text
    return root


# ---------------------------------------------------------------------------
# Validate / apply protocol
# ---------------------------------------------------------------------------

def validate_modifications(
    modifications: Any,
    action: str,
    subtree: Any,
    *,
    registry: Mapping[str, Handler] | None = None,
    templates: Mapping[str, TemplateSpec] | None = None,
) -> tuple[bool, str]:
    """Structural check of one target's modification list.

    Rejects when a directive names an unknown template, targets a node id that
    does not exist in ``subtree`` (serialized via ``serialize_tree``), targets a
    role the template does not apply to, or fails the closed param schema.
    Returns ``(ok, reason)``; on ``ok=False`` the whole plan attempt must be
    rejected (never partially applied).
    """
    if not modifications:
        return True, ""
    if not isinstance(modifications, list):
        return False, "modifications must be a list"
    registry = registry or REGISTRY
    templates = templates or TEMPLATES
    tree = serialize_tree(subtree, kind=f"small/{action}")
    node_ids = {node["node_id"] for node in tree["nodes"]}
    for i, mod in enumerate(modifications):
        if not isinstance(mod, Mapping):
            return False, f"modifications[{i}] must be an object"
        mod_action = mod.get("action")
        if mod_action is not None and str(mod_action) != action:
            return False, (
                f"modifications[{i}]: action {mod_action!r} does not match "
                f"step action {action!r}"
            )
        template = mod.get("template")
        if not isinstance(template, str) or template not in registry:
            return False, f"modifications[{i}]: unknown template {template!r}"
        spec = templates[template]
        target = mod.get("target_node_id")
        if not isinstance(target, str) or target not in node_ids:
            return False, (
                f"modifications[{i}]: target_node_id {target!r} not found in "
                f"{action} tree (serialized ids: {sorted(node_ids)[:6]}...)"
            )
        role = _nested_role_for(action, target)
        if role is None or role not in spec.applies_to:
            return False, (
                f"modifications[{i}]: template {template!r} does not apply to "
                f"node {target!r} (role {role!r})"
            )
        params = mod.get("params", {})
        if not isinstance(params, Mapping):
            return False, f"modifications[{i}]: params must be an object"
        try:
            spec.validate_params(dict(params))
        except ModificationValidationError as exc:
            return False, f"modifications[{i}]: {exc}"
        reason = mod.get("reason")
        if reason is not None and not isinstance(reason, str):
            return False, f"modifications[{i}]: reason must be a string"
    return True, ""


def _step_index_for(mod: Mapping[str, Any], plan: list[Mapping[str, Any]]) -> int | None:
    """Resolve the plan step a modification targets.

    ``step_index`` is the primary key. When absent, fall back to the action
    field (matching the unique step of that action; ambiguous on duplicates).
    Returns None when the mod cannot be unambiguously assigned.
    """
    raw = mod.get("step_index")
    if isinstance(raw, int) and 0 <= raw < len(plan):
        return raw
    action = str(mod.get("action") or "")
    indexes = [i for i, step in enumerate(plan) if str(step.get("action") or "") == action]
    if len(indexes) == 1:
        return indexes[0]
    return None


def group_modifications_by_step(
    plan: list[Mapping[str, Any]],
    modifications: list[Mapping[str, Any]],
) -> dict[int, list[dict[str, Any]]]:
    """Assign plan-level modifications to the step (index) they target.

    A modification targets a step by ``step_index`` (or, when the plan has a
    unique step of that action, by its ``action``). Modifications that cannot be
    matched are dropped here — validation rejects them earlier; this is the safe
    post-validation grouping.
    """
    grouped: dict[int, list[dict[str, Any]]] = {}
    for mod in modifications:
        if not isinstance(mod, Mapping):
            continue
        step_index = _step_index_for(mod, plan)
        if step_index is None:
            continue
        grouped.setdefault(step_index, []).append(dict(mod))
    return grouped


def validate_plan_modifications(
    modifications: Any,
    plan: list[Mapping[str, Any]],
    *,
    registry: Mapping[str, Handler] | None = None,
    templates: Mapping[str, TemplateSpec] | None = None,
) -> tuple[bool, str]:
    """Validate a plan-level ``modifications`` list across all steps.

    Each modification is matched to its step (by ``step_index`` / ``action``),
    the step's small tree is built, and ``validate_modifications`` runs the
    structural + schema checks. Returns ``(ok, reason)``; on ``ok=False`` the
    whole plan attempt must be rejected — never partially applied.
    """
    if not modifications:
        return True, ""
    if not isinstance(modifications, list):
        return False, "modifications must be a list"
    registry = registry or REGISTRY
    templates = templates or TEMPLATES
    grouped = group_modifications_by_step(plan, modifications)
    # every modification must have been assigned to a step
    assigned = sum(len(mods) for mods in grouped.values())
    if assigned != len(modifications):
        unassigned = [i for i, mod in enumerate(modifications) if _step_index_for(mod, plan) is None]
        return False, (
            "modifications could not be matched to a plan step (missing/invalid "
            f"step_index or action): indexes {unassigned}"
        )
    for step_index, mods in sorted(grouped.items()):
        step = plan[step_index]
        action = str(step.get("action") or "")
        factory = small_trees.ACTION_FACTORIES.get(action)
        if factory is None:
            return False, f"step {step_index}: unknown action {action!r}"
        try:
            subtree = factory()
        except Exception as exc:  # noqa: BLE001
            return False, f"step {step_index}: small tree build failed: {exc!r}"
        ok, reason = validate_modifications(
            mods, action, subtree, registry=registry, templates=templates,
        )
        if not ok:
            return False, f"step {step_index} ({action}): {reason}"
    return True, ""


def apply_modifications(
    step_subtree: Any,
    action: str,
    modifications: Any,
    *,
    registry: Mapping[str, Handler] | None = None,
) -> tuple[Any, list[tuple[str, str, str]]]:
    """Apply one step's validated modifications at plan-build time.

    Returns ``(new_subtree, applied)`` where ``applied`` is the list of
    ``(template, node_id, reason)`` audit tuples. A handler error reverts to the
    unmodified subtree (safe) and records nothing. Callers MUST have already run
    ``validate_modifications``; a directive that reaches here but fails is
    treated as a silent no-op so a bad handler never breaks the plan.
    """
    if not modifications:
        return step_subtree, []
    registry = registry or REGISTRY
    result = step_subtree
    applied: list[tuple[str, str, str]] = []
    for mod in modifications:
        if not isinstance(mod, Mapping):
            continue
        template = mod.get("template")
        handler = registry.get(template) if isinstance(template, str) else None
        if handler is None:
            continue
        try:
            candidate = handler(
                result,
                action,
                dict(mod.get("params", {}) or {}),
                str(mod.get("target_node_id") or ""),
            )
        except (ModificationValidationError, TypeError, ValueError, KeyError):
            continue  # revert to previous (unmodified) subtree
        if candidate is not None:
            result = candidate
            applied.append((template, str(mod.get("target_node_id")), str(mod.get("reason") or "")))
    return result, applied


_DIFF_FIELDS = (
    "name", "node_class", "children",
    "semantics", "blackboard_access", "action_context", "effect_contract",
)


def _diff_signature(node: Mapping[str, Any]) -> dict[str, Any]:
    """Stable structural signature — ignores volatile ``type`` (repr-based)."""
    return {field: node.get(field) for field in _DIFF_FIELDS if field in node}


def diff_trees(before: Any, after: Any, kind: str) -> list[dict[str, Any]]:
    """Structural diff between two serialized trees.

    Returns one record per changed/removed/added node: the node id, its type,
    and the reason class (``changed`` / ``removed`` / ``added``). Comparison
    uses the stable structural signature (name, node_class, children,
    semantics, blackboard_access, action_context) — not the raw node dict, whose
    ``type`` reflects Python repr and always differs across a deepcopy. Used by
    the checker LLM and the ten-command harness to pinpoint exactly which nodes
    a modification touched.
    """
    before_doc = serialize_tree(before, kind=kind, label="before")
    after_doc = serialize_tree(after, kind=kind, label="after")
    b = {node["node_id"]: node for node in before_doc["nodes"]}
    a = {node["node_id"]: node for node in after_doc["nodes"]}
    diff: list[dict[str, Any]] = []
    for node_id, node in a.items():
        if node_id not in b:
            diff.append({
                "node_id": node_id, "type": node.get("type"),
                "name": node.get("name"), "reason": "added",
            })
        elif _diff_signature(node) != _diff_signature(b[node_id]):
            diff.append({
                "node_id": node_id, "type": node.get("type"),
                "name": node.get("name"), "reason": "changed",
            })
    for node_id, node in b.items():
        if node_id not in a:
            diff.append({
                "node_id": node_id, "type": node.get("type"),
                "name": node.get("name"), "reason": "removed",
            })
    return diff


def node_id_by_name(action: str, subtree: Any, name_fragment: str) -> str | None:
    """Find the serialized node id of the first node whose name contains a
    fragment — lets a handler/template point at the right leaf without knowing
    the exact structural path."""
    tree = serialize_tree(subtree, kind=f"small/{action}")
    for node in tree["nodes"]:
        if name_fragment in str(node.get("name", "")):
            return node["node_id"]
    return None


__all__ = [
    "REGISTRY",
    "TEMPLATES",
    "TemplateSpec",
    "ModificationValidationError",
    "apply_modifications",
    "diff_trees",
    "group_modifications_by_step",
    "node_id_by_name",
    "validate_modifications",
    "validate_plan_modifications",
]
