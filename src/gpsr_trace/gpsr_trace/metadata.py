"""Immutable agent and proposal metadata shared by trace and planning tools."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Mapping
from uuid import uuid4

from .events import JsonValue, ValidationError, _copy_json, _require_identifier, timestamp_from_string, timestamp_to_string, utc_now


@dataclass(frozen=True)
class AgentMetadata:
    """Identifies the component or agent that produced a proposal or event."""

    agent_id: str
    role: str
    version: str | None = None
    model: str | None = None
    session_id: str | None = None
    capabilities: tuple[str, ...] = ()
    attributes: Mapping[str, JsonValue] = field(default_factory=dict)

    def __post_init__(self) -> None:
        _require_identifier("agent_id", self.agent_id)
        _require_identifier("role", self.role)
        for optional_name in ("version", "model", "session_id"):
            optional_value = getattr(self, optional_name)
            if optional_value is not None:
                _require_identifier(optional_name, optional_value)
        capabilities = tuple(self.capabilities)
        for capability in capabilities:
            _require_identifier("capability", capability)
        object.__setattr__(self, "capabilities", capabilities)
        if not isinstance(self.attributes, Mapping):
            raise ValidationError("agent attributes must be a mapping")
        object.__setattr__(self, "attributes", _copy_json(dict(self.attributes), field_name="agent attributes"))

    def to_dict(self) -> dict[str, JsonValue]:
        return {
            "agent_id": self.agent_id,
            "role": self.role,
            "version": self.version,
            "model": self.model,
            "session_id": self.session_id,
            "capabilities": list(self.capabilities),
            "attributes": _copy_json(self.attributes, field_name="agent attributes"),
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "AgentMetadata":
        if not isinstance(value, Mapping):
            raise ValidationError("agent metadata must be a mapping")
        return cls(
            agent_id=value["agent_id"],
            role=value["role"],
            version=value.get("version"),
            model=value.get("model"),
            session_id=value.get("session_id"),
            capabilities=tuple(value.get("capabilities", ())),
            attributes=value.get("attributes", {}),
        )


@dataclass(frozen=True)
class ProposalMetadata:
    """Provenance for a behavior-tree change proposal."""

    proposal_id: str
    agent: AgentMetadata
    created_at: datetime = field(default_factory=utc_now)
    base_tree_version: int | None = None
    summary: str = ""
    trace_id: str | None = None
    labels: Mapping[str, JsonValue] = field(default_factory=dict)

    def __post_init__(self) -> None:
        _require_identifier("proposal_id", self.proposal_id)
        if not isinstance(self.agent, AgentMetadata):
            raise ValidationError("proposal agent must be AgentMetadata")
        timestamp_to_string(self.created_at)
        if self.base_tree_version is not None and (
            not isinstance(self.base_tree_version, int) or self.base_tree_version < 0
        ):
            raise ValidationError("base_tree_version must be a non-negative integer or None")
        if not isinstance(self.summary, str):
            raise ValidationError("proposal summary must be a string")
        if self.trace_id is not None:
            _require_identifier("trace_id", self.trace_id)
        if not isinstance(self.labels, Mapping):
            raise ValidationError("proposal labels must be a mapping")
        object.__setattr__(self, "labels", _copy_json(dict(self.labels), field_name="proposal labels"))

    @classmethod
    def create(cls, *, agent: AgentMetadata, **kwargs: Any) -> "ProposalMetadata":
        """Create metadata with a UUID proposal id unless one is supplied."""

        return cls(proposal_id=kwargs.pop("proposal_id", str(uuid4())), agent=agent, **kwargs)

    def to_dict(self) -> dict[str, JsonValue]:
        return {
            "proposal_id": self.proposal_id,
            "agent": self.agent.to_dict(),
            "created_at": timestamp_to_string(self.created_at),
            "base_tree_version": self.base_tree_version,
            "summary": self.summary,
            "trace_id": self.trace_id,
            "labels": _copy_json(self.labels, field_name="proposal labels"),
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "ProposalMetadata":
        if not isinstance(value, Mapping):
            raise ValidationError("proposal metadata must be a mapping")
        return cls(
            proposal_id=value["proposal_id"],
            agent=AgentMetadata.from_dict(value["agent"]),
            created_at=timestamp_from_string(value.get("created_at", timestamp_to_string(utc_now()))),
            base_tree_version=value.get("base_tree_version"),
            summary=value.get("summary", ""),
            trace_id=value.get("trace_id"),
            labels=value.get("labels", {}),
        )
