"""Conservative retention policy for debugger trajectory history.

Only unnamed, unpinned, completed trajectories are candidates for deletion.
Named trajectories are deliberately treated as operator-selected evidence, and
active trajectories are never removed even when the database is over budget.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import sqlite3
from typing import Any

from .projection import is_terminal_status


DEFAULT_MAX_AGE_DAYS = 7
# Retention is expressed as 10 GB (decimal), matching the operator-facing
# configuration label rather than silently turning it into 10 GiB.
DEFAULT_MAX_BYTES = 10 * 1000 * 1000 * 1000


@dataclass(frozen=True)
class RetentionPolicy:
    """Limits applied to disposable debugger trajectory history."""

    max_age_days: int = DEFAULT_MAX_AGE_DAYS
    max_bytes: int = DEFAULT_MAX_BYTES

    def __post_init__(self) -> None:
        if self.max_age_days < 0:
            raise ValueError("max_age_days must be non-negative")
        if self.max_bytes < 0:
            raise ValueError("max_bytes must be non-negative")

    def as_dict(self) -> dict[str, int]:
        return {"max_age_days": self.max_age_days, "max_bytes": self.max_bytes}


def coerce_utc(value: datetime | str | None = None) -> datetime:
    """Return an aware UTC time, accepting ISO-8601 values for testability."""

    if value is None:
        return datetime.now(timezone.utc)
    if isinstance(value, str):
        value = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if value.tzinfo is None:
        return value.replace(tzinfo=timezone.utc)
    return value.astimezone(timezone.utc)


def apply_retention(
    connection: sqlite3.Connection,
    *,
    policy: RetentionPolicy | None = None,
    now: datetime | str | None = None,
) -> dict[str, Any]:
    """Delete expired/excess disposable trajectories and return an audit DTO.

    The caller owns transaction boundaries.  ``byte_size`` is an estimate of
    stored event/checkpoint JSON, which permits a per-trajectory budget while
    avoiding misleading attempts to apportion SQLite's shared page/WAL space.
    """

    policy = policy or RetentionPolicy()
    reference = coerce_utc(now)
    cutoff = reference - timedelta(days=policy.max_age_days)
    rows = connection.execute(
        """
        SELECT trajectory_id, name, pinned, completed, status, finished_at, byte_size
        FROM trajectories
        ORDER BY finished_at ASC, trajectory_id ASC
        """
    ).fetchall()

    candidates: list[sqlite3.Row] = []
    protected_bytes = 0
    for row in rows:
        eligible = (
            not str(row["name"] or "").strip()
            and not bool(row["pinned"])
            and bool(row["completed"])
            and is_terminal_status(row["status"])
        )
        if eligible:
            candidates.append(row)
        else:
            protected_bytes += int(row["byte_size"] or 0)

    eligible_before = sum(int(row["byte_size"] or 0) for row in candidates)
    deleted: list[str] = []
    reclaimed = 0

    retained: list[sqlite3.Row] = []
    for row in candidates:
        finished = _parse_timestamp(row["finished_at"])
        if finished is not None and finished < cutoff:
            _delete(connection, row["trajectory_id"])
            deleted.append(row["trajectory_id"])
            reclaimed += int(row["byte_size"] or 0)
        else:
            retained.append(row)

    retained_bytes = sum(int(row["byte_size"] or 0) for row in retained)
    for row in retained:
        if retained_bytes <= policy.max_bytes:
            break
        _delete(connection, row["trajectory_id"])
        deleted.append(row["trajectory_id"])
        size = int(row["byte_size"] or 0)
        reclaimed += size
        retained_bytes -= size

    return {
        "policy": policy.as_dict(),
        "ran_at": reference.isoformat().replace("+00:00", "Z"),
        "deleted_trajectory_ids": deleted,
        "deleted_count": len(deleted),
        "bytes_reclaimed_estimate": reclaimed,
        "eligible_bytes_before": eligible_before,
        "eligible_bytes_after": retained_bytes,
        "protected_bytes_estimate": protected_bytes,
    }


def _parse_timestamp(value: Any) -> datetime | None:
    if not isinstance(value, str) or not value:
        return None
    try:
        return coerce_utc(value)
    except (TypeError, ValueError):
        return None


def _delete(connection: sqlite3.Connection, trajectory_id: str) -> None:
    # Foreign-key cascades remove events, causal links, and local checkpoints.
    connection.execute("DELETE FROM trajectories WHERE trajectory_id = ?", (trajectory_id,))
