"""SQLite/WAL event store for the standalone GPSR mission debugger.

``DebugStore`` is intentionally a small synchronous API.  It can be called
from a ROS callback, a FastAPI worker, or an offline importer without pulling
in either ROS or FastAPI.  Events are immutable; summaries and checkpoints are
disposable acceleration structures rebuilt from those events when necessary.
"""
from __future__ import annotations

from contextlib import contextmanager
from datetime import date, datetime, timezone
from enum import Enum
import base64
import json
import math
from pathlib import Path
import sqlite3
import threading
from typing import Any, Iterator, Mapping

from .projection import (
    PROJECTION_VERSION,
    empty_projection,
    is_terminal_status,
    lifecycle_for_event,
    reduce_events,
)
from .retention import RetentionPolicy, apply_retention


class StoreError(RuntimeError):
    """Base exception for durable debugger storage errors."""


class EventConflictError(StoreError):
    """A sequence or event id was reused for different immutable content."""


class TrajectoryNotFoundError(KeyError, StoreError):
    """The requested trajectory does not exist."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds").replace("+00:00", "Z")


def json_safe(value: Any) -> Any:
    """Convert common Python values to strict JSON-compatible values.

    Storage accepts standard JSON as its native language.  ``datetime``,
    ``date``, ``Path`` and enum values are converted for producer convenience;
    unsupported or non-finite values fail early instead of producing a value a
    web client cannot serialise.
    """

    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("JSON event values cannot be NaN or infinite")
        return value
    if isinstance(value, (datetime, date)):
        if isinstance(value, datetime) and value.tzinfo is not None:
            return value.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")
        return value.isoformat()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Enum):
        return json_safe(value.value)
    if isinstance(value, Mapping):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, set):
        return [json_safe(item) for item in sorted(value, key=repr)]
    raise TypeError(f"value of type {type(value).__name__} is not JSON serialisable")


def _json_dumps(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _cursor_encode(updated_at: str, trajectory_id: str) -> str:
    raw = _json_dumps({"updated_at": updated_at, "trajectory_id": trajectory_id}).encode("utf-8")
    return base64.urlsafe_b64encode(raw).decode("ascii").rstrip("=")


def _cursor_decode(cursor: str) -> tuple[str, str] | None:
    if not cursor:
        return None
    try:
        padded = cursor + "=" * (-len(cursor) % 4)
        value = json.loads(base64.urlsafe_b64decode(padded.encode("ascii")).decode("utf-8"))
        if isinstance(value, dict) and isinstance(value.get("updated_at"), str) and isinstance(value.get("trajectory_id"), str):
            return value["updated_at"], value["trajectory_id"]
    except (UnicodeError, ValueError, json.JSONDecodeError):
        return None
    return None


def _normalise_event(event: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(event, Mapping):
        raise TypeError("event must be a mapping")
    result = json_safe(event)
    # The native GPSR envelope calls this a trajectory, while the lightweight
    # ``gpsr_trace`` SDK uses ``trace_id`` and calls the discriminator
    # ``event_type``.  Normalising at the storage boundary keeps every
    # producer interchangeable without forcing ROS nodes to depend on the
    # debugger package.
    trajectory_id = result.get("trajectory_id") or result.get("run_id") or result.get("trace_id")
    if not isinstance(trajectory_id, str) or not trajectory_id.strip():
        raise ValueError("event.trajectory_id must be a non-empty string")
    sequence = result.get("sequence")
    if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 0:
        raise ValueError("event.sequence must be a non-negative integer")
    event_id = result.get("event_id")
    if not isinstance(event_id, str) or not event_id.strip():
        raise ValueError("event.event_id must be a non-empty string")
    event_type = result.get("type") or result.get("event_type")
    if not isinstance(event_type, str) or not event_type.strip():
        raise ValueError("event.type must be a non-empty string")
    result["trajectory_id"] = trajectory_id
    result["sequence"] = sequence
    result["event_id"] = event_id
    result["type"] = event_type
    result.setdefault("event_type", event_type)
    if "occurred_at" not in result and "timestamp" in result:
        result["occurred_at"] = result["timestamp"]
    result.setdefault("occurred_at", _utc_now())
    result.setdefault("payload", {})
    return result


def _causal_ids(event: Mapping[str, Any]) -> list[str]:
    payload = event.get("payload") if isinstance(event.get("payload"), Mapping) else {}
    values: list[Any] = []
    for source in (event, payload):
        for key in ("causation_id", "parent_event_id", "caused_by", "cause_event_id"):
            if key in source:
                values.append(source[key])
        for key in ("causation_ids", "parent_event_ids", "causes"):
            item = source.get(key)
            if isinstance(item, (list, tuple, set)):
                values.extend(item)
    result: list[str] = []
    for value in values:
        if isinstance(value, str) and value and value not in result:
            result.append(value)
    return result


class DebugStore:
    """Append-only trajectory event store with checkpoint+delta replay."""

    def __init__(
        self,
        path: str | Path,
        *,
        checkpoint_interval: int = 100,
        retention_policy: RetentionPolicy | None = None,
    ) -> None:
        if checkpoint_interval < 1:
            raise ValueError("checkpoint_interval must be at least one")
        database = str(path)
        self.path = None if database == ":memory:" else Path(database)
        if self.path is not None and not database.startswith("file:"):
            self.path.parent.mkdir(parents=True, exist_ok=True)
        self.checkpoint_interval = checkpoint_interval
        self.retention_policy = retention_policy or RetentionPolicy()
        self._lock = threading.RLock()
        self._connection = sqlite3.connect(
            database,
            timeout=30.0,
            isolation_level=None,
            check_same_thread=False,
            uri=database.startswith("file:"),
        )
        self._connection.row_factory = sqlite3.Row
        self._connection.execute("PRAGMA foreign_keys = ON")
        self._connection.execute("PRAGMA busy_timeout = 30000")
        # WAL permits an API reader to reconstruct a historical snapshot while
        # a callback appends an event.  In-memory SQLite reports a different
        # journal mode; that is harmless for unit tests.
        self._connection.execute("PRAGMA journal_mode = WAL")
        self._connection.execute("PRAGMA synchronous = NORMAL")
        self._create_schema()

    def close(self) -> None:
        with self._lock:
            if self._connection is not None:
                self._connection.close()
                self._connection = None  # type: ignore[assignment]

    def __enter__(self) -> "DebugStore":
        return self

    def __exit__(self, _exc_type: Any, _exc: Any, _tb: Any) -> None:
        self.close()

    def append_event(self, event: Mapping[str, Any]) -> bool:
        """Append an immutable event and return whether it was newly stored.

        Re-delivery of byte-for-byte equivalent normalised content returns
        ``False``.  Reusing either ``(trajectory_id, sequence)`` or a global
        ``event_id`` for different content raises
        :class:`EventConflictError`, making a producer bug visible without
        ever rewriting the history.
        """

        normalised = _normalise_event(event)
        encoded = _json_dumps(normalised)
        payload_json = _json_dumps(normalised["payload"])
        trajectory_id = normalised["trajectory_id"]
        sequence = normalised["sequence"]
        event_id = normalised["event_id"]
        with self._transaction():
            trajectory_before = self._connection.execute(
                "SELECT last_sequence FROM trajectories WHERE trajectory_id = ?",
                (trajectory_id,),
            ).fetchone()
            sequence_row = self._connection.execute(
                "SELECT event_id, event_json FROM events WHERE trajectory_id = ? AND sequence = ?",
                (trajectory_id, sequence),
            ).fetchone()
            event_id_row = self._connection.execute(
                "SELECT trajectory_id, sequence, event_json FROM events WHERE event_id = ?",
                (event_id,),
            ).fetchone()
            for row in (sequence_row, event_id_row):
                if row is None:
                    continue
                if row["event_json"] == encoded:
                    return False
                raise EventConflictError(
                    f"immutable event key already exists for trajectory={trajectory_id!r}, "
                    f"sequence={sequence!r}, event_id={event_id!r}"
                )

            now = _utc_now()
            started, completed, lifecycle_status = lifecycle_for_event(normalised)
            status = lifecycle_status or ("running" if started else "active")
            self._connection.execute(
                """
                INSERT INTO trajectories (
                    trajectory_id, created_at, updated_at, started_at, finished_at,
                    status, completed, last_sequence, event_count, byte_size, metadata_json
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, 0, 0, '{}')
                ON CONFLICT(trajectory_id) DO NOTHING
                """,
                (
                    trajectory_id,
                    now,
                    now,
                    normalised["occurred_at"] if started else None,
                    normalised["occurred_at"] if completed else None,
                    status,
                    int(completed),
                    sequence,
                ),
            )
            self._connection.execute(
                """
                INSERT INTO events (
                    trajectory_id, sequence, event_id, event_type, occurred_at,
                    payload_json, event_json, causation_id, correlation_id
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    trajectory_id,
                    sequence,
                    event_id,
                    normalised["type"],
                    normalised["occurred_at"],
                    payload_json,
                    encoded,
                    _first_causation(normalised),
                    _correlation_id(normalised),
                ),
            )
            for parent_event_id in _causal_ids(normalised):
                self._connection.execute(
                    """
                    INSERT OR IGNORE INTO event_links (trajectory_id, child_event_id, parent_event_id)
                    VALUES (?, ?, ?)
                    """,
                    (trajectory_id, event_id, parent_event_id),
                )

            self._update_trajectory_after_append(normalised, len(encoded.encode("utf-8")), now, started, completed, lifecycle_status)
            # A late event invalidates snapshots that were built beyond it.
            self._connection.execute(
                "DELETE FROM checkpoints WHERE trajectory_id = ? AND sequence >= ?",
                (trajectory_id, sequence),
            )
            # In-order live traffic is already reflected by the incremental
            # lifecycle update above. Replaying the entire projection for
            # every event makes a burst import quadratic (and full executor
            # ticks are large). Only pay that cost when a reconnect actually
            # delivers an older causal sequence.
            if trajectory_before is not None and sequence < trajectory_before["last_sequence"]:
                self._refresh_trajectory_lifecycle_locked(trajectory_id)
            event_count = self._connection.execute(
                "SELECT event_count FROM trajectories WHERE trajectory_id = ?", (trajectory_id,)
            ).fetchone()["event_count"]
            if completed or _is_explicit_checkpoint(normalised) or event_count % self.checkpoint_interval == 0:
                self._write_checkpoint_locked(trajectory_id, sequence)
        return True

    def list_trajectories(self, *, limit: int = 50, cursor: str | None = None) -> list[dict[str, Any]]:
        """Return newest trajectories first.

        Each returned row includes an opaque ``cursor`` that can be supplied as
        the next call's ``cursor``.  A trajectory id is also accepted as a
        convenient compatibility cursor.
        """

        if not isinstance(limit, int) or isinstance(limit, bool) or limit < 1:
            raise ValueError("limit must be a positive integer")
        limit = min(limit, 1000)
        with self._lock:
            boundary = _cursor_decode(cursor or "")
            if boundary is None and cursor:
                row = self._connection.execute(
                    "SELECT updated_at, trajectory_id FROM trajectories WHERE trajectory_id = ?", (cursor,)
                ).fetchone()
                boundary = (row["updated_at"], row["trajectory_id"]) if row else None
            parameters: list[Any] = []
            where = ""
            if boundary is not None:
                where = "WHERE (updated_at < ? OR (updated_at = ? AND trajectory_id < ?))"
                parameters.extend((boundary[0], boundary[0], boundary[1]))
            parameters.append(limit)
            rows = self._connection.execute(
                f"""
                SELECT trajectory_id, name, pinned, created_at, updated_at, started_at, finished_at,
                       status, completed, last_sequence, event_count, byte_size
                FROM trajectories {where}
                ORDER BY updated_at DESC, trajectory_id DESC
                LIMIT ?
                """,
                parameters,
            ).fetchall()
            return [self._trajectory_summary(row) for row in rows]

    def trajectory_snapshot(self, trajectory_id: str, at_sequence: int | None = None) -> dict[str, Any]:
        """Rebuild a JSON-safe projection from its newest checkpoint and delta."""

        with self._lock:
            trajectory = self._require_trajectory(trajectory_id)
            if at_sequence is not None and (
                not isinstance(at_sequence, int) or isinstance(at_sequence, bool) or at_sequence < 0
            ):
                raise ValueError("at_sequence must be a non-negative integer or None")
            target = trajectory["last_sequence"] if at_sequence is None else at_sequence
            checkpoint = self._connection.execute(
                """
                SELECT sequence, snapshot_json FROM checkpoints
                WHERE trajectory_id = ? AND sequence <= ?
                ORDER BY sequence DESC LIMIT 1
                """,
                (trajectory_id, target),
            ).fetchone()
            stale_checkpoint = False
            if checkpoint is None:
                base = empty_projection(trajectory_id)
                after = -1
            else:
                candidate = json.loads(checkpoint["snapshot_json"])
                if candidate.get("projection_version") == PROJECTION_VERSION:
                    base = candidate
                    after = checkpoint["sequence"]
                else:
                    base = empty_projection(trajectory_id)
                    after = -1
                    stale_checkpoint = True
            rows = self._connection.execute(
                """
                SELECT event_json FROM events
                WHERE trajectory_id = ? AND sequence > ? AND sequence <= ?
                ORDER BY sequence ASC
                """,
                (trajectory_id, after, target),
            ).fetchall()
            state = reduce_events(trajectory_id, (json.loads(row["event_json"]) for row in rows), initial=base)
            if stale_checkpoint:
                # Projection reducers evolve independently of the immutable
                # event schema. Replace incompatible acceleration state with a
                # checkpoint generated by the current reducer.
                self._connection.execute(
                    "DELETE FROM checkpoints WHERE trajectory_id = ?",
                    (trajectory_id,),
                )
                self._connection.execute(
                    """
                    INSERT INTO checkpoints (trajectory_id, sequence, snapshot_json, created_at)
                    VALUES (?, ?, ?, ?)
                    """,
                    (trajectory_id, target, _json_dumps(state), _utc_now()),
                )
            count = self._connection.execute(
                "SELECT COUNT(*) AS count FROM events WHERE trajectory_id = ? AND sequence <= ?",
                (trajectory_id, target),
            ).fetchone()["count"]
            summary = self._trajectory_summary(trajectory)
            summary["last_sequence"] = state["sequence"]
            summary["event_count"] = count
            summary["status"] = state["status"]
            summary["completed"] = is_terminal_status(state["status"])
            summary["started_at"] = state["started_at"]
            summary["finished_at"] = state["finished_at"]
            state["trajectory"] = summary
            # Handy top-level aliases for small API clients.  The nested
            # trajectory document remains the canonical summary.
            state["name"] = summary["name"]
            state["pinned"] = summary["pinned"]
            return state

    def events(self, trajectory_id: str, *, after: int = -1, limit: int = 1000) -> list[dict[str, Any]]:
        """Return causally ordered (sequence-ascending) immutable events."""

        if not isinstance(after, int) or isinstance(after, bool) or after < -1:
            raise ValueError("after must be an integer greater than or equal to -1")
        if not isinstance(limit, int) or isinstance(limit, bool) or limit < 1:
            raise ValueError("limit must be a positive integer")
        with self._lock:
            self._require_trajectory(trajectory_id)
            rows = self._connection.execute(
                """
                SELECT event_json FROM events
                WHERE trajectory_id = ? AND sequence > ?
                ORDER BY sequence ASC LIMIT ?
                """,
                (trajectory_id, after, min(limit, 10000)),
            ).fetchall()
            return [json.loads(row["event_json"]) for row in rows]

    def causal_events(
        self,
        trajectory_id: str,
        event_id: str,
        *,
        direction: str = "ancestors",
        limit: int = 1000,
    ) -> list[dict[str, Any]]:
        """Return an event's causal ancestors or descendants in sequence order."""

        if direction not in {"ancestors", "descendants"}:
            raise ValueError("direction must be 'ancestors' or 'descendants'")
        if limit < 1:
            raise ValueError("limit must be positive")
        with self._lock:
            self._require_trajectory(trajectory_id)
            known = self._connection.execute(
                "SELECT 1 FROM events WHERE trajectory_id = ? AND event_id = ?", (trajectory_id, event_id)
            ).fetchone()
            if known is None:
                raise KeyError(event_id)
            if direction == "ancestors":
                source_column, target_column = "child_event_id", "parent_event_id"
            else:
                source_column, target_column = "parent_event_id", "child_event_id"
            pending = [event_id]
            visited: set[str] = set()
            related: set[str] = set()
            while pending and len(related) < limit:
                source = pending.pop()
                if source in visited:
                    continue
                visited.add(source)
                rows = self._connection.execute(
                    f"SELECT {target_column} AS event_id FROM event_links WHERE trajectory_id = ? AND {source_column} = ?",
                    (trajectory_id, source),
                ).fetchall()
                for row in rows:
                    target = row["event_id"]
                    if target not in related:
                        related.add(target)
                        pending.append(target)
                    if len(related) >= limit:
                        break
            if not related:
                return []
            placeholders = ",".join("?" for _ in related)
            rows = self._connection.execute(
                f"SELECT event_json FROM events WHERE trajectory_id = ? AND event_id IN ({placeholders}) ORDER BY sequence ASC",
                [trajectory_id, *sorted(related)],
            ).fetchall()
            return [json.loads(row["event_json"]) for row in rows]

    def tree_document(self, trajectory_id: str, revision: str | int) -> dict[str, Any]:
        """Return one projected tree revision, including its latest node deltas."""

        snapshot = self.trajectory_snapshot(trajectory_id)
        key = str(revision)
        tree = snapshot["trees"]["revisions"].get(key)
        if tree is None:
            raise KeyError(f"tree revision {revision!r} does not exist")
        document = json_safe(tree)
        document["trajectory_id"] = trajectory_id
        document["sequence"] = snapshot["sequence"]
        return document

    def set_trajectory_name(self, trajectory_id: str, name: str | None) -> dict[str, Any]:
        """Set an optional operator name; named trajectories are retained."""

        if name is not None and not isinstance(name, str):
            raise TypeError("name must be a string or None")
        value = name.strip() if isinstance(name, str) else None
        with self._transaction():
            self._require_trajectory(trajectory_id)
            self._connection.execute(
                "UPDATE trajectories SET name = ?, updated_at = ? WHERE trajectory_id = ?",
                (value or None, _utc_now(), trajectory_id),
            )
            row = self._require_trajectory(trajectory_id)
            return self._trajectory_summary(row)

    def set_trajectory_pinned(self, trajectory_id: str, pinned: bool) -> dict[str, Any]:
        """Pin or unpin a trajectory without altering its immutable event log."""

        if not isinstance(pinned, bool):
            raise TypeError("pinned must be a bool")
        with self._transaction():
            self._require_trajectory(trajectory_id)
            self._connection.execute(
                "UPDATE trajectories SET pinned = ?, updated_at = ? WHERE trajectory_id = ?",
                (int(pinned), _utc_now(), trajectory_id),
            )
            return self._trajectory_summary(self._require_trajectory(trajectory_id))

    def delete_trajectory(self, trajectory_id: str) -> bool:
        """Delete one trajectory and all of its events/checkpoints, if present."""

        with self._transaction():
            cursor = self._connection.execute("DELETE FROM trajectories WHERE trajectory_id = ?", (trajectory_id,))
            return cursor.rowcount > 0

    def retention(self, *, now: datetime | str | None = None) -> dict[str, Any]:
        """Apply the seven-day/10-GB disposable-history retention policy."""

        with self._transaction():
            return apply_retention(self._connection, policy=self.retention_policy, now=now)

    def _create_schema(self) -> None:
        with self._lock:
            self._connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS trajectories (
                    trajectory_id TEXT PRIMARY KEY,
                    name TEXT,
                    pinned INTEGER NOT NULL DEFAULT 0 CHECK (pinned IN (0, 1)),
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL,
                    started_at TEXT,
                    finished_at TEXT,
                    status TEXT NOT NULL DEFAULT 'active',
                    completed INTEGER NOT NULL DEFAULT 0 CHECK (completed IN (0, 1)),
                    last_sequence INTEGER NOT NULL DEFAULT 0,
                    event_count INTEGER NOT NULL DEFAULT 0,
                    byte_size INTEGER NOT NULL DEFAULT 0,
                    metadata_json TEXT NOT NULL DEFAULT '{}'
                );
                CREATE INDEX IF NOT EXISTS trajectories_recent_idx
                    ON trajectories(updated_at DESC, trajectory_id DESC);
                CREATE INDEX IF NOT EXISTS trajectories_retention_idx
                    ON trajectories(completed, pinned, finished_at);

                CREATE TABLE IF NOT EXISTS events (
                    trajectory_id TEXT NOT NULL,
                    sequence INTEGER NOT NULL,
                    event_id TEXT NOT NULL,
                    event_type TEXT NOT NULL,
                    occurred_at TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    event_json TEXT NOT NULL,
                    causation_id TEXT,
                    correlation_id TEXT,
                    PRIMARY KEY (trajectory_id, sequence),
                    UNIQUE (event_id),
                    FOREIGN KEY (trajectory_id) REFERENCES trajectories(trajectory_id) ON DELETE CASCADE
                );
                CREATE INDEX IF NOT EXISTS events_correlation_idx
                    ON events(trajectory_id, correlation_id, sequence);
                CREATE INDEX IF NOT EXISTS events_causation_idx
                    ON events(trajectory_id, causation_id, sequence);

                CREATE TABLE IF NOT EXISTS event_links (
                    trajectory_id TEXT NOT NULL,
                    child_event_id TEXT NOT NULL,
                    parent_event_id TEXT NOT NULL,
                    PRIMARY KEY (trajectory_id, child_event_id, parent_event_id),
                    FOREIGN KEY (trajectory_id) REFERENCES trajectories(trajectory_id) ON DELETE CASCADE
                );
                CREATE INDEX IF NOT EXISTS event_links_parent_idx
                    ON event_links(trajectory_id, parent_event_id);

                CREATE TABLE IF NOT EXISTS checkpoints (
                    trajectory_id TEXT NOT NULL,
                    sequence INTEGER NOT NULL,
                    snapshot_json TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    PRIMARY KEY (trajectory_id, sequence),
                    FOREIGN KEY (trajectory_id) REFERENCES trajectories(trajectory_id) ON DELETE CASCADE
                );
                """
            )

    @contextmanager
    def _transaction(self) -> Iterator[None]:
        with self._lock:
            try:
                self._connection.execute("BEGIN IMMEDIATE")
                yield
            except BaseException:
                self._connection.rollback()
                raise
            else:
                self._connection.commit()

    def _require_trajectory(self, trajectory_id: str) -> sqlite3.Row:
        row = self._connection.execute("SELECT * FROM trajectories WHERE trajectory_id = ?", (trajectory_id,)).fetchone()
        if row is None:
            raise TrajectoryNotFoundError(trajectory_id)
        return row

    def _update_trajectory_after_append(
        self,
        event: Mapping[str, Any],
        byte_size: int,
        now: str,
        started: bool,
        completed: bool,
        lifecycle_status: str | None,
    ) -> None:
        trajectory_id = event["trajectory_id"]
        current = self._require_trajectory(trajectory_id)
        metadata = json.loads(current["metadata_json"])
        payload = event.get("payload") if isinstance(event.get("payload"), Mapping) else {}
        incoming_metadata = payload.get("metadata")
        if isinstance(incoming_metadata, Mapping):
            metadata.update(json_safe(incoming_metadata))
        for key in ("title", "description", "mode", "source", "tags"):
            if key in payload:
                metadata[key] = json_safe(payload[key])
        name = current["name"]
        pinned = current["pinned"]
        supplied_name = payload.get("name", event.get("name"))
        supplied_pinned = payload.get("pinned", event.get("pinned"))
        if isinstance(supplied_name, str):
            name = supplied_name.strip() or None
        if isinstance(supplied_pinned, bool):
            pinned = int(supplied_pinned)
        status = lifecycle_status or current["status"]
        self._connection.execute(
            """
            UPDATE trajectories
            SET name = ?, pinned = ?, updated_at = ?,
                started_at = COALESCE(started_at, ?),
                finished_at = CASE WHEN ? THEN COALESCE(finished_at, ?) ELSE finished_at END,
                status = ?, completed = CASE WHEN ? THEN 1 ELSE completed END,
                last_sequence = MAX(last_sequence, ?),
                event_count = event_count + 1,
                byte_size = byte_size + ?, metadata_json = ?
            WHERE trajectory_id = ?
            """,
            (
                name,
                pinned,
                now,
                event["occurred_at"] if started else None,
                int(completed),
                event["occurred_at"],
                status,
                int(completed),
                event["sequence"],
                byte_size,
                _json_dumps(metadata),
                trajectory_id,
            ),
        )

    def _write_checkpoint_locked(self, trajectory_id: str, sequence: int) -> None:
        snapshot = self.trajectory_snapshot(trajectory_id, at_sequence=sequence)
        # The trajectory summary is current mutable metadata; checkpoints only
        # need reducer state.  Removing it keeps snapshots independent of a
        # later rename/pin operation.
        snapshot.pop("trajectory", None)
        snapshot.pop("name", None)
        snapshot.pop("pinned", None)
        self._connection.execute(
            """
            INSERT INTO checkpoints (trajectory_id, sequence, snapshot_json, created_at)
            VALUES (?, ?, ?, ?)
            ON CONFLICT(trajectory_id, sequence) DO UPDATE SET snapshot_json = excluded.snapshot_json, created_at = excluded.created_at
            """,
            (trajectory_id, sequence, _json_dumps(snapshot), _utc_now()),
        )

    def _refresh_trajectory_lifecycle_locked(self, trajectory_id: str) -> None:
        state = self.trajectory_snapshot(trajectory_id)
        status = state.get("status") if isinstance(state.get("status"), str) else "active"
        completed = is_terminal_status(status)
        self._connection.execute(
            """
            UPDATE trajectories
            SET status = ?, completed = ?,
                started_at = COALESCE(?, started_at),
                finished_at = CASE WHEN ? THEN COALESCE(?, finished_at) ELSE NULL END
            WHERE trajectory_id = ?
            """,
            (
                status,
                int(completed),
                state.get("started_at"),
                int(completed),
                state.get("finished_at"),
                trajectory_id,
            ),
        )

    @staticmethod
    def _trajectory_summary(row: sqlite3.Row) -> dict[str, Any]:
        value = {
            "trajectory_id": row["trajectory_id"],
            "name": row["name"],
            "pinned": bool(row["pinned"]),
            "created_at": row["created_at"],
            "updated_at": row["updated_at"],
            "started_at": row["started_at"],
            "finished_at": row["finished_at"],
            "status": row["status"],
            "completed": bool(row["completed"]),
            "last_sequence": row["last_sequence"],
            "event_count": row["event_count"],
            "byte_size": row["byte_size"],
        }
        value["cursor"] = _cursor_encode(value["updated_at"], value["trajectory_id"])
        return value


def _is_explicit_checkpoint(event: Mapping[str, Any]) -> bool:
    return str(event.get("type", "")).lower() in {
        "checkpoint",
        "checkpoint.created",
        "trajectory.checkpoint",
        "state.checkpoint",
    }


def _first_causation(event: Mapping[str, Any]) -> str | None:
    values = _causal_ids(event)
    return values[0] if values else None


def _correlation_id(event: Mapping[str, Any]) -> str | None:
    payload = event.get("payload") if isinstance(event.get("payload"), Mapping) else {}
    for source in (event, payload):
        value = source.get("correlation_id")
        if isinstance(value, str) and value:
            return value
    return None


__all__ = [
    "DebugStore",
    "EventConflictError",
    "StoreError",
    "TrajectoryNotFoundError",
    "json_safe",
]
