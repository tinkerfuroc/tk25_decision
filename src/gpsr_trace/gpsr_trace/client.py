"""Non-blocking Unix-domain-socket delivery for :class:`TraceEvent` records."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import queue
import socket
import tempfile
import threading
import time
from typing import Iterable, Mapping
from uuid import uuid4

from .events import ContentPolicy, RedactionPolicy, TraceEvent, ValidationError, _require_identifier


@dataclass(frozen=True)
class TraceClientStats:
    """A snapshot of client delivery counters."""

    emitted: int = 0
    enqueued: int = 0
    dropped: int = 0
    delivered: int = 0
    spooled: int = 0
    replayed: int = 0
    transport_failures: int = 0


class BoundedSpool:
    """A small, atomic-on-rewrite local NDJSON spool with oldest-first eviction."""

    def __init__(self, path: str | os.PathLike[str], *, max_bytes: int = 4 * 1024 * 1024) -> None:
        if not isinstance(max_bytes, int) or max_bytes < 1:
            raise ValidationError("spool max_bytes must be a positive integer")
        self.path = Path(path)
        self.max_bytes = max_bytes
        self._lock = threading.RLock()

    @staticmethod
    def _normalise_lines(lines: Iterable[bytes]) -> list[bytes]:
        normalised: list[bytes] = []
        for line in lines:
            if not isinstance(line, bytes):
                raise ValidationError("spool entries must be bytes")
            stripped = line.rstrip(b"\r\n")
            if stripped:
                normalised.append(stripped + b"\n")
        return normalised

    def _read_lines_locked(self) -> list[bytes]:
        try:
            content = self.path.read_bytes()
        except FileNotFoundError:
            return []
        return self._normalise_lines(content.splitlines())

    def _write_lines_locked(self, lines: list[bytes]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(f".{self.path.name}.{os.getpid()}.tmp")
        with temporary.open("wb") as handle:
            handle.writelines(lines)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, self.path)

    def append(self, lines: Iterable[bytes]) -> int:
        """Append records, evicting oldest whole records; return records retained."""

        additions = self._normalise_lines(lines)
        with self._lock:
            kept = self._read_lines_locked()
            for line in additions:
                if len(line) > self.max_bytes:
                    continue
                kept.append(line)
                while sum(len(item) for item in kept) > self.max_bytes:
                    kept.pop(0)
            self._write_lines_locked(kept)
            addition_ids = {id(line) for line in additions}
            return sum(1 for line in kept if id(line) in addition_ids)

    def peek_batch(self, *, max_records: int, max_bytes: int) -> list[bytes]:
        """Read, but do not remove, an oldest-first bounded batch."""

        if max_records < 1 or max_bytes < 1:
            raise ValidationError("spool batch limits must be positive")
        with self._lock:
            batch: list[bytes] = []
            size = 0
            for line in self._read_lines_locked():
                if batch and (len(batch) >= max_records or size + len(line) > max_bytes):
                    break
                if len(line) > max_bytes:
                    # Entries may only be oversized if a spool file was edited
                    # outside this class. Return it so a caller can still drain it.
                    batch.append(line)
                    break
                batch.append(line)
                size += len(line)
            return batch

    def discard(self, records: int) -> None:
        """Discard the given count of oldest records after successful delivery."""

        if not isinstance(records, int) or records < 0:
            raise ValidationError("records must be a non-negative integer")
        if not records:
            return
        with self._lock:
            lines = self._read_lines_locked()
            self._write_lines_locked(lines[records:])

    def size_bytes(self) -> int:
        """Return the current on-disk byte count, treating a missing file as zero."""

        with self._lock:
            try:
                return self.path.stat().st_size
            except FileNotFoundError:
                return 0


class TraceClient:
    """Accept trace events without blocking application work.

    Each client owns one default ``trace_id`` (the debugger trajectory id),
    while callers may override it for a child causal trace. Producers use
    ``put_nowait`` on a bounded queue.  A daemon worker batches
    events into NDJSON, reconnects per batch to the Unix socket, and moves a
    failed batch into a bounded local spool.  On a later successful connection
    it drains the spool before delivering fresh events, retaining event order.
    """

    def __init__(
        self,
        socket_path: str | os.PathLike[str],
        *,
        source_id: str | None = None,
        trace_id: str | None = None,
        queue_size: int = 1024,
        batch_size: int = 64,
        flush_interval: float = 0.25,
        socket_timeout: float = 0.25,
        spool_path: str | os.PathLike[str] | None = None,
        spool_max_bytes: int = 4 * 1024 * 1024,
        redaction: RedactionPolicy | None = None,
        content_policy: ContentPolicy | None = None,
    ) -> None:
        if not isinstance(queue_size, int) or queue_size < 1:
            raise ValidationError("queue_size must be a positive integer")
        if not isinstance(batch_size, int) or batch_size < 1:
            raise ValidationError("batch_size must be a positive integer")
        if not isinstance(flush_interval, (int, float)) or flush_interval <= 0:
            raise ValidationError("flush_interval must be positive")
        if not isinstance(socket_timeout, (int, float)) or socket_timeout <= 0:
            raise ValidationError("socket_timeout must be positive")
        self.socket_path = os.fspath(socket_path)
        if not self.socket_path:
            raise ValidationError("socket_path must be non-empty")
        self.source_id = source_id or f"gpsr-trace-{socket.gethostname()}-{os.getpid()}"
        _require_identifier("source_id", self.source_id)
        self.trace_id = trace_id or str(uuid4())
        _require_identifier("trace_id", self.trace_id)
        self.batch_size = batch_size
        self.flush_interval = float(flush_interval)
        self.socket_timeout = float(socket_timeout)
        safe_source = "".join(character if character.isalnum() else "_" for character in self.source_id)
        default_spool = Path(tempfile.gettempdir()) / "gpsr_trace" / f"{safe_source}.ndjson"
        self.spool = BoundedSpool(spool_path or default_spool, max_bytes=spool_max_bytes)
        self.redaction = redaction if redaction is not None else RedactionPolicy()
        self.content_policy = content_policy if content_policy is not None else ContentPolicy()
        if not isinstance(self.redaction, RedactionPolicy):
            raise ValidationError("redaction must be RedactionPolicy or None")
        if not isinstance(self.content_policy, ContentPolicy):
            raise ValidationError("content_policy must be ContentPolicy or None")

        self._queue: queue.Queue[TraceEvent] = queue.Queue(maxsize=queue_size)
        self._sequence = 0
        self._sequence_lock = threading.Lock()
        self._stats_lock = threading.Lock()
        self._stats = TraceClientStats()
        self._stop = threading.Event()
        self._closed = False
        self._lifecycle_lock = threading.Lock()
        self._worker = threading.Thread(target=self._run, name="gpsr-trace", daemon=True)
        self._worker.start()

    def _increment(self, **changes: int) -> None:
        with self._stats_lock:
            current = self._stats
            values = {field: getattr(current, field) + changes.get(field, 0) for field in current.__dataclass_fields__}
            self._stats = TraceClientStats(**values)

    @property
    def stats(self) -> TraceClientStats:
        """Return a consistent immutable delivery-counter snapshot."""

        with self._stats_lock:
            return self._stats

    def _next_sequence(self) -> int:
        with self._sequence_lock:
            sequence = self._sequence
            self._sequence += 1
            return sequence

    def emit(
        self,
        event_type: str,
        payload: Mapping[str, object] | None = None,
        *,
        trace_id: str | None = None,
        parent_event_id: str | None = None,
        causation_ids: tuple[str, ...] | list[str] = (),
    ) -> bool:
        """Create, redact, bound, and enqueue an event; never wait for I/O.

        A false return means the client is closed or its bounded producer queue
        is full.  Sequence numbers intentionally remain monotonic across a
        dropped event so drops are observable to a collector.
        """

        if payload is not None and not isinstance(payload, Mapping):
            raise ValidationError("payload must be a mapping or None")
        safe_payload = self.content_policy.apply(self.redaction.redact(dict(payload or {})), path="$.payload")
        event = TraceEvent.create(
            source_id=self.source_id,
            sequence=self._next_sequence(),
            event_type=event_type,
            payload=safe_payload,
            trace_id=trace_id or self.trace_id,
            parent_event_id=parent_event_id,
            causation_ids=causation_ids,
        )
        return self.submit(event)

    def submit(self, event: TraceEvent) -> bool:
        """Queue an already constructed event without modifying its envelope."""

        if not isinstance(event, TraceEvent):
            raise ValidationError("event must be TraceEvent")
        self._increment(emitted=1)
        with self._lifecycle_lock:
            if self._closed:
                self._increment(dropped=1)
                return False
            try:
                self._queue.put_nowait(event)
            except queue.Full:
                self._increment(dropped=1)
                return False
        self._increment(enqueued=1)
        return True

    def flush(self, timeout: float | None = None) -> bool:
        """Wait until accepted producer events are delivered or spooled.

        It does not wait for an absent collector to become available; once an
        event is durably put in the local spool it is considered flushed.
        """

        if timeout is not None and timeout < 0:
            raise ValidationError("flush timeout must be non-negative or None")
        if timeout is None:
            self._queue.join()
            return True
        deadline = time.monotonic() + timeout
        with self._queue.all_tasks_done:
            while self._queue.unfinished_tasks:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._queue.all_tasks_done.wait(remaining)
        return True

    def close(self, *, timeout: float | None = 2.0) -> bool:
        """Stop the worker after draining its queue to the collector or spool."""

        if timeout is not None and timeout < 0:
            raise ValidationError("close timeout must be non-negative or None")
        with self._lifecycle_lock:
            if not self._closed:
                self._closed = True
                self._stop.set()
        self._worker.join(timeout)
        return not self._worker.is_alive()

    def __enter__(self) -> "TraceClient":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _encode(self, events: Iterable[TraceEvent]) -> list[bytes]:
        return [(event.to_json() + "\n").encode("utf-8") for event in events]

    def _send(self, lines: list[bytes]) -> bool:
        if not lines:
            return True
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
                connection.settimeout(self.socket_timeout)
                connection.connect(self.socket_path)
                connection.sendall(b"".join(lines))
            return True
        except OSError:
            self._increment(transport_failures=1)
            return False

    def _store(self, lines: list[bytes]) -> None:
        try:
            retained = self.spool.append(lines)
        except OSError:
            # The producer was non-blocking and the transport failed; a broken
            # filesystem must not take down the daemon worker.
            self._increment(dropped=len(lines))
            return
        self._increment(spooled=retained, dropped=len(lines) - retained)

    def _replay_spool(self) -> bool:
        while True:
            try:
                lines = self.spool.peek_batch(max_records=self.batch_size, max_bytes=self.spool.max_bytes)
            except OSError:
                return False
            if not lines:
                return True
            if not self._send(lines):
                return False
            try:
                self.spool.discard(len(lines))
            except OSError:
                # Leave a duplicate for at-least-once replay rather than risk
                # discarding an event whose successful send cannot be recorded.
                return False
            self._increment(delivered=len(lines), replayed=len(lines))

    def _deliver(self, events: list[TraceEvent]) -> None:
        lines = self._encode(events)
        if not self._replay_spool() or not self._send(lines):
            self._store(lines)
            return
        self._increment(delivered=len(lines))

    def _run(self) -> None:
        batch: list[TraceEvent] = []
        while True:
            if self._stop.is_set() and self._queue.empty():
                if batch:
                    self._deliver(batch)
                    for _ in batch:
                        self._queue.task_done()
                return
            try:
                event = self._queue.get(timeout=self.flush_interval)
                batch.append(event)
                if len(batch) < self.batch_size:
                    continue
            except queue.Empty:
                if not batch:
                    self._replay_spool()
                    continue
            self._deliver(batch)
            for _ in batch:
                self._queue.task_done()
            batch = []
