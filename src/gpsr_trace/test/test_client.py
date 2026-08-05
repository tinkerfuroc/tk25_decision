import json
from pathlib import Path
import socket
import threading
import time

from gpsr_trace import BoundedSpool, TraceClient


def _collector(path: Path, received: list[bytes], ready: threading.Event, stop: threading.Event) -> threading.Thread:
    def run() -> None:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(str(path))
            server.listen()
            server.settimeout(0.05)
            ready.set()
            while not stop.is_set():
                try:
                    connection, _ = server.accept()
                except TimeoutError:
                    continue
                with connection:
                    chunks: list[bytes] = []
                    while True:
                        data = connection.recv(65536)
                        if not data:
                            break
                        chunks.append(data)
                    received.append(b"".join(chunks))

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread


def _wait_for(predicate, timeout: float = 1.5) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def test_trace_client_delivers_batched_ndjson_and_redacts(tmp_path: Path) -> None:
    socket_path = tmp_path / "collector.sock"
    received: list[bytes] = []
    ready, stop = threading.Event(), threading.Event()
    server = _collector(socket_path, received, ready, stop)
    assert ready.wait(1)
    client = TraceClient(socket_path, source_id="test-planner", batch_size=2, flush_interval=0.01)
    try:
        assert client.emit("task.started", {"token": "secret"})
        assert client.emit("task.finished", {"ok": True})
        assert client.flush(1)
        assert _wait_for(lambda: bool(received))
    finally:
        client.close()
        stop.set()
        server.join(1)

    records = [json.loads(line) for line in b"".join(received).splitlines()]
    assert [record["sequence"] for record in records] == [0, 1]
    assert records[0]["payload"]["token"] == "[REDACTED]"
    assert client.stats.delivered == 2


def test_spool_is_bounded_and_replayed_when_collector_returns(tmp_path: Path) -> None:
    socket_path = tmp_path / "later.sock"
    spool_path = tmp_path / "trace.ndjson"
    client = TraceClient(
        socket_path,
        source_id="replay-test",
        spool_path=spool_path,
        spool_max_bytes=4096,
        flush_interval=0.02,
        socket_timeout=0.02,
    )
    try:
        assert client.emit("offline.event", {"message": "queued"})
        assert client.flush(1)
        assert spool_path.exists() and spool_path.stat().st_size > 0

        received: list[bytes] = []
        ready, stop = threading.Event(), threading.Event()
        server = _collector(socket_path, received, ready, stop)
        assert ready.wait(1)
        assert _wait_for(lambda: bool(received))
        stop.set()
        server.join(1)
        assert b"offline.event" in b"".join(received)
        assert client.stats.replayed == 1
    finally:
        client.close()


def test_bounded_spool_evicts_oldest_whole_records(tmp_path: Path) -> None:
    spool = BoundedSpool(tmp_path / "small.ndjson", max_bytes=6)
    assert spool.append([b"a\n", b"bb\n", b"ccc\n"]) == 1
    assert spool.peek_batch(max_records=10, max_bytes=100) == [b"ccc\n"]
