from __future__ import annotations

import asyncio
import json

from gpsr_debug_server.main import MAX_INGEST_EVENT_BYTES, UnixIngest


def test_unix_ingest_accepts_full_tree_sized_event(tmp_path) -> None:
    received: list[dict] = []

    def receive(event: dict) -> None:
        received.append(event)

    ingest = UnixIngest(tmp_path / "ingest.sock", receive)

    document = {
        "trajectory_id": "large-tree",
        "sequence": 1,
        "event_id": "tree-1",
        "event_type": "tree.generated",
        "payload": {"tree": "x" * (2 * 1024 * 1024)},
    }

    class Writer:
        def close(self) -> None:
            pass

        async def wait_closed(self) -> None:
            pass

    async def exercise() -> None:
        reader = asyncio.StreamReader(limit=MAX_INGEST_EVENT_BYTES)
        reader.feed_data(json.dumps(document).encode("utf-8") + b"\n")
        reader.feed_eof()
        await ingest._client(reader, Writer())

    asyncio.run(exercise())
    assert received == [document]
