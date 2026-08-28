# tools/tests/test_app.py
"""
NOTE on test harness: the brief's reference test used
`fastapi.testclient.TestClient`, which imports `starlette.testclient`.
That module raises `RuntimeError` at import time in this environment --
`httpx` is not installed in the project venv (confirmed via
`python -c "import httpx"` -> ModuleNotFoundError, and there is no `pip`
in the venv either, so nothing can be installed). Per the task's global
constraint to install nothing and report a blocker instead, this file
drives the app with a real uvicorn server on a loopback port and
`requests` (both already installed) rather than `TestClient`. The test
names, scenarios and assertions are unchanged from the brief.
"""
from __future__ import annotations

import socket
import threading
import time
from contextlib import contextmanager

import requests
import uvicorn

from gpsr_ui.app import create_app
from gpsr_ui.config import Settings


def _free_port() -> int:
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


class _LiveClient:
    """Thin requests-based stand-in for fastapi.testclient.TestClient."""

    def __init__(self, base_url: str):
        self._base_url = base_url

    def get(self, path: str):
        return requests.get(self._base_url + path, timeout=5)


@contextmanager
def _client(bench_root, tmp_path):
    settings = Settings(
        bench_root=bench_root,
        state_dir=tmp_path / "state",
        sheet_events_path=None,
    )
    app = create_app(settings)
    port = _free_port()
    config = uvicorn.Config(app, host="127.0.0.1", port=port, log_level="error")
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    deadline = time.time() + 5
    while not server.started and time.time() < deadline:
        time.sleep(0.01)
    try:
        yield _LiveClient(f"http://127.0.0.1:{port}")
    finally:
        server.should_exit = True
        thread.join(timeout=5)


def test_healthz(make_run, tmp_path):
    run = make_run(name="s9999-040-x")
    with _client(run.parents[2], tmp_path) as client:
        assert client.get("/healthz").json() == {"ok": True}


def test_tiers_api_groups_attempts(make_run, tmp_path):
    run = make_run(name="s9999-041-x", verdict="PASS")
    make_run(name="s9999-041-x.attempt2-foo", verdict="FAIL")
    with _client(run.parents[2], tmp_path) as client:
        body = client.get("/api/tiers").json()
        entries = body["tiers"][0]["entries"]
        assert len(entries) == 1
        assert len(entries[0]["attempts"]) == 2
        assert entries[0]["attempts"][0]["is_current"] is True


def test_run_api_returns_epochs_and_regeneration_count(make_run, tmp_path):
    run = make_run(name="s9999-042-x", epochs=[["a"], ["a", "b"]])
    with _client(run.parents[2], tmp_path) as client:
        body = client.get("/api/run/t9-test/s9999-042-x").json()
        assert body["tree_regenerations"] == 0
        assert len(body["epochs"]) == 2
        assert body["verdict"] == "PASS"
        assert body["clock_mode"] == "none"


def test_run_api_404s_for_an_unknown_run(make_run, tmp_path):
    run = make_run(name="s9999-043-x")
    with _client(run.parents[2], tmp_path) as client:
        assert client.get("/api/run/t9-test/nope").status_code == 404


def test_run_api_rejects_path_traversal(make_run, tmp_path):
    run = make_run(name="s9999-044-x")
    with _client(run.parents[2], tmp_path) as client:
        assert client.get("/api/run/t9-test/..%2F..%2Fetc").status_code == 404


def test_run_api_resolves_pseudo_tier_with_embedded_slash(make_run, tmp_path):
    """Real corpus fact: t2-2026/runs-invalidated-20260826 and
    t2-2026/runs-diagnostics-20260827 sit alongside t2-2026/runs, and
    corpus.list_tiers names them "t2-2026/invalidated-20260826" etc --
    WITH an embedded slash. A route that only captures a single path
    segment for `tier` can never address these. Simulate one by renaming
    make_run's plain "runs" dir to a "runs-<suffix>" sibling."""
    run = make_run(name="s9999-046-x", verdict="PASS")
    tier_dir = run.parents[1]
    runs_dir = run.parents[0]
    pseudo_runs_dir = tier_dir / "runs-invalidated-20260826"
    runs_dir.rename(pseudo_runs_dir)
    new_run = pseudo_runs_dir / run.name

    with _client(run.parents[2], tmp_path) as client:
        tiers_body = client.get("/api/tiers").json()
        names = [t["name"] for t in tiers_body["tiers"]]
        assert "t9-test/invalidated-20260826" in names

        resp = client.get(
            f"/api/run/t9-test/invalidated-20260826/{new_run.name}"
        )
        assert resp.status_code == 200
        assert resp.json()["verdict"] == "PASS"


def test_index_page_renders_the_entry(make_run, tmp_path):
    run = make_run(name="s9999-045-x", verdict="PASS")
    with _client(run.parents[2], tmp_path) as client:
        page = client.get("/")
        assert page.status_code == 200
        assert "s9999-045-x" in page.text
