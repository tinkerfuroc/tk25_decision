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


def test_run_page_renders_for_a_known_run(make_run, tmp_path):
    run = make_run(name="s9999-050-x", verdict="PASS")
    with _client(run.parents[2], tmp_path) as client:
        page = client.get("/run/t9-test/s9999-050-x")
        assert page.status_code == 200
        assert "s9999-050-x" in page.text
        assert 'id="ribbon"' in page.text


def test_run_page_404s_for_an_unknown_run(make_run, tmp_path):
    run = make_run(name="s9999-051-x")
    with _client(run.parents[2], tmp_path) as client:
        assert client.get("/run/t9-test/nope").status_code == 404


def test_run_page_resolves_pseudo_tier_with_embedded_slash(make_run, tmp_path):
    """Same real-corpus fact as test_run_api_resolves_pseudo_tier_with_
    embedded_slash: the HTML run page must also address runs whose tier
    name embeds a slash (e.g. t2-2026/invalidated-20260826), not just the
    JSON API."""
    run = make_run(name="s9999-052-x", verdict="PASS")
    tier_dir = run.parents[1]
    runs_dir = run.parents[0]
    pseudo_runs_dir = tier_dir / "runs-invalidated-20260826"
    runs_dir.rename(pseudo_runs_dir)
    new_run = pseudo_runs_dir / run.name

    with _client(run.parents[2], tmp_path) as client:
        resp = client.get(
            f"/run/t9-test/invalidated-20260826/{new_run.name}"
        )
        assert resp.status_code == 200
        assert new_run.name in resp.text


def test_index_page_renders_the_entry(make_run, tmp_path):
    run = make_run(name="s9999-045-x", verdict="PASS")
    with _client(run.parents[2], tmp_path) as client:
        page = client.get("/")
        assert page.status_code == 200
        assert "s9999-045-x" in page.text


def test_index_page_links_to_the_html_run_page_not_the_raw_json(
    make_run, tmp_path
):
    """Task 6 pointed index entries at /api/run/... (raw JSON) because no
    HTML run page existed yet. Now that /run/... exists, entries must link
    there instead."""
    run = make_run(name="s9999-057-x", verdict="PASS")
    with _client(run.parents[2], tmp_path) as client:
        page = client.get("/")
        assert 'href="/run/t9-test/s9999-057-x"' in page.text
        assert 'href="/api/run/t9-test/s9999-057-x"' not in page.text


def test_frames_api_lists_labels_for_a_two_camera_run(make_run, tmp_path):
    run = make_run(
        name="s9999-047-x",
        frames={"head": [(0, 1000)], "arena": [(0, 1000)]},
    )
    with _client(run.parents[2], tmp_path) as client:
        body = client.get("/api/frames/t9-test/s9999-047-x").json()
        assert sorted(body["labels"]) == ["arena", "head"]
        assert body["labels"]["head"][0]["file"] == "0000_1000.jpg"


def test_frames_api_handles_a_single_camera_run(make_run, tmp_path):
    """Real corpus fact: s2026-003-findObjInRoom has frames/head but no
    frames/arena. A run may genuinely have only one camera."""
    run = make_run(name="s9999-048-x", frames={"head": [(0, 1000)]})
    with _client(run.parents[2], tmp_path) as client:
        body = client.get("/api/frames/t9-test/s9999-048-x").json()
        assert list(body["labels"]) == ["head"]


def test_frame_route_serves_the_jpeg_bytes(make_run, tmp_path):
    run = make_run(name="s9999-049-x", frames={"head": [(0, 1000)]})
    with _client(run.parents[2], tmp_path) as client:
        resp = client.get("/frame/t9-test/s9999-049-x/head/0000_1000.jpg")
        assert resp.status_code == 200
        assert resp.content == b"\xff\xd8\xff\xd9"
        assert resp.headers["content-type"] == "image/jpeg"


def test_frame_route_404s_for_traversal_and_unknown_files(make_run, tmp_path):
    run = make_run(name="s9999-055-x", frames={"head": [(0, 1000)]})
    with _client(run.parents[2], tmp_path) as client:
        # unknown file
        assert client.get(
            "/frame/t9-test/s9999-055-x/head/nope.jpg"
        ).status_code == 404
        # literal ".." segment between run and label, decoded server-side
        assert client.get(
            "/frame/t9-test/s9999-055-x/../run.json"
        ).status_code == 404
        # escape attempt via label
        assert client.get(
            "/frame/t9-test/s9999-055-x/../head/0000_1000.jpg"
        ).status_code == 404
        # too few segments to even contain a run path
        assert client.get("/frame/onlyonepart").status_code == 404


def test_frame_routes_resolve_pseudo_tier_with_embedded_slash(
    make_run, tmp_path
):
    """Same real-corpus fact as test_run_api_resolves_pseudo_tier_with_
    embedded_slash: t2-2026/invalidated-20260826 style tier names carry
    an embedded slash. Frame routes must address runs under them too."""
    run = make_run(
        name="s9999-056-x", verdict="PASS", frames={"head": [(0, 1000)]},
    )
    tier_dir = run.parents[1]
    runs_dir = run.parents[0]
    pseudo_runs_dir = tier_dir / "runs-invalidated-20260826"
    runs_dir.rename(pseudo_runs_dir)
    new_run = pseudo_runs_dir / run.name

    with _client(run.parents[2], tmp_path) as client:
        frames_resp = client.get(
            f"/api/frames/t9-test/invalidated-20260826/{new_run.name}"
        )
        assert frames_resp.status_code == 200
        assert list(frames_resp.json()["labels"]) == ["head"]

        frame_resp = client.get(
            f"/frame/t9-test/invalidated-20260826/{new_run.name}"
            "/head/0000_1000.jpg"
        )
        assert frame_resp.status_code == 200
        assert frame_resp.content == b"\xff\xd8\xff\xd9"
