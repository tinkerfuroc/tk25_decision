"""Focused HTTP tests for causal replay and fixed vendor asset serving."""
from __future__ import annotations

import pytest


pytest.importorskip("fastapi")
from fastapi.testclient import TestClient

from gpsr_debug_server.store import DebugStore
from gpsr_debug_server.web import create_app


def _event(sequence: int, event_id: str, event_type: str, payload: dict | None = None) -> dict:
    return {
        "trajectory_id": "trace-1",
        "sequence": sequence,
        "event_id": event_id,
        "type": event_type,
        "occurred_at": f"2026-08-03T00:00:{sequence:02d}Z",
        "payload": payload or {},
    }


def test_causal_endpoint_is_authenticated_directional_and_bounded(tmp_path) -> None:
    with DebugStore(":memory:") as store:
        store.append_event(_event(1, "cause", "trajectory.started"))
        child = _event(2, "child", "plan.committed", {"revision": 1})
        child["causation_id"] = "cause"
        store.append_event(child)
        app = create_app(store, webui_dir=tmp_path, secret="test-secret")
        client = TestClient(app, base_url="http://localhost")
        endpoint = "/api/v1/trajectories/trace-1/events/child/causal"

        assert client.get(endpoint).status_code == 403
        headers = {"x-gpsr-session": "test-secret"}
        ancestors = client.get(endpoint, headers=headers)
        assert ancestors.status_code == 200
        assert [item["event_id"] for item in ancestors.json()["events"]] == ["cause"]

        descendants = client.get(
            "/api/v1/trajectories/trace-1/events/cause/causal?direction=descendants&limit=999999",
            headers=headers,
        )
        assert descendants.status_code == 200
        assert [item["event_id"] for item in descendants.json()["events"]] == ["child"]
        assert client.get(endpoint + "?direction=sideways", headers=headers).status_code == 422
        assert client.get(
            "/api/v1/trajectories/trace-1/events/missing/causal", headers=headers
        ).status_code == 404


def test_cytoscape_route_only_serves_the_fixed_local_asset(tmp_path) -> None:
    with DebugStore(":memory:") as store:
        app = create_app(store, webui_dir=tmp_path, secret="test-secret")
        client = TestClient(app, base_url="http://localhost")
        route = "/vendor/cytoscape.min.js"
        assert client.get(route).status_code == 404

        vendor = tmp_path / "vendor"
        vendor.mkdir()
        asset = vendor / "cytoscape.min.js"
        asset.write_text("window.CytoscapeForTest = true;", encoding="utf-8")
        response = client.get(route)
        assert response.status_code == 200
        assert response.text == "window.CytoscapeForTest = true;"
        assert response.headers["content-type"].startswith("application/javascript")


def test_ui_model_route_serves_the_local_module(tmp_path) -> None:
    (tmp_path / "ui_model.js").write_text(
        "window.GpsrUiModel = {};",
        encoding="utf-8",
    )
    with DebugStore(":memory:") as store:
        app = create_app(store, webui_dir=tmp_path, secret="test-secret")
        client = TestClient(app, base_url="http://localhost")

        response = client.get("/ui_model.js")

        assert response.status_code == 200
        assert response.text == "window.GpsrUiModel = {};"
        assert response.headers["content-type"].startswith("application/javascript")


def test_checkpoint_artifacts_are_rooted_and_authenticated(tmp_path) -> None:
    webui = tmp_path / "webui"
    webui.mkdir()
    artifacts = tmp_path / "artifacts"
    artifacts.mkdir()
    (artifacts / "front.jpg").write_bytes(b"fixture-image")
    (tmp_path / "secret.jpg").write_bytes(b"outside-root")
    with DebugStore(":memory:") as store:
        app = create_app(
            store,
            webui_dir=webui,
            artifact_dir=artifacts,
            secret="test-secret",
        )
        client = TestClient(app, base_url="http://localhost")
        route = "/api/v1/artifacts/front.jpg"

        assert client.get(route).status_code == 403
        response = client.get(route, headers={"x-gpsr-session": "test-secret"})
        assert response.status_code == 200
        assert response.content == b"fixture-image"
        assert client.get(
            "/api/v1/artifacts/missing.jpg",
            headers={"x-gpsr-session": "test-secret"},
        ).status_code == 404
        assert client.get(
            "/api/v1/artifacts/%2E%2E%2Fsecret.jpg",
            headers={"x-gpsr-session": "test-secret"},
        ).status_code in {404, 422}
