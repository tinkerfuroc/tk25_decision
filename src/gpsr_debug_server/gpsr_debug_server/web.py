"""FastAPI surface for the standalone GPSR mission debugger.

The server is intentionally a thin adapter over :mod:`store`: all durable
state and replay semantics live below the HTTP layer. The module remains
importable in ROS-free/unit-test environments where FastAPI is not installed.
"""
from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
import hmac
import ipaddress
import secrets
import time
from pathlib import Path
from typing import Any, Callable
from urllib.parse import urlsplit

try:  # Optional for the offline protocol/store tests.
    from fastapi import Body, Depends, FastAPI, Header, HTTPException, Request, WebSocket, WebSocketDisconnect
    from fastapi.responses import FileResponse, JSONResponse
except ImportError:  # pragma: no cover
    FastAPI = None  # type: ignore[assignment]


class EventBroker:
    """Small bounded fan-out broker shared by HTTP and ROS callbacks."""

    def __init__(self) -> None:
        self._queues: set[asyncio.Queue] = set()
        self._loop: asyncio.AbstractEventLoop | None = None

    def attach_loop(self) -> None:
        self._loop = asyncio.get_running_loop()

    async def publish(self, event: dict[str, Any]) -> None:
        for queue in tuple(self._queues):
            if queue.full():
                try:
                    queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            queue.put_nowait(event)

    def publish_threadsafe(self, event: dict[str, Any]) -> None:
        if self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(self.publish(event), self._loop)

    def subscribe(self) -> asyncio.Queue:
        queue: asyncio.Queue = asyncio.Queue(maxsize=512)
        self._queues.add(queue)
        return queue

    def unsubscribe(self, queue: asyncio.Queue) -> None:
        self._queues.discard(queue)


class ControllerLease:
    def __init__(self, ttl_s: float = 60.0) -> None:
        self.ttl_s = ttl_s
        self.owner: str | None = None
        self.expires_at = 0.0

    def acquire(self, owner: str) -> dict[str, Any]:
        now = time.monotonic()
        if self.owner and self.expires_at > now and self.owner != owner:
            raise PermissionError("another debugger session owns the controller lease")
        self.owner = owner
        self.expires_at = now + self.ttl_s
        return self.public()

    def renew(self, owner: str) -> dict[str, Any]:
        if self.owner != owner or self.expires_at <= time.monotonic():
            raise PermissionError("controller lease is not owned by this session")
        self.expires_at = time.monotonic() + self.ttl_s
        return self.public()

    def release(self, owner: str) -> dict[str, Any]:
        if self.owner == owner:
            self.owner = None
            self.expires_at = 0.0
        return self.public()

    def require(self, owner: str) -> None:
        if self.owner != owner or self.expires_at <= time.monotonic():
            raise PermissionError("a live controller lease is required")

    def public(self) -> dict[str, Any]:
        remaining = max(0.0, self.expires_at - time.monotonic()) if self.owner else 0.0
        return {"owner": self.owner, "expires_in_s": round(remaining, 3), "ttl_s": self.ttl_s}


def create_app(
    store: Any,
    *,
    webui_dir: str | Path,
    artifact_dir: str | Path | None = None,
    secret: str | None = None,
    control_submitter: Callable[[dict[str, Any]], Any] | None = None,
    on_ingest: Callable[[dict[str, Any]], None] | None = None,
):
    if FastAPI is None:  # pragma: no cover - exercised by dependency checks
        raise RuntimeError("install gpsr_debug_server[web] to run the web server")

    webui = Path(webui_dir)
    artifacts = Path(artifact_dir).resolve() if artifact_dir is not None else None
    token = secret or secrets.token_urlsafe(32)
    broker = EventBroker()
    lease = ControllerLease()

    @asynccontextmanager
    async def lifespan(_app):
        broker.attach_loop()
        yield

    app = FastAPI(
        title="GPSR Mission Debugger",
        version="0.1.0",
        docs_url=None,
        redoc_url=None,
        openapi_url=None,
        lifespan=lifespan,
    )
    app.state.store = store
    app.state.broker = broker
    app.state.controller_lease = lease
    app.state.session_token = token

    def auth_value(request: Request) -> str:
        return (
            request.headers.get("x-gpsr-session")
            or request.headers.get("x-tinker-session")
            or request.query_params.get("token", "")
        )

    async def read_guard(request: Request) -> None:
        if not hmac.compare_digest(auth_value(request), token):
            raise HTTPException(403, "invalid debugger session token")

    async def mutation_guard(
        request: Request,
        x_gpsr_session: str | None = Header(default=None),
    ) -> None:
        supplied = x_gpsr_session or auth_value(request)
        if not hmac.compare_digest(supplied, token):
            raise HTTPException(403, "invalid debugger session token")
        origin = request.headers.get("origin")
        if origin and urlsplit(origin).netloc != request.headers.get("host", ""):
            raise HTTPException(403, "cross-origin mutation rejected")

    @app.middleware("http")
    async def loopback_only(request: Request, call_next):
        host = urlsplit(f"//{request.headers.get('host', '')}").hostname
        allowed = host == "localhost"
        try:
            allowed = allowed or bool(host and ipaddress.ip_address(host).is_loopback)
        except ValueError:
            pass
        if not allowed:
            return JSONResponse({"error": "loopback host required"}, status_code=403)
        return await call_next(request)

    def publish(event: dict[str, Any]) -> None:
        broker.publish_threadsafe({"type": "gpsr.event", "event": event})

    def append_event(event: dict[str, Any]) -> dict[str, Any]:
        if on_ingest:
            on_ingest(event)
        result = store.append_event(event)
        publish(event)
        return {"accepted": bool(result), "event_id": event.get("event_id"), "sequence": event.get("sequence")}

    @app.get("/")
    @app.get("/debug")
    async def index():
        return FileResponse(webui / "index.html", headers={"Cache-Control": "no-store"})

    @app.get("/app.js")
    async def javascript():
        return FileResponse(webui / "app.js", media_type="application/javascript", headers={"Cache-Control": "no-cache"})

    @app.get("/ui_model.js")
    async def ui_model():
        return FileResponse(
            webui / "ui_model.js",
            media_type="application/javascript",
            headers={"Cache-Control": "no-cache"},
        )

    @app.get("/style.css")
    async def stylesheet():
        return FileResponse(webui / "style.css", media_type="text/css", headers={"Cache-Control": "no-cache"})

    @app.get("/vendor/cytoscape.min.js")
    async def cytoscape_vendor():
        """Serve the optional pinned graph-layout dependency at one fixed path."""

        root = webui.resolve()
        asset = (root / "vendor" / "cytoscape.min.js").resolve()
        if not asset.is_file() or root not in asset.parents:
            raise HTTPException(404, "Cytoscape vendor asset is not installed")
        return FileResponse(
            asset,
            media_type="application/javascript",
            headers={"Cache-Control": "public, max-age=86400"},
        )

    @app.get("/api/v1/artifacts/{name}", dependencies=[Depends(read_guard)])
    async def checkpoint_artifact(name: str):
        """Serve one captured image from the debugger-owned artifact root."""

        if artifacts is None or not name or Path(name).name != name:
            raise HTTPException(404, "unknown checkpoint artifact")
        asset = (artifacts / name).resolve()
        if artifacts not in asset.parents or not asset.is_file():
            raise HTTPException(404, "unknown checkpoint artifact")
        return FileResponse(asset, headers={"Cache-Control": "private, max-age=3600"})

    @app.get("/api/v1/session")
    async def session():
        return {"token": token, "mode": "developer-only", "controller_lease": lease.public()}

    @app.get("/api/v1/trajectories", dependencies=[Depends(read_guard)])
    async def trajectories(limit: int = 50, cursor: str | None = None):
        return {"trajectories": store.list_trajectories(limit=min(max(limit, 1), 500), cursor=cursor)}

    @app.get("/api/v1/trajectories/{trajectory_id}", dependencies=[Depends(read_guard)])
    async def trajectory(trajectory_id: str, at: int | None = None):
        try:
            return store.trajectory_snapshot(trajectory_id, at_sequence=at)
        except KeyError as exc:
            raise HTTPException(404, f"unknown trajectory: {trajectory_id}") from exc

    @app.get("/api/v1/trajectories/{trajectory_id}/events", dependencies=[Depends(read_guard)])
    async def trajectory_events(trajectory_id: str, after: int = -1, limit: int = 1000):
        try:
            events = store.events(trajectory_id, after=max(-1, after), limit=min(max(1, limit), 10000))
        except KeyError as exc:
            raise HTTPException(404, f"unknown trajectory: {trajectory_id}") from exc
        return {"events": events}

    @app.get("/api/v1/trajectories/{trajectory_id}/events/{event_id}/causal", dependencies=[Depends(read_guard)])
    async def causal_events(
        trajectory_id: str,
        event_id: str,
        direction: str = "ancestors",
        limit: int = 200,
    ):
        if direction not in {"ancestors", "descendants"}:
            raise HTTPException(422, "direction must be 'ancestors' or 'descendants'")
        try:
            events = store.causal_events(
                trajectory_id,
                event_id,
                direction=direction,
                limit=min(max(limit, 1), 1000),
            )
        except KeyError as exc:
            raise HTTPException(404, f"unknown trajectory or event: {event_id}") from exc
        return {"events": events}

    @app.get("/api/v1/trajectories/{trajectory_id}/trees/{revision}", dependencies=[Depends(read_guard)])
    async def tree(trajectory_id: str, revision: str):
        try:
            return store.tree_document(trajectory_id, revision)
        except KeyError as exc:
            raise HTTPException(404, f"unknown tree revision: {revision}") from exc

    @app.patch("/api/v1/trajectories/{trajectory_id}", dependencies=[Depends(mutation_guard)])
    async def rename_trajectory(trajectory_id: str, document: dict = Body(...)):
        name = document.get("name")
        if name is not None and (not isinstance(name, str) or len(name) > 128):
            raise HTTPException(422, "name must be a string of at most 128 characters")
        result = store.trajectory_snapshot(trajectory_id).get("trajectory", {})
        if "name" in document:
            result = store.set_trajectory_name(trajectory_id, name)
        if "pinned" in document:
            if not isinstance(document["pinned"], bool):
                raise HTTPException(422, "pinned must be boolean")
            result = store.set_trajectory_pinned(trajectory_id, document["pinned"])
        publish({"schema": "tinker.gpsr.telemetry", "schema_version": 1, "type": "trajectory.renamed", "trajectory_id": trajectory_id, "payload": {"name": name}})
        return result

    @app.post("/api/v1/control/lease", dependencies=[Depends(mutation_guard)])
    async def acquire_lease(request: Request):
        owner = request.headers.get("x-gpsr-controller") or secrets.token_urlsafe(12)
        try:
            value = lease.acquire(owner)
        except PermissionError as exc:
            raise HTTPException(409, str(exc)) from exc
        return {"controller_id": owner, "lease": value}

    @app.post("/api/v1/control/lease/renew", dependencies=[Depends(mutation_guard)])
    async def renew_lease(request: Request):
        owner = request.headers.get("x-gpsr-controller", "")
        try:
            return {"controller_id": owner, "lease": lease.renew(owner)}
        except PermissionError as exc:
            raise HTTPException(409, str(exc)) from exc

    @app.delete("/api/v1/control/lease", dependencies=[Depends(mutation_guard)])
    async def release_lease(request: Request):
        owner = request.headers.get("x-gpsr-controller", "")
        return {"lease": lease.release(owner)}

    @app.post("/api/v1/trajectories/{trajectory_id}/commands", dependencies=[Depends(mutation_guard)])
    async def command(trajectory_id: str, request: Request, document: dict = Body(...)):
        owner = request.headers.get("x-gpsr-controller", "")
        try:
            lease.require(owner)
        except PermissionError as exc:
            raise HTTPException(409, str(exc)) from exc
        allowed = {"pause", "resume", "cancel", "retry_step", "skip_step", "capture_checkpoint", "apply_patch", "rollback", "edit_state", "agent_directive", "activate_proposal"}
        kind = document.get("command")
        if kind not in allowed:
            raise HTTPException(422, f"unsupported command: {kind!r}")
        payload = {"trajectory_id": trajectory_id, "request_id": document.get("request_id") or secrets.token_hex(12), "controller_id": owner, "command": kind, "expected_revision": document.get("expected_revision"), "payload": document.get("payload", {}), "justification": document.get("justification", "")}
        result = control_submitter(payload) if control_submitter else {"status": "unavailable", "message": "GPSR debug gateway is not connected"}
        if asyncio.iscoroutine(result):
            result = await result
        publish({"schema": "tinker.gpsr.telemetry", "schema_version": 1, "type": "control.requested", "trajectory_id": trajectory_id, "payload": payload})
        return {"request": payload, "result": result}

    @app.post("/api/v1/ingest", dependencies=[Depends(mutation_guard)])
    async def ingest(document: dict = Body(...)):
        return append_event(document)

    @app.websocket("/api/v1/stream")
    async def stream(websocket: WebSocket):
        if not hmac.compare_digest(websocket.query_params.get("token", ""), token):
            await websocket.close(code=1008)
            return
        await websocket.accept()
        queue = broker.subscribe()
        try:
            await websocket.send_json({"type": "connected", "lease": lease.public()})
            while True:
                try:
                    value = await asyncio.wait_for(queue.get(), timeout=20.0)
                    await websocket.send_json(value)
                except asyncio.TimeoutError:
                    await websocket.send_json({"type": "heartbeat", "lease": lease.public()})
        except WebSocketDisconnect:
            pass
        finally:
            broker.unsubscribe(queue)

    return app
