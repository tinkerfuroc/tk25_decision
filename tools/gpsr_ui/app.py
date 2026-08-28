# tools/gpsr_ui/app.py
"""FastAPI app: JSON API + HTML corpus browser over the read-only bench.

`_resolve` deviates from the brief on purpose (per controller ruling): the
brief's version calls `find_run` (which itself calls `list_tiers`) and
then calls `list_tiers` a second time to locate the matching `Attempt` --
two full walks of a 2.4 GB, ~92-run corpus per API request. This version
walks the corpus exactly once per request, searching that single
`list_tiers` result for both the run's path and its `Attempt`. Behaviour
is unchanged: an unknown run or a traversal attempt in `dir_name` still
404s. `corpus.find_run` is consequently unused here; it is left in place
in corpus.py, which is out of scope for this task and is exercised by its
own tests.

Route shape deviates too: the brief's `/api/run/{tier}/{dir_name}` assumes
a tier name is always a single path segment, but corpus.py's pseudo-tiers
(runs-invalidated-*, runs-diagnostics-* siblings of a tier's plain `runs`
dir) are named `"<tier>/<suffix>"` -- WITH an embedded slash -- and these
exist in the real corpus today (t2-2026/invalidated-20260826,
t2-2026/diagnostics-20260827). A plain two-segment route can never match
those. The route below instead captures the whole remainder as one
`{path:path}` segment and rpartitions it on the LAST slash into
(tier, dir_name), so a multi-segment tier name resolves correctly while a
single-segment one (as in every brief test) is unaffected.

Task 7's frame routes inherit the same "tier may embed a slash" problem,
plus one of their own. The brief proposes `/api/run/{tier}/{dir_name}/frames`
and `/frame/{tier}/{dir_name}/{label}/{file}` -- both assume a fixed
segment count, which a slash-bearing pseudo-tier already breaks (same
defect as above). Worse, a same-prefix fix along the lines of
`/api/run/{path:path}/frames` is ambiguous given the existing
`/api/run/{path:path}` route: Starlette matches routes in registration
order, and `{path:path}` is greedy, so a request for
`/api/run/t9/some-run/frames` would already satisfy the existing
`api_run` route (with `path="t9/some-run/frames"`, whose rpartition then
sees a bogus dir_name of "frames") before ever reaching a route added
after it -- the more-specific suffix route would only work by accident of
declaration order, and even then only if no run happens to be named
"frames".

The frame routes below dodge this by using a DISTINCT top-level prefix
per resource, so the greedy segment is unambiguously the very last thing
in the URL and never collides with `/api/run/...`:

  GET /api/frames/{path:path}        -- path = "tier[/suffix]/dir_name"
  GET /frame/{path:path}             -- path = "tier[/suffix]/dir_name/label/file"

The frame-serving route peels its trailing `label` and `file` segments
off with `rsplit("/", 2)` (equivalent to two rpartitions from the right)
and resolves the remaining "tier[/suffix]/dir_name" through the same
`_resolve` used by `api_run`, so a run under a slash-bearing pseudo-tier
(e.g. t2-2026/invalidated-20260826/some-run) is addressable for frames
exactly as it is for the plain run API.
"""
from __future__ import annotations

from dataclasses import asdict
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from .cache import cached_run_model
from .clock import load_clock
from .config import Settings, load_settings
from .corpus import Attempt, list_tiers
from .frames import frame_path, list_frames

_HERE = Path(__file__).parent


def _attempt_json(attempt: Attempt) -> dict:
    data = asdict(attempt)
    data["path"] = str(attempt.path)
    return data


def _run_json(run_dir: Path, model, clock, attempt: Attempt | None) -> dict:
    return {
        "dir_name": run_dir.name,
        "trajectory_id": model.trajectory_id,
        "verdict": attempt.verdict if attempt else None,
        "seconds": attempt.seconds if attempt else None,
        "detail": attempt.detail if attempt else "",
        "started_wall": model.started_wall,
        "finished_wall": model.finished_wall,
        "tree_regenerations": model.tree_regenerations,
        "gate_failures": model.gate_failures,
        "clock_mode": clock.mode,
        "clock_labels": clock.labels,
        "announcements": model.announcements,
        "epochs": [
            {
                "ordinal": e.ordinal,
                "wall": e.wall,
                "sequence": e.sequence,
                "root_id": e.root_id,
                "nodes": [asdict(n) for n in e.nodes.values()],
            }
            for e in model.epochs
        ],
        "transitions": [asdict(t) for t in model.transitions],
        "milestones": [asdict(m) for m in model.milestones],
        "judge_events": [asdict(j) for j in model.judge_events],
    }


def create_app(settings: Settings | None = None) -> FastAPI:
    settings = settings or load_settings()
    app = FastAPI(title="GPSR Bench Run Viewer")
    app.state.settings = settings

    templates = Jinja2Templates(directory=str(_HERE / "templates"))
    app.mount(
        "/static", StaticFiles(directory=str(_HERE / "static")), name="static")

    @app.get("/healthz")
    def healthz() -> dict:
        return {"ok": True}

    @app.get("/api/tiers")
    def api_tiers() -> JSONResponse:
        tiers = list_tiers(settings.bench_root)
        return JSONResponse({
            "tiers": [
                {
                    "name": t.name,
                    "entries": [
                        {
                            "entry_id": e.entry_id,
                            "template": e.template,
                            "feasibility": e.feasibility,
                            "text": e.text,
                            "attempts": [_attempt_json(a) for a in e.attempts],
                        }
                        for e in t.entries
                    ],
                }
                for t in tiers
            ]
        })

    def _resolve(path: str) -> tuple[Path, Attempt]:
        """Resolve a raw `tier/dir_name` (or `tier/suffix/dir_name`) path
        with a SINGLE list_tiers() walk.

        Splits on the LAST slash so a pseudo-tier name with an embedded
        slash (e.g. "t2-2026/invalidated-20260826") still resolves, then
        searches one discovery pass for both the run directory and its
        Attempt -- instead of the brief's find_run() + a second
        list_tiers() pass (two full corpus walks per request).

        A dir_name of "", "." or ".." never matches a real Attempt (no
        run directory is named that), so it 404s without needing a
        dedicated traversal check; the same is true of any tier string
        built from ".." segments, since it can never equal a real
        discovered Tier.name.
        """
        tier, _, dir_name = path.rpartition("/")
        if dir_name in {"", ".", ".."}:
            raise HTTPException(status_code=404, detail="run not found")
        for candidate in list_tiers(settings.bench_root):
            if candidate.name != tier:
                continue
            for entry in candidate.entries:
                for attempt in entry.attempts:
                    if attempt.dir_name == dir_name:
                        return attempt.path, attempt
        raise HTTPException(status_code=404, detail="run not found")

    @app.get("/api/run/{path:path}")
    def api_run(path: str) -> JSONResponse:
        run_dir, attempt = _resolve(path)
        model = cached_run_model(run_dir, settings.state_dir)
        clock = load_clock(run_dir)
        return JSONResponse(_run_json(run_dir, model, clock, attempt))

    @app.get("/api/frames/{path:path}")
    def api_frames(path: str) -> JSONResponse:
        run_dir, _ = _resolve(path)
        frames = list_frames(run_dir)
        return JSONResponse({
            "labels": {
                label: [asdict(r) for r in refs]
                for label, refs in frames.items()
            }
        })

    @app.get("/frame/{path:path}")
    def frame(path: str):
        parts = path.rsplit("/", 2)
        if len(parts) != 3:
            raise HTTPException(status_code=404, detail="frame not found")
        run_path, label, file = parts
        run_dir, _ = _resolve(run_path)
        resolved = frame_path(run_dir, label, file)
        if resolved is None:
            raise HTTPException(status_code=404, detail="frame not found")
        return FileResponse(
            resolved,
            media_type="image/jpeg",
            # Frames are immutable once written; cache them hard.
            headers={"Cache-Control": "public, max-age=31536000, immutable"},
        )

    @app.get("/")
    def index(request: Request):
        return templates.TemplateResponse(
            request, "index.html", {"tiers": list_tiers(settings.bench_root)},
        )

    return app


app = create_app()
