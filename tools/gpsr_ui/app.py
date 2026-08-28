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
"""
from __future__ import annotations

from dataclasses import asdict
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from .cache import cached_run_model
from .clock import load_clock
from .config import Settings, load_settings
from .corpus import Attempt, list_tiers

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

    @app.get("/")
    def index(request: Request):
        return templates.TemplateResponse(
            request, "index.html", {"tiers": list_tiers(settings.bench_root)},
        )

    return app


app = create_app()
