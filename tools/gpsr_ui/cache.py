# tools/gpsr_ui/cache.py
"""Derived-model cache keyed on the event log's identity.

The cache never writes inside the corpus it reads: every write this
module performs lands under the caller-supplied `state_dir`, never under
`run_dir`. A live run's events.jsonl grows continuously; the cache key
is built from its path, size and mtime, so it changes on every append
and the model is recomputed while the run is in flight. That is the
intended behaviour, not a bug to optimise away.
"""
from __future__ import annotations

import hashlib
import os
import pickle
from pathlib import Path

from .telemetry import RunModel, load_run_model, newest_events_file


def cache_key(run_dir: Path) -> str:
    """Identity of a run's telemetry: path, size and mtime of its event log.

    A live run's log grows, changing the key on every append, so the model
    is recomputed while it is in flight. That is the intended behaviour.
    """
    run_dir = Path(run_dir)
    events = newest_events_file(run_dir)
    parts = [str(run_dir.resolve())]
    if events is not None:
        try:
            st = events.stat()
            parts += [str(events), str(st.st_size), repr(st.st_mtime)]
        except OSError:
            # Vanished between newest_events_file()'s check and this stat()
            # (concurrent corpus writers can rename/archive mid-read) --
            # degrade to a stable marker rather than raising.
            parts.append("stat-failed")
    else:
        parts.append("no-events")
    return hashlib.sha256("\0".join(parts).encode()).hexdigest()[:32]


def cached_run_model(run_dir: Path, state_dir: Path) -> RunModel:
    """Load run_dir's model, deriving it fresh only on a cache miss.

    All reads/writes performed by this function happen under `state_dir`
    (plus the read-only `load_run_model` call, which only ever reads
    `run_dir`). A cache read or write failure of any kind falls back to
    plain recomputation -- a corrupt, missing or unwritable cache entry
    is never fatal to the caller.
    """
    key = cache_key(run_dir)
    target = Path(state_dir) / "models" / f"{key}.pickle"
    try:
        with target.open("rb") as fh:
            model = pickle.load(fh)
        if isinstance(model, RunModel):
            return model
    except (OSError, pickle.PickleError, EOFError, AttributeError,
            ImportError, ValueError):
        pass  # missing, corrupt or unreadable cache entry: recompute

    model = load_run_model(run_dir)

    try:
        target.parent.mkdir(parents=True, exist_ok=True)
        # Write to a process-unique temp name and atomically rename into
        # place, so a crash mid-write can never leave a partially-written
        # file sitting at the final `target` path where a later reader
        # would try to unpickle it and either crash or silently load a
        # truncated model. os.replace() is atomic on the same filesystem,
        # which this is, since both live under state_dir.
        tmp = target.with_suffix(f".tmp-{os.getpid()}-{id(model)}")
        with tmp.open("wb") as fh:
            pickle.dump(model, fh)
        os.replace(tmp, target)
    except Exception:
        # A cache write failure -- state_dir unwritable, disk full, or any
        # other exception pickle.dump/os.replace can raise -- is never
        # fatal: `model` was already computed correctly above and the
        # caller still gets it, just uncached. Deliberately broad: the
        # promise is that a write problem here can never surface as a
        # crash to a reader of run telemetry. (Exception, not
        # BaseException, so Ctrl-C/SystemExit still propagate normally.)
        try:
            tmp.unlink(missing_ok=True)
        except (OSError, NameError):
            pass
    return model


def snapshot_mtimes(root: Path) -> dict[Path, float]:
    """Every file under `root` with its mtime, for read-only assertions."""
    out: dict[Path, float] = {}
    root = Path(root)
    try:
        walker = root.rglob("*")
    except OSError:
        return out
    while True:
        try:
            path = next(walker)
        except StopIteration:
            break
        except OSError:
            # A directory vanished mid-walk (concurrent corpus writer);
            # skip past it rather than aborting the whole snapshot.
            continue
        try:
            if path.is_file():
                out[path] = path.stat().st_mtime
        except OSError:
            continue
    return out
