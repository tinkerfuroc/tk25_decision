"""Task-neutral score/event tracing."""

from typing import Any


def record_event(
    blackboard: Any,
    *,
    phase: str,
    item: str,
    action: str,
    outcome: str,
    points_est: int,
) -> None:
    """Append an event when the caller exposes a ``score_trace`` key."""
    try:
        trace = list(blackboard.score_trace)
    except (AttributeError, KeyError, TypeError):
        return
    trace.append(
        {
            "phase": phase,
            "item": item,
            "action": action,
            "outcome": outcome,
            "points_est": points_est,
        }
    )
    blackboard.score_trace = trace

