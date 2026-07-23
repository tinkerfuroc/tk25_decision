"""Backward-compatible import for the packaged endpoint verifier."""

from behavior_tree.verify_task_endpoints import *  # noqa: F401,F403
from behavior_tree.verify_task_endpoints import main


if __name__ == "__main__":
    raise SystemExit(main())
