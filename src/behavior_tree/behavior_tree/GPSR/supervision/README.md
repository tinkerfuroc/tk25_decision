# GPSR execution supervisor

This package verifies the terminal result of registered robot-effect leaves,
then either clears execution, compiles a typed local recovery directive, or
requests a minimal remaining-plan revision.

The normal GPSR runtime is unchanged unless supervision is enabled:

```bash
export GPSR_SUPERVISION_MODE=active
export GPSR_SUPERVISION_SUCCESS_MODE=hybrid
export GPSR_SUPERVISOR_MODEL=openai/gpt-5.6-luna
export GPSR_SUPERVISOR_VERIFY_EFFORT=medium
export GPSR_SUPERVISOR_PLAN_EFFORT=high
```

`GPSR_SUPERVISION_MODE=shadow` records decisions without controlling the tree.
`GPSR_SUPERVISION_SUCCESS_MODE=optimistic` lets successful leaves advance until
a negative verdict reaches a safe tick boundary. Failures remain blocking.

For a hardware-free run, select the committed fixture context and the existing
full mock configuration:

```bash
export GPSR_SUPERVISOR_CONTEXT=fixture
export GPSR_RUN_LIVE_LLM_TESTS=1
PYTHONPATH=src/behavior_tree:src/gpsr_trace:src/gpsr_debug_server \
  python -m pytest -m live_openrouter \
  src/behavior_tree/test/test_gpsr_supervision_live.py
```

Real robot deployments must call `configure_default_supervisor()` with a
production `ContextProvider`. That worker is responsible for capturing both
cameras and rendering current map/arm state. It returns the same four-role
`SnapshotBundle` used by the fixture provider.

Local recovery output is never executable model text. It is validated against
one of five schemas (`scan_views`, `reacquire_object`, `retry_navigation`,
`relocalize`, `ask_human`) and passed to a trusted handler. Fixture/full-mock
runs acknowledge those directives without accessing robot services; production
must install the corresponding handlers.
