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

For a hardware-free smoke run, select the committed fixture context and the
existing full mock configuration:

```bash
export GPSR_SUPERVISOR_CONTEXT=fixture
export GPSR_RUN_LIVE_LLM_TESTS=1
PYTHONPATH=src/behavior_tree:src/gpsr_trace:src/gpsr_debug_server \
  python -m pytest -m live_openrouter \
  src/behavior_tree/test/test_gpsr_supervision_live.py
```

The complete regression catalogue contains ten cases and eleven checkpoints
(case 03 has a pre- and post-recovery-budget checkpoint). It covers all four
verdicts, all three BT assessments, all four escalation values, every local
macro and every global action. The opt-in live matrix performs three verifier
samples for all eleven checkpoints plus three planner samples for eight
checkpoints: 57 OpenRouter calls total.

```bash
export GPSR_RUN_LIVE_LLM_TESTS=1
export GPSR_LIVE_REPORT=/tmp/gpsr-vlm-ten-case/live-results.json
PYTHONPATH=src/behavior_tree:src/gpsr_trace:src/gpsr_debug_server \
  python -m pytest -q \
  src/behavior_tree/test/test_gpsr_supervision_matrix_live.py

PYTHONPATH=src/behavior_tree:src/gpsr_trace:src/gpsr_debug_server \
  python src/gpsr_debug_server/tools/seed_supervisor_demo.py \
  --state-dir /tmp/gpsr-vlm-ten-case/dashboard \
  --live-results "$GPSR_LIVE_REPORT" --replace

PYTHONPATH=src/behavior_tree:src/gpsr_trace:src/gpsr_debug_server \
  python src/gpsr_debug_server/tools/render_supervisor_contact_sheet.py \
  --state-dir /tmp/gpsr-vlm-ten-case/dashboard \
  --live-results "$GPSR_LIVE_REPORT" \
  --output /tmp/gpsr-vlm-ten-case/contact-sheet.png
```

Real robot deployments must call `configure_default_supervisor()` with a
production `ContextProvider`. That worker is responsible for capturing both
cameras and rendering current map/arm state. It returns the same four-role
`SnapshotBundle` used by the fixture provider.

The hardware-free arm image is not a schematic: it applies the selected GPSR
runtime pose to the generated `xarm7.urdf` and rasterizes the Tinker base,
xArm, gripper, and D435 visual STL meshes headlessly. The 768×768 high-contrast
image is scene-only—there is no right-side joint dashboard—and includes the
calibrated wrist optical axis. Joint values and the named pose remain in
artifact metadata. The suite exercises `arm_pos_navigating`,
`arm_pos_orbbec_look`, `arm_pos_scan`, and `arm_pos_scan_original` with
direction-matched wrist fixtures. A real AprilTag calibration frame is kept as
`wrist_camera_mismatch.jpg` only for case 08.

Verifier prompt v3 first checks whether all four artifacts could belong to the
same checkpoint. A stale/different-scene camera frame, impossible viewpoint, or
pose contradiction returns `uncertain`, `stop`, and
`sensor_context_mismatch`; it is not misclassified as destructive world
change.

Local recovery output is never executable model text. It is validated against
one of five schemas (`scan_views`, `reacquire_object`, `retry_navigation`,
`relocalize`, `ask_human`) and passed to a trusted handler. Fixture/full-mock
runs acknowledge those directives without accessing robot services; production
must install the corresponding handlers.
