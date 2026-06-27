# HRI arm-pointing and the `pan_bias` fix (2026-06-27)

**Scope:** how `BtNode_PointTo` aims the xArm at a seat/person, why the
seat-pointing call sites used to pass `pan_bias=math.pi`, and why that is now
`0.0`. The root cause lived in the **camera calibration**, not here — see the
vision-side write-up
`tk26_vision/src/pan_tilt/pan_tilt_calibration_deployment.md`.

---

## Symptom

HRI seat-recommendation pointed the arm ~12° clockwise of the seat shown
(correctly) in the vision log, and the error grew the further off-axis the seat
was. Person-pointing in the two-way introduction had the same class of error.

## How `BtNode_PointTo` computes the pan

The arm base (`link_base`) is aligned with `base_link` (identity / 0° yaw), and
the point-to ready pose `ARM_POS_POINT_TO` is FK-verified to aim **exactly along
joint0** (intrinsic pointing offset `C = 0`). So with a correct point in
`base_link`:

```
joint0 = wrap_to_pi( atan2(y, x) - pan_bias )
```

(see `TemplateNodes/pointing_math.compute_point_to_pan`). With a correct camera TF
the seat/person centroid is already correct in `base_link`, so `atan2(y, x)` aims
the arm directly and **`pan_bias` must be 0**.

## Root cause (why `pan_bias=math.pi` existed)

The pan-tilt camera extrinsic was **mis-deployed**: the camera (physically
forward) was modelled ~180° **backward** because a re-calibration was half-applied
(`tilt_offset` updated in `pan_tilt.yaml`, but the matched `camera_mount` never
reached the URDF — full story in the vision doc). Seat/person centroids therefore
came back rotated ~π about base-Z (landing behind the robot).

`pan_bias=math.pi` **cancelled** that ~180° in joint0, so the arm pointed roughly
right — but only roughly: it left a tilt/position-dependent residual (the "~12°,
worse off-axis" symptom) and broke the moment the camera TF was corrected.

## The fix

Once the camera TF was corrected (2026-06-27), the centroid is correct in
`base_link`, so every seat/person-pointing call site changed
`pan_bias=math.pi → 0.0`:

- `HRI/hri.py` → `createEscortAndSeat` (seat pointing)
- `HRI/hri.py` → `createTwoWayIntroduction` (point at guest1 + guest2; also wrapped
  each point in `Retry`/`FailureIsSuccess` so a transient pointing failure is
  best-effort, not a hard task failure)
- `HRI/point_at_seat.py` (standalone seat-pointing harness)
- docstrings corrected in `TemplateNodes/pointing_math.py` and
  `TemplateNodes/Manipulation.py` (`BtNode_PointTo`): the centroid is no longer
  "π-rotated about base Z"; `pan_bias=0` is the production default. The `math.pi`
  path is retained in the helper only for a genuinely π-rotated source frame
  (e.g. an un-fixed backward-camera TF).

## ⚠️ Coupling — do not deploy `pan_bias=0` alone

`pan_bias=0` is **only correct with the fixed camera TF**. If you deploy this
behavior_tree change but the robot still runs the old (backward) `camera_mount`
URDF / un-synced `pan_tilt.yaml`, the arm will point **~180° off**. The three
parts ship together:

1. `tk26_vision` — `pan_tilt/config/pan_tilt.yaml` (`tilt_offset_rad`, `pan_offset_rad`).
2. `tk25_basic` — `tinker_urdf/src/pan_tilt.urdf.xacro` (`camera_mount` rpy
   `3.10886 1.01221 -0.0396491`).
3. `tk25_decision` — the `pan_bias=0` call sites above.

After changing, rebuild with `./tkbuild tk25_decision --packages-select behavior_tree`
(and rebuild `tinker_urdf` / `pan_tilt` for their halves).
