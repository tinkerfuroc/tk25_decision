# HRI 2026: aimed bag-handover arm pose (point at guest 2) — design

Date: 2026-07-03
Scope: `hri-2026` entry point only (`behavior_tree/HRI/hri_2026.py:createBagFlowReal2026`).

## Problem

The hri-2026 bag flow opens the gripper for the bag with the arm left in
whatever pose the previous phase ended in (the two-way introduction's
best-effort point at guest 2). The canonical `hri.createBagFlow` moves to the
fixed `KEY_ARM_HANDOVER` joint pose first, but with no aiming — the arm does
not orient toward the person handing over the bag.

Requested: a dedicated bag-handover arm pose that reuses the same orientation
logic as pointing at people, aimed at the **second guest**.

## Existing building blocks (all reused, none modified)

- `BtNode_PointTo` (`TemplateNodes/Manipulation.py:839`): joint0 (pan) =
  `compute_point_to_pan(x, y, pan_bias)` from a target `PointStamped`;
  joints 1–6 from a blackboard init-pose key. This is exactly "the
  orientation logic used when pointing to people" (intro call sites:
  `hri.py:1052` / `hri.py:1114`, `pan_bias=0.0` since the 2026-06-27
  camera-TF fix).
- `KEY_PERSONS` layout `[host, guest1, guest2]` → guest 2 = `target_id=2`.
  `KEY_PERSON_CENTROIDS` is populated by the intro's
  `BtNode_FeatureMatching` scan (None-padded at index 0) and stays valid
  through the handover: the base does not move between the intro scan and
  the bag grasp (the 180° turn-around runs *after* "Close gripper with bag").
- `ARM_POS_HANDOVER` / `KEY_ARM_HANDOVER` already exist in `config.py`, are
  loaded from `constants.json:arm_pos_handover`, and are seeded by
  `createConstantWriter` (which `createHRITask2026` runs). Joint0 of that
  constant is overwritten by the pointing math, so no constants change is
  needed. To physically differentiate the handover reach later, tune joints
  1–6 of `arm_pos_handover` — the aim at guest 2 is unaffected.

## Design

In `createBagFlowReal2026`, replace the bare "Announce ready for bag" child
with a canonical-style parallel (mirrors `hri.createBagFlow`'s
"Parallel move arm to handover and announce"):

```
Parallel(SuccessOnAll) "Arm to handover pose + announce ready"
├── FailureIsSuccess "Arm to bag-handover pose (best effort)"
│   └── Selector(memory) "Arm to bag-handover pose"
│       ├── Retry×3 → BtNode_PointTo(init=KEY_ARM_HANDOVER, target_id=2,
│       │             persons=KEY_PERSONS, points=KEY_PERSON_CENTROIDS,
│       │             pan_bias=0.0)          # aimed handover pose
│       └── Retry×3 → BtNode_MoveArmSingle(KEY_ARM_HANDOVER)  # canonical fixed fallback
└── BtNode_Announce "I am ready to take your bag."
```

- **Fallback**: `BtNode_PointTo` fast-fails when the guest-2 centroid is
  missing (the intro's feature-match is itself best-effort); the fixed
  handover pose then preserves today's canonical behavior.
- **Best-effort overall**: a total arm refusal must not forfeit the
  gripper handover + follow-to-drop scoring, consistent with the flow's
  other `FailureIsSuccess(Retry×3)` wrappers.
- Announcement texts unchanged (out of scope).

## Alternatives considered

1. Sequential arm move before the announce — simpler tree, but adds dead
   air; canonical flow already masks arm latency with speech. Rejected.
2. New `arm_pos_bag_handover` constants entry — YAGNI; `arm_pos_handover`
   is already the bag-handover pose and is currently unused by hri-2026.
3. Re-scanning guests before the handover — unnecessary; centroids are
   fresh (no base motion since the intro scan) and a re-scan costs time
   against the 6:00 budget.

## Tests

New `test/test_hri_bagflow_handover_point.py` (conventions of
`test_hri_turnaround_goto.py`, `BT_MOCK_MODE=true`):

1. Exactly one `BtNode_PointTo` in the bag flow; `target_id == 2`,
   `pan_bias == 0.0`, persons/points keys correct, init-pose remap →
   `/hri_arm_handover`.
2. The handover-pose parallel precedes "Open gripper for bag".
3. The selector's fallback child is a fixed `BtNode_MoveArmSingle` on
   `KEY_ARM_HANDOVER`, ordered after the aimed primary.

Existing `test_hri_turnaround_goto.py` / `test_hri_2026_start_gate.py`
assertions must stay green (the new subtree replaces one direct child of the
bag flow; "Close gripper with bag" / "Look at host" / turn-around ordering
is untouched).
