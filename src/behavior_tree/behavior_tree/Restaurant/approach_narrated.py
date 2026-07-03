"""Restaurant-2026 approach with parallel spoken narration.

While ``BtNode_Approach`` drives to the customer (via ``go_to_approach_stable``),
a side-child loops short "keep-alive" lines so the approach never feels silent.
Mirrors GPSR's ``_goto_keepalive_announcer`` / ``create_goto``
(``GPSR/small_trees.py:812-870``).

2026-only: consumed by ``order_intake_items.createCollectOneOrderItems``. The
legacy ``restaurants.createApproachCustomer`` is deliberately untouched.

Verified invariants + "announce always finishes" guarantee:
docs/superpowers/specs/2026-07-04-restaurant2026-approach-narration-design.md
"""
from __future__ import annotations

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Navigation import BtNode_Approach
from behavior_tree.TemplateNodes.Vision import BtNode_ShowImage

from .config import (
    KEY_ACTIVE_CUSTOMER_ID,
    KEY_ACTIVE_CUSTOMER_PICTURE,
    KEY_CUSTOMER_LOCATION,
    KEY_CUSTOMER_QUEUE,
    KEY_ORDER_CHECKLIST,
)
from .state_nodes import (
    BtNode_CloseActiveCustomer,
    BtNode_UpdateChecklistFlag,
)

# Short lines (~1 s spoken each) so the guaranteed-completion audio tail past
# arrival stays small on the serialized, no-cancel TTS server.
APPROACH_NARRATION_LINES = [
    "Approaching the customer.",
    "Planning my route.",
    "Adjusting my approach.",
]
APPROACH_NARRATION_INTERVAL_SEC = 5.0


def _approach_narrator() -> py_trees.behaviour.Behaviour:
    """Loop the narration lines with fixed gaps, forever, side-child-safe.

    Only ever returns RUNNING: ``SuccessIsRunning`` over a memory ``Sequence``,
    each ``Announce`` wrapped in ``FailureIsSuccess`` (a TTS error cannot trip
    the parallel's FAILURE priority), and ``Timer`` never fails. So the approach
    action alone decides the parallel's SUCCESS/FAILURE.
    """
    lines = py_trees.composites.Sequence(
        name="approach narration lines", memory=True)
    for i, msg in enumerate(APPROACH_NARRATION_LINES):
        lines.add_child(py_trees.decorators.FailureIsSuccess(
            name=f"say narration {i} (best-effort)",
            child=BtNode_Announce(
                name=f"approach narration {i}",
                bb_source=None,
                message=msg,
            ),
        ))
        lines.add_child(py_trees.timers.Timer(
            name=f"narration gap {i}",
            duration=APPROACH_NARRATION_INTERVAL_SEC,
        ))
    return py_trees.decorators.SuccessIsRunning(
        name="loop approach narration", child=lines)


def _approachCustomerSubtreeNarrated(
    name: str = "Approach customer table",
) -> py_trees.composites.Parallel:
    """Drive to the customer via ``go_to_approach_stable`` while narrating.

    ``SuccessOnSelected([approach])`` makes the drive the sole outcome driver and
    ends the parallel exactly when the drive ends (py_trees then stops the
    narrator). No one-shot pre-announce — the loop's first line replaces it.
    """
    approach = BtNode_Approach(
        name=f"{name}: go to approach",
        bb_target_key=KEY_CUSTOMER_LOCATION,
        action_name="go_to_approach_stable",
        action_timeout_ticks=220,  # 220 x 500 ms = 110 s backstop (legacy parity)
    )
    return py_trees.composites.Parallel(
        name=name,
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
            children=[approach], synchronise=False),
        children=[approach, _approach_narrator()],
    )


def createApproachCustomerNarrated() -> py_trees.composites.Selector:
    """2026 clone of ``restaurants.createApproachCustomer`` whose drive attempts
    narrate in parallel and target ``go_to_approach_stable``.

    Identical to the legacy factory except the ``Retry`` wraps
    ``_approachCustomerSubtreeNarrated`` instead of ``_approachCustomerSubtree``.
    ``partial_score_path`` is unchanged; narration is scoped inside the ``Retry``
    (the drive attempts) so it never overlaps the "couldn't reach" line.
    """
    root = py_trees.composites.Selector(
        name="Approach selected customer", memory=True)

    success_path = py_trees.composites.Sequence(
        name="Reach customer", memory=True)
    success_path.add_child(py_trees.decorators.Retry(
        name="retry goto customer",
        child=_approachCustomerSubtreeNarrated("Approach customer table"),
        num_failures=3,
    ))
    success_path.add_child(BtNode_UpdateChecklistFlag(
        name="Mark reached",
        checklist_key=KEY_ORDER_CHECKLIST,
        flag="reached",
        value=True,
    ))

    partial_score_path = py_trees.composites.Sequence(
        name="Partial score on unreachable caller", memory=True)
    partial_score_path.add_child(BtNode_Announce(
        name="Announce detected-but-unreachable caller",
        bb_source=None,
        message="I saw a caller but couldn't reach the table. Here is who I detected.",
    ))
    partial_score_path.add_child(BtNode_ShowImage(
        name="Show detected caller picture",
        bb_image_path_key=KEY_ACTIVE_CUSTOMER_PICTURE,
    ))
    partial_score_path.add_child(BtNode_UpdateChecklistFlag(
        name="Mark partial score",
        checklist_key=KEY_ORDER_CHECKLIST,
        flag="partial_score",
        value=True,
    ))
    partial_score_path.add_child(BtNode_CloseActiveCustomer(
        name="Close unreachable active caller",
        queue_key=KEY_CUSTOMER_QUEUE,
        active_id_key=KEY_ACTIVE_CUSTOMER_ID,
    ))

    root.add_child(success_path)
    root.add_child(partial_score_path)
    return root
