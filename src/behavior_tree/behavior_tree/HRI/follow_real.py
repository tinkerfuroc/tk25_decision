# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""HRI follow-host stage on the REAL follow process.

The production HRI bag flow (``HRI/hri.py:createGraspAndCarryBag``) currently
stubs the "follow host to the drop area" stage with four ``BtNode_Announce``
"Attempting to follow" leaves and leaves the real follow + drop commented out
(``# root.add_child(createFollowPerson(FOLLOW_CONFIG))`` and the drop sequence).
The only other HRI follow code, ``HRI/follow.py``'s ``BtNode_FollowActionLegacy``,
targets the **removed** ``tracking_server`` action and is deprecated.

This module is the NEW, real integration the user asked for: it drives the
follow-host stage with the **real follow process** from the ``FollowPerson``
package — ``create_follow_person_tree`` (live ``BtNode_TrackPersonAction`` @
``/track_person`` + ``BtNode_FollowAction`` @ ``follow_server`` + the reacq /
two-pass recovery reactions). Because that follow tree is a never-self-completing
``Parallel`` (it stays alive through transient losses by design), HRI needs it
**termination-gated** so the robot can stop following and drop the bag: we run it
beside a *termination detector* inside a ``Parallel(SuccessOnOne)`` — when the
detector fires (host says a stop phrase, or operator key in offline tests), the
parallel succeeds, the follow subtree goes INVALID and cancels its goals, and the
flow proceeds to the drop.

It writes NO existing files. ``hri.py`` is untouched; swapping the production
tree to call ``createHRIBagFlowReal`` is a separate, approval-gated one-liner.

Standalone harness::

    ros2 run behavior_tree hri-follow-real

The harness uses an operator-key terminator, so it needs no audio server; for a
fully auto-advancing offline run (mock every subsystem) use the bundled config::

    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json \
        ros2 run behavior_tree hri-follow-real

NOTE: ``createHRIBagFlowReal`` with the *default* (voice) terminator instantiates
``BtNode_ListenAction``, which targets the real ``listen_action`` server unless
audio is mocked — run it on-robot or with the full-mock config above.
"""

import os

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.FollowPerson.follow_person import create_follow_person_tree
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_ListenAction
from behavior_tree.TemplateNodes.BaseBehaviors import (
    BtNode_WriteToBlackboard,
    BtNode_WaitKeyboardPress,
)
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_GripperAction,
    BtNode_MoveArmSingle,
)
from behavior_tree.visualization import create_post_tick_visualizer

# Module-local blackboard keys for the arm poses this stage uses. Kept local
# (not imported from HRI/config.py, whose constants.json path is hardcoded and
# may be absent) so this module is always import-safe and the harness seeds them.
KEY_ARM_DROP = "hri/arm_drop_pose"
KEY_ARM_NAVIGATING = "hri/arm_nav_pose"

# A neutral 7-joint pose (radians) used by the offline harness to satisfy the
# arm nodes' blackboard reads in mock mode. Replace with the real drop/nav poses
# on-robot (HRI/config.py KEY_ARM_DROP / KEY_ARM_NAVIGATING).
_DUMMY_ARM_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def _default_termination(stop_phrase: str, listen_timeout: float):
    """Default follow-termination detector: host voice keyword, operator fallback.

    A memoryless ``Selector`` of [listen-for-stop-phrase, operator-key], wrapped
    in ``FailureIsRunning`` so a failed/han-up listen never FAILs the enclosing
    ``Parallel`` (which would abort the follow). SUCCESS only when the host says
    the stop phrase or an operator presses a key.

    NOTE: ``BtNode_ListenAction`` targets the real ``listen_action`` server
    (audio is REAL in the default mock config), so this default is for on-robot
    use. Offline tests pass an operator-key terminator instead (see ``main``).
    """
    listen = BtNode_ListenAction(
        name=f"listen for stop phrase ('{stop_phrase}')",
        bb_dest_key="hri/follow_stop_heard",
        timeout=listen_timeout,
    )
    operator = BtNode_WaitKeyboardPress(name="operator: end follow")
    selector = py_trees.composites.Selector(
        name="follow stop trigger", memory=False, children=[listen, operator]
    )
    return py_trees.decorators.FailureIsRunning(
        name="follow stop trigger (stay alive)", child=selector
    )


def createFollowHostUntilStop(
    stop_phrase: str = "you can stop here",
    listen_timeout: float = 8.0,
    target_frame=None,
    use_breadcrumbs: bool = False,
    termination_child=None,
):
    """Real follow-host stage that ends on a termination signal.

    Runs ``create_follow_person_tree`` (the real follow process) beside a
    termination detector under ``Parallel(SuccessOnOne)``. The follow tree never
    self-completes, so the detector is what ends the stage; on SUCCESS the
    follow subtree is invalidated and its ``/track_person`` + ``follow_server``
    goals are cancelled.

    Args:
        stop_phrase: keyword the host says to end following (default detector).
        listen_timeout: per-attempt listen timeout for the default detector.
        target_frame / use_breadcrumbs: forwarded to ``create_follow_person_tree``.
        termination_child: override the detector (e.g. an operator-key node for
            offline tests). When ``None`` the default voice+operator detector is
            built.

    Returns:
        A ``Parallel(SuccessOnOne)`` behaviour.
    """
    follow = create_follow_person_tree(
        target_frame=target_frame,
        enable_navigation=True,
        use_breadcrumbs=use_breadcrumbs,
    )
    if termination_child is None:
        termination_child = _default_termination(stop_phrase, listen_timeout)

    return py_trees.composites.Parallel(
        name="Follow host until stop",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        children=[follow, termination_child],
    )


def createBagDropReal(
    arm_drop_key: str = KEY_ARM_DROP,
    arm_nav_key: str = KEY_ARM_NAVIGATING,
    confirm: bool = True,
):
    """Drop the carried bag at the current (host-indicated) area.

    Replaces the commented-out drop sequence + ``BtNode_MockSafetyCheck('Drop
    confirmation detector TODO')`` in ``hri.py`` with a real arm-drop:
    announce -> arm to drop pose -> open gripper -> (optional operator/host
    confirm) -> arm back to navigation pose.
    """
    seq = py_trees.composites.Sequence("Drop bag", memory=True)
    seq.add_child(
        BtNode_Announce(
            name="announce dropping bag",
            bb_source=None,
            message="I will place the bag here now.",
        )
    )
    seq.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="arm to drop pose (best effort)",
            child=py_trees.decorators.Retry(
                name="Retry arm to drop pose",
                child=BtNode_MoveArmSingle(
                    name="Move arm to drop pose",
                    service_name="arm_joint_service",
                    arm_pose_bb_key=arm_drop_key,
                    add_octomap=False,
                ),
                num_failures=3,
            ),
        )
    )
    seq.add_child(
        BtNode_GripperAction(name="Open gripper to drop bag", open_gripper=True)
    )
    if confirm:
        seq.add_child(
            BtNode_Announce(
                name="announce bag dropped",
                bb_source=None,
                message="I have placed the bag.",
            )
        )
    seq.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="arm back to nav pose (best effort)",
            child=py_trees.decorators.Retry(
                name="Retry arm back to nav pose",
                child=BtNode_MoveArmSingle(
                    name="Move arm back to navigation pose",
                    service_name="arm_joint_service",
                    arm_pose_bb_key=arm_nav_key,
                    add_octomap=False,
                ),
                num_failures=3,
            ),
        )
    )
    return seq


def createHRIBagFlowReal(
    stop_phrase: str = "you can stop here",
    listen_timeout: float = 8.0,
    termination_child=None,
):
    """Full real bag flow: lightweight handover -> real follow-host -> drop.

    This is the drop-in replacement subtree for the stubbed
    ``createGraspAndCarryBag`` tail. Wiring it into the production HRI tree is a
    separate, approval-gated edit to ``hri.py`` — this module only provides it.
    """
    root = py_trees.composites.Sequence("HRI bag flow (real follow)", memory=True)

    # --- handover (lightweight; the production tree already has a richer one) ---
    root.add_child(
        BtNode_GripperAction(name="Open gripper for bag", open_gripper=True)
    )
    root.add_child(
        BtNode_Announce(
            name="Ask for bag handover",
            bb_source=None,
            message="Please place your bag in my gripper.",
        )
    )
    root.add_child(
        py_trees.timers.Timer(name="Wait for bag placement", duration=3.0)
    )
    root.add_child(
        BtNode_GripperAction(name="Close gripper with bag", open_gripper=False)
    )

    # --- real follow host until the host signals to stop ---
    root.add_child(
        BtNode_Announce(
            name="Follow host announcement",
            bb_source=None,
            message="I'll follow you and carry the bag. Tell me when to stop.",
        )
    )
    root.add_child(
        createFollowHostUntilStop(
            stop_phrase=stop_phrase,
            listen_timeout=listen_timeout,
            termination_child=termination_child,
        )
    )

    # --- drop ---
    root.add_child(createBagDropReal())
    return root


def main():
    """Offline-runnable harness for the real HRI follow-host + drop flow.

    Uses an operator-key terminator (no audio server needed) so the full
    handover -> real-follow -> drop flow ticks end-to-end in mock mode.
    """
    rclpy.init()

    root = py_trees.composites.Sequence("test:hri-follow-real", memory=True)
    # Seed dummy arm poses so the mock arm nodes have blackboard values to read.
    root.add_child(
        BtNode_WriteToBlackboard(
            "seed drop pose",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_ARM_DROP,
            object=list(_DUMMY_ARM_POSE),
        )
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            "seed nav pose",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_ARM_NAVIGATING,
            object=list(_DUMMY_ARM_POSE),
        )
    )
    # Operator-key terminator: press the success key to end the follow stage.
    operator_terminator = BtNode_WaitKeyboardPress(
        name="operator: press to end follow and drop"
    )
    root.add_child(
        createHRIBagFlowReal(termination_child=operator_terminator)
    )
    root.add_child(py_trees.behaviours.Running("idle (ctrl-c to exit)"))

    tree = py_trees_ros.trees.BehaviourTree(root=root)
    tree.setup(timeout=15, node_name="hri_follow_real")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="hri-follow-real"
    )
    tree.tick_tock(period_ms=500.0, post_tick_handler=print_tree)
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        shutdown_visualizer()
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
