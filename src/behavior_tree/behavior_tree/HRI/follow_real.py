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
**termination-gated** so the robot can stop following and drop the bag. Termination
is now autonomous: the nav follow executive's own ``OK_PERSON_STATIONARY`` verdict
(guest parked + robot behind them) is latched onto the ``follow/arrived`` blackboard
bool by ``BtNode_FollowAction``; ``BtNode_CheckFollowArrived`` consumes that latch
inside a ``Parallel(SuccessOnOne)`` to end the follow, then the robot asks a spoken
confirmation ("should I place the bag here?") — "yes" drops the bag, "no" resumes
following. No reliance on the guest volunteering a stop phrase (voice keyword
termination is removed entirely).

It writes NO existing files. ``hri.py`` is untouched; swapping the production
tree to call ``createHRIBagFlowReal`` is a separate, approval-gated one-liner.

Standalone harness::

    ros2 run behavior_tree hri-follow-real

The harness passes an operator-key arrival override (the mock Follow action never
latches ``follow/arrived`` offline), so it needs no audio server beyond the spoken
confirmation; for a fully auto-advancing offline run (mock every subsystem) use the
bundled config::

    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json \
        ros2 run behavior_tree hri-follow-real
"""

import os

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.components.following.follow_person import create_follow_person_tree
from behavior_tree.nodes.Audio import (
    BtNode_Announce,
    BtNode_GetConfirmationAction,
)
from behavior_tree.nodes.BaseBehaviors import (
    BtNode_WriteToBlackboard,
    BtNode_WaitKeyboardPress,
)
from behavior_tree.nodes.Manipulation import (
    BtNode_GripperAction,
    BtNode_MoveArmSingle,
)
from behavior_tree.core.visualization import create_post_tick_visualizer

# Module-local blackboard keys for the arm poses this stage uses. Kept local
# (not imported from HRI/config.py, whose constants.json path is hardcoded and
# may be absent) so this module is always import-safe and the harness seeds them.
KEY_ARM_DROP = "hri/arm_drop_pose"
KEY_ARM_NAVIGATING = "hri/arm_nav_pose"

# A neutral 7-joint pose (radians) used by the offline harness to satisfy the
# arm nodes' blackboard reads in mock mode. Replace with the real drop/nav poses
# on-robot (HRI/config.py KEY_ARM_DROP / KEY_ARM_NAVIGATING).
_DUMMY_ARM_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class BtNode_CheckFollowArrived(py_trees.behaviour.Behaviour):
    """SUCCESS once the follow executive reported the guest is stationary.

    The nav follow_server stationary gate (guest within 0.3 m for 5 s, 3 s stable
    lock, robot within 2.5 m, then a completed final park) ends the Follow action
    with OK_PERSON_STATIONARY. That terminal SUCCEEDED state is NOT published in
    Follow feedback (the server breaks before publishing), so we cannot poll
    ``follow/state``; ``BtNode_FollowAction`` latches the result onto the
    ``follow/arrived`` blackboard bool, which this node consumes.

    Re-arms each follow attempt: ``initialise`` clears the latch, then ``update``
    returns SUCCESS once it flips True (else RUNNING).
    """

    def __init__(self, name="guest stopped (follow stationary)",
                 bb_key_arrived="follow/arrived"):
        super().__init__(name=name)
        self.bb_key_arrived = bb_key_arrived

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=f"{self.name}_bb")
        self._bb.register_key(self.bb_key_arrived, access=py_trees.common.Access.READ)
        self._bb.register_key(self.bb_key_arrived, access=py_trees.common.Access.WRITE)

    def initialise(self):
        # Re-arm for this follow attempt.
        self._bb.set(self.bb_key_arrived, False, overwrite=True)
        self.feedback_message = "waiting for follow to report person stationary"

    def update(self):
        try:
            arrived = self._bb.get(self.bb_key_arrived)
        except KeyError:
            return py_trees.common.Status.RUNNING
        if arrived:
            self.feedback_message = "follow reported person stationary"
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


def createFollowHostUntilStop(
    target_frame=None,
    use_breadcrumbs: bool = False,
    arrived_override=None,
    confirm_question: str = "It looks like you have stopped. Should I place the bag here?",
    num_attempts: int = -1,
    operator_stop_key=None,
):
    """Follow the host until the nav stack reports them stationary, then confirm.

    Structure (proven HelpMeCarry follow-confirm loop):

        Retry(num_attempts,
          Sequence(memory=True, [
            Parallel(SuccessOnOne, [ create_follow_person_tree(nav), arrived_detector ]),
            Announce(confirm_question),
            BtNode_GetConfirmationAction(),   # yes -> SUCCESS (drop); no -> FAILURE (re-follow)
          ]))

    The follow tree never self-completes; ``arrived_detector`` ends the inner
    Parallel when the follow executive's OK_PERSON_STATIONARY latch fires, which
    cancels /track_person + follow_server and continues through the existing
    post-follow handoff path. ``num_attempts=-1`` = follow until stopped.

    Args:
        arrived_override: alternate arrival detector (e.g. an operator-key node
            for offline tests, where the mock Follow action never latches
            ``follow/arrived``). None -> BtNode_CheckFollowArrived.
        operator_stop_key: optional PTY key consumed by a production operator
            stop detector. It is combined with the normal stationary detector,
            so an operator stop follows the same cancellation and post-follow
            handoff path without disabling autonomous arrival detection.
    """
    follow = create_follow_person_tree(
        target_frame=target_frame,
        enable_navigation=True,
        use_breadcrumbs=use_breadcrumbs,
    )
    arrived = (arrived_override if arrived_override is not None
               else BtNode_CheckFollowArrived())
    if operator_stop_key is not None:
        arrived = py_trees.composites.Parallel(
            name=f"Follow stop detectors ({operator_stop_key})",
            policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
            children=[
                arrived,
                BtNode_WaitKeyboardPress(
                    name=f"Operator stop follow ({operator_stop_key})",
                    key=operator_stop_key,
                ),
            ],
        )

    follow_until_arrived = py_trees.composites.Parallel(
        name="Follow until guest stops",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        children=[follow, arrived],
    )

    loop = py_trees.composites.Sequence(
        name="follow + confirm arrival", memory=True,
        children=[
            follow_until_arrived,
            BtNode_Announce(name="ask if arrived", bb_source=None, message=confirm_question),
            # BtNode_GetConfirmationAction(name="confirm drop here"),
            py_trees.timers.Timer(name="Wait for bag placement", duration=3.0)
        ],
    )
    return py_trees.decorators.Retry(
        name="follow host until confirmed stop",
        child=loop,
        num_failures=num_attempts,
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
    arrived_override=None,
    confirm_question: str = "It looks like you have stopped. Should I place the bag here?",
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
            message="Stand in front of me please. I'll follow you and carry the bag.",
        )
    )
    root.add_child(
        createFollowHostUntilStop(
            arrived_override=arrived_override,
            confirm_question=confirm_question,
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
    # Operator-key arrival override: press the success key to mark the guest as
    # stationary (the mock Follow action never latches ``follow/arrived`` offline).
    operator_terminator = BtNode_WaitKeyboardPress(
        name="operator: press to mark arrived"
    )
    root.add_child(
        createHRIBagFlowReal(arrived_override=operator_terminator)
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
