"""Existing HRI/Restaurant/PickAndPlace leaves with task-local strict guards."""
import math
import time
import py_trees

from behavior_tree.TemplateNodes.Audio import (
    BtNode_Announce, BtNode_ListenAction, BtNode_PhraseExtractionAction)
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction, BtNode_Approach
from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle
from behavior_tree.TemplateNodes.Vision import BtNode_FeatureExtraction, BtNode_TurnPanTilt
from .perception import DetectScene, IdentifyOwners
from .state import PREFIX, S, get, put, stamp_seconds, valid_point


class SettledPanTilt(BtNode_TurnPanTilt):
    """Existing command protocol, but no 'timeout means success' for photography."""
    def initialise(self):
        self._feedback_received = 0.0
        self._command_at = time.monotonic()
        super().initialise()

    def _state_cb(self, message):
        super()._state_cb(message)
        self._feedback_received = time.monotonic()

    def update(self):
        result = super().update()
        if result != S.SUCCESS:
            return result
        state = self._latest_state
        if (self.mock_mode or state is None or not state.feedback_ok
                or self._feedback_received < self._command_at
                or time.monotonic() - self._feedback_received > 0.5
                or abs(math.degrees(state.pan_rad) - self.x) >= self.SETTLE_EPS_DEG
                or abs(math.degrees(state.tilt_rad) - self.y) >= self.SETTLE_EPS_DEG):
            self.feedback_message = "No fresh in-tolerance pan/tilt feedback"
            # A previously aligned sample can make the base leaf report success
            # on the command tick. Wait for a post-command sample within the
            # original deadline, rather than failing before ROS can deliver it.
            if not self.mock_mode and time.time() <= self._settle_deadline:
                return S.RUNNING
            return S.FAILURE
        self.feedback_message = ""
        put("capture_after", self.node.get_clock().now().nanoseconds / 1e9)
        return S.SUCCESS


class Tk27MoveArm(BtNode_MoveArmSingle):
    """Reuse action lifecycle/result handling with the real TK27 joint schema."""
    def __init__(self, **kwargs):
        from tk27_manipulation_msgs.action import JointMove
        super().__init__(**kwargs)
        self.action_type = JointMove

    def send_goal(self):
        goal = self.action_type.Goal()
        goal.joint_names = ["joint" + str(i) for i in range(1, 8)]
        goal.positions = [float(v) for v in self.blackboard.arm_joint_pose]
        self.send_goal_request(goal)

    def update(self):
        status = super().update()
        if status == S.FAILURE:
            # Log before terminate() replaces feedback with cancellation state.
            self.node.get_logger().error(
                f"{self.name} [{self.action_name}]: {self.feedback_message}")
        return status

    def terminate(self, new_status):
        # A rejected or completed goal has nothing left to cancel. Preserve
        # the original result for the GUI instead of obscuring it with a cancel.
        if (not self.mock_mode and
                (getattr(self, "result_status", None) is not None or
                 (getattr(self, "goal_handle", None) is not None and
                  not self.goal_handle.accepted))):
            return
        super().terminate(new_status)


class FreshTarget(py_trees.behaviour.Behaviour):
    def __init__(self, context):
        super().__init__("Check person target frame and capture age")
        self.context = context

    def setup(self, **kwargs):
        self.node = kwargs["node"]

    def update(self):
        target = get("target")
        if not get("target_valid") or not valid_point(target):
            return S.FAILURE
        age = self.node.get_clock().now().nanoseconds / 1e9 - stamp_seconds(target.header)
        return S.SUCCESS if 0 <= age <= self.context.config["target_max_age_seconds"] else S.FAILURE


class RosLeaves:
    def __init__(self, context):
        self.context = context
        self.config = context.config
        self.endpoints = self.config["endpoints"]

    def bounded(self, name, node, seconds=None):
        return py_trees.decorators.Timeout(name=name + " timeout", child=node,
            duration=float(seconds or self.config["service_timeout_seconds"]))

    def say(self, name, text=None):
        return self.bounded(name, BtNode_Announce(name=name,
            bb_source=PREFIX + "speech" if text is None else None,
            message=text, service_name=self.endpoints["announce"]))

    def listen(self, name):
        return self.bounded(name, BtNode_ListenAction(name=name,
            bb_dest_key=PREFIX + "heard", timeout=self.config["listen_timeout_seconds"],
            action_name=self.endpoints["listen"]))

    def command(self):
        return self.bounded("Listen switch command", BtNode_PhraseExtractionAction(
            name="Listen switch command", wordlist=["开", "关"],
            bb_dest_key=PREFIX + "command", timeout=self.config["listen_timeout_seconds"],
            action_name=self.endpoints["phrase"]))

    def feature(self):
        return self.bounded("Extract owner features", BtNode_FeatureExtraction(
            name="Extract owner features", bb_dest_key=PREFIX + "features",
            bb_image_key=PREFIX + "reference_image",
            service_name=self.endpoints["feature_extraction"], use_orbbec=True))

    def head(self, pan, tilt):
        return SettledPanTilt(name=f"Pan {pan} tilt {tilt}", x=float(pan), y=float(tilt))

    def settle(self):
        return py_trees.timers.Timer(name="Wait for a post-settle image",
                                     duration=self.config["settle_seconds"])

    def detect(self, task):
        return DetectScene("Detect " + task, self.context, task)

    def identify(self):
        return IdentifyOwners("Identify owners in behaviour image", self.context)

    def goto(self, name):
        return self.bounded(name, BtNode_GotoAction(name=name, key=PREFIX + "nav_goal",
            action_name=self.endpoints["navigate"]), self.config["navigation_timeout_seconds"])

    def approach(self):
        return py_trees.composites.Sequence(name="Approach detected owner", memory=True, children=[
            FreshTarget(self.context),
            self.bounded("Approach owner", BtNode_Approach(
                name="Navigate around obstacles to owner", bb_target_key=PREFIX + "target",
                desired_distance=0.7, min_distance=0.55, max_distance=1.2,
                timeout_sec=self.config["navigation_timeout_seconds"],
                action_name=self.endpoints["approach"]),
                self.config["navigation_timeout_seconds"] + 2)])

    def arm(self, name):
        return self.bounded(name, Tk27MoveArm(name=name,
            arm_pose_bb_key=PREFIX + "arm_goal", action_name=self.endpoints["arm"]),
            self.config.get("arm_timeout_seconds", 120.0))
