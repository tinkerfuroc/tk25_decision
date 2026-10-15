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
from .control import MotionBarrier


class MotionDiagnostics:
    """Keep terminal action errors visible; never cancel a completed goal."""
    def initialise(self):
        self._transport_error = None
        try:
            super().initialise()
        except Exception as exc:
            self._transport_error = "Action initialization failed: " + str(exc)

    def goal_response_callback(self, future):
        try:
            super().goal_response_callback(future)
        except Exception as exc:
            self._transport_error = "Goal response failed: " + str(exc)

    def get_result_callback(self, future):
        try:
            super().get_result_callback(future)
        except Exception as exc:
            self._transport_error = "Action result failed: " + str(exc)

    def update(self):
        try:
            if getattr(self, "_transport_error", None):
                raise RuntimeError(self._transport_error)
            status = super().update()
        except Exception as exc:
            self.feedback_message = str(exc)
            status = S.FAILURE
        if status == S.FAILURE:
            self.node.get_logger().error(
                f"{self.name} [{self.action_name}]: {self.feedback_message}")
        return status

    def terminate(self, new_status):
        if (not self.mock_mode and
                (getattr(self, "result_status", None) in (4, 5, 6) or
                 (getattr(self, "goal_handle", None) is not None and
                  not self.goal_handle.accepted))):
            return
        super().terminate(new_status)


class ServiceDiagnostics:
    """Convert service transport exceptions into recoverable BT failures."""
    def initialise(self):
        self._request_error = None
        try:
            super().initialise()
        except Exception as exc:
            self._request_error = "Service request failed: " + str(exc)

    def update(self):
        try:
            if self._request_error:
                raise RuntimeError(self._request_error)
            return super().update()
        except Exception as exc:
            self.feedback_message = str(exc)
            self.node.get_logger().error(f"{self.name}: {exc}")
            return S.FAILURE


class HangzhouAnnounce(ServiceDiagnostics, BtNode_Announce):
    pass


class HangzhouFeature(MotionDiagnostics, BtNode_FeatureExtraction):
    """Feature extraction is an action; reject images from before head settling."""
    def initialise(self):
        self.capture_started = get("capture_after")
        super().initialise()

    def process_result(self):
        status = super().process_result()
        if status == S.SUCCESS:
            image = self.blackboard.comparison_image
            if image is None or image.width <= 0 or image.height <= 0 or stamp_seconds(image.header) < self.capture_started:
                self.feedback_message = "Feature image is empty or predates head settling"
                return S.FAILURE
        return status


class HangzhouGoto(MotionDiagnostics, BtNode_GotoAction):
    def process_result(self):
        result = getattr(self.result_message, "result", None)
        code = getattr(result, "error_code", 0)
        if self.result_status == 4 and code == 0:
            self.feedback_message = "Navigation succeeded"
            return S.SUCCESS
        self.feedback_message = (f"Navigation failed: status={self.result_status}, "
                                 f"error_code={code}, {getattr(result, 'error_msg', '')}")
        return S.FAILURE


class HangzhouApproach(MotionDiagnostics, BtNode_Approach):
    def process_result(self):
        if self.result_status != 4:
            self.feedback_message = f"Approach action failed: status={self.result_status}"
            return S.FAILURE
        return super().process_result()


class HangzhouListen(MotionDiagnostics, BtNode_ListenAction):
    pass


class HangzhouCommand(MotionDiagnostics, BtNode_PhraseExtractionAction):
    pass


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


class Tk27MoveArm(MotionDiagnostics, BtNode_MoveArmSingle):
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

class FreshTarget(py_trees.behaviour.Behaviour):
    def __init__(self, context):
        super().__init__("Check person target frame and capture age")
        self.context = context

    def setup(self, **kwargs):
        self.node = kwargs["node"]

    def update(self):
        try:
            target = get("target")
            if not get("target_valid") or not valid_point(target):
                self.feedback_message = "Person target has no valid map position"
                return S.FAILURE
            age = self.node.get_clock().now().nanoseconds / 1e9 - stamp_seconds(target.header)
        except (KeyError, AttributeError, TypeError) as exc:
            self.feedback_message = "Invalid person target: " + str(exc)
            return S.FAILURE
        if not 0 <= age <= self.context.config["target_max_age_seconds"]:
            self.feedback_message = f"Person observation expired or future-dated: age={age:.2f}s"
            return S.FAILURE
        return S.SUCCESS


class RosLeaves:
    def __init__(self, context):
        self.context = context
        self.config = context.config
        self.endpoints = self.config["endpoints"]

    def bounded(self, name, node, seconds=None):
        return py_trees.decorators.Timeout(name=name + " timeout", child=node,
            duration=float(seconds or self.config["service_timeout_seconds"]))

    def motion(self, name, node, seconds, guards=()):
        self.context.motion_actions.append(node)
        return py_trees.composites.Sequence(name=name + " guarded motion", memory=True,
            children=[MotionBarrier(self.context), *guards, self.bounded(name, node, seconds)])

    def audio(self, name, node):
        self.context.audio_actions.append(node)
        return py_trees.composites.Sequence(name=name + " guarded audio", memory=True,
            children=[MotionBarrier(self.context, "audio"), self.bounded(name, node)])

    def say(self, name, text=None):
        return self.audio(name, HangzhouAnnounce(name=name,
            bb_source=PREFIX + "speech" if text is None else None,
            message=text, service_name=self.endpoints["announce"]))

    def listen(self, name):
        return self.audio(name, HangzhouListen(name=name,
            bb_dest_key=PREFIX + "heard", timeout=self.config["listen_timeout_seconds"],
            action_name=self.endpoints["listen"]))

    def command(self):
        return self.audio("Listen switch command", HangzhouCommand(
            name="Listen switch command", wordlist=["开", "关"],
            bb_dest_key=PREFIX + "command", timeout=self.config["listen_timeout_seconds"],
            action_name=self.endpoints["phrase"]))

    def feature(self):
        from .state import StateNode
        action = HangzhouFeature(name="Extract owner features",
            bb_dest_key=PREFIX + "features", bb_image_key=PREFIX + "reference_image",
            service_name=self.endpoints["feature_extraction"], use_orbbec=True)
        self.context.feature_actions.append(action)
        return py_trees.composites.Sequence(name="Capture owner features", memory=True, children=[
            MotionBarrier(self.context, "feature"),
            StateNode("Clear previous feature result", lambda: [put(k, None) for k in
                      ("features", "reference_image")]),
            self.bounded("Extract owner features", action)])

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
        return self.motion(name, HangzhouGoto(name=name, key=PREFIX + "nav_goal",
            action_name=self.endpoints["navigate"]), self.config["navigation_timeout_seconds"])

    def approach(self):
        return self.motion("Approach owner", HangzhouApproach(
                name="Navigate around obstacles to owner", bb_target_key=PREFIX + "target",
                desired_distance=0.7, min_distance=0.55, max_distance=1.2,
                timeout_sec=self.config["navigation_timeout_seconds"],
                action_name=self.endpoints["approach"]),
                self.config["navigation_timeout_seconds"] + 2, guards=[FreshTarget(self.context)])

    def arm(self, name):
        return self.motion(name, Tk27MoveArm(name=name,
            arm_pose_bb_key=PREFIX + "arm_goal", action_name=self.endpoints["arm"]),
            self.config.get("arm_timeout_seconds", 120.0))
