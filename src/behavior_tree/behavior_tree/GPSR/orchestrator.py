"""GPSR orchestrator.

Receives a natural-language command, asks an LLM to break it into an ordered
list of atomic actions, then dispatches each action to the corresponding small
tree from ``small_trees.py``. Adds a self-monitor guard after every step and a
self-correction sub-tree that re-prompts the LLM for a fresh plan when a step
fails.

Blackboard contract:
    Inputs:
        gpsr/command            (str) — raw user instruction
    Internal:
        gpsr/plan               (list[dict]) — ordered actions to run
        gpsr/plan_index         (int) — next action to dispatch
        gpsr/current_action     (str) — name of the action being dispatched
        gpsr/current_params     (dict) — its parameters
        gpsr/state_log          (list[str]) — completed-step descriptions
        gpsr/correction_count   (int) — bumped on every self-correction
        gpsr/last_failure       (str) — feedback text from the failed step
"""

import json
import math
import textwrap
import threading
from typing import Any, Dict, List, Optional, Tuple

import py_trees
import rclpy
from py_trees.behaviour import Behaviour
from py_trees.blackboard import Blackboard
from py_trees.common import Access, Status
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

import openai

from .config import (
    OPENAI_API_KEY,
    OPENAI_MODEL,
    OPENAI_TEMPERATURE,
    OPENAI_MAX_TOKENS,
)
from .small_trees import ACTION_FACTORIES, bb_keys


# ---------------------------------------------------------------------------
# Knowledge available to the LLM (poses + objects loaded once at import).
# Each entry maps the GPSR-vocabulary name -> resolved PoseStamped or vision
# prompt string. The orchestrator uses these to materialise concrete BB values
# when popping a step from the plan.
# ---------------------------------------------------------------------------

KNOWN_LOCATIONS: Dict[str, PoseStamped] = {}
KNOWN_OBJECT_PROMPTS: Dict[str, str] = {}


def _parse_pose_stamped(json_dict: dict) -> PoseStamped:
    point = json_dict["point"]
    orientation = json_dict["orientation"]
    return PoseStamped(
        header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
        pose=Pose(
            position=Point(x=point["x"], y=point["y"], z=0.0),
            orientation=Quaternion(
                x=orientation['x'], y=orientation['y'],
                z=orientation['z'], w=orientation['w'],
            ),
        ),
    )


def load_knowledge_from_constants(constants_path: str) -> None:
    """Populate KNOWN_LOCATIONS / KNOWN_OBJECT_PROMPTS from constants.json."""
    KNOWN_LOCATIONS.clear()
    KNOWN_OBJECT_PROMPTS.clear()
    with open(constants_path, "r") as fh:
        constants = json.load(fh)
    for key, value in constants.get("possible_poses", {}).items():
        KNOWN_LOCATIONS[key] = _parse_pose_stamped(value)
    for key, value in constants.get("egpsr_rooms", {}).items():
        # Don't overwrite if already present in possible_poses.
        KNOWN_LOCATIONS.setdefault(key, _parse_pose_stamped(value))
    for key, value in constants.get("possible_objects", {}).items():
        KNOWN_OBJECT_PROMPTS[key] = value


# ---------------------------------------------------------------------------
# LLM planning
# ---------------------------------------------------------------------------

ACTION_CATALOGUE_DESCRIPTION = textwrap.dedent("""
    Available atomic actions and their parameter schemas:

    - goto(location: str)
        Navigate to a known room or placement. ``location`` must be one of the known locations.
    - find_object(object: str, location?: str)
        Visually search for an object. If ``location`` is given, goto() is assumed beforehand.
    - find_person(descriptor: str)
        Search for a person matching ``descriptor`` (e.g. "waving person",
        "person in a red shirt", "John"). Use "waving person" when the command
        mentions a waving/gesture cue.
    - follow(person?: str)
        Continuously follow a person. ``person`` is optional and informational.
    - guide(location: str)
        Lead a person to ``location``. Plan a find_person beforehand if needed.
    - greet(person: str)
        Speak a greeting that names ``person``.
    - grasp(object: str)
        Pick up an object that is currently in view of the arm/vision system.
        Always plan find_object + goto first.
    - place(location: str)
        Place the currently-held object at ``location``.
    - deliver(object: str, recipient: str, recipient_location?: str)
        Hand the currently-held object to ``recipient`` at their location.
    - count(object: str)
        Count visible instances of ``object`` and announce the number.
    - answer_question()
        Listen to a question and answer it.
    - tell_info(text: str)
        Speak the literal ``text`` (use for "tell me the time", team info, etc.;
        resolve the actual content in ``text``).
    - say(text: str)
        Generic spoken announcement.

    Compose composite commands by emitting multiple atomic actions in order.
    Example: "bring me the coke from the kitchen" =>
        [
          {"action": "goto", "params": {"location": "kitchen"}},
          {"action": "find_object", "params": {"object": "coke"}},
          {"action": "grasp", "params": {"object": "coke"}},
          {"action": "goto", "params": {"location": "instruction_point"}},
          {"action": "deliver", "params": {"object": "coke", "recipient": "user"}}
        ]
""").strip()


SYSTEM_PROMPT = textwrap.dedent("""
    You are the task-planning module of a household service robot competing in
    RoboCup@Home GPSR. You translate a free-form natural-language instruction
    into an ordered list of atomic actions the robot can execute.

    Respond with JSON only, in this exact shape:
    {
      "reasoning": "<short explanation of the plan>",
      "plan": [
        {"action": "<action_name>", "params": {<key>: <value>, ...}},
        ...
      ]
    }

    Use only the action names listed below. Use only known locations and objects.
    If the command is impossible, return an empty plan with a reasoning that
    explains why.
""").strip()


def _build_planner_user_prompt(
    command: str,
    state_log: List[str],
    failure_msg: Optional[str] = None,
) -> str:
    known_loc = ", ".join(sorted(KNOWN_LOCATIONS.keys())) or "(none)"
    known_obj = ", ".join(sorted(KNOWN_OBJECT_PROMPTS.keys())) or "(none)"
    body = (
        f"Known locations: {known_loc}\n"
        f"Known objects: {known_obj}\n\n"
        f"{ACTION_CATALOGUE_DESCRIPTION}\n\n"
        f"Command:\n{command}\n\n"
        f"Completed steps so far:\n{json.dumps(state_log, indent=2)}\n"
    )
    if failure_msg:
        body += (
            f"\nThe previous attempt failed with: {failure_msg}\n"
            "Re-plan from the current state. Do not repeat completed steps.\n"
        )
    body += "\nReturn the JSON plan now."
    return body


class BtNode_PlanActions(Behaviour):
    """Call the LLM asynchronously, parse the returned plan, write it to BB."""

    def __init__(
        self,
        name: str = "Plan actions",
        rephrase_on_failure: bool = False,
    ):
        super().__init__(name)
        self._client_oai = openai.OpenAI(
            api_key=OPENAI_API_KEY,
            base_url="https://openrouter.ai/api/v1",
        )
        self._bb = None
        self._thread: Optional[threading.Thread] = None
        self._response: Optional[Dict[str, Any]] = None
        self._error: Optional[str] = None
        self._rephrase_on_failure = rephrase_on_failure

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.COMMAND, access=Access.READ)
        self._bb.register_key(bb_keys.PLAN, access=Access.WRITE)
        self._bb.register_key(bb_keys.PLAN_INDEX, access=Access.WRITE)
        self._bb.register_key(bb_keys.STATE_LOG, access=Access.READ)
        self._bb.register_key(bb_keys.LAST_FAILURE, access=Access.READ)
        self._bb.register_key(bb_keys.CORRECTION_COUNT, access=Access.WRITE)

    def initialise(self):
        self._response = None
        self._error = None
        try:
            command = self._bb.get(bb_keys.COMMAND)
        except KeyError:
            command = ""
        try:
            state_log = self._bb.get(bb_keys.STATE_LOG) or []
        except KeyError:
            state_log = []
        try:
            failure_msg = self._bb.get(bb_keys.LAST_FAILURE)
        except KeyError:
            failure_msg = None
        if not self._rephrase_on_failure:
            failure_msg = None

        user_prompt = _build_planner_user_prompt(command, state_log, failure_msg)

        def _call():
            try:
                resp = self._client_oai.chat.completions.create(
                    model=OPENAI_MODEL,
                    messages=[
                        {"role": "system", "content": SYSTEM_PROMPT},
                        {"role": "user", "content": user_prompt},
                    ],
                    temperature=OPENAI_TEMPERATURE,
                    max_tokens=max(OPENAI_MAX_TOKENS, 1024),
                    response_format={"type": "json_object"},
                )
                raw = resp.choices[0].message.content.strip()
                self._response = json.loads(raw)
            except Exception as exc:  # noqa: BLE001 — surface anything for retry
                self._error = repr(exc)

        self._thread = threading.Thread(target=_call, daemon=True)
        self._thread.start()
        self.feedback_message = "LLM planning..."

    def update(self):
        if self._error is not None:
            self.feedback_message = f"LLM error: {self._error}"
            return Status.FAILURE
        if self._response is None:
            return Status.RUNNING

        plan_raw = self._response.get("plan", [])
        if not isinstance(plan_raw, list):
            self.feedback_message = "LLM returned non-list plan"
            return Status.FAILURE

        cleaned: List[Dict[str, Any]] = []
        for step in plan_raw:
            if not isinstance(step, dict):
                continue
            action = step.get("action")
            params = step.get("params", {}) or {}
            if action in ACTION_FACTORIES:
                cleaned.append({"action": action, "params": params})

        self._bb.set(bb_keys.PLAN, cleaned, overwrite=True)
        self._bb.set(bb_keys.PLAN_INDEX, 0, overwrite=True)
        try:
            count = self._bb.get(bb_keys.CORRECTION_COUNT)
        except KeyError:
            count = 0
        self._bb.set(bb_keys.CORRECTION_COUNT, count, overwrite=True)
        self.feedback_message = (
            f"Planned {len(cleaned)} step(s): {[s['action'] for s in cleaned]}"
        )
        return Status.SUCCESS

    def terminate(self, new_status):
        self._thread = None


# ---------------------------------------------------------------------------
# Step pop / parameter materialisation
# ---------------------------------------------------------------------------

class BtNode_PopNextAction(Behaviour):
    """Pop ``plan[plan_index]``, resolve its params into BB targets.

    Returns:
        SUCCESS — a step is ready to execute.
        FAILURE — plan exhausted; outer loop should terminate.
    """

    def __init__(self, name: str = "Pop next action"):
        super().__init__(name)
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.PLAN, access=Access.READ)
        self._bb.register_key(bb_keys.PLAN_INDEX, access=Access.WRITE)
        self._bb.register_key(bb_keys.CURRENT_ACTION, access=Access.WRITE)
        self._bb.register_key(bb_keys.CURRENT_PARAMS, access=Access.WRITE)
        self._bb.register_key(bb_keys.TARGET_POSE, access=Access.WRITE)
        self._bb.register_key(bb_keys.TARGET_LOCATION, access=Access.WRITE)
        self._bb.register_key(bb_keys.TARGET_OBJECT_NAME, access=Access.WRITE)
        self._bb.register_key(bb_keys.TARGET_OBJECT_PROMPT, access=Access.WRITE)
        self._bb.register_key(bb_keys.TARGET_PERSON_PROMPT, access=Access.WRITE)
        self._bb.register_key(bb_keys.ANNOUNCE_TEXT, access=Access.WRITE)

    def update(self):
        try:
            plan = self._bb.get(bb_keys.PLAN) or []
            index = self._bb.get(bb_keys.PLAN_INDEX)
        except KeyError:
            self.feedback_message = "Plan not initialised"
            return Status.FAILURE

        if index >= len(plan):
            self.feedback_message = f"Plan exhausted ({index}/{len(plan)})"
            return Status.FAILURE

        step = plan[index]
        action = step.get("action")
        params = step.get("params", {}) or {}

        self._bb.set(bb_keys.CURRENT_ACTION, action, overwrite=True)
        self._bb.set(bb_keys.CURRENT_PARAMS, params, overwrite=True)
        self._materialise_params(action, params)
        self._bb.set(bb_keys.PLAN_INDEX, index + 1, overwrite=True)
        self.feedback_message = f"step {index+1}/{len(plan)}: {action}({params})"
        return Status.SUCCESS

    def _materialise_params(self, action: str, params: Dict[str, Any]) -> None:
        """Translate the LLM's params into the BB keys the small trees consume."""
        # Location → PoseStamped lookup
        loc_name = params.get("location") or params.get("recipient_location")
        if loc_name:
            self._bb.set(bb_keys.TARGET_LOCATION, loc_name, overwrite=True)
            pose = KNOWN_LOCATIONS.get(loc_name)
            if pose is not None:
                self._bb.set(bb_keys.TARGET_POSE, pose, overwrite=True)

        # Object → vision prompt + name
        obj_name = params.get("object")
        if obj_name:
            self._bb.set(bb_keys.TARGET_OBJECT_NAME, obj_name, overwrite=True)
            prompt = KNOWN_OBJECT_PROMPTS.get(obj_name, obj_name)
            self._bb.set(bb_keys.TARGET_OBJECT_PROMPT, prompt, overwrite=True)

        # Person descriptor
        person = params.get("descriptor") or params.get("person") or params.get("recipient")
        if person:
            self._bb.set(bb_keys.TARGET_PERSON_PROMPT, person, overwrite=True)

        # tell_info / say carry text directly
        if action in ("tell_info", "say"):
            text = params.get("text") or params.get("message") or ""
            self._bb.set(bb_keys.ANNOUNCE_TEXT, text, overwrite=True)


class BtNode_ActionRouter(Behaviour):
    """Guard that succeeds only if ``CURRENT_ACTION`` matches this branch."""

    def __init__(self, expected_action: str):
        super().__init__(f"router:{expected_action}")
        self._expected = expected_action
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.CURRENT_ACTION, access=Access.READ)

    def update(self):
        try:
            current = self._bb.get(bb_keys.CURRENT_ACTION)
        except KeyError:
            return Status.FAILURE
        return Status.SUCCESS if current == self._expected else Status.FAILURE


# ---------------------------------------------------------------------------
# Self-monitor + self-correction
# ---------------------------------------------------------------------------

class BtNode_LogStepResult(Behaviour):
    """Append ``current_action(params) <result>`` to the state log."""

    def __init__(self, name: str, succeeded: bool):
        super().__init__(name)
        self._succeeded = succeeded
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.CURRENT_ACTION, access=Access.READ)
        self._bb.register_key(bb_keys.CURRENT_PARAMS, access=Access.READ)
        self._bb.register_key(bb_keys.STATE_LOG, access=Access.WRITE)
        self._bb.register_key(bb_keys.LAST_FAILURE, access=Access.WRITE)

    def update(self):
        try:
            action = self._bb.get(bb_keys.CURRENT_ACTION)
            params = self._bb.get(bb_keys.CURRENT_PARAMS)
        except KeyError:
            action, params = "unknown", {}
        try:
            log = self._bb.get(bb_keys.STATE_LOG)
        except KeyError:
            log = None
        if log is None:
            log = []
        verdict = "SUCCEEDED" if self._succeeded else "FAILED"
        log.append(f"{action}({params}) {verdict}")
        self._bb.set(bb_keys.STATE_LOG, log, overwrite=True)
        if not self._succeeded:
            self._bb.set(
                bb_keys.LAST_FAILURE,
                f"{action}({params}) failed",
                overwrite=True,
            )
        return Status.SUCCESS


class BtNode_BumpCorrectionCounter(Behaviour):
    """Increment correction counter; FAIL if it exceeds ``max_corrections``."""

    def __init__(self, name: str, max_corrections: int):
        super().__init__(name)
        self._max = max_corrections
        self._bb = None

    def setup(self, **kwargs):
        self._bb = self.attach_blackboard_client(name=self.name)
        self._bb.register_key(bb_keys.CORRECTION_COUNT, access=Access.WRITE)

    def update(self):
        try:
            count = self._bb.get(bb_keys.CORRECTION_COUNT)
        except KeyError:
            count = 0
        count = (count or 0) + 1
        self._bb.set(bb_keys.CORRECTION_COUNT, count, overwrite=True)
        if count > self._max:
            self.feedback_message = f"correction limit reached ({count} > {self._max})"
            return Status.FAILURE
        self.feedback_message = f"correction #{count}"
        return Status.SUCCESS


# ---------------------------------------------------------------------------
# Tree composition
# ---------------------------------------------------------------------------

def create_dispatcher() -> py_trees.composites.Selector:
    """Dispatcher Selector: one [Router → SmallTree] branch per registered action.

    On a successful match the small tree's status (SUCCESS/FAILURE) propagates
    up through the selector. If no router matches, the selector returns FAILURE
    (which the orchestrator interprets as "unknown action — trigger correction").
    """
    selector = py_trees.composites.Selector("dispatcher", memory=False)
    for action_name, factory in ACTION_FACTORIES.items():
        branch = py_trees.composites.Sequence(f"branch:{action_name}", memory=True)
        branch.add_child(BtNode_ActionRouter(action_name))
        branch.add_child(factory())
        selector.add_child(branch)
    return selector


def create_self_correction(max_corrections: int = 3) -> py_trees.composites.Sequence:
    """On step failure: log, bump counter (fail if exhausted), announce, re-plan."""
    from behavior_tree.TemplateNodes.Audio import BtNode_Announce
    seq = py_trees.composites.Sequence("self_correction", memory=True)
    seq.add_child(BtNode_LogStepResult("log failure", succeeded=False))
    seq.add_child(BtNode_BumpCorrectionCounter(
        "bump correction counter", max_corrections=max_corrections,
    ))
    seq.add_child(BtNode_Announce(
        "announce correcting", bb_source=None,
        message="I had trouble with that step. Let me reconsider.",
    ))
    seq.add_child(BtNode_PlanActions(
        name="replan after failure",
        rephrase_on_failure=True,
    ))
    return seq


def create_execute_one_step(max_corrections: int = 3) -> py_trees.composites.Sequence:
    """One iteration of the orchestrator loop: pop → dispatch+monitor → maybe correct."""
    # Pop: SUCCESS continues, FAILURE = no more steps (caller decides what to do).
    pop = BtNode_PopNextAction()

    dispatch = create_dispatcher()

    # Self-monitor: log a SUCCESS step after the dispatcher returned SUCCESS.
    monitor_then_log = py_trees.composites.Sequence("monitor+log success", memory=True)
    monitor_then_log.add_child(dispatch)
    monitor_then_log.add_child(BtNode_LogStepResult("log success", succeeded=True))

    # Self-correction triggered only when the dispatch+monitor returned FAILURE.
    correction = create_self_correction(max_corrections=max_corrections)

    dispatch_or_correct = py_trees.composites.Selector(
        "dispatch_or_correct", memory=False,
        children=[monitor_then_log, correction],
    )

    step = py_trees.composites.Sequence("execute_step", memory=True)
    step.add_child(pop)
    step.add_child(dispatch_or_correct)
    return step


def create_execute_command(
    max_steps: int = 25,
    max_corrections: int = 3,
) -> py_trees.behaviour.Behaviour:
    """Plan once, then run the step loop until the plan is exhausted.

    The plan-loop relies on ``BtNode_PopNextAction`` returning FAILURE when
    nothing is left to do. That failure bubbles up through ``Repeat`` (which
    aborts on any child failure), and the outer ``FailureIsSuccess`` decorator
    converts it back to SUCCESS so the parent tree treats command completion as
    normal success.
    """
    plan = BtNode_PlanActions(name="plan initial")

    loop_body = create_execute_one_step(max_corrections=max_corrections)
    loop = py_trees.decorators.Repeat(
        name="step loop",
        child=loop_body,
        num_success=max_steps,
    )
    loop_done_ok = py_trees.decorators.FailureIsSuccess(
        "plan-exhausted = done", loop,
    )

    root = py_trees.composites.Sequence("execute_command", memory=True)
    root.add_child(plan)
    root.add_child(loop_done_ok)
    return root


def create_orchestrator_init() -> py_trees.composites.Sequence:
    """Reset blackboard state before a new command."""
    seq = py_trees.composites.Sequence("orchestrator_init", memory=True)
    from .small_trees import BtNode_BlackboardSet
    seq.add_child(BtNode_BlackboardSet("clear plan", bb_keys.PLAN, []))
    seq.add_child(BtNode_BlackboardSet("clear plan_index", bb_keys.PLAN_INDEX, 0))
    seq.add_child(BtNode_BlackboardSet("clear state_log", bb_keys.STATE_LOG, []))
    seq.add_child(BtNode_BlackboardSet("clear correction", bb_keys.CORRECTION_COUNT, 0))
    seq.add_child(BtNode_BlackboardSet("clear last_failure", bb_keys.LAST_FAILURE, ""))
    return seq
