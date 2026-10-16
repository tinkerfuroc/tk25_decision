"""Phase factories in the same py_trees idiom as HRI, Restaurant and PickAndPlace."""
import py_trees
from .config import load_config, room_order
from .state import Context, StateNode, get, put, pose, extract_owner_name
from .control import PacedRetry, ReportFailure
from .speech import NAME_PROMPT, HELP_PROMPT, ASSISTANCE_PROMPT


def sequence(name, *children):
    return py_trees.composites.Sequence(name=name, memory=True, children=list(children))


def selector(name, *children):
    return py_trees.composites.Selector(name=name, memory=True, children=list(children))


def step(name, callback):
    return StateNode(name, callback)


def create_task(config=None, leaves_factory=None):
    config = config if config is not None else load_config()
    context = Context(config)
    if leaves_factory is None:
        from .ros_nodes import RosLeaves
        leaves_factory = RosLeaves
    io = leaves_factory(context)
    level = config["level_tilt_deg"]

    def retry(node, key, default):
        return PacedRetry("Retry " + node.name,
            ReportFailure("Report " + node.name, node, context),
            config.get(key + "_attempts", default), config.get("retry_delay_seconds", 1.0))

    def optional(node):
        return py_trees.decorators.FailureIsSuccess(name="Best effort " + node.name,
            child=ReportFailure("Report " + node.name, node, context))

    def head(pan, tilt):
        return retry(io.head(pan, tilt), "head", 3)

    def dialogue(prompt, text, listen_name, initial_prompt=HELP_PROMPT):
        return retry(sequence(prompt,
            step("Clear previous speech input", lambda: put("heard", None)),
            io.say(prompt, text), io.listen(listen_name, initial_prompt=initial_prompt),
            step("Validate and prepare request repeat", context.repeat_request)), "dialogue", 3)

    def arm(name, value, stow=False):
        return sequence(name,
            step("Set saved arm pose", lambda: put("arm_pose_name", value())),
            step("Mark arm requires retraction", lambda: setattr(context, "arm_dirty", True)),
            retry(io.arm(name), "arm", 3),
            step("Record arm state", lambda: setattr(context, "arm_dirty", not stow)))

    def retract():
        return arm("Retract arm", lambda: config["arm_stow"], stow=True)

    def base_motion(name, motion):
        return sequence(name,
            retract(),
            step("Require retracted arm", lambda: not context.arm_dirty),
            head(0, level),
            motion)

    def navigate(name, value):
        return base_motion(name, sequence("Navigate with stowed arm",
            step("Set navigation goal", lambda: put("nav_goal", pose(value()))),
            retry(io.goto(name), "navigation", 5)))

    def return_to_scan():
        return navigate("Return to room scan point", lambda: config["room_poses"][context.room])

    def enrollment(owner):
        intake = sequence("Acquire owner " + str(owner),
            step("Clear enrollment inputs", lambda: [put(k, None) for k in
                 ("heard", "features", "reference_image")]),
            head(0, level), io.settle(),
            io.say("Call owner " + str(owner),
                "请第" + str(owner) + "位主人单独走到我的正前方，看向摄像头。请说您的名字：杰克、安东或者李华，只说您自己的名字。"),
            io.listen("Listen owner name", initial_prompt=NAME_PROMPT),
            step("Match candidate owner name", context.match_owner_name),
            retry(io.feature(), "feature", 4),
            step("Remember owner " + str(owner), lambda: context.remember(owner)))
        return sequence("Enroll owner " + str(owner),
            retry(intake, "dialogue", 3),
            optional(io.say("Repeat registered name")))

    def identify_before_service():
        def fresh(name):
            return sequence(name,
                retry(io.feature(), "feature", 4),
                step("Use fresh facing person crop", context.prepare_facing_identity),
                io.identify(),
                step("Accept facing-image identity", lambda: context.accept_identity("facing_image")))

        def face_camera(prompt_name, text, wait_name):
            return [io.say(prompt_name, text), head(0, level),
                    py_trees.timers.Timer(name=wait_name,
                        duration=config.get("identity_facing_wait_seconds", 2.0))]

        original = sequence("Match original behaviour person crop",
            step("Restore selected behaviour crop", lambda: put("scene", get("behaviour_identity_scene"))),
            io.identify(),
            step("Accept behaviour-image identity", lambda: context.accept_identity("behaviour_image")))
        # Match the observed person before asking them to change their posture.
        # Capture a facing image only when the original crop is inconclusive.
        again = retry(sequence("Re-prompt and match facing image",
            *face_camera("Ask person to face top camera again",
                "我还没看清您的脸。请站到我的正前方，抬头看向我顶部的摄像头，保持两秒不动。",
                "Wait for person to face camera again"),
            fresh("Match repeated facing image")), "identity_retry", 2)
        facing = sequence("Capture facing identity before service",
            *face_camera("Ask person to face top camera",
                "\u8bf7\u60a8\u9762\u5411\u6211\u9876\u90e8\u7684\u6444\u50cf\u5934\uff0c\u8ba9\u6211\u770b\u6e05\u60a8\u7684\u8138\uff0c\u65b9\u4fbf\u6211\u8ba4\u51fa\u60a8\u3002",
                "Wait for person to face camera"),
            fresh("Match facing image"))
        return selector("Identify person before service",
            original, facing, again,
            step("Record unidentified person", lambda: context.event("identity_unresolved")))

    def interaction(role):
        if role == 1:
            request = sequence("Ask switch command",
                step("Clear previous switch request", lambda: [put(k, None) for k in ("command", "heard")]),
                io.say("Ask electrical request", "\u60a8\u5e0c\u671b\u6211\u6253\u5f00\u8fd8\u662f\u5173\u95ed\u7535\u5668\uff1f\u8bf7\u544a\u8bc9\u6211\u60a8\u7684\u9700\u6c42\u3002"),
                io.command(), step("Choose switch operation", context.switch_command))
            operate = sequence("Operate requested switch",
                navigate("Navigate to switch mark", lambda: config["switches"][context.room][get("command")]["navigation"]),
                arm("Move end effector over switch mark", lambda: get("switch_arm")),
                retract(),
                optional(io.say("Electrical operation done", "\u5df2\u5b8c\u6210\u7535\u5668\u5f00\u5173\u6a21\u62df\u64cd\u4f5c\u3002")))
            return sequence("Resting person electrical control",
                retry(request, "dialogue", 3),
                io.say("Repeat electrical request"),
                selector("Leave unchanged or operate",
                    step("No switch change requested", lambda: get("command") == "none"),
                    operate))
        if role == 2:
            return sequence("Fallen person simulated assistance",
                arm("Extend assistance arm over person", lambda: config["arm_assist"]),
                dialogue("Ask fallen owner", "我已伸出机械臂模拟扶起您。您现在感觉怎么样，需要什么帮助？",
                         "Listen fallen owner", initial_prompt=ASSISTANCE_PROMPT),
                io.say("Acknowledge fallen owner"), retract())
        return sequence("Waving person help consultation",
            dialogue("Ask waved owner", "请问您需要什么帮助？", "Listen help request"),
            io.say("Speak original help request"))

    def handle_role(role, room):
        report = sequence("Report observed person",
            identify_before_service(),
            step("Prepare location, owner and behaviour report", lambda: context.prepare_observation_report(role)),
            io.say("Announce location, owner and behaviour"))
        if role == 1 and room not in config["switches"]:
            return selector("Observe resting person without switch service",
                step("No pending resting observation",
                     lambda: role not in context.active or role in context.serviced_roles),
                sequence("Report resting person only",
                    step("Select resting person", lambda: context.select_behaviour(role)),
                    report,
                    step("Record switch service not applicable",
                         lambda: context.event("switch_service_not_applicable"))))
        run = sequence("Perform behaviour " + str(role) + " interaction",
            report,
            base_motion("Approach with stowed arm", retry(io.approach(), "approach", 3)),
            interaction(role),
            step("Record completed behaviour service", lambda: context.finish_behaviour(role)))
        recover = sequence("Recover unsuccessful role interaction",
            retract(),
            step("Record incomplete interaction", lambda: context.event("interaction_failed", role=role)),
            optional(io.say("Report interaction failure", "这次交互没有完成，我会继续巡游后再尝试。")))
        selected = sequence("Selected behaviour " + str(role),
            step("Select person by behaviour", lambda: context.select_behaviour(role)),
            selector("Attempt interaction and recover", run, recover),
            return_to_scan(),
            step("Discard observations after robot movement", lambda: context.active.clear()))
        # Only an absent/already serviced behaviour is skippable. A failed retraction or
        # return navigation must propagate FAILURE, not get swallowed by a selector.
        return selector("Behaviour " + str(role) + " dispatch",
            sequence("No pending behaviour",
                step("No pending matching behaviour", lambda: role in context.serviced_roles
                     or role not in context.active)),
            selected)

    def scan_angle(angle, room):
        recognise = sequence("Recognise behaviour without identity gate",
            retry(io.detect("behaviour"), "perception", 2),
            step("Select observed behaviour targets", context.observe_behaviours))
        scan = sequence("Pan observation " + str(angle),
            step("Clear active observations", lambda: context.active.clear()),
            head(angle, level), io.settle(),
            selector("Detection result or explicit error",
                recognise,
                sequence("Report unavailable detection",
                    step("Clear failed perception", lambda: context.active.clear()),
                    step("Log perception failure", lambda: context.event("perception_failed", pan=angle)),
                    optional(io.say("Perception uncertain", "这次图像识别未完成，继续检查下一个角度。")))),
            *(handle_role(i, room) for i in (1, 2, 3)))
        return scan

    def room_task(room):
        litter = sequence("Inspect floor litter",
            head(0, level - config["litter_depression_deg"]), io.settle(),
            retry(io.detect("litter"), "perception", 2), step("Prepare litter report", context.litter),
            io.say("Announce floor litter"), step("Record litter announcement", context.litter_announced))
        return sequence("Patrol " + config["room_names"][room],
            step("Select room", lambda: context.select_room(room)),
            navigate("Navigate to " + room + " scan point", lambda: config["room_poses"][room]),
            *(scan_angle(angle, room) for angle in config["pan_angles_deg"]),
            selector("Litter observation or explicit error",
                litter,
                sequence("Litter detection unavailable",
                    step("Log litter failure", lambda: context.event("litter_detection_failed")),
                    optional(io.say("Litter uncertain", "本次地面图像检查未完成，不能判断是否有垃圾。")))),
            head(0, level),
            step("Mark room visited", lambda: context.visited.add(room)))

    passes = []
    def visit_room(room):
        return selector("Visit or skip unavailable " + room,
            room_task(room),
            sequence("Recover before skipping room",
                retract(),
                step("Record skipped room", lambda: context.event("room_skipped", skipped_room=room)),
                optional(io.say("Room unavailable", "这个房间本轮检查未完成，我会继续检查其他房间。"))))

    for index in range(config["max_passes"]):
        passes.append(selector("Patrol pass " + str(index + 1),
            step("All scoped tasks completed", lambda: len(context.visited) == 4
                 and len(context.completed) == 3 and len(context.litter_rooms) >= 3),
            sequence("Visit four rooms", *(visit_room(room) for room in room_order(config)))))
    patrol = sequence("Room patrol", *passes)
    mission = sequence("Hangzhou home service",
        step("Validate and initialise", context.initialise),
        py_trees.timers.Timer(name="Wait silently before registration", duration=5.0),
        navigate("Navigate to registration point", lambda: config["registration_pose"]),
        *(enrollment(i) for i in (1, 2, 3)),
        optional(io.say("Enrollment completed", "三位主人的信息已记住，请前往各自房间。我开始巡游。")),
        selector("Patrol or recovery",
            patrol,
            sequence("Stop incomplete patrol", retract(),
                step("Record patrol stopped", lambda: context.event("patrol_stopped")),
                optional(io.say("Patrol stopped", "巡游未全部完成，我将收回机械臂并结束本轮任务。")))),
        retract(), step("Summarise task", context.summary), optional(io.say("Speak task summary")),
        step("Record mission completion", lambda: context.event("mission_completed")))
    recovery = sequence("Mission failure recovery",
        step("Only recover after successful preflight", lambda: context.ready),
        step("Record failure", lambda: context.event("mission_failed")),
        selector("Recover arm or report unsafe stop",
            sequence("Retract and report", retract(),
                optional(io.say("Announce task stopped", "任务已停止，机械臂已收回。"))),
            sequence("Report unsuccessful retraction",
                step("Record recovery failed", lambda: context.event("retraction_failed")),
                optional(io.say("Request operator help", "任务已停止，无法确认机械臂收回，请操作员检查。")))),
        py_trees.behaviours.Failure(name="Keep failed outcome"))
    root = py_trees.decorators.OneShot(
        name="Hangzhou mission (never restart on terminal ticks)",
        child=selector("Mission with failure retraction",
                       ReportFailure("Report mission failure", mission, context), recovery),
        policy=py_trees.common.OneShotPolicy.ON_COMPLETION)
    root.context = context
    return root
