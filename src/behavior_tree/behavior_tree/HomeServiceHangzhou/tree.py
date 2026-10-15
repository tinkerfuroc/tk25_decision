"""Phase factories in the same py_trees idiom as HRI, Restaurant and PickAndPlace."""
import py_trees
from .config import EXPECTED, load_config, room_order
from .state import Context, PatrolBudget, StateNode, get, put, pose, extract_owner_name
from .control import PacedRetry, ReportFailure


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

    def dialogue(prompt, text, listen_name):
        return retry(sequence(prompt,
            step("Clear previous speech input", lambda: put("heard", None)),
            io.say(prompt, text), io.listen(listen_name),
            step("Validate and prepare request repeat", context.repeat_request)), "dialogue", 3)

    def arm(name, value, stow=False):
        return sequence(name,
            step("Set arm goal", lambda: put("arm_goal", value())),
            step("Mark arm requires retraction", lambda: setattr(context, "arm_dirty", True)),
            retry(io.arm(name), "arm", 3),
            step("Record arm state", lambda: setattr(context, "arm_dirty", not stow)))

    def retract():
        return arm("Retract arm", lambda: config["arm_stow"], stow=True)

    def navigate(name, value):
        return sequence(name,
            retract(),
            step("Require retracted arm", lambda: not context.arm_dirty),
            step("Set navigation goal", lambda: put("nav_goal", pose(value()))),
            head(0, level), retry(io.goto(name), "navigation", 5))

    def return_to_scan():
        return navigate("Return to room scan point", lambda: config["room_poses"][context.room])

    def enrollment(owner):
        intake = sequence("Acquire owner " + str(owner),
            step("Clear enrollment inputs", lambda: [put(k, None) for k in
                 ("heard", "features", "reference_image")]),
            head(0, level), io.settle(),
            io.say("Call owner " + str(owner),
                "请第" + str(owner) + "位主人单独走到我的正前方，看向摄像头。请说您的名字：杰克、安东或者李华，只说您自己的名字。"),
            io.listen("Listen owner name"),
            step("Match candidate owner name", lambda: put("heard", extract_owner_name(get("heard")))),
            retry(io.feature(), "feature", 4),
            step("Remember owner " + str(owner), lambda: context.remember(owner)))
        return sequence("Enroll owner " + str(owner),
            retry(intake, "dialogue", 3),
            optional(io.say("Repeat registered name")))

    def interaction(owner):
        if owner == 1:
            request = sequence("Ask switch command",
                step("Clear previous switch command", lambda: put("command", None)),
                io.say("Ask electrical request", "您希望打开还是关闭电器？请说开或者关。"),
                io.command(), step("Choose on/off mark", context.switch_command))
            return sequence("Owner 1 electrical control",
                retry(request, "dialogue", 3),
                io.say("Repeat electrical request"),
                navigate("Navigate to switch mark", lambda: config["switches"][context.room][
                    "on" if get("command") == "开" else "off"]["navigation"]),
                arm("Move end effector over switch mark", lambda: get("switch_arm")),
                retract(),
                optional(io.say("Electrical operation done", "已完成电器开关模拟操作。")))
        if owner == 2:
            return sequence("Owner 2 simulated assistance",
                retract(), head(0, level), retry(io.approach(), "approach", 3),
                arm("Extend assistance arm over person", lambda: config["arm_assist"]),
                dialogue("Ask fallen owner", "我已伸出机械臂模拟扶起您。您现在感觉怎么样，需要什么帮助？",
                         "Listen fallen owner"),
                io.say("Acknowledge fallen owner"), retract())
        return sequence("Owner 3 help consultation",
            retract(), head(0, level), retry(io.approach(), "approach", 3),
            dialogue("Ask waved owner", "请问您需要什么帮助？", "Listen help request"),
            io.say("Speak original help request"))

    def handle_owner(owner):
        run = sequence("Perform owner " + str(owner) + " interaction",
            io.say("Announce owner name and behaviour"), interaction(owner),
            step("Mark owner completed", lambda: context.complete(owner)))
        recover = sequence("Recover unsuccessful owner interaction",
            retract(),
            step("Record incomplete interaction", lambda: context.event("interaction_failed", owner=owner)),
            optional(io.say("Report interaction failure", "这次交互没有完成，我会继续巡游后再尝试。")))
        selected = sequence("Selected owner " + str(owner),
            step("Select matching owner/action", lambda: context.select_owner(owner)),
            selector("Attempt interaction and recover", run, recover),
            return_to_scan(),
            step("Discard observations after robot movement", lambda: context.active.clear()))
        # Only an absent/done/mismatched owner is skippable. A failed retraction or
        # return navigation must propagate FAILURE, not get swallowed by a selector.
        return selector("Owner " + str(owner) + " dispatch",
            sequence("No matching pending owner",
                step("No pending matching behaviour", lambda: owner in context.completed
                     or owner not in context.active
                     or not bool(set(context.active[owner].labels) & EXPECTED[owner]))),
            selected)

    def scan_angle(angle):
        recognise = sequence("Recognise behaviour and identity",
            retry(io.detect("behaviour"), "perception", 2),
            selector("Only identify positive scenes",
                step("No behaviour present", lambda: not get("scene").observations),
                sequence("Identify positive scene", io.identify(),
                         step("Bind owners to observations", context.match_scene))))
        scan = sequence("Pan observation " + str(angle),
            step("Clear active observations", lambda: context.active.clear()),
            head(angle, level), io.settle(),
            selector("Detection result or explicit error",
                recognise,
                sequence("Report unavailable detection",
                    step("Clear failed perception", lambda: context.active.clear()),
                    step("Log perception failure", lambda: context.event("perception_failed", pan=angle)),
                    optional(io.say("Perception uncertain", "这次图像识别未完成，继续检查下一个角度。")))),
            *(handle_owner(i) for i in (1, 2, 3)))
        return scan

    def room_task(room):
        litter = sequence("Inspect floor litter",
            head(0, level - config["litter_depression_deg"]), io.settle(),
            retry(io.detect("litter"), "perception", 2), step("Prepare litter report", context.litter),
            io.say("Announce floor litter"), step("Record litter announcement", context.litter_announced))
        return sequence("Patrol " + config["room_names"][room],
            step("Select room", lambda: context.select_room(room)),
            navigate("Navigate to " + room + " scan point", lambda: config["room_poses"][room]),
            *(scan_angle(angle) for angle in config["pan_angles_deg"]),
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
    patrol = PatrolBudget(child=sequence("Room patrol", *passes), context=context)
    mission = sequence("Hangzhou home service",
        step("Validate and initialise", context.initialise),
        navigate("Enter arena to enrollment point", lambda: config["entry_pose"]),
        *(enrollment(i) for i in (1, 2, 3)),
        optional(io.say("Enrollment completed", "三位主人的信息已记住，请前往各自房间。我开始巡游。")),
        selector("Patrol or bounded recovery",
            patrol,
            sequence("Stop incomplete patrol", retract(),
                step("Record patrol stopped", lambda: context.event("patrol_stopped")),
                optional(io.say("Patrol stopped", "巡游未全部完成，我将收回机械臂并结束本轮任务。")))),
        retract(), step("Summarise task", context.summary), optional(io.say("Speak task summary")),
        navigate("Leave through separate exit", lambda: config["exit_pose"]),
        step("Record exit", lambda: context.event("exited")))
    timed = py_trees.decorators.Timeout(name="Eight-minute match limit", child=mission,
                                        duration=config["mission_timeout_seconds"])
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
                       ReportFailure("Report mission failure", timed, context), recovery),
        policy=py_trees.common.OneShotPolicy.ON_COMPLETION)
    root.context = context
    return root
