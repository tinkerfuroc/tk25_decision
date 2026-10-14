"""Real-service dry-run with navigation, approach and arm replaced at construction."""
import argparse
import copy
import json
import os
import signal
import time
from pathlib import Path

from .config import load_config, validate


def mock_config(config):
    result = copy.deepcopy(config)
    for task in ("behaviour", "litter"):
        result["endpoints"][task + "_detection"] = "/hangzhou/" + task + "_detection/detect"
    result.update(entry_pose=[0, 0, 0], exit_pose=[1, 0, 0],
                  arm_stow=[0] * 7, arm_assist=[0] * 7,
                  evidence_directory="~/hangzhou_dry_runs")
    for room in result["rooms"]:
        result["room_poses"][room] = [0, 0, 0]
        for command in ("on", "off"):
            result["switches"][room][command] = {"navigation": [0, 0, 0], "arm": [0] * 7}
    return result


def motion_mock_factory(context):
    from .ros_nodes import RosLeaves
    from .state import StateNode, get

    class MotionMockLeaves(RosLeaves):
        def record(self, name, operation, key):
            def execute():
                if operation in ("navigate", "approach") and context.arm_dirty:
                    return False
                context.event("mock_motion", operation=operation, node=name, target=repr(get(key)))
                print("[MOCK MOTION] " + operation + ": " + name, flush=True)
                return True
            return StateNode(name + " [MOCK]", execute)

        def goto(self, name):
            return self.record(name, "navigate", "nav_goal")

        def approach(self):
            return self.record("Approach owner", "approach", "target")

        def arm(self, name):
            return self.record(name, "arm", "arm_goal")
    return MotionMockLeaves(context)


def configure_real_perception():
    os.environ["BT_MOCK_MODE"] = "false"
    os.environ["BT_MOCK_CONFIG"] = str(Path(__file__).with_name("live_config.json"))


def build_tree(config):
    configure_real_perception()
    from .tree import create_task
    from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction, BtNode_Approach
    from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle
    root = create_task(mock_config(config), leaves_factory=motion_mock_factory)
    for node in root.iterate():
        if isinstance(node, (BtNode_GotoAction, BtNode_Approach, BtNode_MoveArmSingle)):
            raise RuntimeError("Real motion node found in dry-run: " + node.name)
        if getattr(node, "mock_mode", False):
            raise RuntimeError("Non-motion node unexpectedly mocked: " + node.name)
    return root


def main():
    import py_trees
    import rclpy
    from .bringup import discover, missing_requirements
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config")
    parser.add_argument("--preflight", action="store_true")
    parser.add_argument("--readiness-timeout", type=float, default=90)
    args = parser.parse_args()
    config = mock_config(load_config(args.config))
    validate(config)
    configure_real_perception()
    rclpy.init()
    node = rclpy.create_node("hangzhou_dry_run")
    root = None
    report = {"mode": "motion-only", "mocked": ["navigation", "approach", "arm"],
              "real": ["audio", "vision", "identity", "pan_tilt"]}
    directory = Path("~/hangzhou_dry_runs").expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    report_path = directory / ("run_" + time.strftime("%Y%m%d_%H%M%S") + ".json")
    outcome = 1

    def stop(signum, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, stop)
    try:
        deadline = time.monotonic() + args.readiness_timeout
        while True:
            missing = missing_requirements(discover(node, 1), config)
            if not missing or time.monotonic() >= deadline:
                break
        report["missing_services"] = missing
        if missing:
            report["status"] = "PREFLIGHT_FAILED"
        elif args.preflight:
            report["status"] = "PREFLIGHT_OK"
            outcome = 0
        else:
            root = build_tree(config)
            tree = py_trees.trees.BehaviourTree(root)
            tree.setup(timeout=20, node=node)
            deadline = time.monotonic() + config["mission_timeout_seconds"] + 70
            previous_tip = None
            failures = []
            while time.monotonic() < deadline:
                tree.tick()
                for leaf in root.iterate():
                    if (not leaf.children and leaf.status == py_trees.common.Status.FAILURE
                            and leaf.feedback_message):
                        failure = {"node": leaf.name, "reason": leaf.feedback_message}
                        if failure not in failures:
                            failures.append(failure)
                            print("[FAILED LEAF] " + json.dumps(failure, ensure_ascii=False), flush=True)
                tip = root.tip()
                current_tip = (tip.name, tip.status.name, tip.feedback_message) if tip else None
                if current_tip != previous_tip:
                    print("[TREE] " + repr(current_tip), flush=True)
                    previous_tip = current_tip
                if root.status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
                    break
                rclpy.spin_once(node, timeout_sec=0.1)
            report.update(status=root.status.name, completed=sorted(root.context.completed),
                          visited=sorted(root.context.visited),
                          registered=[owner.name for owner in root.context.owners],
                          evidence=str(root.context.evidence_dir), last_tip=previous_tip,
                          failed_leaves=failures)
            outcome = 0 if root.status == py_trees.common.Status.SUCCESS else 1
    except KeyboardInterrupt:
        report["status"] = "INTERRUPTED"
    except Exception as exc:
        report.update(status="ERROR", error=str(exc))
        raise
    finally:
        if root is not None:
            root.stop(py_trees.common.Status.INVALID)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print("DRY_RUN_REPORT=" + str(report_path), flush=True)
        print(json.dumps(report, ensure_ascii=False), flush=True)
    raise SystemExit(outcome)


if __name__ == "__main__":
    main()

