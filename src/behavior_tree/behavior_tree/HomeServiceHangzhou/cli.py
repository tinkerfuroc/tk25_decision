"""ros2 run behavior_tree hangzhou-home [--config FILE | --check | --draw]."""
import argparse
import os
import sys
from pathlib import Path
from .config import load_config, validate


def main():
    parser = argparse.ArgumentParser(description="杭州居家生活机器人决策树")
    parser.add_argument("--config", help="现场标定 JSON；默认包内 tkmr_live.json")
    parser.add_argument("--check", action="store_true", help="仅校验配置，不连接机器人")
    parser.add_argument("--draw", action="store_true", help="仅打印树结构，允许占位配置")
    args, ros_args = parser.parse_known_args()
    config = load_config(args.config or os.environ.get("HANGZHOU_CONFIG")
                         or Path(__file__).with_name("tkmr_live.json"))
    if not args.draw:
        try:
            validate(config)
        except ValueError as exc:
            parser.exit(2, "配置尚未就绪（没有向机器人发送指令）：\n" + str(exc) + "\n")
    if args.check:
        print("Hangzhou configuration OK")
        return
    os.environ["BT_MOCK_MODE"] = "false"
    os.environ["BT_MOCK_CONFIG"] = str(Path(__file__).with_name("live_config.json"))
    from .tree import create_task
    if args.draw:
        import py_trees
        print(py_trees.display.unicode_tree(create_task(config)))
        return

    def build():
        # Check generated interfaces before any hardware leaf can run.
        import cv2
        from cv_bridge import CvBridge
        from kimi_api._env import load_env, resolve_qwen_target
        from tinker_vision_msgs_26.srv import DetectRgbScene
        CvBridge()
        if not cv2.__version__:
            raise RuntimeError("OpenCV is not available")
        load_env()
        resolve_qwen_target(config["identity_qwen_backend"], config.get("identity_model", ""))
        if not hasattr(DetectRgbScene.Request(), "target_frame"):
            raise RuntimeError("Rebuild/source current tinker_vision_msgs_26 first")
        root = create_task(config)
        for node in root.iterate():
            if getattr(node, "mock_mode", False):
                raise RuntimeError("Live mission contains mocked node: " + node.name)
            for attr in ("action_type", "service_type"):
                if getattr(getattr(node, attr, None), "__module__", "").startswith("behavior_tree.mock_messages"):
                    raise RuntimeError("Missing real ROS interface for " + node.name)
        return root

    sys.argv = [sys.argv[0]] + ros_args
    from behavior_tree.runtime import run_tree
    run_tree(build, period_ms=200.0, title="杭州居家生活服务", node_name="hangzhou_home_tree",
             show_during_setup=True, setup_timeout=120.0)


if __name__ == "__main__":
    main()
