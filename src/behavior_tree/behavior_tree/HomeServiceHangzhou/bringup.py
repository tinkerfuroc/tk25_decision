"""Foreground dependency bringup: real Chinese audio/vision/head, no base or arm."""
import argparse
import os
import time
from pathlib import Path


def discover(node, seconds=2.0):
    import rclpy
    from rclpy.action import get_action_names_and_types
    from tinker_vision_msgs_26.msg import PanTiltState
    feedback = []
    subscription = node.create_subscription(PanTiltState, "/pan_tilt_controller/state",
        lambda msg: feedback.append((time.monotonic(), msg)), 1)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(subscription)
    return {
        "services": {name for name, _ in node.get_service_names_and_types()},
        "actions": {name for name, _ in get_action_names_and_types(node)},
        "camera": node.count_publishers("/camera/color/image_raw") > 0,
        "head": node.count_publishers("/pan_tilt_controller/state") > 0
                and node.count_subscribers("/pan_tilt_controller/cmd") > 0,
        "head_feedback": bool(feedback and feedback[-1][1].connected
                              and feedback[-1][1].feedback_ok
                              and time.monotonic() - feedback[-1][0] < 0.5),
    }


def missing_requirements(graph, config):
    endpoints = config["endpoints"]
    missing = []
    for key in ("announce", "behaviour_detection", "litter_detection"):
        if "/" + endpoints[key].strip("/") not in graph["services"]:
            missing.append(key)
    for key in ("listen", "feature_extraction"):
        if "/" + endpoints[key].strip("/") not in graph["actions"]:
            missing.append(key)
    missing.extend(key for key in ("camera", "head", "head_feedback") if not graph[key])
    return missing


def main():
    import rclpy
    from launch import LaunchDescription, LaunchService
    from launch.actions import ExecuteProcess
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", default="/home/tinker/tk25_ws")
    parser.add_argument("--device", default="/dev/ttyUSB0")
    parser.add_argument("--skip-head", action="store_true", help="Start other dependencies without a USB head device")
    parser.add_argument("--group", choices=("all", "audio", "vision-hardware", "perception"), default="all")
    args = parser.parse_args()
    ws = Path(args.workspace)
    rclpy.init()
    node = rclpy.create_node("hangzhou_dependency_probe")
    graph = discover(node, 3.0)
    node.destroy_node()
    rclpy.shutdown()
    vision = ws / "src/tk26_vision"
    audio = ws / "src/tk_24_audio"
    inherited = os.environ.get("PYTHONPATH", "")
    vision_env = {
        "PYTHONNOUSERSITE": "1",
        "PYTHONPATH": ":".join(str(vision / "src" / p) for p in
             ("vision_util", "kimi_api", "pan_tilt", "tk_vision_specialized")) + ":" + inherited,
    }
    audio_env = {"PYTHONNOUSERSITE": "1",
                 "LISTEN_INITIAL_PROMPT": "这是中文家庭服务对话。请准确记录主人姓名和请求：客厅、卧室、餐厅、厨房，打开或关闭电器，需要帮助。",
                 "PYTHONPATH": str(audio / "src/audio_pakage") + ":" + inherited}
    vision_python = str(vision / ".venv-vision-main/bin/python")
    audio_python = str(audio / ".venv_audio/bin/python")
    processes = []

    def start(name, command, env):
        group = ("audio" if name in ("hangzhou_tts", "hangzhou_listen_ac", "hangzhou_phrase_extraction_ac")
                 else "vision-hardware" if name in ("hangzhou_camera", "hangzhou_pan_controller", "hangzhou_pan_state")
                 else "perception")
        if args.group not in ("all", group):
            return
        print("Starting " + name, flush=True)
        processes.append(ExecuteProcess(name=name, cmd=command, cwd=str(ws),
                         additional_env=env, output="screen", respawn=False))

    for endpoint, module in (("/announce", "tts"), ("/listen_action", "listen_ac"),
                             ("/phrase_extraction_action", "phrase_extraction_ac")):
        collection = graph["services"] if endpoint == "/announce" else graph["actions"]
        if endpoint not in collection:
            start("hangzhou_" + module, [audio_python, "-u", "-m", "audio_pakage." + module], audio_env)
    if not graph["camera"]:
        start("hangzhou_camera", ["ros2", "launch", "orbbec_camera", "femto_bolt.launch.py",
             "depth_registration:=true", "enable_ir:=false"], vision_env)
    if "/object_detection_generalist" not in graph["services"]:
        start("hangzhou_generalist", ["ros2", "run", "object_detection_generalist", "generalist_node"], vision_env)
    if "/feature_extraction_service" not in graph["actions"]:
        start("hangzhou_features", [vision_python, "-u", "-m", "kimi_api.feature_recognition"], vision_env)
    if not args.skip_head and not graph["head"]:
        params = str(vision / "src/pan_tilt/config/pan_tilt.yaml")
        start("hangzhou_pan_controller", [vision_python, "-u", "-m", "pan_tilt.pan_tilt_controller",
              "--ros-args", "--params-file", params, "-p", "device:=" + args.device], vision_env)
        start("hangzhou_pan_state", [vision_python, "-u", "-m", "pan_tilt.pan_tilt_state_publisher",
              "--ros-args", "--params-file", params], vision_env)
    for task in ("behaviour", "litter"):
        if "/hangzhou/" + task + "_detection/detect" not in graph["services"]:
            start("hangzhou_" + task, [vision_python, "-u", "-c",
                  "from tk_vision_specialized." + task + "_detection import main; main()", "--ros-args",
                  "-r", "__ns:=/hangzhou"], vision_env)
    print("Real dependency group: " + args.group + "; no mock services are started.", flush=True)
    if processes:
        service = LaunchService()
        service.include_launch_description(LaunchDescription(processes))
        raise SystemExit(service.run())
    print("All dependencies already available.", flush=True)


if __name__ == "__main__":
    main()

