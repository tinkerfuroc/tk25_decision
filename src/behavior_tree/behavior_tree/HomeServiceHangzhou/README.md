# HomeServiceHangzhou：杭州居家生活服务

2026-10-15 静态检查、HRI 对照、重试与恢复策略、离线验证说明见
[STATIC_REVIEW.md](STATIC_REVIEW.md)。

与 HRI、Restaurant、PickAndPlace 并列的 Python 任务子包，沿用同一个 ROS 包
`behavior_tree` 和 `py_trees` 写法。入口：
`ros2 run behavior_tree hangzhou-home`。

## 分支和工作区

基础分支实际名称为 `tinker2-net`，起点 `da8583c09f15ca07474a9c3c39ca6f20ebcb16eb`；
新分支为 `tinker2_hangzhou`。
已迁入 `/home/tinker/tk25_ws/src/tk25_decision` 主工作区的 `tinker2_hangzhou` 分支。
原 GPSR 分支的未提交改动单独保存在 Git stash，未混入此分支。
音频仓库实际中文分支名是 `tinker2_zh`。

The current task covers four rooms, three owners, identity/behaviour/litter
reports and service requests. Garbage collection is excluded. There is no
mission time limit or exit navigation.

The Hangzhou tmux launcher starts the decision process in its decision window.
Localize the robot, then press Enter there to begin. The CLI also waits for
Enter when launched manually; Ctrl+C or EOF exits without running the mission.
The five-second silent delay starts after the interfaces have initialized.

## Task flow

~~~text
OneShot (terminal outcomes remain latched)
+- Mission
|  +- Validate configuration and initialize evidence
|  +- Wait silently for 5 seconds
|  +- Stow arm, then navigate to register
|  +- Enroll three owners using speech and reference images
|  +- Patrol four rooms, up to max_passes
|  |  +- Navigate to the room scan point with the arm stowed
|  |  +- Observe at pan -90, -30, +30, +90 degrees
|  |  |  +- Select a person by behaviour
|  |  |  +- Match identity from the observed crop, with facing-image fallback
|  |  |  +- Announce room, owner name and behaviour before approach
|  |  |  +- Resting: interpret on/off/keep-state, repeat and simulate operation
|  |  |  +- Fallen: assistance pose, ask and repeat, retract
|  |  |  +- Waving: ask for help request and repeat
|  |  |  +- Return to scan point
|  |  +- Inspect and report floor litter
|  +- Retract arm, report results, finish in place
+- On failure: attempt retraction, report, preserve FAILURE
~~~


三种行为出现顺序不限。登记顺序定义“主人1/2/3”；姓名通过现场中文语音从
“杰克、安东、李华”中匹配，外貌来自现场提取，不按候选词顺序预先分配身份。
匹配不到、同一句包含多个候选姓名或姓名重复时重新询问。
使用 `pypinyin` 将连续汉字转换为不带声调的拼音音节，支持“李花/梨华→李华”、
“安冬→安东”、“杰可→杰克”等同音匹配，匹配唯一候选后直接归一化并沿用登记结果播报。
Exact and homophone matching takes precedence. If neither matches, a complete
short name response may differ by one phonetic edit in one syllable, with the
other syllable unchanged. Ambiguous matches are not assigned to an owner.
An explicit self-introduction after a correction supersedes the earlier text.
Punctuation and spaces cannot join unrelated syllables into a name.
`pypinyin>=0.51,<1` is declared in setup.py.
部署时需在实际运行 ROS 节点的 Python 环境安装该依赖；colcon 构建本身不保证自动安装 pip 依赖。
姓名词表位于 `config.py:OWNER_NAMES`，采用已有 phrase extraction 的候选词匹配思路；
仍调用 `listen_action`，避免本地旧 `phrase_extraction_ac.py` 的英文专用模型限制。
Identity is resolved before approach and service. Match the selected person's
original detection ROI against the three enrollment images first. If inconclusive,
request a facing image and retry up to identity_retry_attempts additional times.
Report the room, the recognized name and the observed behaviour together before
approaching. An unknown person is explicitly announced as unidentified; no name is
inferred from behaviour. Capturing a new image does not overwrite enrollment images.
Person coordinates must be finite and expressed in map; capture timestamps do not
expire coordinates or images. Missing diagnostic log paths do not invalidate a scene.
完成的行为服务去重，已识别主人按实际匹配 ID 记账；不根据行为推断其登记编号。
所有尝试均不确定仍记录“已服务但身份未确认”，不计入已确认主人，也不重复已完成的同类服务。
交互动作本身失败则保持该行为未完成，可在后续扫描/下一轮重试。
所有四房间至少巡游一轮；若三主人交互完成、到访四房间且三个房间发现垃圾，
跳过额外巡游轮。默认最多两轮，也可配置一轮。

## Switch interpretation

SwitchIntent classifies the complete listen_action transcript as on, off, none,
or unknown. A negated instruction alone leaves the current state unchanged;
an ambiguous request triggers another question. Explicit corrections are resolved
from the whole utterance. No device state is inferred from the absence of a command.
The task does not use phrase_extraction_action for switch control.

## 节点与接口

| 功能 | 复用或新增 | 接口 |
|---|---|---|
| 中文播报 | BtNode_Announce | announce，tinker2_zh 上为中文 TTS |
| 中文自由语音 | BtNode_ListenAction | listen_action |
| Switch intent | SwitchIntent | Full listen_action transcript, semantic classification through the existing model backend |
| 主人特征、登记图 | BtNode_FeatureExtraction | feature_extraction_service，Orbbec |
| 固定点导航 | BtNode_GotoAction | navigate_to_pose |
| 接近主人 | BtNode_Approach | go_to_approach，目标 map PointStamped，距离 0.7m |
| Saved arm poses | Tk27SavedPose (direct TK27 action client) | /manipulation/move_to_saved_pose |
| 云台 | SettledPanTilt（基于 BtNode_TurnPanTilt） | pan_tilt_controller 的既有 command/state |
| 行为/垃圾检测 | DetectScene | /behaviour_detection/detect、/litter_detection/detect，DetectRgbScene |
| Identity | IdentifyOwners | Before service: original person ROI first, facing image fallback; compare against three enrollment references |
| Task state | StateNode | /hangzhou/ blackboard and event log |

已有 FeatureMatching 服务会将未匹配项修补成有效候选编号，房间只有一个人时可能
把三个登记身份都指向他，因此本任务不使用该服务决定主人身份。新身份节点复用
`kimi_api._env` 的模型/认证配置，允许 `null`（未知），不依据动作推断身份；
畸形 JSON、重复身份、接口错误均拒绝。原图兜底只使用所接近人物的 `bbox`，
不会把同帧其他人的匹配结果混作该当事人的身份。新图使用特征服务返回的前景人物裁剪图。

DetectRgbScene 的请求只有 target_frame，不接受外部图像。调用时由检测服务冻结当前
RGB and aligned depth/TF are returned with capture timestamps for diagnostics; no capture-age gate is applied.
服务异常不能当成“未发现”，单独中文提示并记录错误。

云台协议：pan 正方向为向右；当前安装记录水平约为 firmware tilt=+30°，
firmware tilt=0° 是物理向下30°。配置 `level_tilt_deg` 表示现场校准的水平命令角；
垃圾角为 `level_tilt_deg - 30`。不要将物理俯角直接当作未经校准的舵机命令。

## Navigation arm pose

The decision tree sends TK27 MoveToSavedPose goals to
/manipulation/move_to_saved_pose. It sends pose names only:

- Navigation and retraction: navigation
- Simulated assistance: help
- Electrical switch interaction: click

TK27 resolves each name under its configured web_manip/export directory and reads
the seven arm joints when handling the request. The decision tree neither loads
these YAML files nor copies their joint angles. Collision avoidance remains enabled.

Every base-motion branch shares the same sequence: request navigation, wait for
successful MoveToSavedPose completion, then start navigation or person approach.
This includes registration, room visits, switch visits, and return-to-scan.
A failed stow prevents base motion. TK27 verifies measured joint convergence before
returning success; accepting the goal is insufficient.

## Map Studio navigation points

Set map_yaml to the navigation map YAML. Navigation goals in registration_pose,
room_poses, and switches.<room>.<on|off>.navigation can be Map Studio
pose names. At task startup the loader resolves names from the adjacent
<map>.poses.yaml through Map Studio's PoseStore. Inline [x, y, yaw] values
remain supported for offline tests and standalone configurations.

The live map is robocup_home_2026_measured_5cm, at 0.05 m/cell. The basic
launch script reads map_yaml from the same task config. Existing running
navigation services are not switched merely by editing configuration.
The live map's currently unassigned destinations are documented in TKMR_LIVE.md.

## Configuration placeholders

编辑 `settings.json` 的副本，并通过 `--config /absolute/path/site.json` 加载。
Navigation waypoints are null placeholders; arm poses use the saved names above.
`--check` 和正式入口都会列出未填写项并退出，任何动作之前必须填全。

- registration_pose, room_poses: Map Studio names or map-frame [x, y, yaw].
- current_room：登记位置所属房间；rooms 从这里旋转后按序巡游。
- switches.<room>.on/off.navigation：每个房间“开”“关”标记的底盘停靠点。
- switches.<room>.on/off.arm: saved switch pose name, normally click.
- arm_assist: saved assistance pose name (help); arm_stow: saved navigation pose name (navigation).
- level_tilt_deg：水平云台命令角；pan_angles_deg 固定为四个角。
- max_passes=2. There is no overall mission or patrol timeout. Startup waits
  silently for five seconds before navigating to registration_pose (register in
  the live map). The task finishes in place after its summary.
- service_timeout_seconds=65, identity_vlm_timeout_seconds=25. These bound pending
  requests, not the age of returned images or coordinates. target_max_age_seconds was removed.
- identity_qwen_backend：dashscope（默认）或 openrouter；
  identity_model 空字符串沿用对应 backend 的既有默认模型。
- evidence_directory：默认 ~/hangzhou_runs。

## 构建与运行

正式运行使用主工作区 install，不再需要 worktree overlay。实机配置与无参数
basic 启动说明见 [TKMR_LIVE.md](TKMR_LIVE.md)。

```bash
cd /home/tinker/tk25_ws
source /opt/ros/humble/setup.bash
source /home/tinker/tk25_ws/install/setup.bash
export PYTHONNOUSERSITE=1
colcon build \
  --paths src/tk25_decision/src/behavior_tree \
    /home/tinker/tk25_ws/src/tk26_vision/src/tinker_vision_msgs_26 \
    /home/tinker/tk25_ws/src/tk26_vision/src/vision_util \
    /home/tinker/tk25_ws/src/tk26_vision/src/kimi_api \
  --packages-select tinker_vision_msgs_26 vision_util kimi_api behavior_tree \
  --allow-overriding behavior_tree tinker_vision_msgs_26
source install/setup.bash

# 查看结构不控制机器人；正式默认配置为 tkmr_live.json
ros2 run behavior_tree hangzhou-home --draw
ros2 run behavior_tree hangzhou-home --check --config /absolute/path/site.json

# 已启动导航、机械臂、相机、云台、特征提取、两个场景检测器及中文语音后：
ros2 run behavior_tree hangzhou-home --config /absolute/path/site.json
```

中文音频使用 tk_24_audio 的 `tinker2_zh` 版本，对应完整启动文件
`ros2 launch audio_pakage audio.launch.py`（同时提供 announce / listen_action /
phrase_extraction_action）。没有切换或改写音频仓库。
The entrypoint follows the shared mock_config.json and respects explicit
BT_MOCK_MODE / BT_MOCK_CONFIG overrides. It does not force live mode or reject
mock leaves. Hangzhou wrappers reuse their underlying nodes' mock settings;
Tk27SavedPose follows BtNode_MoveArmSingle.

身份 VLM 使用现有环境变量（DASHSCOPE_API_KEY / 兼容旧拼写 DASHCOPE_API_KEY，
或 OpenRouter 对应配置）。不在文件中保存密钥。python-dotenv 可选；没有它时需让
启动进程继承已经配置的环境变量。身份请求使用标准库 HTTP，不需要额外安装 OpenAI SDK。

The production entry point uses tk25_decision/.venv_decision/bin/python,
as selected by tkbuild. This environment contains NumPy 1.26.4 and pypinyin
0.55.0. PYTHONNOUSERSITE=1 excludes incompatible packages from the user site;
it does not select the Python interpreter.

Keep pip dependencies in .venv_decision, never in install/behavior_tree.
setup.py declares pypinyin>=0.51,<1, but colcon does not install that dependency.
The exact environment is recorded in .venv_decision.uv-project/pyproject.toml
and uv.lock and restored through the workspace scripts/restore_venvs.sh.
After an intentional environment change, refresh its lock with:

~~~bash
cd /home/tinker/tk25_ws
python3 scripts/lock_venvs.py --only src/tk25_decision/.venv_decision --force
~~~

## 留证与验证

每次运行产生时间命名目录：三张主人登记 PNG，以及 `events.jsonl`。
日志包括姓名、特征、行为、身份对应、原始/复述语音、成功/失败、房间、检测捕获时间、
检测服务输出的 vision_log JSON 路径。检测服务自己保存原图和标注图。
按比赛规则核对得分所需姓名、行为、垃圾全部中文播报；没有伪报垃圾抓取/放置完成。

```bash
# 在上述 source 环境下
export PYTHONPATH="$PWD/src/behavior_tree:$PYTHONPATH"
PYTHONNOUSERSITE=1 .venv_decision/bin/python -m pytest -q \
  src/behavior_tree/test/test_hangzhou_home_service.py
```

测试使用真实树和状态逻辑、模拟 I/O，不向硬件发命令，也不调用付费 VLM。
覆盖随机 3→1→2 顺序、4房间16次扫描、4次垃圾检查、开/关分流、中文姓名播报、
需求逐字复述、去重、未知/空/失败观测、缺失位姿、收臂失败、超时迟到结果、
Pre-service reporting, real service schemas, accepted old timestamps, rejected invalid positions,
silent startup, registration navigation, switch interpretation and terminal latching are covered.
尚未做真实人像识别准确率、相机/云台标定、导航和机械臂实机联调。

本次验证：新增25项测试全部通过；隔离构建4个包通过。另跑公共入口迁移测试时，
发现两个原有失败（fetch_points 缺少 main、PickAndPlace CLI 旧测试桩不匹配），
在未修改的 tinker2-net 基线副本上也复现相同两个失败，未在本任务中改动它们。

## 仅导航与机械臂 mock 的现场 dry-run

basic 新增入口（本包也保存同名脚本副本）：

```bash
# 终端1：前台运行依赖；Ctrl-C 只结束此次启动的进程
/home/tinker/tk25_ws/src/tk25_basic/src/scripts/master_hangzhou.sh services

# 终端2：先检查发现的接口，再运行一次完整树；终态自动退出
/home/tinker/tk25_ws/src/tk25_basic/src/scripts/master_hangzhou.sh check
/home/tinker/tk25_ws/src/tk25_basic/src/scripts/master_hangzhou.sh run
```

脚本 source tk25_ws 和隔离 overlay，使用 tk_24_audio 的中文源码及原有音频 venv，
视觉使用 .venv-vision-main。语音识别显式继承中文家庭服务提示词。
相机、通用目标检测等已存在服务会复用；缺少时才启动。
本脚本不启动导航或机械臂，不调用 HRI master 的硬件全量 bringup 或 tmux 清理。

行为和垃圾服务固定使用独立 /hangzhou/behaviour_detection/detect 和
/hangzhou/litter_detection/detect，确保加载本任务 overlay 中的消息定义，
避免和共享 install 的旧版 DetectRgbScene 服务混用。
正式 settings.json 的服务端点保持原值；此命名空间仅由 dry_run 的内存配置注入。

dry_run.py 在建树时替换 goto、approach、arm，且检查树中没有真实运动执行节点。
Only navigation placeholders are filled in memory; saved arm pose names are retained.
云台、TTS、ASR、特征提取、行为检测、垃圾检测、身份匹配仍是真实接口。
因此需现场主人按提示报名字；底盘没有真实进入其他房间时，镜头看到的仍是当前场景，
不能用这次 dry-run 判断房间导航和完整场地覆盖是否正确。
输出 ~/hangzhou_dry_runs/run_*.json，逐动作证据在同目录下带时间戳子目录的 events.jsonl。
预检同时检查接口存在和云台实时 connected/feedback_ok；云台具体角度到位、
图像有效和 VLM 返回会在执行时再检查。

云台设备可通过 services --device /dev/serial/by-id/... 指定。
当前串口无权限时，管理员可对该设备给当前用户读写 ACL；脚本不执行 sudo 或修改权限。

2026-10-12 现场运行记录：
- 导航/机械臂隔离与任务测试共28项通过。
- 中文 TTS、listen_action、phrase_extraction_action、feature_extraction_service、
  云台控制/状态发布以及两个独立场景服务已启动。
- 初次服务发现预检通过；新增实时云台反馈检查后，预检准确报告 head_feedback 缺失。
- dry-run 已实际 tick；在第一步 Pan 0 tilt 30 等待真实反馈后失败，
  进入 mock 收臂和中文停止播报，最终 FAILURE，没有登记主人或开始巡游。
- 云台串口权限已由用户处理；驱动可以打开 /dev/ttyUSB0，
  但报告没有 T:1001 反馈。未将云台超时伪装成到位成功。
- 该次报告：/home/tinker/hangzhou_dry_runs/run_20261012_102943.json。
  需要先恢复云台供电/通信及反馈，再重跑 run；不能据此声称整场任务通过。
- 刷新最新 DetectRgbScene 消息后，两个独立视觉服务实测均返回 status=0：
  行为检测1个目标，垃圾检测0个目标；返回1280×720图像及留证JSON。
  结果在 /home/tinker/hangzhou_dry_runs/scene_smoke.json。
- 最终反馈预检报告：/home/tinker/hangzhou_dry_runs/run_20261012_103330.json。

