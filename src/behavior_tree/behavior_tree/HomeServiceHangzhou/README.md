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

依据用户给出的《家庭服务赛道》2.5“比赛中”和 2.6：
4 房间、3 位主人、姓名/行为/垃圾播报、需求原意复述、8 分钟、
入口与出口不同。按用户要求，本次不含垃圾抓取和投桶。

## 决策树

```text
OneShot（成功或失败均锁存，不随 runtime 下一次 tick 重新比赛）
└─ Selector：比赛 / 失败收臂
   ├─ Timeout：480 秒
   │  └─ Sequence
   │     ├─ 配置检查、黑板和留证目录初始化
   │     ├─ 收臂 → 自主入场到登记位置
   │     ├─ 依次登记主人 1、2、3
   │     │  └─ 中文召唤 → 听名字 → 特征提取 → 参考图和姓名留存 → 播报姓名
   │     ├─ 巡游（从 current_room 开始轮转四房间；最多 max_passes 轮）
   │     │  └─ 每房间
   │     │     ├─ 导航到固定扫描点
   │     │     ├─ 依次 pan=-90,-30,+30,+90 度（左到右）
   │     │     │  ├─ 到位反馈 → 等新帧 → behaviour_detection
   │     │     │  ├─ 只按行为选人，保存该人物框和行为图，不要求露出脸
   │     │     │  ├─ 不带姓名的行为播报 → 收臂 → approach 当事人
   │     │     │  └─ 按行为交互，在离开当事人前匹配身份
   │     │     │     ├─ 休息/睡觉：询问开/关 → 复述 → 身份匹配 → 开关点导航 → 伸臂 → 收臂
   │     │     │     ├─ 躺地：扶人位姿 → 中文问答/复述 → 收臂 → 身份匹配
   │     │     │     └─ 挥手：询问需求 → 原话复述 → 身份匹配
   │     │     │        每次服务后返回本房间扫描点，再继续后面的角度
   │     │     ├─ pan=0，物理俯视30度 → 新帧 → litter_detection → 中文播报
   │     │     └─ 云台回水平，记录到访
   │     ├─ 收臂 → 如实总结完成数（说明未抓取和放置垃圾）
   │     └─ 导航到独立出口 → 记录离场
   └─ 失败后尝试收臂、播报、保持 FAILURE
```

三种行为出现顺序不限。登记顺序定义“主人1/2/3”；姓名通过现场中文语音从
“杰克、安东、李华”中匹配，外貌来自现场提取，不按候选词顺序预先分配身份。
匹配不到、同一句包含多个候选姓名或姓名重复时重新询问。
使用 `pypinyin` 将连续汉字转换为不带声调的拼音音节，支持“李花/梨华→李华”、
“安冬→安东”、“杰可→杰克”等同音匹配，匹配唯一候选后直接归一化并沿用登记结果播报。
精确姓名和同音姓名若指向不同候选，也拒绝本次输入。没有添加英文别名、拼写距离或近音猜测；
标点、空格不能把分开的汉字拼接成姓名。`pypinyin>=0.51,<1` 已声明在 setup.py 中，
部署时需在实际运行 ROS 节点的 Python 环境安装该依赖；colcon 构建本身不保证自动安装 pip 依赖。
姓名词表位于 `config.py:OWNER_NAMES`，采用已有 phrase extraction 的候选词匹配思路；
仍调用 `listen_action`，避免本地旧 `phrase_extraction_ac.py` 的英文专用模型限制。
身份匹配统一先播报“请您面向我顶部的摄像头，让我看清您的脸，方便我认出您”，
云台回水平，等待 `identity_facing_wait_seconds`（默认 2 秒），复用 Orbbec 人物特征提取
获取当面人物的新裁剪图，再与三张登记参考图做 VLM 匹配。
新图匹配成功就播报姓名；新图为空、超时或不确定时，用最初保存的那个人物框补充匹配。
两路各自允许未知，任何一路明确匹配即可；正面图成功后不再请求旧图，不做多候选投票。
本次特征图不会覆盖三位主人的登记参考图。靠近仍要求有效、未过期的 map 坐标。
完成的行为服务去重，已识别主人按实际匹配 ID 记账；不根据行为推断其登记编号。
两图均不确定仍记录“已服务但身份未确认”，不计入已确认主人，也不重复已完成的同类服务。
交互动作本身失败则保持该行为未完成，可在后续扫描/下一轮重试。
所有四房间至少巡游一轮；若三主人交互完成、到访四房间且三个房间发现垃圾，
跳过额外巡游轮。默认最多两轮，也可配置一轮。

## 节点与接口

| 功能 | 复用或新增 | 接口 |
|---|---|---|
| 中文播报 | BtNode_Announce | announce，tinker2_zh 上为中文 TTS |
| 中文自由语音 | BtNode_ListenAction | listen_action |
| “开/关”选择 | BtNode_PhraseExtractionAction | phrase_extraction_action；候选词为中文“开”“关” |
| 主人特征、登记图 | BtNode_FeatureExtraction | feature_extraction_service，Orbbec |
| 固定点导航 | BtNode_GotoAction | navigate_to_pose |
| 接近主人 | BtNode_Approach | go_to_approach，目标 map PointStamped，距离 0.7m |
| 机械臂 | Tk27MoveArm（复用 BtNode_MoveArmSingle 生命周期） | /manipulation/joint_move |
| 云台 | SettledPanTilt（基于 BtNode_TurnPanTilt） | pan_tilt_controller 的既有 command/state |
| 行为/垃圾检测 | DetectScene | /behaviour_detection/detect、/litter_detection/detect，DetectRgbScene |
| 身份匹配 | IdentifyOwners | 交互后正面人物新图优先，原行为人物 ROI 兜底；与三张登记图比对 |
| 状态/时间预算 | StateNode、PatrolBudget | /hangzhou/ 黑板和事件日志 |

已有 FeatureMatching 服务会将未匹配项修补成有效候选编号，房间只有一个人时可能
把三个登记身份都指向他，因此本任务不使用该服务决定主人身份。新身份节点复用
`kimi_api._env` 的模型/认证配置，允许 `null`（未知），不依据动作推断身份；
畸形 JSON、重复身份、接口错误均拒绝。原图兜底只使用所接近人物的 `bbox`，
不会把同帧其他人的匹配结果混作该当事人的身份。新图使用特征服务返回的前景人物裁剪图。

DetectRgbScene 的请求只有 target_frame，不接受外部图像。调用时由检测服务冻结当前
RGB（行为服务也冻结匹配深度/TF），返回图像及时间戳；本包验证图像时间晚于云台到位。
服务异常不能当成“未发现”，单独中文提示并记录错误。

云台协议：pan 正方向为向右；当前安装记录水平约为 firmware tilt=+30°，
firmware tilt=0° 是物理向下30°。配置 `level_tilt_deg` 表示现场校准的水平命令角；
垃圾角为 `level_tilt_deg - 30`。不要将物理俯角直接当作未经校准的舵机命令。

## 配置占位

编辑 `settings.json` 的副本，并通过 `--config /absolute/path/site.json` 加载。
所有固定导航点、机械臂位姿均为 `null`，不会把占位替换成地图原点。
`--check` 和正式入口都会列出未填写项并退出，任何动作之前必须填全。

- entry_pose、exit_pose、room_poses：map 下 `[x米, y米, yaw弧度]`。
- current_room：登记位置所属房间；rooms 从这里旋转后按序巡游。
- switches.<room>.on/off.navigation：每个房间“开”“关”标记的底盘停靠点。
- switches.<room>.on/off.arm：对应末端位于标记上方的7轴关节角，弧度。
- arm_assist：扶人模拟位姿，末端应在人体上方；arm_stow：导航收臂位姿。
- level_tilt_deg：水平云台命令角；pan_angles_deg 固定为四个角。
- max_passes=2；patrol_timeout_seconds=420；mission_timeout_seconds=480；
  exit_reserve_seconds=60。巡游截止取“巡游自身预算”与“比赛开始+480-60”的较早值。
  总超时会取消正在运行的行为并进入收臂恢复；恢复收臂的安全动作可能超出比赛计时，
  不将它记为比赛任务完成。突发 SIGINT/SIGTERM 沿用 runtime 的取消逻辑，不保证自动收臂。
- service_timeout_seconds=65，identity_vlm_timeout_seconds=25；
  target_max_age_seconds=45：只有有效 map 坐标且拍摄时间未过期，才接近主人。
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
本包默认使用独立 live_config.json，禁用模拟；显式指定模拟环境时正式入口会拒绝运行，
避免把模拟成功当成比赛完成。不会改写共享 mock_config.json。

身份 VLM 使用现有环境变量（DASHSCOPE_API_KEY / 兼容旧拼写 DASHCOPE_API_KEY，
或 OpenRouter 对应配置）。不在文件中保存密钥。python-dotenv 可选；没有它时需让
启动进程继承已经配置的环境变量。身份请求使用标准库 HTTP，不需要额外安装 OpenAI SDK。

tkmr 的用户 Python 目录存在 NumPy 2，和系统 ROS cv_bridge/OpenCV 的 NumPy 1 ABI
冲突；上面的 PYTHONNOUSERSITE=1 使用系统兼容环境，不卸载/降级用户已有包。

## 留证与验证

每次运行产生时间命名目录：三张主人登记 PNG，以及 `events.jsonl`。
日志包括姓名、特征、行为、身份对应、原始/复述语音、成功/失败、房间、检测捕获时间、
检测服务输出的 vision_log JSON 路径。检测服务自己保存原图和标注图。
按比赛规则核对得分所需姓名、行为、垃圾全部中文播报；没有伪报垃圾抓取/放置完成。

```bash
# 在上述 source 环境下
export PYTHONPATH="$PWD/src/behavior_tree:$PYTHONPATH"
PYTHONNOUSERSITE=1 /usr/bin/python3 -m pytest -q \
  src/behavior_tree/test/test_hangzhou_home_service.py
```

测试使用真实树和状态逻辑、模拟 I/O，不向硬件发命令，也不调用付费 VLM。
覆盖随机 3→1→2 顺序、4房间16次扫描、4次垃圾检查、开/关分流、中文姓名播报、
需求逐字复述、去重、未知/空/失败观测、缺失位姿、收臂失败、超时迟到结果、
新帧时间检查、真实服务消息类型、过期/无效人物目标、离场预留时间与终态锁存。
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
仅在内存中生成模拟位姿，settings.json 的 null 占位不变。
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

