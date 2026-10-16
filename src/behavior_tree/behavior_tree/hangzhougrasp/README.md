# hangzhougrasp

与 `HRI/`、`PickAndPlace/` 并列的任务模块，入口：
`ros2 run behavior_tree hangzhougrasp`。多房间任务：每个房间先扫地面点（只扫描播报），
再逐个桌子扫描并尝试抓取；房间顺序由 enterRoom profile 决定。
支持两张已知地图（`--map`）：`robocup_home_2026_measured_5cm`（默认）和
`8902_2_edit`，每张地图有自己的入口房间与 profile。

## 决策树

两种模式都先写常量，再按 profile 的房间顺序依次执行 `Room <name>`：

```text
Write constants
→ [--grasp] Arm to nav before entering the arena
→ Room A: Ground phase → Tables phase
→ Room B: Ground phase → Tables phase
→ ...
```

不抓取（默认 `--no-grasp`）：桌子阶段对每个桌子的每个视点依次
`Go to <pose>` → `Scan or miss`（Orbbec + yolo26_sam 扫描，中文播报；
没检测到则跳过，不算失败）。不构建 Pick、夹爪节点。

抓取（`--grasp`，TK27 机械臂）：桌子阶段对每个桌子：

```text
Visit table:  对每个视点（升序 m）
  Skip or visit：已持有物体则跳过剩余视点
    Go to <table 视点>
    Clear previous scan result
    Orbbec 扫描 + 中文播报（机械臂保持 navigation 姿态）
    只有 Orbbec 看到物体才继续（没看到：该视点算 miss，机械臂不动）：
      取 Orbbec 扫描中最近目标的标签（只取一次；本任务不再做 RealSense 扫描，
      Pick 自己观测和分割）
      整个抓取尝试最多 grasp_retry_limit（3）次，每次都重新执行：
        MoveToSavedPose(pick45)
        → 打开夹爪（严格检查）→ TK27 Pick(标签, high)
      → 记为持有 → MoveToSavedPose(navigation)
      任一步失败：MoveToSavedPose(navigation)；成功则该视点算 miss，
      收回本身失败则整个任务失败
Deliver if holding:  持有物体才执行（规则见“放置规则”）
```

所有视点都没抓到则跳过这个桌子（记录，不算失败）。
地面阶段：按 `(n, m)` 升序逐个 `Go to <ground pose>` → `Scan or miss`，
只扫描播报，不抓取、不放置。

每次 Orbbec 扫描前都先把云台转到绝对角度（固件角度：30 = 水平，0 = 向下约 30°）：
桌子 `table_scan_tilt_deg`=0，地面 `ground_scan_tilt_deg`=0，pan `head_pan_deg`=0，
范围限定 [-30, 30]。云台重试 `head_retry_limit`（3）次仍未到位则记 ERROR 并照常扫描。

`--grasp` 是**构建参数**：关闭时不构建 Pick、夹爪节点，
也不等待这些 action server（`--no-grasp` 不发任何机械臂指令）。`--grasp` 时每次导航前都先 MoveToSavedPose(navigation)，进入场地前也先收回。导航失败会让任务失败；扫描/抓取 miss 不会，
但失败后的收回机械臂（cleanup）若重试后仍失败，任务失败并停止，
不会带着伸出的机械臂继续导航。
播报（TTS）重试后仍失败只记一条 ERROR 日志（`Log announce failure`），
不算 miss：照常继续抓取和后续路线。

### TK27 机械臂接口

| 节点 | Action | 目标 | 成功条件 |
|---|---|---|---|
| Pick | `/manipulation/pick`（`tk27_manipulation_msgs/Pick`） | `prompt`=Orbbec 扫描最近目标的标签，`plan_effort`=`high` | SUCCEEDED 且 `success` 且 `status==OK` |
| 机械臂姿态 | `/manipulation/move_to_saved_pose`（`MoveToSavedPose`） | `pose_name`=`pick45` / `navigation`，`avoid_collisions`=`false`（固定） | SUCCEEDED 且 `success` 且 `status==SUCCESS` |
| 打开夹爪 | `/xarm_gripper/gripper_action`（`GripperCommand`） | `position`=0.0，`max_effort`=50 | SUCCEEDED 且 `reached_goal` 且位置与 0.0 相差 ≤0.025（与 TK27 自身的判断相同） |

- 每次 Pick 之前都单独打开夹爪（语音指令要求），打开失败则不发送 Pick。
- Pick 只接收文字 prompt。服务器自己调用 `/object_detection_generalist`
  （RealSense，生产配置 `vision_force_vlm_sam: true`：VLM 定位 + MobileSAM
  分割），抓取最近的目标；本任务两次 yolo26 扫描的结果都不会传给 Pick。
  因此每次 Pick 都需要 `generalist_node` 及其 VLM API 正常工作。
- 放下物体（destination 或其他房间的放置点）：打开夹爪重试
  `arm_retry_limit` 次；仍失败则记 ERROR 并继续任务（剩下的扫描播报照样得分），
  持有状态无论如何都清除，否则后面所有视点都会被当作“已持有”而跳过。
- mock 模式里扫描节点返回空物体列表，所以 mock 跑不到 Pick；Pick 的行为
  由单元测试（打桩）覆盖，真实验证见下面的上机顺序。

TK27 启动（`ROBOT_NAME=tinker2`，各自终端 source 后运行）：

```bash
ros2 launch manipulation_bringup arm.launch.py           # 机械臂硬件、夹爪、TF、RealSense
ros2 launch manipulation_bringup manipulation.launch.py  # GPU worker + Pick / MoveToSavedPose
ros2 run object_detection_generalist generalist_node     # Pick 使用的检测服务
ros2 run object_detection_generalist yolo26_sam_node     # 本任务自己的扫描
```

不要同时启动 TK25 的 `grasp_bringup` / `arm_api`。保存的姿态来自
`tk27_manipulation/src/web_manip/export/`（需要 `navigation.yaml`、`pick45.yaml`）。

上机验证顺序：确认三个 action 类型 → 确认运行中服务器的 `export_directory`
里有 `navigation`、`pick45` → 单独一次 `plan_effort="high"` 的 Pick 被接受
（不是 `INVALID_GOAL`）→ `--grasp --part scan-grasp` → `--part visit-table`
→ 完整任务。建树、`--print-tree`、mock 成功都不能证明真实接口正确。

## 点位命名

Map Studio 点位名为 `<room>_<furniture>_<n>_<m>`，例如
`bedroom_table_0_0`、`kitchen_ground_1_2`：

- `furniture` 只能是 `table` 或 `ground`；
- `n` 是家具编号，`m` 是该家具的视点编号；
- `room` 只能是小写字母和数字、以字母开头（写 `livingroom`，不是
  `living_room` 或 `LivingRoom`）；profile 的 `rooms` 里写了不合规的房间名
  会启动即报错。

顺序：房间内桌子按 `n` 升序，一个桌子的视点按 `m` 升序，地面点按
`(n, m)` 升序。不符合命名的点位不进入索引：profile 房间里**看起来像路线点**
的错误命名——以 `<room>_table` / `<room>_ground` 开头，或以 `_<数字>_<数字>`
结尾（如 `diningroom_table1_1`、`bedroom_tabel_1_0`）——会启动即报错并列出
点位名；其他点位（如 `bedroom_search`、`livingroom_switch`、`register`）
只记一条 warning。房间在 profile 里却没有任何
桌子/地面点也会启动即报错；`rooms` 中重复的房间名同样报错。destination 点位直接按名称从原始点位表读取，
不要求符合上面的命名。

## 地图、enter_profiles 与 enter_room

`constants.json`（或 `--config` 的 JSON）中，每张已知地图在 `maps` 下有自己的
`enter_room`（默认入口）和 `enter_profiles`；`map_name` 选默认地图：

```json
{
  "map_name": "robocup_home_2026_measured_5cm",
  "maps": {
    "robocup_home_2026_measured_5cm": {
      "enter_room": "livingroom",
      "enter_profiles": {
        "livingroom": {"rooms": ["livingroom", "kitchen", "diningroom", "bedroom"],
                       "destination": "livingroom_destination"},
        "diningroom": {"rooms": ["diningroom", "kitchen", "bedroom", "livingroom"],
                       "destination": "diningroom_destination"}
      }
    },
    "8902_2_edit": {
      "enter_room": "room1",
      "enter_profiles": {"room1": {"rooms": ["room1", "room2"], "destination": "room1_destination"}}
    }
  }
}
```

- `--map NAME` 覆盖 `map_name`；不在 `maps` 里的地图是启动错误，并列出已知地图。
- robocup 地图只有客厅（livingroom）和餐厅（diningroom）两个入口。
  `livingroom_destination`、`diningroom_destination` 需要在 Map Studio 里标出
  （名字可在 `constants.json` 改）；没标之前启动会报错并给出缺失的点位名。
- `8902_2_edit`（实验室地图）：点位已改名为 `room1_table_0_0`、`room2_table_0_0`
  （原 `grasp1`、`grasp2`）和 `room1_destination`（原 `point_area`）；
  入口 room1，顺序 room1 → room2。
- 每个 profile 给出覆盖房间的顺序 `rooms`（必须包含自身）和本次的
  `destination` 点位名。profile 不是对象、缺 `destination` 都是启动错误。
- `--enter-room NAME` 覆盖所选地图的 `enter_room`；未知名称是启动错误。
- 不做任何回退：缺 `maps`、未知地图、缺 `enter_profiles`、未知 `enter_room`、
  点位名找不到、点位文件不存在，都会在 setup 与任何运动之前以清晰信息退出
  （`--print-tree` 同样需要有效 profile）。顶层的旧键 `enter_room` /
  `enter_profiles` 以及地图条目里的其他键都会被拒绝。
- 外部 JSON 只需提供要覆盖的字段；未知键会被拒绝（覆盖 `maps` 时整体替换）。

## 放置规则

抓到物体后（`Deliver if holding`）：

- **在进入房间（enter room）抓到**：机械臂回导航姿态，导航到 profile 的
  `destination`，打开夹爪放下，然后继续剩余的桌子、地面点和房间。
- **在其他房间抓到**：到不了真正的放置位置。机械臂回导航姿态，导航到
  **下一个扫描点**（路线中该桌子之后的下一个点：本房间下一个桌子的第一个视点，
  否则下一个房间的第一个点——即它的第一个地面点），播报
  “无法到达真正的放置位置”，打开夹爪放在那里。导航姿态下相机看不到被放下的
  物体，所以该点正常扫描，不会重复检测。
- **没有下一个扫描点**（路线最后一个点）：回到 destination 做真正放置。
- 一次运行可抓多个物体；每次放置后继续，最后一个点之后结束。
  跳过的桌子不会中断任务。

## 检测器 yolo26_sam

扫描使用 `BtNode_ScanForYolo26`，服务 `object_detection_yolo26_sam`
（`constants.json` 的 `detection_service`）。需先在独立终端启动：

```bash
ros2 run object_detection_generalist yolo26_sam_node
```

该节点用仅 bbox 的 YOLO26 权重加 MobileSAM 掩码。非空 `prompt` 必须与
checkpoint 的标签完全一致，`label_map` 的键同理。`label_map`（`constants.json`）
是 YOLO 类别 → 普通话的词典，覆盖 `yolo26n-20261016-171034-yolo26n-c8ec-best.pt`
的全部 12 类：Banana 香蕉、Pringles 品客薯片、Shampoo 洗发水、Coke 可乐、
Biscuits 饼干、IceTea 冰红茶、Apple 苹果、Cup 杯子、HandSanitizer 洗手液、
SoapBox 肥皂盒、TeddyBear 泰迪熊、Detergent 洗衣液；换模型时要同步更新；
`prompt=""` 时使用 `label_map` 的全部键，两者都空则拒绝启动。
本任务的播报必须是普通话：`prompt` 里的每个标签都必须在 `label_map` 中有
中文名，否则启动即报错并列出缺少的标签。
扫描无检测结果会失败，所以在各阶段被包在 `Scan or miss` 里。
本任务的扫描都使用 Orbbec；`--grasp` 时 RealSense 只由 TK27 Pick 自己使用。
`label_map` 只控制中文播报，不控制抓取类别；每次抓最近的目标，重新检测时
选同类的最近实例，不保证跨帧跟踪。

## 配置与启动

```bash
cd /home/tinker/tk25_ws
./tkbuild tk25_decision --packages-select behavior_tree
./tkbuild tk26_navigation --packages-select navigation_bringup
./tkbuild tk26_vision --packages-select object_detection_generalist
source install/setup.bash

# 全流程（还需要 TK27 的 Pick、MoveToSavedPose 和夹爪 action，见上文）
ros2 run behavior_tree hangzhougrasp --enter-room bedroom --grasp

# 只打印树，不 setup，不连接服务，不发运动指令
ros2 run behavior_tree hangzhougrasp --enter-room bedroom --grasp --print-tree
```

点位文件的 yaw 单位是弧度；`map_studio.poses.load_poses` 转为 map frame 的
PoseStamped。

**Map Studio 重建陷阱**：默认读取已安装的 navigation_bringup/maps。在
Map Studio 里新增/改名点位后，必须重新构建 navigation_bringup
（`./tkbuild tk26_navigation --packages-select navigation_bringup`），
否则任务读到的还是旧点位；也可设 `MAP_STUDIO_MAPS_DIR` 指向源 maps 目录。

Python 工厂：

```python
from behavior_tree.hangzhougrasp.config import load_config
from behavior_tree.hangzhougrasp.hangzhougrasp import createHangzhouGrasp

config = load_config(map_name="robocup_home_2026_measured_5cm", enter_room="livingroom")
root = createHangzhouGrasp(config=config, enable_grasp=True)
```

### 已移除的参数

- `--scan-pose`、`--destination-pose`：删除，传入会报错（不是忽略；`--ros-args ...` 尾部参数仍被容忍）。
- JSON 键 `pose_scan`、`pose_destination`：删除，出现即为未知键错误。
- `scan_camera`：删除（从未被读取；相机由是否 `--grasp` 决定）。
- YOLO-World 检测器（`object_detection_yolo_world`）已被 yolo26_sam 取代。

## 先测小部分

`--part NAME`（默认 `full`）单独运行树的一部分。每个 part 与完整树使用
同一批工厂函数，只写自己需要的点位常量，互不依赖对方的黑板输出，
`--print-tree` 对所有 part 都可用。相关选项：`--room`、`--table N`、
`--at POSE`（先导航到该点）、`--next-pose POSE`。缺必需选项、点位/房间不存在、
需要机械臂却没加 `--grasp`，都在 setup 之前报错。
加 `--grasp` 时，`room`、`visit-table`、`ground` 以及任何带 `--at` 的 part
在第一次导航前先把机械臂收回导航姿态。

| part | 作用 | 示例 |
|---|---|---|
| `scan-announce` | 原地用 Orbbec 扫描并播报（不会把机械臂移到桌面扫描姿态） | `--part scan-announce` |
| `scan-grasp` | 扫描、选择、抓取，失败收回（需 `--grasp`） | `--grasp --part scan-grasp` |
| `visit-table` | 某房间某桌子的全部视点，不放置（需 `--grasp`） | `--grasp --part visit-table --room bedroom --table 0` |
| `deliver` | 臂回导航姿态，去 destination，开夹爪（需 `--grasp`） | `--grasp --part deliver` |
| `drop-elsewhere` | 去 `--next-pose` 放下（需 `--grasp`） | `--grasp --part drop-elsewhere --next-pose kitchen_table_0_0` |
| `ground` | 某房间的地面阶段 | `--no-grasp --part ground --room bedroom` |
| `room` | 某房间：桌子阶段 + 地面阶段 | `--grasp --part room --room bedroom` |
| `full` | 完整多房间任务 | `--enter-room bedroom --grasp` |

建议顺序（从小到大，遇到第一个失败就停下）：

1. 现场准备：Map Studio 标好 `<room>_table_<n>_<m>`、`<room>_ground_<n>_<m>`
   和每个 profile 的 destination，重建 navigation_bringup。
2. 编辑对应地图的 `enter_profiles`，运行
   `ros2 run behavior_tree hangzhougrasp --map <map> --enter-room <room> --grasp --print-tree`，
   确认路线点位顺序；名称或 profile 写错会清晰报错退出。
3. 启动 `yolo26_sam_node`，然后依次：
   `--part scan-announce`（不动）→ `--part scan-announce --at <pose>` →
   `--no-grasp --part ground --room <r>` → `--grasp --part scan-grasp` →
   `--grasp --part visit-table --room <r> --table 0` → `--grasp --part deliver` →
   `--grasp --part drop-elsewhere --next-pose <pose>` →
   `--grasp --part room --room <r>` → 完整运行。

## 与现有任务保持一致的约定

- `config.py` 管理配置转换和 `KEY_*`；`constants.json` 管理现场数据。
  配置从模块路径读取，支持安装后的包，不依赖工作目录。
- `hangzhougrasp.py` 只做组合，使用 `createConstantWriter`、`createScanAnnounce`、
  `createVisitTable`、`createDeliver`、`createDropElsewhere`、`createRoom` 等工厂；
  `parts.py` 的 `createPart` 复用同一批工厂。
- 根与阶段均使用 `Sequence(memory=True)`；常量写入使用
  `Parallel(SuccessOnAll()) + BtNode_WriteToBlackboard`。
- 原语复用 TemplateNodes：GotoAction、ScanForYolo26、AnnounceDetections；
  TK27 机械臂节点在本目录 `tk27_nodes.py`（参照
  `HomeServiceHangzhou.ros_nodes.Tk27SavedPose`），mock 模式分别复用
  `BtNode_Grasp`、`BtNode_MoveArmSingle`、`BtNode_GripperAction` 的 mock 配置。
  共享的 `TemplateNodes.BtNode_GripperAction`（总是报成功）不再被本任务使用，
  也未修改。
- `_gotoRetry`、`_savedPoseRetry`、`_scanYolo26Retry` 与
  PickAndPlace helper 写法一致，均使用 `Retry(num_failures=...)`。
  默认导航 3、扫描 5、机械臂 3、抓取 3、播报 2；均可在 JSON 覆盖。
  数字是允许失败的总次数，例如 3 表示最多 3 次失败，不是额外重试 3 次。
- 抓取 Retry 只包住“打开夹爪 + Pick”：每轮都重新发送打开和新的 Pick 目标；
  Pick 自己观测，所以不重复扫描。
- 抓取场景失败收尾沿用 PickAndPlace 的 Selector + cleanup Sequence。
  `scan-grasp` part 保留失败结果（Orbbec 没看到物体时为成功、机械臂不动）；
  桌子流程（visit-table）中 cleanup 成功视为该视点 miss 并继续，cleanup
  失败则整体失败。Pick 成功后立即记为持有，即使随后的收回失败。
  drop/deliver 的导航节点以目标点位命名
  （`Go to destination <name>`、`Go to drop pose <name>`）。
- 相机响应保持原始 camera frame，`transform_to_map=False`；
  导航目标才在 map frame。两次扫描都不请求 RGB/depth/mask。
- 配置键：`pick_action`、`saved_pose_action`、`gripper_action`、
  `pick_plan_effort`（low/medium/high/auto）、`arm_pose_navigating`、
  `arm_pose_table`、`gripper_open_position`、`gripper_open_tolerance`、
  `gripper_max_effort`。旧键 `arm_pos_navigating`、`arm_pos_table`、
  `arm_action`、`grasp_action` 已删除，出现即为未知键错误。
- 人工中断沿用共享 runtime 的取消行为，不会在退出后另发收回。
- 黑板键均用 `hg_` 前缀；`hg_scan_result` 是 Orbbec 检测，
  `hg_target_label`
  是 Pick 的 prompt，`hg_announcement` 是中文文本。
- CLI 使用共享 `run_tree`，500 ms tick、可视化和原有 mock 配置。
  `stop_on_terminal=True` 防止完成后重复导航/抓取，GUI 保留至 Ctrl+C。
- Mock 遵守原节点的配置。普通视觉 mock 返回空 objects，不伪造可抓取物体；
  抓取流程离线验证需注入带 RGB/depth/mask 的检测响应（见单元测试）。

## 验证

```bash
source /opt/ros/humble/setup.bash
source /home/tinker/tk25_ws/install/setup.bash
cd /home/tinker/tk25_ws/src/tk25_decision
PYTHONPATH="$PWD/src/behavior_tree:$PYTHONPATH" .venv_decision/bin/python -m pytest \
  src/behavior_tree/test/ -q -k hangzhougrasp
```

