# hangzhougrasp

与 `HRI/`、`PickAndPlace/` 并列的任务模块，入口：
`ros2 run behavior_tree hangzhougrasp`。多房间任务：每个房间先逐个桌子
扫描并尝试抓取，再扫地面；房间顺序由 enterRoom profile 决定。

## 决策树

两种模式都先写常量，再按 profile 的房间顺序依次执行 `Room <name>`：

```text
Write constants
→ [--grasp] Arm to navigation pose
→ Room A: Tables phase → Ground phase
→ Room B: Tables phase → Ground phase
→ ...
```

不抓取（默认 `--no-grasp`）：桌子阶段对每个桌子的每个视点依次
`Go to <pose>` → `Scan or miss`（Orbbec + yolo26_sam 扫描，中文播报；
没检测到则跳过，不算失败）。不构建任何机械臂、抓取、夹爪节点。

抓取（`--grasp`）：桌子阶段对每个桌子：

```text
Visit table:  对每个视点（升序 m）
  Skip or visit：已持有物体则跳过剩余视点
    Go to <table 视点>
    Grasp or miss:
      Arm to table scan pose → RealSense 扫描 + 中文播报
      → 选最近目标 → 重新扫描并抓取 → 抓取一成功立即记为持有 → 收回机械臂
      任一步失败：收回机械臂；收回成功则该视点算 miss（已持有则后续视点
      直接跳过），收回本身失败则整个任务失败
Deliver if holding:  持有物体才执行（规则见“放置规则”）
```

所有视点都没抓到则跳过这个桌子（记录，不算失败）。
地面阶段：按 `(n, m)` 升序逐个 `Go to <ground pose>` → `Scan or miss`，
只扫描播报，不抓取、不放置。

`--grasp` 是**构建参数**：关闭时不构建机械臂、抓取、收回、夹爪节点，
也不等待这些 action server。导航失败会让任务失败；扫描/抓取 miss 不会，
但失败后的收回机械臂（cleanup）若重试后仍失败，任务失败并停止，
不会带着伸出的机械臂继续导航。
抓取分支用 `Selector(memory=True)`：失败后先收回机械臂再换视点。
播报（TTS）重试后仍失败只记一条 ERROR 日志（`Log announce failure`），
不算 miss：照常继续抓取和后续路线。

## 点位命名

Map Studio 点位名为 `<room>_<furniture>_<n>_<m>`，例如
`bedroom_table_0_0`、`kitchen_ground_1_2`：

- `furniture` 只能是 `table` 或 `ground`；
- `n` 是家具编号，`m` 是该家具的视点编号；
- `room` 只能是小写字母和数字、以字母开头（写 `livingroom`，不是
  `living_room` 或 `LivingRoom`）；profile 的 `rooms` 里写了不合规的房间名
  会启动即报错。

顺序：房间内桌子按 `n` 升序，一个桌子的视点按 `m` 升序，地面点按
`(n, m)` 升序。不符合命名的点位不进入索引：以 profile 中某个房间名加下划线开头
（如 `bedroom_tabel_1_0`）的错误命名会启动即报错并列出点位名；其他不符合
命名的点位只记一条 warning。房间在 profile 里却没有任何
桌子/地面点也会启动即报错；`rooms` 中重复的房间名同样报错。destination 点位直接按名称从原始点位表读取，
不要求符合上面的命名。

## enter_profiles 与 enter_room

`constants.json`（或 `--config` 的 JSON）中：

```json
{
  "map_name": "8902_2_edit",
  "enter_room": "bedroom",
  "enter_profiles": {
    "bedroom":    {"rooms": ["bedroom", "livingroom", "kitchen"], "destination": "point_area"},
    "livingroom": {"rooms": ["livingroom", "kitchen", "bedroom"], "destination": "point_area"}
  }
}
```

- 每个 profile 给出覆盖房间的顺序 `rooms`（必须包含自身）和本次的
  `destination` 点位名。profile 不是对象、缺 `destination` 都是启动错误。
- `--enter-room NAME` 覆盖 JSON 的 `enter_room`；未知名称是启动错误。
- 不做任何回退：缺 `enter_profiles`、未知 `enter_room`、点位名找不到、
  点位文件不存在，都会在 setup 与任何运动之前以清晰信息退出
  （`--print-tree` 同样需要有效 profile）。
- 外部 JSON 只需提供要覆盖的字段；未知键会被拒绝。

## 放置规则

抓到物体后（`Deliver if holding`）：

- **在进入房间（enter room）抓到**：机械臂回导航姿态，导航到 profile 的
  `destination`，打开夹爪放下，然后继续剩余的桌子、地面点和房间。
- **在其他房间抓到**：到不了真正的放置位置。机械臂回导航姿态，导航到
  **下一个扫描点**（路线中该桌子之后的下一个点：下一个桌子的第一个视点，
  否则本房间第一个地面点，否则下一个房间的第一个点），播报
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
checkpoint 的标签完全一致，`label_map` 的键同理（当前 `coke` → 可乐）；
`prompt=""` 时使用 `label_map` 的全部键，两者都空则拒绝启动。
本任务的播报必须是普通话：`prompt` 里的每个标签都必须在 `label_map` 中有
中文名，否则启动即报错并列出缺少的标签。
扫描无检测结果会失败，所以在各阶段被包在 `Scan or miss` 里。
`--grasp` 的桌面扫描使用 RealSense（含 RGB/depth/mask），其余扫描使用 Orbbec。
`label_map` 只控制中文播报，不控制抓取类别；每次抓最近的目标，重新检测时
选同类的最近实例，不保证跨帧跟踪。

## 配置与启动

```bash
cd /home/tinker/tk25_ws
./tkbuild tk25_decision --packages-select behavior_tree
./tkbuild tk26_navigation --packages-select navigation_bringup
./tkbuild tk26_vision --packages-select object_detection_generalist
source install/setup.bash

# 全流程（还需要 start_grasp 和 joint_move_action）
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

config = load_config(enter_room="bedroom")
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
2. 编辑 `enter_profiles`，运行
   `ros2 run behavior_tree hangzhougrasp --enter-room <room> --grasp --print-tree`，
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
- 原语直接复用 TemplateNodes：GotoAction、MoveArmSingle、Grasp、
  ScanForYolo26、AnnounceDetections。
- `_gotoRetry`、`_moveArmRetry`、`_scanYolo26Retry` 与
  PickAndPlace helper 写法一致，均使用 `Retry(num_failures=...)`。
  默认导航 3、扫描 5、机械臂 3、抓取 3、播报 2；均可在 JSON 覆盖。
  数字是允许失败的总次数，例如 3 表示最多 3 次失败，不是额外重试 3 次。
- 抓取 Retry 包住“重新扫描 + 选目标 + 抓取”，因此上一次抓取失败后不会
  重用之前的 RGB/depth/mask。内部扫描 Retry 与抓取 Retry 是嵌套预算。
- 抓取场景失败收尾沿用 PickAndPlace 的 Selector + cleanup Sequence。
  `scan-grasp` part 保留失败结果；桌子流程（visit-table）中 cleanup 成功
  视为该视点 miss 并继续，cleanup 失败则整体失败。抓取成功后立即记为持有，
  即使随后的收回失败。drop/deliver 的导航节点以目标点位命名
  （`Go to destination <name>`、`Go to drop pose <name>`）。
- 相机响应保持原始 camera frame，`transform_to_map=False`；
  导航目标才在 map frame。抓取只传同一目标对应的一个 mask 和 label，
  检查 RGB/depth/mask 尺寸一致。原始扫描响应不被修改。
- `arm_pos_navigating` / `arm_pos_table` 初始值取自当前
  PickAndPlace/constants.json，同样以度存储，写入黑板前转弧度。
  可在任务配置独立调整，不修改其他任务。
- 抓取使用已有 `stay=False, use_mesh=True` 行为，随后显式收回到本任务的
  导航姿态。人工中断沿用共享 runtime 的取消行为，不会在退出后另发收回。
- 黑板键均用 `hg_` 前缀；`hg_scan_result` 是初次检测，
  `hg_grasp_vision_result` 是选出的抓取包，`hg_announcement` 是中文文本。
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

