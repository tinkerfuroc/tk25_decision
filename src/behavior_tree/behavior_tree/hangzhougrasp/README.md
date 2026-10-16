# hangzhougrasp

与 `HRI/`、`PickAndPlace/` 并列的任务模块，入口：
`ros2 run behavior_tree hangzhougrasp`。

## 决策树

不抓取（默认 `--no-grasp`）：

```text
Write constants
→ Navigate to grasp2 [Retry]
→ YOLO-World scan coke [Retry]
→ 中文 announce [Retry]
→ Navigate to point_area [Retry]
```

抓取（`--grasp`）：

```text
Write constants
→ Arm to navigation pose [Retry]
→ Navigate to grasp2 [Retry]
→ Arm to table scan pose [Retry]
→ RealSense YOLO-World scan coke [Retry]
→ 中文 announce [Retry]
→ Select nearest detected object
→ [重新扫描目标 label → 选取最近实例及对应 mask → Grasp] [Retry]
→ Retract arm with object [Retry]
→ Navigate to point_area [Retry]
```

`--grasp` 是**构建参数**：关闭时不构建任何机械臂、抓取、收回节点，
也不等待这些 action server。开启时始终使用 RealSense；未开启时由
`scan_camera` 选择相机（默认 Orbbec）。

抓取分支的 `Selector(memory=True)` 在扫描/播报/抓取失败后执行收回。
收回后保留 FAILURE，不继续去第二个点；收回失败同样停止。
正常抓取成功也必须收回成功后才能导航。
不自动打开夹爪或放下物体，抵达第二个点后结束。

## 配置与启动

编辑同目录 `constants.json`，或通过 `--config /absolute/path/run.json`
覆盖字段。外部 JSON 只需提供想覆盖的字段。

默认从 Map Studio 的 `8902_2_edit.poses.yaml` 按名称读取点位：
`pose_scan="grasp2"`，`pose_destination="point_area"`。
配置如下，可直接启动：

```json
{
  "map_name": "8902_2_edit",
  "pose_scan": "grasp2",
  "pose_destination": "point_area",
  "prompt": "coke",
  "label_map": {"coke": "可乐"},
  "detection_service": "object_detection_yolo_world"
}
```

点位文件的 yaw 单位是弧度；通过 `map_studio.poses.load_poses` 转为
map frame 的 PoseStamped。默认读取安装的 navigation_bringup/maps，
更新源点位后应重新构建 navigation_bringup；也可用
`MAP_STUDIO_MAPS_DIR` 指向源 maps 目录。
仍支持 HRI / PickAndPlace 格式的 point/orientation 字典覆盖 pose；
也可通过 CLI 指定两个点：
`--scan-pose X Y YAW_DEG --destination-pose X Y YAW_DEG`，
位置单位为米，朝向为度，frame 为 `map`。CLI 覆盖 JSON。

```bash
cd /home/tinker/tk25_ws
./tkbuild tk25_decision --packages-select behavior_tree
./tkbuild tk26_navigation --packages-select navigation_bringup
./tkbuild tk26_vision --packages-select object_detection_generalist
source install/setup.bash

# 独立终端：运行视觉服务（相机、导航和播报服务按原有 bringup 启动）
ros2 run object_detection_generalist generalist_node --ros-args \
  -r __node:=hangzhougrasp_yolo_world \
  --params-file "$(ros2 pkg prefix object_detection_generalist)/share/object_detection_generalist/config/yolo_world.yaml"

# 检测与播报，不构建抓取行为
ros2 run behavior_tree hangzhougrasp --no-grasp

# 增加抓取、收回，再导航；还需要 start_grasp 和 joint_move_action
ros2 run behavior_tree hangzhougrasp --grasp

# 只打印树，不 setup，不连接服务，不发运动指令
ros2 run behavior_tree hangzhougrasp --grasp --print-tree
```

Python 工厂：

```python
from behavior_tree.hangzhougrasp.config import load_config
from behavior_tree.hangzhougrasp.hangzhougrasp import createHangzhouGrasp

config = load_config()
root = createHangzhouGrasp(config=config, enable_grasp=True)
```

`label_map` 就是传给 `BtNode_AnnounceDetections` 的 dictionary。
当前 `prompt="coke"`，检测到后播报“我看到了可乐。”。
YOLO-World 使用文本词表；`prompt=""` 时本任务使用 dictionary 的全部英文键，
非空时使用指定 label（多类可用 `cup . bottle`）；空 prompt 且空字典会拒绝启动。
服务配置关闭 `enable_vlm`，请求关闭 `use_vlm_sam_fallback` 和
`force_vlm_sam`，因此 `coke` 走 YOLO-World + MobileSAM。
该独立 endpoint 不改变其他 generalist 服务的配置；
仍复用 generalist 路由（若改成 COCO 原生类别，会先尝试预训练 YOLO）。
不需要自训 YOLO26 checkpoint，使用已有通用 YOLO-World 权重。
字典只控制中文播报，不控制抓取类别；抓取类别由 prompt 和检测排序决定。
按相机距离最近的目标优先，每次抓一个；重新检测时选择同类的最近实例，
并不保证跨帧实例跟踪。

## 与现有任务保持一致的约定

- `config.py` 管理配置转换和 `KEY_*`；`constants.json` 管理现场数据。
  配置从模块路径读取，支持安装后的包，不依赖工作目录。
- `hangzhougrasp.py` 只做组合，使用 `createConstantWriter`、
  `createNavigateToScan`、`createScanAndAnnounce`、`createGrasp` 等工厂。
- 根与阶段均使用 `Sequence(memory=True)`；常量写入使用
  `Parallel(SuccessOnAll()) + BtNode_WriteToBlackboard`。
- 原语直接复用 TemplateNodes：GotoAction、MoveArmSingle、Grasp、
  ScanForGeneralist、AnnounceDetections。
- `_gotoRetry`、`_moveArmRetry`、`_scanForYoloWorldRetry` 与
  PickAndPlace helper 写法一致，均使用 `Retry(num_failures=...)`。
  默认导航 3、扫描 5、机械臂 3、抓取 3、播报 2；均可在 JSON 覆盖。
  数字是允许失败的总次数，例如 3 表示最多 3 次失败，不是额外重试 3 次。
- 抓取 Retry 包住“重新扫描 + 选目标 + 抓取”，因此上一次抓取失败后不会
  重用之前的 RGB/depth/mask。内部扫描 Retry 与抓取 Retry 是嵌套预算。
- 抓取场景失败收尾沿用 PickAndPlace 的 Selector + cleanup Sequence。
  本任务保留失败结果，避免把 cleanup 成功当作抓取成功后继续运输。
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
  src/behavior_tree/test/test_hangzhougrasp.py -q
```

