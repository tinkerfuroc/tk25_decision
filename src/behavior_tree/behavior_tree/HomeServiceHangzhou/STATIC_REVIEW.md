# Hangzhou 静态检查与恢复策略（2026-10-15）

本次仅修改本地 `tinker2_hangzhou`，没有部署到 tkmr/tkserver，没有启动实机。

后续流程调整：身份匹配已由“行为检测后、交互前”移到“接近并交互后、离开前”。
行为不再与登记编号强绑定；正面新图优先、原行为人物框兜底。最新结构与去重语义见 README。

## 对照 HRI 的具体写法

- `HRI/hri.py:createArrivalTrigger`、主人迎接/沙发导航：将导航叶节点放入有限 `Retry`，默认 5 次；不是重启整个任务。
- `HRI/hri.py` 的登记和特征提取：带记忆的 Sequence 将问答、提取、存储分阶段；失败在所在阶段处理。
- `HRI/hri.py:_armToPoseBestEffort` 和 `HRI/hri_2026.py`：区分关键动作与 best-effort 分支。杭州任务仅将辅助播报设为 best-effort；不能照搬 HRI 中忽略收臂失败的分支，否则会破坏本任务导航前必须收臂的要求。
- 保持 `Sequence/Selector(memory=True)` 和根节点 `OneShot(ON_COMPLETION)`，终态继续 tick 不重跑比赛。

## 检查发现与修改

| 阶段/问题 | 修改后的行为 |
|---|---|
| 导航 abort 一次就退出 | 每个固定点最多 5 次，失败间隔默认 1 秒，耗尽后才进入阶段恢复。 |
| canceled/unknown 被 Goto 当成成功 | 本任务适配器仅接受 ROS `SUCCEEDED`，存在 error_code 时还要求为 0；Approach 同时检查 ROS 和业务结果。 |
| 超时取消尚未完成就重发动作 | 所有本任务底盘/机械臂请求共用终态检查；收到 cancel acknowledgement 不算停止，等待结果终态。默认 10 秒仍未确认则锁止本任务后续运动，并记录事件。 |
| 机械臂和云台没有局部重试 | 各最多 3 次，绝对目标位姿不变；收臂耗尽不能进入导航。 |
| 登记重名在 Retry 外失败 | 清空本次输入、问姓名、特征提取、校验并存储放进同一登记重试；登记后播报不会触发重复存储。 |
| 临时特征提取错误导致重新询问姓名 | 特征提取局部最多 4 次，失败后再按登记重试策略重新问答。每次提取前清空旧结果。 |
| 空识别结果被转成字符串 `None` | 姓名和需求必须为非空字符串，拒绝 `(empty)` 和非字符串输入。 |
| 扶人/挥手问答没有重试 | 问答最多 3 次，仅重做提问、收听和文本校验；不重复接近、伸臂。复述仍为必需步骤，失败不计主人完成。 |
| 播报失败导致无关任务结束 | 登记确认、阶段提示、失败提示和总结允许失败并记录；姓名行为播报、指令/需求复述、垃圾播报仍影响相应完成记录。 |
| 超时录音/TTS 与下一轮语音重叠 | 音频单独等待上一操作结束；音频停止无法确认时停止后续音频操作，但不伪造播报成功，也不妨碍机械臂收回。 |
| 服务/Action 异常抛出导致进程退出 | 本任务的语音/特征和动作适配器将请求、结果异常转为节点 FAILURE，保留错误；动作停止不明时运动检查仍会阻止后续动作。 |
| VLM 请求异常、旧帧未统一处理 | 请求异常变为失败；检测最多 2 次；成功帧必须晚于云台到位且不过期、不来自未来；身份结果再次校验数量、范围和重复匹配。 |
| 服务完一个主人后复用移动前其他人坐标 | 返回扫描点后清空该帧 active observations，后续角度重新识别；Approach 每次重试在等待前一动作结束之后重新检查目标年龄。 |
| 一个房间不可达导致整段巡游结束 | 房间失败后先确认收臂，记录 `room_skipped`，继续其他房间；失败房间不标记 visited，后续巡游轮可再尝试。入场/退场失败仍不可吞掉。 |
| 垃圾播报失败也计分 | 检测只记 `litter_observed`；播报成功后才记录 `litter_announced` 并计入 litter_rooms。 |
| 恢复收臂失败时没有 mission_failed 事件 | 先记录原始任务失败，再尝试收臂；失败时明确记录 retraction_failed 并播报无法确认收回，不谎报已收臂。 |
| 原始失败被取消目标反馈覆盖 | 已结束/拒绝的动作不再发送 cancel；每次失败记录 step_failed 和原始反馈，GUI 可查看。 |
| 重试参数与接口不一致 | 新增可校验的重试/取消预算，模板 endpoint 改为 TK27 JointMove 和 go_to_approach。 |

## 配置

`config.py:RETRY_DEFAULTS` 向旧 JSON 兼容补齐以下默认项，可在现场 JSON 覆盖：

| 参数 | 默认值 |
|---|---:|
| navigation_attempts | 5 |
| approach_attempts / arm_attempts / head_attempts / dialogue_attempts | 3 |
| feature_attempts | 4 |
| perception_attempts | 2 |
| retry_delay_seconds | 1.0 |
| cancel_timeout_seconds | 10.0 |
| arm_timeout_seconds | 120.0 |

attempts 包括第一次执行，不是额外重试次数；范围 1–10。时间必须有限且为正，只有 retry_delay_seconds 允许为 0。
每次动作的原有超时、巡游截止、退场预留和比赛总时限仍然生效。因此临近比赛截止时可能未用完尝试次数就结束。
失败恢复可能超出比赛时限，目的只在停止动作和收臂，不计比赛完成。

正常离场的树状态 SUCCESS 表示流程已结束，不等于所有得分项已完成；`patrol_summary.scope_complete`、completed、visited、litter_rooms 明确记录实际完成情况。

## 验证及边界

新增 `test/test_hangzhou_portable.py` 使用真实 py_trees、任务树、状态和控制逻辑，以确定性 I/O 替代机器人边界。
ROS 适配器结果映射测试只替换其 ROS 父类导入，执行本次修改的方法；不是 ROS 通信集成测试。
Windows 无 ROS 消息、cv_bridge 等依赖，因此原有 ROS 依赖测试本轮未运行，不能据此宣称实机/仿真通过。

PowerShell：

```powershell
$env:PYTHONPATH='src/behavior_tree'
$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD='1'
.\.venv-static\Scripts\python -m pytest -q src/behavior_tree/test/test_hangzhou_portable.py
```

保留真实服务和地图配置，不用 mock 掩盖 Nav2 规划失败。重试不能修复错误定位、地图障碍或不可达目标。
视觉服务无法通过 ROS service 取消远端推理，超时仅丢弃本地结果；身份线程的晚到结果不写黑板。本次没有将身份 VLM 超时盲目并发重试。
