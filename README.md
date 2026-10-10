# 2025年Tinker决策代码
进度：Help Me Carry和Receptionist正在写的过程中


## 接口
接口使用action，接受的request内容不变，feedback和return分别为：
```
# same request as service
---
# feed back message

# what stage the task is at
int32 stage
# name of the stage (ex. "calculating grasp pos")
string stage_name
# status code, 0 for OK, 1 for device disconnected, 2 for other
int32 status
# maximum number of seconds in which the next feedback will be received
# if next feedback is not received within the limit, the behavior tree will treat this as an error
int32 delay_limit
---
...
# same return message as before
# including the following:
int32 status
string error_msg
```

**Action server应在接受到请求后三秒内发出第一次feedback**

## GPSR command bench

Generates a seeded corpus of official-grammar GPSR commands for the rcw2026 sim vocabulary and
scores it per template at increasing realism. Tier 0 = planner only (LLM, no ROS); tier 1 = the real
`gpsr-orchestrator` process with every ROS boundary mocked (`mock_config.bench.json`). Tiers 2/3
(simulation) live in tinker-sim.

    gpsr-bench gen   --seed 42 --per-template 3 --edge --out corpus-42.jsonl
    gpsr-bench tier0 --corpus corpus-42.jsonl --out gpsr_runs/bench/t0-42
    gpsr-bench tier1 --corpus corpus-42.jsonl --out gpsr_runs/bench/t1-42   # after colcon build

Read `SUMMARY.md`: rows are templates (class A/B/C = sim feasibility, see the spec), cells are
passed/total per tier. A template that passes T0 but fails T1 is an executor/BT problem; one that
fails T0 is a planner-prompt problem. Committed baselines: `GPSR/gpsr_runs/bench/`.