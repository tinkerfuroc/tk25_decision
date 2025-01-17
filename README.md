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