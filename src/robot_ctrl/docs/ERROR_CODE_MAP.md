# ERROR_CODE_MAP.md
- 日期: 2026-01-23
- 执行者: Codex
- 说明: robot_ctrl 行为树错误码分类对照表
- 来源: src/robot_ctrl/include/robot_ctrl/bt_plugins/error_log_queue.hpp

## 约定
- error_code 仅表示错误类别，不承诺具体子类型；error_msg 必须包含节点前缀与上下文。
- 适配 RobotState.msg 的 error_id / error_msg 字段，用于前端统一解析。

## 分类对照表

| 编号 | 类别 | 说明 | 典型触发场景（示例 error_msg） |
| --- | --- | --- | --- |
| 1000 | kInputMissing | 输入缺失/黑板读取失败 | `Nav2Pose: Goal is not set` |
| 1100 | kServiceUnavailable | 服务/动作服务不可用 | `SetIcpActive: Service [/icp_active] not available` |
| 1200 | kActionFailure | 动作/调用失败（非超时） | `DockRobotActionBT: Send goal failed` |
| 1300 | kTimeout | 超时类错误 | `CancelNavToPose: cancel request timed out or failed` |
| 1400 | kDataMissing | 数据缺失/无有效数据 | `IsGoalReached: No /tacked_pose received` |
| 1500 | kInvalidParam | 参数非法/超出范围 | `LaunchManagerAction: Unknown command restart` |
| 1600 | kStateConflict | 状态冲突/业务互斥 | `TaskUpdate: Task is locked, ignoring new command` |
| 1700 | kRecoveryFailed | 恢复/重试失败 | `RecoveryNode: Max retries (3) exhausted` |
| 1999 | kUnknown | 未分类错误 | `IsTimeForAction: Failed to get localtime` |

## 维护建议
- 新增错误类别时先在 error_log_queue.hpp 增加常量，再补齐本表。
- error_msg 统一使用 `节点名: 描述` 形式，方便前端定位节点来源。
