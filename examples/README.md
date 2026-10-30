# 离线运行证据

这些文件由本轮本机内存模拟实际运行生成，不是实车日志。

| 文件 | 演示 |
|---|---|
| dry_run_prepared.json | 未确认，在 PREPARE 返回 |
| dry_run_confirmed.json | 已确认但未 dispatch，在 CONFIRM 返回 |
| dry_run_succeeded.json | 内存目标单次发布，模拟关联状态 SUCCEEDED |
| dry_run_preempted.json | 模拟位移达到保护阈值，cancel 后观察 PREEMPTED |
| self_check.txt | 9 项自检输出，含实际调用发布函数拦截 cmd_vel |

日志保留生成时的时间和原 log_path；复制到 examples 仅作展示，不是可恢复的任务凭据。
复现请使用 README 的 dry-run 命令；真实日志继续写入被 Git 忽略的 logs 目录。
