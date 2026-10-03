# 任务与证据协议

沿用 `HIT-WXM/researchops-orchestrator`；本仓库是工程资产快照。公开仓库不复制内部原始状态、账本、任务记录，也不另建协调总线。

接手现有项目时先读合法可访问的 `researchops/state/STATE.json` 与 reducer 生成的项目入口，核对任务版本、在途工作与源哈希。任务/事件/结果按官方不可变记录契约进入 `researchops/outbox/{gpt,codex,human}`；只有官方 reducer 写 STATE、LEDGER 和项目 INDEX。使用官方 validator 检查一致性。

本次公开验证说明仅记录离线工程证据，不代表硬件、安全或现场验收 PASS。私有状态继续保存在原项目，不应把此快照初始化成第二套任务系统。
