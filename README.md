# 机器狗导航与作业框架

面向轮足机器狗的自有 Python 工程框架公开快照。提供航点跟随、感知接口、行为线程、执行回执关联、离线预检与 Mock 演示。公开材料使用合成坐标和匿名配置。

## 快速离线验证

需要 Python 3.10 或更新版本；本次验证环境为 Python 3.12。依赖见 `d50ws_nav/requirements.txt`。自行在隔离环境准备依赖后执行：

```bash
cd d50ws_nav
python -B -m pytest -p no:cacheprovider -q
python -B tools/offline_l2l3_replay.py --output ../reports/offline/replay.json
python -B tools/offline_config_preflight.py config/preflight_template.json
```

空白预检模板预期返回 MISSING（退出码 1），缺资料时不能放行；需填入本地真实材料后重新检查。离线回放覆盖 9 个正常动作与 8 个异常场景，并包含非零粗对位角。测试证据见 [验证记录](docs/VALIDATION.md)。

## 内容

| 目录 | 用途 |
|---|---|
| `d50ws_nav/common` | 地理投影与 NMEA/RTK 数据接口 |
| `d50ws_nav/sdk_adapter` | 抽象接口、Mock 及自有厂商 SDK 桥接源码 |
| `d50ws_nav/perception` | P1–P4 注入式感知接口与帧缓存 |
| `d50ws_nav/nav` | 跟随、安全监护及动作类 |
| `d50ws_nav/tools`、`tests` | 离线预检、回放和回归验证 |
| `d50ws_nav/config` | 匿名模板与合成路线 |

## 能力与待验证边界

已实现的接口可验证线程生命周期、数据新鲜度检查、逐命令关联、超时/取消请求、粗细对位 Mock 推进和异常拒绝。真实视觉/OCR/点云算法、标定、设备数据接入及上装执行器仍需外部实现方注入；本仓库不提供完整现场任务调度器。

**Mock 成功不证明实际动作完成、停止或避障安全。** 当前实际 SDK 桥接对应 v5.0.4 Fast DDS，选定的 301 TCP 尚须适配；[SDK 外部准备说明](docs/SDK_EXTERNAL.md)列出已知差异。既有真实适配层的 `speed_cmd` 是发送速度，主机时间戳与调用成功也不能证明设备状态新鲜。

前后避障、转向扫掠区、制动距离、失联及陈旧传感器时的安全停机，都需要隔离低速场地、近旁人员实体急停和实测停稳证据。源码中的速度/距离阈值均未作为现场验收值发布。

不含厂商 SDK、二进制、模型、真实录制数据、内部要求/合同或原始任务账本。第三方依赖与权利说明见 [NOTICE](NOTICE.md)。任务与证据继续使用 [唯一 ResearchOps 协议](docs/RESEARCHOPS.md)。
