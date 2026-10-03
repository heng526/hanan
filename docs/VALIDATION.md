# 离线验证记录

验证日期：2026-10-03。验证对象为本仓库的脱敏公开副本，Python 3.12。使用既有环境，未安装软件、导入厂商库、打开相机或连接设备。

| 项目 | 结果 | 证据范围 |
|---|---|---|
| `python -B -m pytest -p no:cacheprovider -q` | 106 passed，66.07 秒 | 原有离线回归；SDK 方法契约检查未使用外部厂商头文件 |
| `python -B tools/offline_l2l3_replay.py --output ../reports/offline/replay.json` | 17 场景：9 正常、8 异常 | 正常链全部 SUCCESS；异常链保留预期 FAIL/ABORTED，包含非零粗对位角 |
| 匿名空白预检模板 | MISSING，退出码 1 | 未填现场资料不能判为就绪；现场安全仍 UNVERIFIED |
| 公开内容检查 | 自有文本源码及脱敏说明 | 无 SDK/二进制/模型/实测数据；所有文件低于 1 MiB，总体低于 20 MiB |

硬件动作、三维感知精度、后方净空、转向扫掠区、取消后的停稳和传感器/通信失效处理均未经过现场验证。测试中的安全放行来自合成 Mock，不构成实际操作授权。

逐文件大小和 SHA-256 见 [发布清单](release_manifest.json)。清单不列自身哈希；实际快照由 Git commit 标识。
