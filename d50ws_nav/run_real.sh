#!/usr/bin/env bash
# 外部 SDK 启动模板；未经过真机验收，禁止把离线 PASS 当成现场放行。
# 当前桥接仅对应 v5.0.4 Fast DDS；301 TCP 请先完成独立适配。
set -euo pipefail
SDK_DIR=${SDK_DIR:?请在私有环境中显式配置外部 SDK 路径}
if [ ! -d "$SDK_DIR/lib" ]; then
    echo "SDK_DIR 必须包含匹配版本的 lib 目录"; exit 1
fi
export LD_LIBRARY_PATH="$SDK_DIR/lib:$SDK_DIR/third/fastdds/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=${ROBOT_ROS_DOMAIN_ID:?请显式配置现场 ROS_DOMAIN_ID}
export FASTRTPS_DEFAULT_PROFILES_FILE=${ROBOT_DDS_PROFILE:?请显式配置现场 DDS 配置文件}
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"
echo "启动前须独立完成现场网络、实体急停、后方覆盖及停稳确认验收。"
python3 tools/teleop.py "$@"
