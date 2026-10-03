#!/usr/bin/env bash
# 编译 lingsi pybind 桥接模块（Ubuntu 20.04/22.04）
#
# 当前构建目标为 v5.0.4 Fast DDS；301 TCP 需要独立适配，见 docs/SDK_EXTERNAL.md
# 用法:
#   SDK_DIR=~/robot_sdk_client_wheel_v5.0.4_x86_64_20260817 ./build_bridge.sh
set -euo pipefail

SDK_DIR=${SDK_DIR:?用法: SDK_DIR=<SDK解压目录(含include/和lib/)> ./build_bridge.sh}
if [ ! -f "$SDK_DIR/include/high_level_base.h" ]; then
    echo "错误: $SDK_DIR/include/high_level_base.h 不存在"; exit 1
fi

HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

echo "== 1/3 依赖 =="
python3 -m pip install --user -q pybind11 || true

echo "== 2/3 CMake 编译 =="
cmake -B build -DSDK_DIR="$(realpath "$SDK_DIR")" .
cmake --build build -j"$(nproc)"

SO=$(ls build/lingsi*.so 2>/dev/null | head -1)
if [ -z "$SO" ]; then echo "错误: 未找到编译产物 lingsi*.so"; exit 1; fi
cp "$SO" "$HERE/../../"
echo "== 3/3 验证 =="
cd "$HERE/../.."
# v5.0.4 环境示例；本次发布未编译、导入或运行实际 SDK
export LD_LIBRARY_PATH="$SDK_DIR/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$SDK_DIR/config/fastdds_no_shm.xml"
python3 -c "import lingsi; print('lingsi 模块导入成功:', lingsi.__doc__)"
echo "完成。真机运行请用: ./run_real.sh（自动设置环境变量）"
