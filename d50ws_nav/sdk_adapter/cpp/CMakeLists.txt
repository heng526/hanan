cmake_minimum_required(VERSION 3.10)
project(lingsi_bridge CXX)

# 灵锶高层 SDK 的 pybind11 桥接。
# 用法（Ubuntu，SDK 已解压）：
#   SDK_DIR=/path/to/robot_sdk_client_wheel_v5.0.4 ./build_bridge.sh
# 产物：lingsi<abi>.so（拷回 d50ws_nav/ 根目录供 python import）

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT DEFINED SDK_DIR)
    message(FATAL_ERROR "请用 -DSDK_DIR=<解压的SDK目录> 指定（需含 include/high_level_base.h 和 lib/）")
endif()

find_package(pybind11 QUIET)
if(NOT pybind11_FOUND)
    include(FetchContent)
    FetchContent_Declare(pybind11
        GIT_REPOSITORY https://gitee.com/mirrors/pybind11.git
        GIT_TAG v2.11.1)
    FetchContent_MakeAvailable(pybind11)
endif()

pybind11_add_module(lingsi lingsi_pybind.cpp)

target_include_directories(lingsi PRIVATE ${SDK_DIR}/include)
target_link_directories(lingsi PRIVATE ${SDK_DIR}/lib)
target_link_libraries(lingsi PRIVATE Crobotcontroller)

# 运行期 rpath 指向 SDK lib 目录，避免 LD_LIBRARY_PATH 依赖
set_target_properties(lingsi PROPERTIES
    BUILD_RPATH "${SDK_DIR}/lib"
    INSTALL_RPATH "${SDK_DIR}/lib")
