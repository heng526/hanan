// lingsi_pybind.cpp — 灵锶高层 SDK 的 Python 桥接层（pybind11）
//
// 用途：把 C++ 高层接口（high_level_base.h 的 Robot 抽象类）暴露为 Python 模块
// `lingsi`，供 d50ws_nav/sdk_adapter/lingsi_tcp.py 调用。
// 编译（Ubuntu 20.04/22.04，解压 SDK zip 后）：
//   pip install pybind11
//   g++ -O2 -std=c++17 -shared -fPIC \
//       -I<-sdk>/include $(python3 -m pybind11 --includes) \
//       lingsi_pybind.cpp -L<sdk>/lib -lCrobotcontroller \
//       -Wl,-rpath,'$ORIGIN/../lib' -o lingsi$(python3-config-extension-suffix)
// 注：方法签名以 M0 实际解包的 high_level_base.h 为准，发现差异只需改本文件。

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <memory>
#include "high_level_base.h"

namespace py = pybind11;

static std::shared_ptr<Robot> g_robot;   // 单实例：狗控制通道只有一条

static Robot* ensure() {
    if (!g_robot) g_robot = createQuadruped();
    return g_robot.get();
}

PYBIND11_MODULE(lingsi, m) {
    m.doc() = "LingSi wheel-legged dog high-level SDK bridge (v5.0.4; v301 requires adaptation)";

    // ---- 生命周期 ----
    m.def("init", []() { return ensure()->init(); });

    // ---- 基础动作（对应手柄：站立/趴下/阻尼/卸力） ----
    m.def("standup", []() { return ensure()->Standup(); });
    m.def("getdown", []() { return ensure()->Getdown(); });
    m.def("damp", []() { return ensure()->Damp(); });
    m.def("zero_torque", []() { return ensure()->ZeroTorque(); });
    m.def("stop_move", []() { return ensure()->StopMove(); });

    // ---- 运动控制 ----
    // Move(lm, vm, lrm)：以指定速度移动 1 秒（lm 前后, vm 左右, lrm 转向）
    m.def("move_1s", [](double lm, double vm, double lrm) {
        return ensure()->Move(lm, vm, lrm);
    });
    m.def("set_height", [](float h) { return ensure()->SetHeight(h); });
    // 以下三个在 v3.0.1 文档中被划线（不承诺开放），头文件存在，M0 真机验证：
    m.def("move_forward", [](float d) { return ensure()->MoveForward(d); });
    m.def("set_speed", [](float s) { return ensure()->SetSpeed(s); });
    m.def("rotate_right_left", [](double rad) { return ensure()->Rotate_right_and_left(rad); });
    // 模式类候选（过轨步态 M0 验证项）：
    m.def("switch_rl_mode", [](bool v) { return ensure()->SwitchToRLMode(v); });
    m.def("high_knee", [](bool v) { return ensure()->Highknee(v); });
    m.def("switch_crawl_mode", [](bool v) { return ensure()->SwitchToCrawlMode(v); });
    m.def("joint_lock", [](bool v) { return ensure()->JointLockMode(v); });

    // ---- 状态读取 ----
    m.def("get_battery", []() { return ensure()->GetRobotBatteryPercentage(); });
    m.def("get_speed", []() { return ensure()->GetSpeed(); });
    m.def("get_angle", []() { return ensure()->GetAngle(); });
    m.def("get_height", []() { return ensure()->GetHeight(); });
    m.def("get_action_status", []() { return ensure()->GetActionStatus(); });
    m.def("get_sdk_switch", []() { return ensure()->GetSdkSwitch(); });
    m.def("get_charging_state", []() { return ensure()->GetRobotChargingState(); });
    m.def("get_sdk_version", []() { return ensure()->GetSdkVersion(); });

    m.def("get_odometry", []() {
        auto o = ensure()->GetOdometry();
        py::dict d;
        d["position"] = std::vector<double>(o.position, o.position + 3);
        d["orientation"] = std::vector<double>(o.orientation, o.orientation + 4);
        d["linear"] = std::vector<double>(o.linear, o.linear + 3);
        d["angular"] = std::vector<double>(o.angular, o.angular + 3);
        return d;
    });
    m.def("get_robot_speed", []() {
        auto s = ensure()->GetRobotSpeed();
        py::dict d;
        d["x"] = s.body_x; d["y"] = s.body_y; d["z"] = s.body_z; d["ang"] = s.body_ang;
        return d;
    });
    m.def("get_joint_state", []() {
        auto j = ensure()->GetJointState();
        py::dict d;
        d["angles"] = std::vector<float>(j.joint_angles, j.joint_angles + 12);
        d["velocities"] = std::vector<float>(j.joint_velocities, j.joint_velocities + 12);
        return d;
    });
    m.def("get_imu_accel", []() {
        auto a = ensure()->GetImuLinearAcceleration();
        return std::vector<double>{a.imu_ax, a.imu_ay, a.imu_az};
    });
    // GPS：狗自带 /gps/location（NavSatFix）。非 RTK，仅作备份位置源。
    m.def("get_gps", []() {
        auto g = ensure()->GetGpsLocation();
        py::dict d;
        d["received"] = g.is_received;
        d["lat"] = g.latitude; d["lon"] = g.longitude; d["alt"] = g.altitude;
        return d;
    });
    m.def("get_all_alerts", []() {
        std::vector<py::dict> out;
        for (auto &a : ensure()->GetAllAlerts()) {
            py::dict d;
            d["key"] = a.key;
            d["code"] = a.code;
            d["key_str"] = a.key_str;
            d["timestamp"] = a.timestamp;
            out.push_back(d);
        }
        return out;
    });

    // ---- 充电（对接自主回充） ----
    m.def("start_charging", []() { return ensure()->StartRobotCharging(); });
    m.def("stop_charging", []() { return ensure()->StopRobotCharging(); });

    // ---- 地图（pro 版建图导航的文件读取） ----
    m.def("get_map_list", []() {
        std::vector<std::string> names;
        for (auto &mi : ensure()->GetRobotMapList()) names.push_back(mi.map_name);
        return names;
    });
    m.def("get_map_files", [](const std::string &name, const std::string &dir) {
        return ensure()->GetRobotMapFiles(name, dir);
    });
}
