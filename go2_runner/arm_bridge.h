#pragma once
/**
 * arm_bridge.h — C++ 调用 Python 机械臂任务脚本的桥接
 *
 * 每个阶段函数通过 popen 执行 Python 脚本并等待返回。
 * 脚本路径：arm_task/task_planner.py（相对于工作目录或源码目录）
 */

#include <string>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <array>
#include <memory>

#include <unitree/robot/go2/sport/sport_client.hpp>

// =============================================================================
// 工具函数：执行 shell 命令并捕获 stdout 输出
// =============================================================================
static inline std::string execCommand(const std::string &cmd)
{
    std::array<char, 256> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(
        popen(cmd.c_str(), "r"), pclose);
    if (!pipe)
    {
        std::cerr << "[ArmBridge] popen() failed for: " << cmd << std::endl;
        return "";
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }
    return result;
}

// =============================================================================
// 阶段1: 抓取平台装货 + 识别标志
// 返回: 识别标志 ID (1 或 2)，失败返回 -1
//
// 在 C++ FSM 中的调用时机：
//   四足机器人到达抓取平台并停稳后，
//   在跳转到下一个 case 之前调用。
//   例如在 case3 phase1 结束、或新增 case 中。
// =============================================================================
static inline int armCallStage1()
{
    std::cout << "\n[ArmBridge] ====== Stage 1: 抓取平台装货 ======" << std::endl;
    std::string output = execCommand("sudo python3 arm_task/task_planner.py --stage 1");
    std::cout << output << std::endl;

    // 解析 MARKER_RESULT= 行
    size_t pos = output.find("MARKER_RESULT=");
    if (pos != std::string::npos)
    {
        int marker = std::atoi(output.c_str() + pos + 14);
        std::cout << "[ArmBridge] Stage 1 完成，marker_id=" << marker << std::endl;
        return marker;
    }
    std::cerr << "[ArmBridge] Stage 1 失败：未找到 MARKER_RESULT" << std::endl;
    return -1;
}

// =============================================================================
// 阶段2: 中转平台卸货 + 抓取场地物资
// 返回: true 成功
//
// 在 C++ FSM 中的调用时机：
//   阶段1完成后，行走至中转平台并停稳后调用。
// =============================================================================
static inline bool armCallStage2()
{
    std::cout << "\n[ArmBridge] ====== Stage 2: 中转平台卸货+装货 ======" << std::endl;
    int ret = std::system("sudo python3 arm_task/task_planner.py --stage 2");
    if (ret == 0)
    {
        std::cout << "[ArmBridge] Stage 2 成功" << std::endl;
        return true;
    }
    std::cerr << "[ArmBridge] Stage 2 失败 (exit=" << ret << ")" << std::endl;
    return false;
}

// =============================================================================
// 阶段3: 检测点 — 警示标志检测（机械臂拍照识别，回传参数）
// 返回: 警示标志 ID (0/1/2)，-1 失败
//
// 在 C++ FSM 中的调用时机：
//   阶段2完成后，行走至检测点并停稳后调用。
//   要求：四足机器人垂直投影完全覆盖检测点。
//   识别完成后，C++ 端根据返回的 warning_id 执行机器狗动作。
// =============================================================================
static inline int armCallStage3()
{
    std::cout << "\n[ArmBridge] ====== Stage 3: 检测点警示标志识别 ======" << std::endl;
    std::string output = execCommand("sudo python3 arm_task/task_planner.py --stage 3");
    std::cout << output << std::endl;

    // 解析 WARNING_ID= 行
    size_t pos = output.find("WARNING_ID=");
    if (pos != std::string::npos)
    {
        int warning_id = std::atoi(output.c_str() + pos + 11);
        std::cout << "[ArmBridge] Stage 3 完成，warning_id=" << warning_id << std::endl;
        return warning_id;
    }
    std::cerr << "[ArmBridge] Stage 3 失败：未找到 WARNING_ID" << std::endl;
    return -1;
}

// =============================================================================
// 机器狗警示动作（由 C++ 端执行，机械臂只负责识别）
// 这些函数在 stage3 识别完成后根据 warning_id 调用。
// =============================================================================

/** 警示动作: 伸懒腰 (warning_id=0, 当心触电) */
static inline void dogActionStretch(unitree::robot::go2::SportClient &sc)
{
    std::cout << "[DogAction] 执行: 伸懒腰 (stretch)" << std::endl;
    // TODO: 实现机器狗伸懒腰动作
    // 可参考：sc.Stretch() 或通过 Move/Euler 组合实现
}

/** 警示动作: 打招呼 (warning_id=1, 当心强氧化物) */
static inline void dogActionWaveHello(unitree::robot::go2::SportClient &sc)
{
    std::cout << "[DogAction] 执行: 打招呼 (wave_hello)" << std::endl;
    // TODO: 实现机器狗打招呼动作
    // 可参考：sc.Hello() 或前腿抬起摆动
}

/** 警示动作: 闪烁前灯三次 (warning_id=2, 当心辐射) */
static inline void dogActionFlashLights(unitree::robot::go2::SportClient &sc)
{
    std::cout << "[DogAction] 执行: 闪烁前灯三次 (flash_lights)" << std::endl;
    // TODO: 实现机器狗前灯闪烁
    // 可参考：sc.LedControl() 或系统灯光接口
}

/** 根据 warning_id 执行对应的机器狗警示动作 */
static inline void dogDoAlertAction(unitree::robot::go2::SportClient &sc, int warning_id)
{
    switch (warning_id)
    {
    case 0: dogActionStretch(sc);      break;
    case 1: dogActionWaveHello(sc);    break;
    case 2: dogActionFlashLights(sc);  break;
    default:
        std::cerr << "[DogAction] 未知警示标志ID: " << warning_id << std::endl;
        break;
    }
}

// =============================================================================
// 阶段4: 放置平台卸货
// 参数: target_platform — 1=一号放置平台, 2=二号放置平台
// 返回: true 成功
//
// 在 C++ FSM 中的调用时机：
//   阶段3完成后，根据阶段1识别到的 marker_id，
//   行走至对应放置平台（1号或2号）并停稳后调用。
//   target_platform 由阶段1返回的 marker_id 决定。
// =============================================================================
static inline bool armCallStage4(int target_platform)
{
    std::cout << "\n[ArmBridge] ====== Stage 4: 放置平台卸货 (平台" 
              << target_platform << ") ======" << std::endl;
    std::string cmd = "sudo python3 arm_task/task_planner.py --stage 4 --target "
                      + std::to_string(target_platform);
    int ret = std::system(cmd.c_str());
    if (ret == 0)
    {
        std::cout << "[ArmBridge] Stage 4 成功" << std::endl;
        return true;
    }
    std::cerr << "[ArmBridge] Stage 4 失败 (exit=" << ret << ")" << std::endl;
    return false;
}

// =============================================================================
// 集成示例（在 main.cpp 的 FSM 中插入）:
//
// // --- 全局变量: 记录阶段1返回的识别标志 ---
// static int g_platform_marker_id = -1;  // 1或2，-1表示未识别
//
// // --- 在到达抓取平台后（如 case3 phase1 结束后）---
// case ARM_PICKUP_1: {  // 或与现有 case 合并
//     sc.StopMove();
//     g_platform_marker_id = armCallStage1();
//     if (g_platform_marker_id > 0)
//         Flag_Task = NEXT_CASE;  // 继续行走至中转平台
//     else
//         Flag_Task = ERROR_CASE;
//     break;
// }
//
// // --- 在到达中转平台后 ---
// case ARM_TRANSIT: {
//     sc.StopMove();
//     if (armCallStage2())
//         Flag_Task = NEXT_CASE;  // 继续行走至检测点
//     else
//         Flag_Task = ERROR_CASE;
//     break;
// }
//
// // --- 在到达检测点后 ---
// case ARM_CHECKPOINT: {
//     sc.StopMove();
//     if (armCallStage3())
//         Flag_Task = NEXT_CASE;  // 继续行走至放置平台
//     else
//         Flag_Task = ERROR_CASE;
//     break;
// }
//
// // --- 在到达放置平台后 ---
// case ARM_PLACE: {
//     sc.StopMove();
//     if (armCallStage4(g_platform_marker_id))
//         Flag_Task = NEXT_CASE;  // 继续行走
//     else
//         Flag_Task = ERROR_CASE;
//     break;
// }
// =============================================================================