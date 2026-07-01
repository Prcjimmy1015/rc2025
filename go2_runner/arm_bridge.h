#pragma once
/**
 * arm_bridge.h — C++ 调用 Python 机械臂任务脚本的桥接（3阶段）
 *
 * 每个阶段函数通过 popen 执行 Python 脚本并等待返回。
 * 脚本路径：arm_task/task_planner.py
 *
 * 识别标志和警示标志的识别完全由 C++ 端机器狗前视摄像头完成，
 * 机械臂只负责抓取/卸载动作。
 * 阶段1结束后至阶段3前，机械臂始终保持抓取行走姿态（抓手28°载货）。
 */

#include <string>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <array>
#include <memory>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <opencv2/opencv.hpp>

// =============================================================================
// 工具函数
// =============================================================================
static inline std::string execCommand(const std::string &cmd)
{
    std::array<char, 256> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(
        popen(cmd.c_str(), "r"), pclose);
    if (!pipe) { std::cerr << "[ArmBridge] popen() failed: " << cmd << std::endl; return ""; }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
        result += buffer.data();
    return result;
}

// =============================================================================
// 机器狗前视摄像头识别函数（C++ 实现，与机械臂无关）
// =============================================================================

/** 识别抓取平台正面的识别标志（1号标识或2号标识） */
static inline int dogDetectPlatformMarker(cv::Mat &frame)
{
    // TODO: 实现识别标志检测
    std::cout << "[DogVision] detect_platform_marker() 未实现，返回默认值 1" << std::endl;
    return 1;
}

/** 识别检测平台的警示标志类型（当心触电/强氧化物/辐射） */
static inline int dogDetectWarningMarker(cv::Mat &frame)
{
    // TODO: 实现警示标志检测
    std::cout << "[DogVision] detect_warning_marker() 未实现，返回默认值 0" << std::endl;
    return 0;
}

// =============================================================================
// 阶段1: 抓取平台装货
// =============================================================================
static inline bool armCallStage1(int marker_id)
{
    std::cout << "\n[ArmBridge] ====== Stage 1: 抓取平台装货 (marker=" 
              << marker_id << ") ======" << std::endl;
    std::string cmd = "sudo python3 arm_task/task_planner.py --stage 1 --marker "
                      + std::to_string(marker_id);
    int ret = std::system(cmd.c_str());
    if (ret == 0) { std::cout << "[ArmBridge] Stage 1 成功" << std::endl; return true; }
    std::cerr << "[ArmBridge] Stage 1 失败 (exit=" << ret << ")" << std::endl;
    return false;
}

// =============================================================================
// 阶段2: 中转平台卸货 + 抓取场地物资
// =============================================================================
static inline bool armCallStage2()
{
    std::cout << "\n[ArmBridge] ====== Stage 2: 中转平台卸货+装货 ======" << std::endl;
    int ret = std::system("sudo python3 arm_task/task_planner.py --stage 2");
    if (ret == 0) { std::cout << "[ArmBridge] Stage 2 成功" << std::endl; return true; }
    std::cerr << "[ArmBridge] Stage 2 失败 (exit=" << ret << ")" << std::endl;
    return false;
}

// =============================================================================
// 阶段3: 放置平台卸货
// =============================================================================
static inline bool armCallStage3(int target_platform)
{
    std::cout << "\n[ArmBridge] ====== Stage 3: 放置平台卸货 (平台" 
              << target_platform << ") ======" << std::endl;
    std::string cmd = "sudo python3 arm_task/task_planner.py --stage 3 --target "
                      + std::to_string(target_platform);
    int ret = std::system(cmd.c_str());
    if (ret == 0) { std::cout << "[ArmBridge] Stage 3 成功" << std::endl; return true; }
    std::cerr << "[ArmBridge] Stage 3 失败 (exit=" << ret << ")" << std::endl;
    return false;
}

// =============================================================================
// 机器狗警示动作（C++ 端执行，在检测点用 dogDetectWarningMarker 识别后调用）
// =============================================================================

static inline void dogActionStretch(unitree::robot::go2::SportClient &sc)
{ std::cout << "[DogAction] 伸懒腰 (stretch)" << std::endl; /* TODO */ }

static inline void dogActionWaveHello(unitree::robot::go2::SportClient &sc)
{ std::cout << "[DogAction] 打招呼 (wave_hello)" << std::endl; /* TODO */ }

static inline void dogActionFlashLights(unitree::robot::go2::SportClient &sc)
{ std::cout << "[DogAction] 闪烁前灯三次 (flash_lights)" << std::endl; /* TODO */ }

static inline void dogDoAlertAction(unitree::robot::go2::SportClient &sc, int warning_id)
{
    switch (warning_id)
    {
    case 0: dogActionStretch(sc);      break;
    case 1: dogActionWaveHello(sc);    break;
    case 2: dogActionFlashLights(sc);  break;
    default: std::cerr << "[DogAction] 未知警示标志ID: " << warning_id << std::endl; break;
    }
}

// =============================================================================
// 集成示例（在 main.cpp FSM 中插入）:
//
// static int g_marker_id = -1;
//
// // 阶段1: 提前用机器狗摄像头识别 → 传入机械臂
// g_marker_id = dogDetectPlatformMarker(frame);
// if (armCallStage1(g_marker_id)) Flag_Task = NEXT;
//
// // 阶段2: 中转平台
// if (armCallStage2()) Flag_Task = NEXT;
//
// // 检测点 (C++ 独立，无需 Python):
// int wid = dogDetectWarningMarker(frame);
// dogDoAlertAction(sc, wid);
//
// // 阶段3: 放置平台
// if (armCallStage3(g_marker_id)) Flag_Task = NEXT;
// =============================================================================