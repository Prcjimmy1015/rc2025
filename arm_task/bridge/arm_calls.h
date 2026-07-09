#pragma once
/**
 * bridge/arm_calls.h — 机械臂 Python 脚本桥接调用
 */
#include "arm_utils.h"

// =============================================================================
// 阶段1: 抓取平台装货 (完整流程)
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
// 阶段2: 中转平台卸货+装货 (完整流程)
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
// 阶段1/2 仅检测模式 (popen + 解析 stdout)
// =============================================================================

static inline float armStage1Detect(int marker_id, int &out_class_id,
                                     float &out_x, float &out_z, float &out_depth)
{
    std::cout << "\n[ArmBridge] ====== Stage 1 detect (marker="
              << marker_id << ") ======" << std::endl;
    std::string cmd = "sudo python3 arm_task/task_planner.py --stage 1 --mode detect --marker "
                      + std::to_string(marker_id);
    std::string output = popenRead(cmd);
    float ratio = parseRatioFromOutput(output, 0.5f);
    if (!parseGeometryFromOutput(output, out_class_id, out_x, out_z, out_depth))
        { out_class_id = -1; out_x = 0; out_z = 0; out_depth = 0; }
    std::cout << "[ArmBridge] Stage 1 detect: ratio=" << ratio
              << " class_id=" << out_class_id << std::endl;
    return ratio;
}

static inline float armStage2Detect(int &out_class_id,
                                     float &out_x, float &out_z, float &out_depth)
{
    std::cout << "\n[ArmBridge] ====== Stage 2 detect ======" << std::endl;
    std::string output = popenRead("sudo python3 arm_task/task_planner.py --stage 2 --mode detect");
    float ratio = parseRatioFromOutput(output, 0.5f);
    if (!parseGeometryFromOutput(output, out_class_id, out_x, out_z, out_depth))
        { out_class_id = -1; out_x = 0; out_z = 0; out_depth = 0; }
    std::cout << "[ArmBridge] Stage 2 detect: ratio=" << ratio
              << " class_id=" << out_class_id << std::endl;
    return ratio;
}