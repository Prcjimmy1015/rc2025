#pragma once
/**
 * arm_bridge.h — 机械臂任务桥接入口
 *
 * 本文件只包含 3 个任务编排函数，每个函数内部调用 bridge/ 子文件中的模块。
 * 子文件结构:
 *   bridge/params.h       — 常量 + extern 声明
 *   bridge/arm_utils.h    — popen、解析、ONNX推理、视觉识别
 *   bridge/arm_calls.h    — armCallStage1/2/3 + armStage1Detect/armStage2Detect
 *   bridge/dog_turn.h     — dogTurn90Degrees
 *   bridge/dog_align.h    — dogAlignToPlatform
 *   bridge/dog_alerts.h   — 警示动作
 */
#include "bridge/arm_calls.h"
#include "bridge/dog_turn.h"
#include "bridge/dog_align.h"
#include "bridge/dog_alerts.h"

// =============================================================================
// Task 1: 左转90→检测→比值对齐→识别标志→(抓取留空)→右转90回正
// =============================================================================
static inline bool dogTask1Execute(unitree::robot::go2::SportClient &sc,
                                    cv::VideoCapture &cap,
                                    unitree::robot::go2::VuiClient &vc,
                                    int marker_id)
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Task 1 开始 — 抓取平台" << std::endl;
    std::cout << "========================================" << std::endl;

    // Step 1: 左转 90°
    if (!dogTurn90Degrees(sc, cap, +1))
    {
        std::cerr << "[Task1] 左转 90° 失败" << std::endl;
        return false;
    }

    // Step 2: 机械臂拍照检测
    int class_id;
    float wx, wz, depth;
    float ratio = armStage1Detect(marker_id, class_id, wx, wz, depth);

    // Step 3: 显示机械臂 D435 标注图像
    cv::Mat arm_view = cv::imread(ARM_ANNOTATED_IMAGE_PATH);
    if (!arm_view.empty())
    {
        cv::imshow("Arm D435", arm_view);
        cv::waitKey(1);
    }
    else
    {
        std::cerr << "[Task1] 无法加载机械臂标注图像: " << ARM_ANNOTATED_IMAGE_PATH << std::endl;
    }

    // Step 4: 机器狗比值对齐
    if (!dogAlignToPlatform(sc, cap, ratio))
    {
        std::cerr << "[Task1] 比值对齐失败" << std::endl;
    }

    // Step 5: 识别平台正面标志 — 此时正对平台，识别最准确
    cv::Mat mark_frame;
    cap.read(mark_frame);
    if (!mark_frame.empty())
    {
        marker_id = dogDetectPlatformMarker(mark_frame);
        std::cout << "[Task1] 🔍 识别标志: " << marker_id << "号平台" << std::endl;
    }
    else
    {
        std::cerr << "[Task1] 识别标志时相机帧为空，使用传入 marker_id=" << marker_id << std::endl;
    }

    // Step 6: [TODO: 用户后期指定] 机械臂抓取动作
    // ================================================================
    // 示例:
    //   armCallStage1(marker_id);
    //   或分步:
    //   armCallStage1Grasp(class_id, wx, wz, depth);
    // ================================================================

    cv::destroyWindow("Arm D435");

    // Step 7: 右转 90° 回正
    if (!dogTurn90Degrees(sc, cap, -1))
    {
        std::cerr << "[Task1] 右转 90° 回正失败" << std::endl;
    }

    std::cout << "\n[Task1] ✅ 完成！" << std::endl;
    return true;
}

// =============================================================================
// Task 2: 右转90→检测→比值对齐→(中转留空)→左转90回正
// =============================================================================
static inline bool dogTask2Execute(unitree::robot::go2::SportClient &sc,
                                    cv::VideoCapture &cap,
                                    unitree::robot::go2::VuiClient &vc)
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Task 2 开始 — 中转平台" << std::endl;
    std::cout << "========================================" << std::endl;

    // Step 1: 右转 90°
    if (!dogTurn90Degrees(sc, cap, -1))
    {
        std::cerr << "[Task2] 右转 90° 失败" << std::endl;
        return false;
    }

    // Step 2: 机械臂拍照检测
    int class_id;
    float wx, wz, depth;
    float ratio = armStage2Detect(class_id, wx, wz, depth);

    // Step 3: 显示机械臂 D435 标注图像
    cv::Mat arm_view = cv::imread(ARM_ANNOTATED_IMAGE_PATH);
    if (!arm_view.empty())
    {
        cv::imshow("Arm D435", arm_view);
        cv::waitKey(1);
    }
    else
    {
        std::cerr << "[Task2] 无法加载机械臂标注图像: " << ARM_ANNOTATED_IMAGE_PATH << std::endl;
    }

    // Step 4: 机器狗比值对齐
    if (!dogAlignToPlatform(sc, cap, ratio))
    {
        std::cerr << "[Task2] 比值对齐失败" << std::endl;
    }

    // Step 5: [TODO: 用户后期指定] 中转平台卸货+抓取
    // ================================================================
    // 示例:
    //   armCallStage2();
    //   或分步:
    //   armCallStage2Transit(class_id, wx, wz, depth);
    // ================================================================

    cv::destroyWindow("Arm D435");

    // Step 6: 左转 90° 回正
    if (!dogTurn90Degrees(sc, cap, +1))
    {
        std::cerr << "[Task2] 左转 90° 回正失败" << std::endl;
    }

    std::cout << "\n[Task2] ✅ 完成！" << std::endl;
    return true;
}

// =============================================================================
// Task 3: 放置平台卸货 — (卸货前姿态留空)→放置平台卸货
// =============================================================================
static inline bool dogTask3Execute(unitree::robot::go2::SportClient &sc,
                                    cv::VideoCapture &cap,
                                    unitree::robot::go2::VuiClient &vc,
                                    int target_platform)
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Task 3 开始 — 放置平台卸货 (平台" << target_platform << ")" << std::endl;
    std::cout << "========================================" << std::endl;

    // [TODO: 用户后期指定] 卸货前如果需要在放置平台前调整姿态，在此添加
    // 例如: dogAlignToPlatform(sc, cap, ratio);

    bool ok = armCallStage3(target_platform);

    std::cout << "\n[Task3] " << (ok ? "✅ 完成！" : "❌ 失败") << std::endl;
    return ok;
}