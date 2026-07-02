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
#include <unitree/robot/go2/vui/vui_client.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

// =============================================================================
// ONNX 模型路径（cv_Sign/Res18_5in2）
// =============================================================================
static const char* MODEL_2IN1_PATH = "cv_Sign/Res18_5in2/2in1.onnx";  // 识别标志（1号/2号）
static const char* MODEL_3IN1_PATH = "cv_Sign/Res18_5in2/3in1.onnx";  // 警示标志（触电/强氧化物/辐射）

// =============================================================================
// ONNX 推理辅助函数
// =============================================================================
static inline int onnxInfer(const cv::Mat &frame, const char* modelPath)
{
    // 加载 ONNX 模型（首次调用时加载，后续复用静态变量）
    static std::map<std::string, cv::dnn::Net> netCache;
    cv::dnn::Net net;
    auto it = netCache.find(modelPath);
    if (it == netCache.end())
    {
        net = cv::dnn::readNetFromONNX(modelPath);
        if (net.empty())
        {
            std::cerr << "[DogVision] 无法加载模型: " << modelPath << "，返回默认值" << std::endl;
            return -1;
        }
        netCache[modelPath] = net;
        std::cout << "[DogVision] 模型已加载: " << modelPath << std::endl;
    }
    else
    {
        net = it->second;
    }

    // 预处理：RGB → resize 720x720 → 归一化 → blob
    cv::Mat rgb;
    cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(720, 720));
    cv::Mat blob = cv::dnn::blobFromImage(resized, 1.0/255.0, cv::Size(720, 720),
                                           cv::Scalar(), true, false);

    // 推理
    net.setInput(blob);
    cv::Mat output = net.forward();

    // Softmax → class_id
    double maxVal;
    cv::Point maxLoc;
    cv::minMaxLoc(output.reshape(1, 1), nullptr, &maxVal, nullptr, &maxLoc);
    int classId = maxLoc.x;

    std::cout << "[DogVision] 推理完成 model=" << modelPath
              << " class_id=" << classId << " conf=" << maxVal << std::endl;
    return classId;
}

// =============================================================================
// 机器狗前视摄像头识别函数（C++ 实现，与机械臂无关）
// =============================================================================

/** 识别抓取平台正面的识别标志（1号标识或2号标识） */
static inline int dogDetectPlatformMarker(cv::Mat &frame)
{
    int id = onnxInfer(frame, MODEL_2IN1_PATH);
    if (id < 0) { std::cout << "[DogVision] 识别标志失败，返回默认值 1" << std::endl; return 1; }
    // 2in1.onnx 输出: class 0 → 1号标识, class 1 → 2号标识
    return id + 1;  // 映射到 1/2
}

/** 识别检测平台的警示标志类型（当心触电/强氧化物/辐射） */
static inline int dogDetectWarningMarker(cv::Mat &frame)
{
    int id = onnxInfer(frame, MODEL_3IN1_PATH);
    if (id < 0) { std::cout << "[DogVision] 警示标志失败，返回默认值 0" << std::endl; return 0; }
    // 3in1.onnx 输出: class 0 → 当心触电, 1 → 当心强氧化物, 2 → 当心辐射
    return id;
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
// 机器狗警示动作（C++ 端执行，基于 sport/sport_test.cpp）
// =============================================================================

/** 伸懒腰 (warning_id=0, 当心触电) */
static inline void dogActionStretch(unitree::robot::go2::SportClient &sc)
{
    std::cout << "[DogAction] 执行: 伸懒腰 (stretch)" << std::endl;
    sc.Stretch();
    sleep(4);
}

/** 打招呼 (warning_id=1, 当心强氧化物) */
static inline void dogActionWaveHello(unitree::robot::go2::SportClient &sc)
{
    std::cout << "[DogAction] 执行: 打招呼 (wave_hello)" << std::endl;
    sc.Hello();
    sleep(4);
}

/** 闪烁前灯三次 (warning_id=2, 当心辐射) */
static inline void dogActionFlashLights(unitree::robot::go2::VuiClient &vc)
{
    std::cout << "[DogAction] 执行: 闪烁前灯三次 (flash_lights)" << std::endl;
    for (int i = 0; i < 3; i++)
    {
        vc.SetBrightness(10);
        usleep(400000);
        vc.SetBrightness(0);
        usleep(400000);
    }
    vc.SetBrightness(0);
}

/** 根据 warning_id 执行对应的机器狗警示动作 */
static inline void dogDoAlertAction(unitree::robot::go2::SportClient &sc,
                                    unitree::robot::go2::VuiClient &vc,
                                    int warning_id)
{
    switch (warning_id)
    {
    case 0: dogActionStretch(sc);      break;
    case 1: dogActionWaveHello(sc);    break;
    case 2: dogActionFlashLights(vc);  break;
    default: std::cerr << "[DogAction] 未知警示标志ID: " << warning_id << std::endl; break;
    }
}

// =============================================================================
// 集成示例（在 main.cpp FSM 中插入）:
//
// // 需要在 AppRuntime 或 main 中初始化 VuiClient
// unitree::robot::go2::VuiClient vui_client;
// vui_client.SetTimeout(10.0f);
// vui_client.Init();
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
// dogDoAlertAction(sc, vui_client, wid);
//
// // 阶段3: 放置平台
// if (armCallStage3(g_marker_id)) Flag_Task = NEXT;
// =============================================================================