#pragma once
/**
 * bridge/params.h — 共用常量与外部声明
 */

#include <string>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <array>
#include <memory>
#include <sstream>
#include <regex>
#include <cmath>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/vui/vui_client.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

// =============================================================================
// ONNX 模型路径
// =============================================================================
static const char* MODEL_2IN1_PATH = "arm_task/sign_model/2in1.onnx";
static const char* MODEL_3IN1_PATH = "arm_task/sign_model/3in1.onnx";

// =============================================================================
// 标注图像保存路径
// =============================================================================
static const char* ARM_ANNOTATED_IMAGE_PATH = "arm_task/detect_output.jpg";

// =============================================================================
// 外部传感器引用 (来自 go2_runner/globals.h)
// =============================================================================
extern float ob_x_f;  // 雷达前向滤波距离 (m)

// =============================================================================
// dogAlignToPlatform 小步伐控制参数
// =============================================================================
static constexpr float kAlignVxMax       = 0.04f;
static constexpr float kAlignVyMax       = 0.03f;
static constexpr float kAlignWMax        = 0.15f;
static constexpr float kAlignRatioTol    = 0.03f;
static constexpr float kAlignPGain       = 2.5f;
static constexpr float kAlignVyRatioGain = 0.08f;
static constexpr float kAlignTargetDistM = 0.5f;
static constexpr float kAlignVxGain      = 1.5f;
static constexpr int   kAlignMaxFrames   = 600;

// =============================================================================
// dogTurn90Degrees 转弯参数
// =============================================================================
static constexpr float  kTurn90Vx        = 0.10f;
static constexpr double kTurn90R         = 0.5;
static constexpr double kTurn90Tol       = 0.08;
static constexpr int    kTurn90MaxFrames = 500;