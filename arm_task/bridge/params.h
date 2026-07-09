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
#include <thread>
#include <chrono>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/vui/vui_client.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

// ONNX 模型路径
static const char* MODEL_3IN1_PATH = "arm_task/sign_model/3in1.onnx";

// 外部传感器引用 (来自 go2_runner/globals.h)
extern float ob_x_f;

// dogTurn90Degrees 原地转弯参数
static constexpr float  kTurn90W        = 1.2f;
static constexpr int    kTurn90Frames   = 25;