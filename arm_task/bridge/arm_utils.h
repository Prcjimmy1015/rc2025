#pragma once
/**
 * bridge/arm_utils.h — ONNX 推理 + 警告标志识别
 */
#include "params.h"

static inline int onnxInfer(const cv::Mat &frame, const char* modelPath)
{
    static std::map<std::string, cv::dnn::Net> netCache;
    cv::dnn::Net net;
    auto it = netCache.find(modelPath);
    if (it == netCache.end())
    {
        net = cv::dnn::readNetFromONNX(modelPath);
        if (net.empty()) { std::cerr << "[DogVision] 无法加载模型: " << modelPath << std::endl; return -1; }
        netCache[modelPath] = net;
        std::cout << "[DogVision] 模型已加载: " << modelPath << std::endl;
    }
    else { net = it->second; }

    cv::Mat rgb, resized;
    cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
    cv::resize(rgb, resized, cv::Size(720, 720));
    cv::Mat blob = cv::dnn::blobFromImage(resized, 1.0/255.0, cv::Size(720, 720),
                                           cv::Scalar(), true, false);
    net.setInput(blob);
    cv::Mat output = net.forward();

    double maxVal; cv::Point maxLoc;
    cv::minMaxLoc(output.reshape(1, 1), nullptr, &maxVal, nullptr, &maxLoc);
    int classId = maxLoc.x;
    return classId;
}

static inline int dogDetectWarningMarker(cv::Mat &frame)
{
    int id = onnxInfer(frame, MODEL_3IN1_PATH);
    if (id < 0) { std::cout << "[DogVision] 警示标志失败，默认 0" << std::endl; return 0; }
    return id;
}