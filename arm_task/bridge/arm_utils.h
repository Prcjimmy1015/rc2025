#pragma once
/**
 * bridge/arm_utils.h — popen辅助、stdout解析、ONNX推理、视觉识别
 */
#include "params.h"

// =============================================================================
// popen 辅助
// =============================================================================
static inline std::string popenRead(const std::string &cmd)
{
    std::array<char, 256> buffer;
    std::string result;
    FILE *pipe = popen(cmd.c_str(), "r");
    if (!pipe) { std::cerr << "[popen] 无法执行: " << cmd << std::endl; return ""; }
    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr)
        result += buffer.data();
    int ret = pclose(pipe);
    std::cout << "[popen] 退出码=" << ret << " cmd=" << cmd << std::endl;
    return result;
}

// =============================================================================
// stdout 解析
// =============================================================================
static inline float parseRatioFromOutput(const std::string &output, float default_val = 0.5f)
{
    std::regex re("RATIO_RESULT=([0-9.]+)");
    std::smatch m;
    if (std::regex_search(output, m, re))
    {
        float r = std::stof(m[1].str());
        std::cout << "[parse] RATIO_RESULT=" << r << std::endl;
        return r;
    }
    std::cerr << "[parse] RATIO_RESULT 未找到，使用默认值 " << default_val << std::endl;
    return default_val;
}

static inline bool parseGeometryFromOutput(const std::string &output,
                                            int &class_id, float &x, float &z, float &depth)
{
    std::regex re("GEOMETRY=(-?\\d+),([0-9.-]+),([0-9.-]+),([0-9.-]+)");
    std::smatch m;
    if (std::regex_search(output, m, re))
    {
        class_id = std::stoi(m[1].str());
        x = std::stof(m[2].str());
        z = std::stof(m[3].str());
        depth = std::stof(m[4].str());
        std::cout << "[parse] GEOMETRY= class_id=" << class_id
                  << " x=" << x << " z=" << z << " depth=" << depth << std::endl;
        return true;
    }
    std::cerr << "[parse] GEOMETRY 未找到" << std::endl;
    return false;
}

// =============================================================================
// ONNX 推理
// =============================================================================
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
    std::cout << "[DogVision] 推理完成 model=" << modelPath
              << " class_id=" << classId << " conf=" << maxVal << std::endl;
    return classId;
}

// =============================================================================
// 视觉识别
// =============================================================================
static inline int dogDetectPlatformMarker(cv::Mat &frame)
{
    int id = onnxInfer(frame, MODEL_2IN1_PATH);
    if (id < 0) { std::cout << "[DogVision] 识别标志失败，默认 1" << std::endl; return 1; }
    return id + 1;  // class 0→1号, class 1→2号
}

static inline int dogDetectWarningMarker(cv::Mat &frame)
{
    int id = onnxInfer(frame, MODEL_3IN1_PATH);
    if (id < 0) { std::cout << "[DogVision] 警示标志失败，默认 0" << std::endl; return 0; }
    return id;
}