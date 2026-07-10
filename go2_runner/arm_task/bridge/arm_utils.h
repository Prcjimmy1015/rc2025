#pragma once
/**
 * bridge/arm_utils.h — ONNX 推理 + 警告标志识别
 * 通过 popen 调用 Python onnx_infer.py 脚本（使用 onnxruntime），
 * 因为 C++ OpenCV 4.5.4 的 DNN 模块不支持 opset 25 的 ONNX 模型。
 */
#include "params.h"

static inline int onnxInfer(const cv::Mat &frame, const char* modelPath)
{
    // 将帧写入临时图片文件，供 Python 脚本读取
    std::string tmpImg = "/tmp/onnx_frame_" + std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())) + ".png";
    cv::imwrite(tmpImg, frame);

    // 构造命令
    std::string cmd = "python3 " + std::string(MODEL_INFER_SCRIPT) + " " + modelPath + " " + tmpImg + " 2>/dev/null";
    FILE *pipe = popen(cmd.c_str(), "r");
    if (!pipe)
    {
        std::cerr << "[DogVision] popen 失败" << std::endl;
        std::remove(tmpImg.c_str());
        return -1;
    }

    char buf[32] = {0};
    if (fgets(buf, sizeof(buf), pipe) == nullptr)
    {
        std::cerr << "[DogVision] 推理进程无输出" << std::endl;
        pclose(pipe);
        std::remove(tmpImg.c_str());
        return -1;
    }
    pclose(pipe);
    std::remove(tmpImg.c_str());

    int classId = std::atoi(buf);
    return classId;
}

static inline int dogDetectWarningMarker(cv::Mat &frame)
{
    int id = onnxInfer(frame, MODEL_3IN1_PATH);
    if (id < 0) { std::cout << "[DogVision] 警示标志失败，默认 0" << std::endl; return 0; }
    return id;
}