#pragma once
/**
 * bridge/dog_turn.h — 机器狗 90° 原地转弯（开环时间控制）
 */
#include "params.h"
using namespace unitree::robot;

static inline bool dogTurn90Degrees(unitree::robot::go2::SportClient &sc,
                                     cv::VideoCapture &cap,
                                     int direction)
{
    const char *dir_name = (direction >= 0) ? "LEFT" : "RIGHT";
    float w_cmd = (direction >= 0) ? kTurn90W : -kTurn90W;

    std::cout << "\n[DogTurn] ====== 转弯 " << dir_name << " 90° ======" << std::endl;
    std::cout << "[DogTurn] w=" << w_cmd << " rad/s, 持续 " << kTurn90Frames << " 帧" << std::endl;

    cv::Mat dummy;
    cap.read(dummy); cap.read(dummy);  // 预热 pipeline
    sc.StaticWalk();
    for (int frame = 0; frame < kTurn90Frames; ++frame)
    {
        sc.Move(0.f, 0.f, w_cmd);
        cap.read(dummy);
        if (frame % 15 == 0)
            std::cout << "[DogTurn] 转弯中... " << frame << "/" << kTurn90Frames << std::endl;
    }
    sc.StopMove();
    std::cout << "[DogTurn] ✅ 转弯完成" << std::endl;
    return true;
}

/** 无摄像头版本转弯 — 用 usleep 替代 cap.read() 做帧间定时 */
static inline bool dogTurn90DegreesNoCam(unitree::robot::go2::SportClient &sc,
                                          int direction)
{
    const char *dir_name = (direction >= 0) ? "LEFT" : "RIGHT";
    float w_cmd = (direction >= 0) ? kTurn90W : -kTurn90W;

    std::cout << "\n[DogTurn] ====== 转弯 " << dir_name << " 90° ======" << std::endl;
    std::cout << "[DogTurn] w=" << w_cmd << " rad/s, 持续 " << kTurn90Frames << " 帧" << std::endl;

    sc.StaticWalk();
    for (int frame = 0; frame < kTurn90Frames; ++frame)
    {
        sc.Move(0.f, 0.f, w_cmd);
        usleep(33333);  // ~30fps 帧间隔
        if (frame % 15 == 0)
            std::cout << "[DogTurn] 转弯中... " << frame << "/" << kTurn90Frames << std::endl;
    }
    sc.StopMove();
    std::cout << "[DogTurn] ✅ 转弯完成" << std::endl;
    return true;
}
