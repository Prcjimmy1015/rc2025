#pragma once
/**
 * bridge/dog_turn.h — 机器狗 90° 协调弧线转弯
 */
#include "params.h"

static inline bool dogTurn90Degrees(unitree::robot::go2::SportClient &sc,
                                     int direction, const double *yaw_ptr)
{
    const char *dir_name = (direction >= 0) ? "LEFT" : "RIGHT";
    double target_angle = *yaw_ptr + direction * M_PI / 2.0;
    while (target_angle > M_PI) target_angle -= 2 * M_PI;
    while (target_angle <= -M_PI) target_angle += 2 * M_PI;

    std::cout << "\n[DogTurn] ====== 转弯 " << dir_name << " 90° ======" << std::endl;
    std::cout << "[DogTurn] 当前 yaw=" << *yaw_ptr * 180 / M_PI
              << "deg → 目标 yaw=" << target_angle * 180 / M_PI << "deg" << std::endl;

    sc.StaticWalk();
    int frame = 0;
    while (frame < kTurn90MaxFrames)
    {
        ++frame;
        double err = target_angle - *yaw_ptr;
        while (err > M_PI) err -= 2 * M_PI;
        while (err <= -M_PI) err += 2 * M_PI;

        if (std::fabs(err) <= kTurn90Tol)
        {
            sc.StopMove();
            std::cout << "[DogTurn] ✅ 转弯完成 err=" << err * 180 / M_PI
                      << "deg (frames=" << frame << ")" << std::endl;
            return true;
        }

        double w_cmd = err * 3.0;
        int w_sign = (err > 0) ? 1 : -1;
        if (std::fabs(w_cmd) < 0.4) w_cmd = w_sign * 0.4;
        w_cmd = std::max(-1.2, std::min(1.2, w_cmd));

        float vx = kTurn90Vx;
        double w_mag = (double)vx / kTurn90R;
        if (w_mag < 0.4) w_mag = 0.4;
        w_mag = std::min(w_mag, 1.2);
        double w_final = (double)w_sign * w_mag;

        sc.Move(vx, 0.f, w_final);

        if (frame % 30 == 0)
            std::cout << "[DogTurn] 转弯中... err=" << err * 180 / M_PI
                      << "deg w=" << w_final << std::endl;
    }
    sc.StopMove();
    std::cerr << "[DogTurn] ❌ 转弯超时" << std::endl;
    return false;
}