#include "case2.h"
#include "../globals.h"
#include "../utils.h"

#include <cmath>
#include <iostream>

using namespace unitree::robot;
using namespace std;

// checkpoint A1: lx=1.30, ly=2.93, yaw=-2.757
static const double CP_A1_LX = 1.30;
static const double CP_A1_LY = 2.93;
static const double CP_A1_YAW = -2.757;

static bool stairs_done = false;
static bool aligning = false;
static int align_timer = 0;

void case2_reset()
{
    stairs_done = false;
    aligning = false;
    align_timer = 0;
}

bool case2_tick(go2::SportClient &sc,
                double lx,
                double ly)
{
    // Phase 0: 上台阶 (placeholder - 留空等后续实现)
    if (!stairs_done) {
        // TODO: 在这里实现上台阶逻辑
        // 目前跳过，直接标记完成
        stairs_done = true;
        aligning = true;
        align_timer = 0;
        cout << "[case2] Stairs (placeholder) done, aligning to A1..." << endl;
        return false;
    }

    // Phase 1: 回正到 checkpoint A1 的方向
    if (aligning) {
        align_timer++;
        double ey = CP_A1_YAW - yaw;
        if (ey > M_PI) ey -= 2*M_PI;
        if (ey < -M_PI) ey += 2*M_PI;
        double steer = ey * 2.0;
        steer = max(-0.5, min(0.5, steer));
        sc.Move(0, 0, steer);

        if (align_timer % 30 == 0) {
            printf("[case2] Aligning to A1: yaw=%.3f yaw_target=%.3f diff=%.1fdeg\n",
                   yaw, CP_A1_YAW, ey*180/M_PI);
        }

        // 方向对准后完成
        if (fabs(ey) < 0.15 && align_timer > 30) {
            sc.StopMove();
            cout << "[case2] Aligned to A1, entering case3 V22巡线" << endl;
            return true;  // → Flag_Task=3
        }
        return false;
    }

    return false;
}