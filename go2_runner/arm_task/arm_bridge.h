#pragma once
/**
 * arm_bridge.h — 机器狗控制入口
 *
 * 原地转弯 90° + 警示动作 + 完整任务流程
 */
#include "bridge/dog_turn.h"
#include "bridge/dog_alerts.h"
#include "bridge/arm_utils.h"

/**
 * dogFullTaskManual — 完整任务流程（手动模式，无摄像头依赖）
 *
 * 可用于 test_task 和 case3 集成:
 *   dogFullTaskManual(sc, vc, action_id);
 *
 * 流程: 左转90° → 等1秒 → 执行动作 → (仅stretch)后退一步 → 右转90°
 *
 * 特点:
 *   - 转弯不使用摄像头（dogTurn90DegreesNoCam）
 *   - 可直接传入 case3 中已有的 sc + 临时构造的 vc
 */
static inline void dogFullTaskManual(unitree::robot::go2::SportClient &sc,
                                     unitree::robot::go2::VuiClient &vc,
                                     int action_id)
{
    using namespace unitree::robot;
    using namespace std;

    cout << "\n[DogTask] ====== 开始完整任务流程 ======" << endl;
    cout << "[DogTask] 动作ID: " << action_id
         << " (0=伸懒腰, 1=打招呼, 2=闪烁前灯)" << endl;

    // ─── 步骤1: 左转 90° ───
    dogTurn90DegreesNoCam(sc, +1);

    // ─── 等待 3 秒 ───
    cout << "[DogTask] 识别图像..." << endl;
    sleep(3);

    // ─── 步骤2: 执行动作 ───
    cout << "[DogTask] 执行动作..." << endl;
    switch (action_id)
    {
    case 0:
        cout << "[DogTask] 执行: 伸懒腰 (stretch)" << endl;
        sc.Stretch();

        // 后退一步（约 0.2m）
        {
            cout << "[DogTask] 后退一步..." << endl;
            sc.StaticWalk();
            for (int f = 0; f < 18; ++f)
            {
                sc.Move(-0.2f, 0.f, 0.f);
                usleep(33333);
            }
            sc.StopMove();
        }
        break;
    case 1:
        cout << "[DogTask] 执行: 打招呼 (wave_hello)" << endl;
        sc.Hello();
        break;
    case 2:
        cout << "[DogTask] 执行: 闪烁前灯三次 (flash_lights)" << endl;
        sleep(4);
        for (int i = 0; i < 3; i++)
        {
            cout << '[' << i << ']' << endl;
            vc.SetBrightness(10);
            usleep(400000);
            vc.SetBrightness(0);
            usleep(400000);
        }
        vc.SetBrightness(0);
        break;
    default:
        cerr << "[DogTask] 未知动作ID: " << action_id << endl;
        break;
    }

    // ─── 步骤3: 右转 90° ───
    dogTurn90DegreesNoCam(sc, -1);

    sc.StopMove();
    cout << "[DogTask] ====== 任务完成 ======" << endl;
}