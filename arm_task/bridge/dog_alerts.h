#pragma once
/**
 * bridge/dog_alerts.h — 机器狗警示动作
 */
#include "params.h"

static inline void dogActionStretch(unitree::robot::go2::SportClient &sc)
{
    std::cout << "[DogAction] 执行: 伸懒腰 (stretch)" << std::endl;
    sc.Stretch();
    sleep(4);
}

static inline void dogActionWaveHello(unitree::robot::go2::SportClient &sc)
{
    std::cout << "[DogAction] 执行: 打招呼 (wave_hello)" << std::endl;
    sc.Hello();
    sleep(4);
}

static inline void dogActionFlashLights(unitree::robot::go2::VuiClient &vc)
{
    std::cout << "[DogAction] 执行: 闪烁前灯三次 (flash_lights)" << std::endl;
    for (int i = 0; i < 4; i++)
    {
        std::cout << '[' << i << ']' << std::endl;
        vc.SetBrightness(10);
        usleep(500000);
        vc.SetBrightness(0);
        usleep(500000);
    }
    vc.SetBrightness(0);
}

static inline void dogDoAlertAction(unitree::robot::go2::SportClient &sc,
                                    unitree::robot::go2::VuiClient &vc,
                                    int warning_id)
{
    switch (warning_id)
    {
    case 0: dogActionWaveHello(sc);    break;  // 打招呼
    case 1: dogActionStretch(sc);      break;  // 伸懒腰
    case 2: dogActionFlashLights(vc);  break;  // 闪烁前灯
    default: std::cerr << "[DogAction] 未知警示标志ID: " << warning_id << std::endl; break;
    }
}