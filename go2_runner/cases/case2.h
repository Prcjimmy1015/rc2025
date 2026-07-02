#pragma once

#include <unitree/robot/go2/sport/sport_client.hpp>

// case2: 上台阶（placeholder）+ 回正到 A1 后进入 case3
// 返回 true → 进入 case3 (V22巡线)
void case2_reset();
bool case2_tick(unitree::robot::go2::SportClient &sc,
                double lx,
                double ly);