#pragma once

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <opencv2/opencv.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

// case2: 上下三级台阶 + 回正到 A1 后进入 case3
// 返回 true → Flag_Task=3 (V22巡线)
void case2_reset();
bool case2_tick(unitree::robot::go2::SportClient &sc,
                const cv::Mat &undist,
                const unitree_go::msg::dds_::SportModeState_ &state,
                int fcount,
                double lx,
                double ly);
