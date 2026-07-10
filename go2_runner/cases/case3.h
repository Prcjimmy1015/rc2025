#pragma once

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <opencv2/opencv.hpp>

// case3：V22巡线 + checkpoint任务中断
// 返回:
//   0 → 巡线中
//   1 → 检查到 case4 结束信号（暂不使用）
void case3_reset();
int case3_tick(unitree::robot::go2::SportClient &sc, unitree::robot::go2::VuiClient &vui_client,
               cv::Mat &undist,
               double lx,
               double ly);
