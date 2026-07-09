#pragma once
/**
 * bridge/dog_align.h — 机器狗比值对齐（Canny 边缘检测 + 两阶段控制）
 */
#include "params.h"
using namespace unitree::robot;

static inline void detectByCanny(cv::Mat &frame, int &left_bound, int &right_bound,
                                  int &visual_center_x, int img_w, int img_h)
{
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // ROI: 图像下 1/3
    int roi_y_start = img_h * 2 / 3;
    cv::Mat roi = gray(cv::Rect(0, roi_y_start, img_w, img_h - roi_y_start));

    cv::Mat edges;
    cv::Canny(roi, edges, 40, 120);

    // 水平投影
    std::vector<int> col_sum(img_w, 0);
    int total = 0;
    for (int y = 0; y < edges.rows; ++y)
    {
        const uchar *row = edges.ptr(y);
        for (int x = 0; x < img_w; ++x)
            if (row[x]) { col_sum[x]++; total++; }
    }

    // 找 col_sum 的峰值
    int peak = 0;
    for (int x = 0; x < img_w; ++x)
        if (col_sum[x] > peak) peak = col_sum[x];

    // 默认边界 (检测失败时用)
    left_bound = img_w / 4;
    right_bound = img_w * 3 / 4;

    if (peak >= 5 && total >= 50)
    {
        int thresh = std::max(5, peak / 3);
        // 从左向右扫描找真实左边缘
        for (int x = img_w / 8; x < img_w; ++x)
        {
            if (col_sum[x] > thresh) { left_bound = x; break; }
        }
        // 从右向左扫描找真实右边缘
        for (int x = img_w * 7 / 8; x >= 0; --x)
        {
            if (col_sum[x] > thresh) { right_bound = x; break; }
        }
        // 边界合理性检查
        int w = right_bound - left_bound;
        if (w < 30 || w > img_w * 3 / 4)
        {
            left_bound = img_w / 4;
            right_bound = img_w * 3 / 4;
        }
    }
    visual_center_x = img_w / 2;
}

static inline bool dogAlignToPlatform(unitree::robot::go2::SportClient &sc,
                                       cv::VideoCapture &cap,
                                       float target_ratio)
{
    std::cout << "\n[DogAlign] ====== 开始比值对齐 target=" << target_ratio << " ======" << std::endl;

    sc.StaticWalk();

    cv::Mat frame;
    int frame_count = 0;

    while (frame_count < kAlignMaxFrames)
    {
        ++frame_count;
        if (!cap.read(frame) || frame.empty())
        {
            std::cerr << "[DogAlign] 相机帧读取失败" << std::endl;
            continue;
        }

        int img_w = frame.cols;
        int img_h = frame.rows;

        int left_bound, right_bound, visual_center_x;
        detectByCanny(frame, left_bound, right_bound, visual_center_x, img_w, img_h);

        // 计算比值
        float current_ratio = 0.5f;
        if (right_bound > left_bound + 1)
        {
            current_ratio = (float)(visual_center_x - left_bound) / (float)(right_bound - left_bound);
            current_ratio = std::max(0.0f, std::min(1.0f, current_ratio));
        }

        float ratio_err = target_ratio - current_ratio;

        // 两阶段控制
        static int phase = 0;
        float w_cmd = 0, vy = 0, vx = 0;

        if (phase == 0)
        {
            w_cmd = -ratio_err * kAlignPGain;
            w_cmd = std::max(-kAlignWMax, std::min(kAlignWMax, w_cmd));
            vy = -ratio_err * kAlignVyRatioGain;
            vy = std::max(-kAlignVyMax, std::min(kAlignVyMax, vy));
            vx = 0;

            static int p1_stable = 0;
            if (std::fabs(ratio_err) <= kAlignRatioTol)
            {
                ++p1_stable;
                if (p1_stable >= 5)
                {
                    sc.StopMove();
                    std::cout << "[DogAlign] ✅ 朝向对齐完成! err=" << ratio_err << std::endl;
                    phase = 1;
                    p1_stable = 0;
                }
            }
            else { p1_stable = 0; }
        }
        else
        {
            w_cmd = 0; vy = 0;
            float dist_err = ob_x_f - kAlignTargetDistM;
            if (!std::isfinite(ob_x_f)) dist_err = 0;
            vx = dist_err * kAlignVxGain;
            vx = std::max(-kAlignVxMax, std::min(kAlignVxMax, vx));

            static int p2_stable = 0;
            if (std::fabs(dist_err) <= 0.05f)
            {
                ++p2_stable;
                if (p2_stable >= 5)
                {
                    sc.StopMove();
                    std::cout << "[DogAlign] ✅ 距离对齐完成! dist=" << ob_x_f
                              << " (frames=" << frame_count << ")" << std::endl;
                    cv::destroyWindow("Go2 Platform Alignment");
                    phase = 0; p2_stable = 0;
                    return true;
                }
            }
            else { p2_stable = 0; }
        }

        sc.StaticWalk();
        sc.Move(vx, vy, w_cmd);

        // 可视化
        cv::Mat display = frame.clone();
        int edge_y = img_h * 2 / 3;
        cv::line(display, cv::Point(left_bound, edge_y), cv::Point(right_bound, edge_y),
                 cv::Scalar(0, 255, 255), 2);
        cv::putText(display, "0.0", cv::Point(left_bound - 30, edge_y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        cv::putText(display, "1.0", cv::Point(right_bound + 5, edge_y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        cv::line(display, cv::Point(visual_center_x, 0), cv::Point(visual_center_x, img_h - 1),
                 cv::Scalar(0, 255, 0), 2);

        int target_x = left_bound + (int)(target_ratio * (right_bound - left_bound));
        for (int y = 0; y < img_h - 1; y += 12)
            cv::line(display, cv::Point(target_x, y), cv::Point(target_x, std::min(y+6, img_h-1)),
                     cv::Scalar(255, 0, 0), 2);

        char info[128];
        snprintf(info, sizeof(info), "P%d t=%.3f c=%.3f e=%.3f W=%d",
                 phase + 1, target_ratio, current_ratio, ratio_err, right_bound - left_bound);
        cv::putText(display, info, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        cv::imshow("Go2 Platform Alignment", display);
        cv::waitKey(1);

        if (frame_count % 30 == 0)
            std::cout << "[DogAlign] P" << (phase + 1) << " target=" << target_ratio
                      << " cur=" << current_ratio << " err=" << ratio_err
                      << " L=" << left_bound << " R=" << right_bound
                      << " W=" << (right_bound - left_bound) << std::endl;
    }

    sc.StopMove();
    std::cerr << "[DogAlign] ❌ 对齐超时" << std::endl;
    cv::destroyWindow("Go2 Platform Alignment");
    return false;
}

// =============================================================================
// 观察模式
// =============================================================================
static inline void dogAlignObserve(unitree::robot::go2::SportClient &sc, cv::VideoCapture &cap)
{
    std::cout << "\n[DogAlign] ====== 观察模式 (不移动) ======\n[DogAlign] ESC 键退出" << std::endl;
    sc.StaticWalk();
    cv::Mat frame;

    while (true)
    {
        if (!cap.read(frame) || frame.empty()) { continue; }
        int img_w = frame.cols, img_h = frame.rows;
        int left_bound, right_bound, visual_center_x;
        detectByCanny(frame, left_bound, right_bound, visual_center_x, img_w, img_h);

        float current_ratio = 0.5f;
        if (right_bound > left_bound + 1)
            current_ratio = (float)(visual_center_x - left_bound) / (float)(right_bound - left_bound);

        sc.Move(0.f, 0.f, 0.f);

        cv::Mat display = frame.clone();
        int edge_y = img_h * 2 / 3;
        cv::line(display, cv::Point(left_bound, edge_y), cv::Point(right_bound, edge_y),
                 cv::Scalar(0, 255, 255), 2);
        cv::putText(display, "0.0", cv::Point(left_bound - 30, edge_y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        cv::putText(display, "1.0", cv::Point(right_bound + 5, edge_y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        cv::line(display, cv::Point(visual_center_x, 0), cv::Point(visual_center_x, img_h - 1),
                 cv::Scalar(0, 255, 0), 2);

        char info[128];
        snprintf(info, sizeof(info), "Canny r=%.3f L=%d R=%d W=%d",
                 current_ratio, left_bound, right_bound, right_bound - left_bound);
        cv::putText(display, info, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        cv::imshow("Go2 Observe", display);
        int key = cv::waitKey(1);
        if (key == 27) break;

        static int cnt = 0;
        if (++cnt % 30 == 0)
            std::cout << "[Observe] ratio=" << current_ratio << " L=" << left_bound
                      << " R=" << right_bound << " W=" << (right_bound - left_bound) << std::endl;
    }
    cv::destroyWindow("Go2 Observe");
    sc.StopMove();
}