#pragma once
/**
 * bridge/dog_align.h — 机器狗比值对齐（三自由度小步伐控制）
 */
#include "params.h"

static inline bool dogAlignToPlatform(unitree::robot::go2::SportClient &sc,
                                       cv::VideoCapture &cap,
                                       float target_ratio,
                                       const double *yaw_ptr)
{
    std::cout << "\n[DogAlign] ====== 开始比值对齐 target=" << target_ratio << " ======" << std::endl;

    sc.StaticWalk();
    sc.Euler(0, 0.25, 0);

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

        // ---- 检测平台侧面在图像中的位置 ----
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        int roi_y_start = img_h * 2 / 3;
        cv::Mat roi = gray(cv::Rect(0, roi_y_start, img_w, img_h - roi_y_start));

        cv::Mat edges;
        cv::Canny(roi, edges, 40, 120);

        std::vector<int> col_sum(img_w, 0);
        for (int y = 0; y < edges.rows; ++y)
        {
            const uchar *row = edges.ptr(y);
            for (int x = 0; x < img_w; ++x)
                if (row[x]) col_sum[x]++;
        }

        // 滑动窗口找平台侧面左右边界
        int left_bound = img_w / 4, right_bound = img_w * 3 / 4;
        int max_sum = 0, max_x = img_w / 2;
        int win_size = img_w / 8;
        for (int x = img_w / 8; x < img_w - img_w / 8 - win_size; ++x)
        {
            int s = 0;
            for (int i = 0; i < win_size; ++i) s += col_sum[x + i];
            if (s > max_sum) { max_sum = s; max_x = x + win_size / 2; }
        }
        if (max_sum > 5)
        {
            left_bound = max_x - win_size / 2;
            right_bound = max_x + win_size / 2;
        }

        int visual_center_x = img_w / 2;
        float current_ratio = 0.5f;
        if (right_bound > left_bound)
        {
            current_ratio = (float)(visual_center_x - left_bound) / (float)(right_bound - left_bound);
            current_ratio = std::max(0.0f, std::min(1.0f, current_ratio));
        }

        float ratio_err = target_ratio - current_ratio;

        // ---- 三自由度控制 ----
        float w_cmd = -ratio_err * kAlignPGain;
        w_cmd = std::max(-kAlignWMax, std::min(kAlignWMax, w_cmd));

        float vy = -ratio_err * kAlignVyRatioGain;
        vy = std::max(-kAlignVyMax, std::min(kAlignVyMax, vy));

        float dist_err = ob_x_f - kAlignTargetDistM;
        if (!std::isfinite(ob_x_f)) dist_err = 0;
        float vx = dist_err * kAlignVxGain;
        vx = std::max(-kAlignVxMax, std::min(kAlignVxMax, vx));

        sc.Move(vx, vy, w_cmd);

        // ---- 可视化标注 ----
        cv::Mat display = frame.clone();
        int edge_y = roi_y_start + edges.rows / 2;

        cv::line(display, cv::Point(left_bound, edge_y),
                 cv::Point(right_bound, edge_y), cv::Scalar(0, 255, 255), 2);
        cv::putText(display, "0.0", cv::Point(left_bound - 30, edge_y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        cv::putText(display, "1.0", cv::Point(right_bound + 5, edge_y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);

        // 绿色竖直线 — 当前视觉中心
        cv::line(display, cv::Point(visual_center_x, 0),
                 cv::Point(visual_center_x, img_h - 1), cv::Scalar(0, 255, 0), 2);

        // 蓝色虚线 — 目标位置
        int target_x = (int)(left_bound + target_ratio * (right_bound - left_bound));
        target_x = std::max(0, std::min(img_w - 1, target_x));
        for (int y = 0; y < img_h - 1; y += 12)
        {
            int y2 = std::min(y + 6, img_h - 1);
            cv::line(display, cv::Point(target_x, y), cv::Point(target_x, y2),
                     cv::Scalar(255, 0, 0), 2);
        }

        char info[128];
        snprintf(info, sizeof(info), "target=%.3f cur=%.3f err=%.3f",
                 target_ratio, current_ratio, ratio_err);
        cv::putText(display, info, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

        cv::imshow("Go2 Platform Alignment", display);
        cv::waitKey(1);

        if (frame_count % 30 == 0)
            std::cout << "[DogAlign] target=" << target_ratio << " cur=" << current_ratio
                      << " err=" << ratio_err << " w=" << w_cmd
                      << " vx=" << vx << " vy=" << vy << " dist=" << ob_x_f
                      << " left=" << left_bound << " right=" << right_bound << std::endl;

        // 连续稳定帧收敛判断
        static int stable_count = 0;
        if (std::fabs(ratio_err) <= kAlignRatioTol)
        {
            ++stable_count;
            if (stable_count >= 5)
            {
                sc.StopMove();
                std::cout << "[DogAlign] ✅ 对齐完成! err=" << ratio_err
                          << " (frames=" << frame_count << ")" << std::endl;
                cv::destroyWindow("Go2 Platform Alignment");
                stable_count = 0;
                return true;
            }
        }
        else { stable_count = 0; }
    }

    sc.StopMove();
    std::cerr << "[DogAlign] ❌ 对齐超时" << std::endl;
    cv::destroyWindow("Go2 Platform Alignment");
    return false;
}