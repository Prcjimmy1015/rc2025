#include "case2.h"
#include "../globals.h"
#include "../utils.h"
#include "../params.h"
#include <opencv2/aruco.hpp>
#include <cmath>
#include <iostream>
#include <algorithm>
using namespace unitree::robot;
using namespace cv;
using namespace std;

static int stair_step = 0;
static int stair_cnt = 0;

void case2_reset()
{
    stair_step = 0;
    stair_cnt = 0;
}

bool case2_tick(go2::SportClient &sc,
                const Mat &undist,
                const unitree_go::msg::dds_::SportModeState_ &state,
                int fcount,
                double lx,
                double ly)
{
    (void)fcount;(void)lx;(void)ly;
    stair_cnt++;
    double pitch = state.imu_state().rpy()[1];
    double roll  = state.imu_state().rpy()[0];

    // ====== ArUcoTag 检测 (10cm marker, DICT_4X4_50, ID=0) ======
    Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
    Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    vector<int> ids;
    vector<vector<Point2f>> corners;
    cv::aruco::detectMarkers(undist, dict, corners, ids, params);

    static double aruco_dist  = 999;
    static double aruco_angle = 999;
    static bool   aruco_detected = false;
    static int    aruco_hold = 0;

    bool detected_this_frame = false;
    if (!ids.empty()){
        for (size_t i = 0; i < ids.size(); i++){
            if (ids[i] == 0){
                double marker_size_px = norm(corners[i][0] - corners[i][1]);
                aruco_dist = 0.10 * K.at<double>(0,0) / marker_size_px;
                vector<Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(corners, 0.10, K, D, rvecs, tvecs);
                aruco_angle = atan2(tvecs[i][0], tvecs[i][2]);
                detected_this_frame = true;
                aruco_hold = 30;
                break;
            }
        }
    }

    if (detected_this_frame)
        aruco_detected = true;
    else if (aruco_hold > 0)
        aruco_hold--;
    else
        aruco_detected = false;

    if (aruco_detected)
        cout << "C++ ARUCO dist=" << aruco_dist << " angle=" << aruco_angle << endl;

    // ====== Aruco 方向修正 (分档增益) ======
    double yaw_corr = 0;
    if (aruco_detected && fabs(aruco_angle) > 0.05){
        double k_yaw = (fabs(aruco_angle) > 0.15) ? 0.30 : 0.15;
        yaw_corr = -k_yaw * aruco_angle;
        yaw_corr = max(-0.40, min(0.40, yaw_corr));
    }

    // ====== IMU 航向锁定 (ArUco 丢失时兜底) ======
    static double s1_start_yaw = 0;
    double imu_yaw_err = yaw - s1_start_yaw;
    if (imu_yaw_err >  M_PI) imu_yaw_err -= 2.0 * M_PI;
    if (imu_yaw_err < -M_PI) imu_yaw_err += 2.0 * M_PI;
    double imu_corr = -0.60 * imu_yaw_err;
    imu_corr = max(-0.60, min(0.60, imu_corr));

    double heading_corr = aruco_detected ? yaw_corr : imu_corr;

    // ====== 共享状态 ======
    static double px_start     = 0;
    static double py_start     = 0;
    static double peak_pitch   = 0;
    static int    obx_far_at   = 0;
    static int    settle_cnt   = 0;
    static int    prev_step    = -1;

    static int s1_settle = 0;
    static bool s5_timed_out = false;  // S5 超时, 下台阶带左偏

    if (stair_step != prev_step){
        px_start    = px;
        py_start    = py;
        peak_pitch  = 0;
        obx_far_at  = 0;
        settle_cnt  = 0;
        prev_step   = stair_step;
        if (stair_step == 1){
            s1_start_yaw = yaw;
            s1_settle    = 0;
        }
    }

    if (fabs(pitch) > peak_pitch) peak_pitch = fabs(pitch);

    // ====== 侧倾过大恢复 ======
    bool roll_emergency = (fabs(roll) > 0.50);
    bool in_climb       = (stair_step >= 1 && stair_step <= 3);
    bool in_descend     = (stair_step >= 6 && stair_step <= 8);

    if (roll_emergency && (in_climb || in_descend)){
        sc.StopMove();
        cout << "!! ROLL EMERGENCY: roll=" << roll << " at step=" << stair_step << endl;
        stair_cnt = 0;
        stair_step = 10;
    }
    // ===========================================================

    else if (stair_step == 0){
        // S0: 退出case1后对准台阶
        // phase 0=评估(盲退/粗调/直走) 1=粗调(原地旋转) 2=前进+微调 3=精调(原地旋转)
        static int  s0_phase     = 0;
        static int  s0_align_cnt = 0;

        if (s0_phase == 0){
            // ---- 先看ArUco决定策略 ----
            if (!aruco_detected){
                // 没看到ArUco(太近出画了) → 后退直到看到, timeout 50帧
                sc.ClassicWalk(true);
                sc.Move(-0.15, 0, 0);

                if (stair_cnt % 10 == 0)
                    cout << "[S0-BACK] cnt=" << stair_cnt
                         << " ob_x=" << ob_x
                         << " aruco=" << aruco_detected << endl;

                if (stair_cnt > 35 || aruco_detected){
                    sc.StopMove();
                    if (aruco_detected && fabs(aruco_angle) > 0.08){
                        s0_phase     = 1;
                        s0_align_cnt = 0;
                        cout << "[S0-BACK→COARSE] aruco_angle=" << aruco_angle << endl;
                    }else{
                        s0_phase  = 2;
                        cout << "[S0-BACK→FWD] aruco_angle=" << aruco_angle << endl;
                    }
                    stair_cnt = 0;
                }
            }else if (fabs(aruco_angle) > 0.08){
                // 看到了但角度偏差大 → 原地粗调
                sc.ClassicWalk(true);
                double cyaw = max(-0.40, min(0.40, -0.35 * aruco_angle));
                sc.Move(0, 0, cyaw);
                s0_align_cnt++;

                if (s0_align_cnt % 10 == 0)
                    cout << "[S0-COARSE] cnt=" << s0_align_cnt
                         << " angle=" << aruco_angle
                         << " cyaw=" << cyaw << endl;

                if ((aruco_detected && fabs(aruco_angle) < 0.08)
                    || s0_align_cnt > 60
                    || !aruco_detected){
                    sc.StopMove();
                    s0_phase  = 2;
                    stair_cnt = 0;
                    cout << "[S0-COARSE→FWD]" << endl;
                }
            }else{
                // 角度OK → 直接前进微调
                s0_phase  = 2;
                stair_cnt = 0;
                cout << "[S0→FWD] angle=" << aruco_angle << " straight" << endl;
            }
        }else if (s0_phase == 1){
            // ---- 粗调: 原地旋转对准ArUco (阈值0.08, timeout 60帧) ----
            sc.ClassicWalk(true);
            double cyaw = (aruco_detected)
                ? max(-0.40, min(0.40, -0.35 * aruco_angle))
                : 0.0;
            sc.Move(0, 0, cyaw);
            s0_align_cnt++;

            if (s0_align_cnt % 10 == 0)
                cout << "[S0-COARSE] cnt=" << s0_align_cnt
                     << " angle=" << aruco_angle
                     << " cyaw=" << cyaw << endl;

            if ((aruco_detected && fabs(aruco_angle) < 0.08)
                || s0_align_cnt > 60
                || !aruco_detected){
                sc.StopMove();
                s0_phase  = 2;
                stair_cnt = 0;
                cout << "[S0-COARSE→FWD]" << endl;
            }
        }else if (s0_phase == 2){
            // ---- 前进 + yaw微调 ----
            sc.ClassicWalk(true);
            double vx = (isfinite(ob_x) && ob_x < 0.70) ? 0.10 : 0.18;
            sc.Move(vx, 0, yaw_corr);

            if (stair_cnt % 10 == 0)
                cout << "[S0-FWD] cnt=" << stair_cnt
                     << " ob_x=" << ob_x << " py=" << py
                     << " vx=" << vx
                     << " yaw_corr=" << yaw_corr
                     << " angle=" << aruco_angle << endl;

            if (isfinite(ob_x) && ob_x < 0.55 && ob_x > 0.1){
                if (aruco_detected && fabs(aruco_angle) > 0.06){
                    sc.StopMove();
                    s0_phase     = 3;
                    s0_align_cnt = 0;
                    cout << "[S0-FWD→FINE] aruco_angle=" << aruco_angle
                         << " → fine align" << endl;
                }else{
                    sc.StopMove();
                    cout << "[S0-FWD→CLIMB] ob_x=" << ob_x << endl;
                    stair_cnt   = 0;
                    s0_phase    = 0;
                    stair_step  = 1;
                }
            }else if (stair_cnt > 200){
                s0_phase = 0;
                cout << "[S0-FWD→CLIMB] TIMEOUT" << endl;
                stair_cnt  = 0;
                stair_step = 1;
            }
        }else{ // s0_phase == 3
            // ---- 精调: 靠近后原地细调 (阈值0.05, timeout 50帧, ArUco丢失也退出) ----
            sc.ClassicWalk(true);
            double f_yaw = (aruco_detected)
                ? max(-0.35, min(0.35, -0.30 * aruco_angle))
                : 0.0;
            sc.Move(0, 0, f_yaw);
            s0_align_cnt++;

            if (s0_align_cnt % 10 == 0)
                cout << "[S0-FINE] cnt=" << s0_align_cnt
                     << " angle=" << aruco_angle
                     << " fyaw=" << f_yaw << endl;

            if ((aruco_detected && fabs(aruco_angle) < 0.05)
                || s0_align_cnt > 50
                || !aruco_detected){
                sc.StopMove();
                cout << "[S0-FINE→CLIMB]" << endl;
                stair_cnt   = 0;
                s0_phase    = 0;
                stair_step  = 1;
            }
        }
    }
    else if (stair_step == 1){
        s1_settle++;

        // 前6帧抑制航向修正 (StopMove→ClassicWalk 切换时 IMU yaw 有跳变)
        double s1_hdg = heading_corr;
        if (s1_settle <= 6){
            s1_hdg = 0;
            if (s1_settle == 6){
                s1_start_yaw = yaw;
                cout << "[S1] YAW RELOCK: " << s1_start_yaw << endl;
            }
        }
        // 机械臂负重偏载, 爬台阶方向修正增益×1.9
        s1_hdg = max(-0.85, min(0.85, s1_hdg * 1.9));
        // 左脚打滑检测: roll 变负(左倾) → 右移压回去
        double roll_corr = 0;
        if (s1_settle > 6 && fabs(roll) > 0.10){
            roll_corr = 0.15 * roll;
            roll_corr = max(-0.18, min(0.18, roll_corr));
        }

        sc.ClassicWalk(true);
        sc.Move(0.25, roll_corr, s1_hdg);
        double dpx = px - px_start, dpy = py - py_start;
        double d2d = sqrt(dpx*dpx + dpy*dpy);

        if (obx_far_at == 0 && isfinite(ob_x) && ob_x > 1.5) obx_far_at = stair_cnt;

        if (stair_cnt % 10 == 0)
            cout << "[S1] cnt=" << stair_cnt
                 << " d2d=" << d2d
                 << " ob_x=" << ob_x
                 << " roll=" << roll << " rcorr=" << roll_corr
                 << " obx_far_at=" << obx_far_at << endl;

        bool A = (d2d > 1.30);
        bool B = (obx_far_at > 0 && stair_cnt > obx_far_at + 170);
        bool C = (stair_cnt > 370);

        if (A || B || C){
            cout << "[S1→2] A=" << A << " B=" << B << " C=" << C
                 << " d2d=" << d2d
                 << " obx_far_at=" << obx_far_at
                 << " cnt=" << stair_cnt
                 << " → BACKUP" << endl;
            stair_cnt = 0;
            stair_step = 2;
        }
    }
    else if (stair_step == 2){
        if (stair_cnt < 30){
            sc.StopMove();
        }else{
            sc.ClassicWalk(true);
            sc.Move(-0.08, 0, heading_corr);
        }

        if (stair_cnt % 10 == 0)
            cout << "[S2] cnt=" << stair_cnt
                 << " d2d=" << sqrt(pow(px-px_start,2)+pow(py-py_start,2)) << endl;

        if (stair_cnt > 55){
            sc.StopMove();
            cout << "[S2→5] → TURN" << endl;
            stair_cnt = 0;
            stair_step = 5;
        }
    }
    else if (stair_step == 5){
        static double target_yaw = 0;
        static bool turn_inited = false;
        static double px_turn_start = 0, py_turn_start = 0;

        if (!turn_inited){
            target_yaw = yaw + M_PI / 2.0;
            turn_inited = true; stair_cnt = 0;
            px_turn_start = px; py_turn_start = py;
            cout << "[S5] TURN START yaw=" << yaw << " target=" << target_yaw << endl;
        }

        double err = target_yaw - yaw;
        if (err >  M_PI) err -= 2.0 * M_PI;
        if (err < -M_PI) err += 2.0 * M_PI;

        const double k_p = 0.45;
        double vyaw = k_p * err;
        vyaw = max(-0.55, min(0.55, vyaw));

        double abs_err = fabs(err);
        double vx;
        if      (abs_err > 1.0)  vx = 0.04;
        else if (abs_err > 0.3)  vx = 0.08;
        else                      vx = 0.12;

        double dpx = px-px_turn_start, dpy = py-py_turn_start;
        double d2d_turn = sqrt(dpx*dpx + dpy*dpy);

        sc.ClassicWalk(true);
        sc.Move(vx, 0, vyaw);

        if(stair_cnt%10==0)
            cout<<"[S5] cnt="<<stair_cnt<<" err="<<err<<" vyaw="<<vyaw<<" vx="<<vx<<" d2d="<<d2d_turn<<" yaw="<<yaw<<endl;

        bool angle_ok=(abs_err<0.08), rear_ok=(d2d_turn>0.25), min_time=(stair_cnt>45);
        if(angle_ok&&rear_ok&&min_time){
            sc.StopMove();
            cout<<"[S5] DONE  err="<<err<<" d2d="<<d2d_turn<<" → DESCEND"<<endl;
            turn_inited=false; s5_timed_out=false; stair_cnt=0; stair_step=6;
        }else if(stair_cnt>100){
            sc.StopMove();
            cout<<"[S5] TIMEOUT  err="<<err<<" → DESCEND with drift"<<endl;
            turn_inited=false; s5_timed_out=true; stair_cnt=0; stair_step=6;
        }
    }
    else if (stair_step == 6)  // 下第一级 (顶层→中层)
    {
        double descend_vyaw = s5_timed_out ? 0.04 : 0;
        sc.ClassicWalk(true);
        sc.Move(0.10, 0, descend_vyaw);

        bool dropped = (peak_pitch > 0.30);
        bool settled = (fabs(pitch) < 0.15 && stair_cnt > 20);
        if(dropped&&settled)settle_cnt++;else settle_cnt=0;
        bool on_ground = (stair_cnt > 30 && (!isfinite(ob_x) || ob_x > 2.0));
        if(settle_cnt>12||on_ground||stair_cnt>150){
            sc.StopMove();
            stair_cnt=0;
            stair_step = on_ground ? 9 : 7;
        }
    }
    else if (stair_step == 7)  // 下第二级 (中层→底层)
    {
        double descend_vyaw = s5_timed_out ? 0.04 : 0;
        sc.ClassicWalk(true);
        sc.Move(0.10, 0, descend_vyaw);

        bool dropped = (peak_pitch > 0.30);
        bool settled = (fabs(pitch) < 0.15 && stair_cnt > 20);
        if(dropped&&settled)settle_cnt++;else settle_cnt=0;
        bool on_ground = (stair_cnt > 30 && (!isfinite(ob_x) || ob_x > 2.0));
        if(settle_cnt>12||on_ground||stair_cnt>150){
            sc.StopMove();
            stair_cnt=0;
            stair_step = on_ground ? 9 : 8;
        }
    }
    else if (stair_step == 8)  // 下第三级 (底层→地面)
    {
        double descend_vyaw = s5_timed_out ? 0.04 : 0;
        sc.ClassicWalk(true);
        sc.Move(0.10, 0, descend_vyaw);

        bool dropped = (peak_pitch > 0.30);
        bool settled = (fabs(pitch) < 0.15 && stair_cnt > 20);
        if(dropped&&settled)settle_cnt++;else settle_cnt=0;
        bool on_ground = (stair_cnt > 30 && (!isfinite(ob_x) || ob_x > 2.0));
        if(settle_cnt>12||on_ground||stair_cnt>150){
            sc.StopMove();
            stair_cnt=0;
            stair_step = 9;
        }
    }
    else if (stair_step == 9){
        static double s9_target_yaw = 0;
        static bool s9_turn_inited = false;
        static int s9_phase = 0;  // 0=rotate to south, 1=go straight

        if (!s9_turn_inited){
            double dy = yaw0 + M_PI;
            if(dy > M_PI) dy -= 2*M_PI;
            if(dy < -M_PI) dy += 2*M_PI;
            s9_target_yaw = dy;
            s9_turn_inited = true;
            s9_phase = 0;
            stair_cnt = 0;
            cout << "[S9] TURN TO SOUTH start yaw=" << yaw << " target=" << s9_target_yaw << endl;
        }

        if(s9_phase == 0){
            double err = s9_target_yaw - yaw;
            if(err > M_PI) err -= 2*M_PI;
            if(err < -M_PI) err += 2*M_PI;
            double vyaw = 0.45 * err;
            vyaw = max(-0.55, min(0.55, vyaw));
            sc.ClassicWalk(true);
            sc.Move(0, 0, vyaw);
            if(stair_cnt%10==0)
                cout << "[S9] ROT cnt=" << stair_cnt << " err=" << err
                     << " vyaw=" << vyaw << " yaw=" << yaw << endl;
            if(fabs(err) < 0.08 || stair_cnt > 100){
                sc.StopMove();
                cout << "[S9] ROT DONE → GO STRAIGHT" << endl;
                s9_phase = 1;
                stair_cnt = 0;
            }
            stair_cnt++;
        }else{
            sc.ClassicWalk(true);
            sc.Move(0.15, 0, 0);
            if(stair_cnt > 50){
                sc.StopMove();
                s9_turn_inited = false;
                stair_cnt=0; stair_step=0;
                return true;  // → Flag_Task=3
            }
        }
    }
    else if (stair_step == 10){
        if(stair_cnt<30){sc.StopMove();}
        else{
            double recover_dir = (pitch>0)?0.08:-0.08;
            double side_recover=0;
            if(fabs(roll)>0.2)side_recover=(roll>0)?-0.08:0.08;
            sc.Move(recover_dir,side_recover,0);
            if(fabs(pitch)<0.1&&fabs(roll)<0.1&&stair_cnt>60){
                sc.StopMove();
                stair_cnt=0; stair_step=9;
            }
        }
    }

    return false;
}