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

    // ArucoTag 检测
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
                aruco_dist = 0.044 * K.at<double>(0,0) / marker_size_px;
                vector<Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(corners, 0.044, K, D, rvecs, tvecs);
                aruco_angle = rvecs[i][2];
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

    // Aruco 方向修正
    double yaw_corr = 0;
    if (aruco_detected && fabs(aruco_angle) > 0.05){
        yaw_corr = -0.06 * aruco_angle;
        yaw_corr = max(-0.3, min(0.3, yaw_corr));
    }

    // 共享状态
    static double px_start     = 0;
    static double py_start     = 0;
    static double peak_pitch   = 0;
    static int    obx_far_at   = 0;
    static int    settle_cnt   = 0;
    static int    prev_step    = -1;

    if (stair_step != prev_step){
        px_start    = px;
        py_start    = py;
        peak_pitch  = 0;
        obx_far_at  = 0;
        settle_cnt  = 0;
        prev_step   = stair_step;
    }

    if (fabs(pitch) > peak_pitch) peak_pitch = fabs(pitch);

    // 全局安全检测: 侧倾过大恢复
    bool roll_emergency = (fabs(roll) > 0.50);
    bool in_climb       = (stair_step >= 1 && stair_step <= 3);
    bool in_descend     = (stair_step >= 6 && stair_step <= 8);

    if (roll_emergency && (in_climb || in_descend)){
        sc.StopMove();
        cout << "!! ROLL EMERGENCY: roll=" << roll << " at step=" << stair_step << endl;
        stair_cnt = 0;
        stair_step = 10;
    }
    else if (stair_step == 0){
        sc.ClassicWalk(true);
        sc.Move(0.18, 0, yaw_corr);
        if (stair_cnt % 10 == 0)
            cout << "[S0] cnt=" << stair_cnt << " ob_x=" << ob_x << " py=" << py << endl;
        if (isfinite(ob_x) && ob_x < 0.55 && ob_x > 0.1){
            sc.StopMove();
            cout << "[S0→1] ob_x=" << ob_x << " py=" << py << " → CLIMB" << endl;
            stair_cnt = 0; stair_step = 1;
        }else if (stair_cnt > 200){
            cout << "[S0→1] TIMEOUT → CLIMB" << endl;
            stair_cnt = 0; stair_step = 1;
        }
    }
    else if (stair_step == 1){
        sc.ClassicWalk(true);
        sc.Move(0.15, 0, yaw_corr);
        double dpx = px - px_start, dpy = py - py_start;
        double d2d = sqrt(dpx*dpx + dpy*dpy);
        if (obx_far_at == 0 && isfinite(ob_x) && ob_x > 1.5) obx_far_at = stair_cnt;
        if (stair_cnt % 10 == 0)
            cout << "[S1] cnt=" << stair_cnt << " d2d=" << d2d << " ob_x=" << ob_x << " obx_far_at=" << obx_far_at << endl;
        bool A = (d2d > 0.85);
        bool B = (obx_far_at > 0 && stair_cnt > obx_far_at + 100);
        bool C = (stair_cnt > 270);
        if (A || B || C){
            cout << "[S1→5] A=" << A << " B=" << B << " C=" << C << " d2d=" << d2d << " → TURN" << endl;
            stair_cnt = 0; stair_step = 5;
        }
    }
    else if (stair_step == 5){
        static double target_yaw = 0;
        static bool turn_inited = false;
        static double px_turn_start = 0, py_turn_start = 0;
        if (!turn_inited){
            target_yaw = yaw + M_PI/2.0;
            turn_inited = true; stair_cnt = 0;
            px_turn_start = px; py_turn_start = py;
            cout << "[S5] TURN START yaw=" << yaw << " target=" << target_yaw << endl;
        }
        const int PUSH_FRAMES = 5;
        double err = target_yaw - yaw;
        if(err>M_PI)err-=2*M_PI; if(err<-M_PI)err+=2*M_PI;
        double vyaw = 0.45 * err;
        vyaw = max(-0.55, min(0.55, vyaw));
        double abs_err = fabs(err);
        double vx = (abs_err>1.0)?0.04:(abs_err>0.3)?0.08:0.12;
        double dpx = px-px_turn_start, dpy = py-py_turn_start;
        double d2d_turn = sqrt(dpx*dpx+dpy*dpy);
        if (stair_cnt < PUSH_FRAMES) vyaw = 0;
        sc.ClassicWalk(true);
        sc.Move(vx,0,vyaw);
        if(stair_cnt%10==0)
            cout<<"[S5] cnt="<<stair_cnt<<" err="<<err<<" vyaw="<<vyaw<<" vx="<<vx<<" d2d="<<d2d_turn<<endl;
        stair_cnt++;
        bool angle_ok=(abs_err<0.08), rear_ok=(d2d_turn>0.25), min_time=(stair_cnt>45);
        if(angle_ok&&rear_ok&&min_time){
            sc.StopMove();
            cout<<"[S5] DONE → DESCEND"<<endl;
            turn_inited=false; stair_cnt=0; stair_step=6;
        }else if(stair_cnt>300){
            sc.StopMove();
            cout<<"[S5] TIMEOUT → DESCEND"<<endl;
            turn_inited=false; stair_cnt=0; stair_step=6;
        }
    }
    else if (stair_step == 6 || stair_step == 7 || stair_step == 8){
        sc.ClassicWalk(true);
        sc.Move(0.10, 0, 0);
        bool dropped = (peak_pitch > 0.30);
        bool settled = (fabs(pitch) < 0.15 && stair_cnt > 20);
        if(dropped&&settled)settle_cnt++;else settle_cnt=0;
        bool on_ground = (stair_cnt > 30 && (!isfinite(ob_x) || ob_x > 1.5));
        if(settle_cnt>12||on_ground||stair_cnt>80){
            sc.StopMove();
            stair_cnt=0;
            stair_step = on_ground ? 9 : stair_step+1;
        }
    }
    else if (stair_step == 9){
        sc.ClassicWalk(true);
        sc.Move(0.15, 0, 0);
        if(stair_cnt > 2){
            sc.StopMove();
            stair_cnt=0; stair_step=0;
            return true;  // → Flag_Task=3
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