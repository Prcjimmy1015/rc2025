#include <opencv2/opencv.hpp>

#include <opencv2/aruco.hpp>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/ros2/PointStamped_.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <atomic>

#define TOPIC_RANGE_INFO "rt/utlidar/range_info"
#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;
using namespace cv;
using namespace std;

/* Camera intrinsics for the **front** RGB camera (replace with yours) */
static const Mat K = (Mat_<double>(3, 3) << 929.7797, 0, 629.6662,
                      0, 926.7584, 335.6207,
                      0, 0, 1);
static const Mat D = (Mat_<double>(1, 4) << -0.4157, 0.1327, 0, 0);

/* ------------------------------------------------------------------ */
/* -------------------  Global navigation state  -------------------- */
float ob_x = 0, ob_y = 0, ob_z = 0; // lidar ranges
double px = 0, py = 0, yaw = 0;     // body pose
double px0 = 0, py0 = 0, yaw0 = 0;  // pose at start
int Flag_Task = 0;                  // main FSM flag
int turn_step = 0;
int sub_step = 0;
int frame_cnt = 0;
double jump_yaw = 0;  // 起跳时的方向
bool can_turn = false;

/* -------------------  Safety Zone  -------------------- */
int start_jump_times = 0;
int end_jump_times = 0;
bool found_turn = false;
int obstacle_avoidance_state = 0;

/* Global variable to store last detected marker id */
std::atomic<int> g_last_aruco_id(-1);
float g_aruco_dist = 0;
float g_aruco_angle = 0;

void aruco_socket_server(int port = 5005)
{
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr("127.0.0.1");
    address.sin_port = htons(port);
    bind(server_fd, (struct sockaddr *)&address, sizeof(address));
    listen(server_fd, 1);
    std::cout << "Aruco socket server listening on 127.0.0.1:" << port << std::endl;
    while (true)
    {
        int client_fd = accept(server_fd, nullptr, nullptr);
        char buffer[32];
        while (true)
        {
            ssize_t len = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
            if (len > 0)
            {
                buffer[len] = '\0';
                int id;
                float dist, angle;
                if (sscanf(buffer, "%d,%f,%f", &id, &dist, &angle) == 3)
                {
                    g_last_aruco_id = id;
                    g_aruco_dist = dist;
                    g_aruco_angle = angle;
                    std::cout << "Received aruco id: " << id << " dist: " << dist << " angle: " << angle << std::endl;
                }
                else
                {
                    id = atoi(buffer);
                    g_last_aruco_id = id;
                    std::cout << "Received aruco id: " << id << std::endl;
                }
            }
            else if (len == 0)
            {
                // client disconnected
                close(client_fd);
                break;
            }
        }
    }
    close(server_fd);
}

/* Utility: convert pose to local frame aligned with (px0,py0,yaw0) */
static inline void transformLocal(double x, double y, double yaw_now,
                                  double &lx, double &ly, double &dyaw)
{
    double c = cos(yaw0), s = sin(yaw0);
    lx = (x - px0) * c + (y - py0) * s;
    ly = -(x - px0) * s + (y - py0) * c;
    dyaw = yaw_now - yaw0;
    if (dyaw > M_PI)
        dyaw -= 2 * M_PI;
    if (dyaw < -M_PI)
        dyaw += 2 * M_PI;
}

/* ------------------------------------------------------------------ */
/* ------------------  ROS2 / RTDDS Call-backs  --------------------- */
void rangeCB(const void *m)
{
    auto *p = (const geometry_msgs::msg::dds_::PointStamped_ *)m;
    ob_x = p->point().x();
    ob_y = p->point().y();
    ob_z = p->point().z();
}

class StateCB
{
public:
    unitree_go::msg::dds_::SportModeState_ state;
    void operator()(const void *m)
    {
        state = *(const unitree_go::msg::dds_::SportModeState_ *)m;
        px = state.position()[0];
        py = state.position()[1];
        yaw = state.imu_state().rpy()[2];
    }
};

// Add PID_Yaw and PID_Yaw1 functions from v1_code.cpp
float PID_Yaw(float expect, float err)
{
    static float integral = 0, error_last = 0;
    float p = 5.0, i = 0, d = 0;
    float error_current = err - expect;
    integral += error_current;
    float output = -(p * error_current + i * integral + d * (error_current - error_last));
    error_last = error_current;
    return std::max(-2.0f, std::min(2.0f, output));
}

float PID_Yaw1(float expect, float err)
{
    static float integral = 0, error_last = 0;
    float p = 0.025, i = 0, d = 0;
    float error_current = err - expect;
    integral += error_current;
    float output = -(p * error_current + i * integral + d * (error_current - error_last));
    error_last = error_current;
    return std::max(-2.0f, std::min(2.0f, output));
}

/* ------------------------------------------------------------------ */
/* --------------------------  Main program  ------------------------ */
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        cerr << "Usage: " << argv[0] << " <ethernet_if>\n";
        return -1;
    }

    /* Init Unitree DDS */
    ChannelFactory::Instance()->Init(0, argv[1]);

    /* Subscribers */
    ChannelSubscriber<geometry_msgs::msg::dds_::PointStamped_> sub_range(TOPIC_RANGE_INFO);
    sub_range.InitChannel(rangeCB);

    StateCB stateCB;
    ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> sub_state(TOPIC_HIGHSTATE);
    sub_state.InitChannel(stateCB);

    /* Sport client */
    go2::SportClient sc;
    sc.SetTimeout(10.0f);
    sc.Init();
    sc.BalanceStand();
    Flag_Task = 2;
    // this_thread::sleep_for(500ms);

    /* Save initial pose */
    px0 = px;
    py0 = py;
    yaw0 = yaw;

    /* Front-RGB stream (same pipeline as before) */
    VideoCapture cap(
        "udpsrc address=230.1.1.1 port=1720 multicast-iface=ens33 "
        "! application/x-rtp, media=video, encoding-name=H264 "
        "! rtph264depay ! h264parse ! avdec_h264 ! videoconvert "
        "! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1",
        CAP_GSTREAMER);
    if (!cap.isOpened())
    {
        cerr << "Front camera stream not opened\n";
        return -1;
    }

    Mat frame, undist;
    int fcount = 0;
    auto t0 = chrono::steady_clock::now();

    // Start the aruco socket server in a background thread
    std::thread aruco_thread(aruco_socket_server, 5005);
    aruco_thread.detach();

    /* ---------------------------  LOOP  --------------------------- */
    while (true)
    {
        if (!cap.read(frame) || frame.empty())
            break;
        fcount++;
        undistort(frame, undist, K, D);

        cout << "Flag_Task=" << Flag_Task
             << " ob_x=" << ob_x << " ob_y=" << ob_y << " ob_z=" << ob_z
             << " px=" << px << " py=" << py << " yaw=" << yaw
             << " pitch=" << stateCB.state.imu_state().rpy()[1]
             << " roll=" << stateCB.state.imu_state().rpy()[0] << endl;

        /* FPS overlay */
        double fps = fcount / chrono::duration<double>(chrono::steady_clock::now() - t0).count();
        putText(undist, format("FPS %.1f", fps), {10, 30},
                FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);

        /*****************   MAIN FSM  *****************/
        double lx, ly, dyaw;
        transformLocal(px, py, yaw, lx, ly, dyaw);
        double yaw_pid = dyaw * -5.0; // crude P controller (rad→cmd)

        switch (Flag_Task)
        {
        
        case 0: /* 起点区 —— line following until obstacle-avoid */
        {
            //Jump once only to get out of the starting line
            static int pre_jump = 0;
            static int pause_before_jump = 0;
            static int jump_wait = 0;

            if (pre_jump < 20)
            {
                if (pre_jump == 0) jump_yaw = yaw;
                sc.StaticWalk();
                sc.Move(0.2, 0, 0);
                pre_jump++;
                break;
            }
            
            if (pause_before_jump < 10)
            {
                sc.StopMove();
                pause_before_jump++;
                break;
            }
            if (start_jump_times == 0)
            {
                sc.FrontJump();
                start_jump_times++;
                break;
            }
            if (start_jump_times == 1 && jump_wait < 15)
            {
                jump_wait++;
                break;
            }

            /* 用颜色范围检测黑线 */
            undist = undist(Rect(0, undist.rows * 2 / 3, undist.cols, undist.rows / 3));
            Mat hsv, mask;
            cvtColor(undist, hsv, COLOR_BGR2HSV);

            Scalar lower_black(0, 0, 0);
            Scalar upper_black(180, 255, 25);
            inRange(hsv, lower_black, upper_black, mask);

            Mat kernel = getStructuringElement(MORPH_RECT, {5, 5});
            morphologyEx(mask, mask, MORPH_CLOSE, kernel);
            morphologyEx(mask, mask, MORPH_OPEN, kernel);

            imshow("Binary", mask);

            double err = 0;
            int cnt = 0;
            for (int r = mask.rows - 1; r >= mask.rows - 120; --r)
            {
                const uchar *row = mask.ptr(r);
                for (int c = 0; c < mask.cols; ++c)
                    if (row[c])
                    {
                        err += c - 640;
                        cnt++;
                    }
            }
            err = cnt ? err / cnt : 0;

            double steer = -0.001 * err;
            cout << "cnt: " << cnt << endl;

            sc.StaticWalk();
            sc.Euler(0, 0.25, 0);
            sc.Move(0.25, 0, steer);

            static int no_line_count = 0;
            static int slow_cnt = 0;

            static bool aligning = false;
            static int align_cnt = 0;
            static bool centering = false;
            static int center_cnt = 0;

            if (centering)
            {
                sc.Move(0, 0.08, 0);
                center_cnt++;
                if (center_cnt > 8)
                {
                    sc.StopMove();
                    centering = false;
                    center_cnt = 0;
                    can_turn = false;
                    turn_step = 0;
                    sub_step = 0;
                    frame_cnt = 0;
                    Flag_Task = 1;
                }
            }
            else if (aligning)
            {
                double yaw_err = yaw - jump_yaw;
                double yaw_corr = (yaw_err > 0) ? -0.3 : 0.3;
                double side_corr = 0;
                if (!isfinite(ob_y)) side_corr = -0.08;
                else if (!isfinite(ob_z)) side_corr = 0.08;
                sc.Move(0, side_corr, yaw_corr);
                align_cnt++;
                if (abs(yaw_err) < 0.05 || align_cnt > 60)
                {
                    sc.StopMove();
                    aligning = false;
                    align_cnt = 0;
                    centering = true;
                }
            }
            else if (no_line_count > 10 || (isfinite(ob_x) && ob_x < 1.8))
            {
                sc.Move(0.08, 0, 0);
                slow_cnt++;
                
                if (slow_cnt > 10)
                {
                    sc.StopMove();
                    slow_cnt = 0;
                    aligning = true;
                    align_cnt = 0;
                }
            }
            else if (cnt < 500)
            {
                no_line_count++;
            }
            else
            {
                no_line_count = 0;
            }
        }
        break;

        case 1: /* 雷达全程控制转弯 */
        {
            frame_cnt++;            

            static double last_ox = 2.0, last_oy = 0.3, last_oz = 0.3;
            if (isfinite(ob_x)) last_ox = ob_x; else ob_x = last_ox;
            if (isfinite(ob_y)) last_oy = ob_y; else ob_y = last_oy;
            if (isfinite(ob_z)) last_oz = ob_z; else ob_z = last_oz;

            static double ox_hist[5] = {2.0, 2.0, 2.0, 2.0, 2.0};
            static double oy_hist[5] = {0.3, 0.3, 0.3, 0.3, 0.3};
            static double oz_hist[5] = {0.3, 0.3, 0.3, 0.3, 0.3};
            static int hist_idx = 0;

            if (isfinite(ob_x)) ox_hist[hist_idx] = ob_x;
            if (isfinite(ob_y)) oy_hist[hist_idx] = ob_y;
            if (isfinite(ob_z)) oz_hist[hist_idx] = ob_z;
            hist_idx = (hist_idx + 1) % 5;

            double ox_sorted[5], oy_sorted[5], oz_sorted[5];
            for (int i = 0; i < 5; i++) {
                ox_sorted[i] = ox_hist[i];
                oy_sorted[i] = oy_hist[i];
                oz_sorted[i] = oz_hist[i];
            }
            sort(ox_sorted, ox_sorted + 5);
            sort(oy_sorted, oy_sorted + 5);
            sort(oz_sorted, oz_sorted + 5);
            double filtered_ob_x = ox_sorted[2];
            double filtered_ob_y = oy_sorted[2];
            double filtered_ob_z = oz_sorted[2];

            cout << "CASE1 turn=" << turn_step << " sub=" << sub_step
                 << " cnt=" << frame_cnt << " ox=" << filtered_ob_x << " oy=" << filtered_ob_y << " oz=" << filtered_ob_z << endl;

            putText(undist, format("ox=%.2f", filtered_ob_x), {10, 60}, FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 255}, 2);
            putText(undist, format("oy=%.2f", filtered_ob_y), {10, 85}, FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
            putText(undist, format("oz=%.2f", filtered_ob_z), {10, 110}, FONT_HERSHEY_SIMPLEX, 0.7, {255, 0, 0}, 2);

            if (turn_step >= 5)
            {
                sc.StopMove();
                Flag_Task = 2;
            }
            else if (sub_step == 0)  // 直走
            {
                static int enter_cnt = 0;
                enter_cnt++;
                if (enter_cnt > 60) can_turn = true;

                sc.StaticWalk();

                double yaw_corr = 0;
                if (turn_step == 0)
                {
                    double yaw_err = yaw - jump_yaw;
                    yaw_corr = -0.8 * yaw_err;
                }

                double side_corr = 0;
                if (!isfinite(filtered_ob_y)) side_corr = -0.08;
                else if (!isfinite(filtered_ob_z)) side_corr = 0.08;

                if (filtered_ob_x < 0.35) { side_corr = 0; yaw_corr = 0; }
                sc.Move(0.16, side_corr, yaw_corr);
                sc.Euler(0, 0, 0);

                double side_diff = (turn_step == 2 || turn_step == 3) ? (filtered_ob_z - filtered_ob_y) : (filtered_ob_y - filtered_ob_z);
                double diff_threshold = (turn_step == 0) ? 0.60 : -999;

                static int low_count1 = 0;
                static int low_count2 = 0;

                if (turn_step == 1)
                {
                    if (filtered_ob_x < 0.55)
                        low_count2++;
                    else
                        low_count2 = 0;
                }
                else
                {
                    if (filtered_ob_x <= 0.45 || side_diff > diff_threshold)
                        low_count1++;
                    else
                        low_count1 = 0;
                }

                if (can_turn && ((turn_step == 1 && low_count2 > 2) || (turn_step != 1 && low_count1 > 3)))
                {
                    low_count1 = 0;
                    low_count2 = 0;
                    frame_cnt = 0;
                    if (turn_step == 0)
                        sub_step = 1;
                    else if (turn_step == 1)
                        sub_step = 6;
                    else if (turn_step == 3)
                        sub_step = 9;
                    else if (turn_step == 2)
                        sub_step = 12;
                    else if (turn_step == 4)
                        sub_step = 11;
                    else
                        sub_step = 2;
                }
            }
            else if (sub_step == 1)  // 侧移靠近内墙
            {
                double side_dir = (turn_step == 2 || turn_step == 3) ? -0.10 : 0.10;
                sc.Move(0, side_dir, 0);
                if (frame_cnt > 10) { frame_cnt = 0; sub_step = 2; }
            }
            else if (sub_step == 6)  // 后退
            {
                sc.Move(-0.12, 0, 0);
                if (frame_cnt > 10) { frame_cnt = 0; sub_step = 5; }
            }
            else if (sub_step == 7)  // 后退（第四个弯）
            {
                sc.Move(-0.08, 0, 0);
                if (frame_cnt > 15) { frame_cnt = 0; sub_step = 2; }
            }
            else if (sub_step == 9)  // 右移（第四个弯）
            {
                sc.Move(0, -0.10, 0);
                if (frame_cnt > 6) { frame_cnt = 0; sub_step = 2; }
            }            
            else if (sub_step == 8)  // 前进（第三个弯）
            {
                sc.Move(0.08, 0, 0);
                if (frame_cnt > 15) { frame_cnt = 0; sub_step = 2; }
            }
                        else if (sub_step == 12)  // 右移（第三个弯）
            {
                sc.Move(0, -0.10, 0);
                if (frame_cnt > 8) { frame_cnt = 0; sub_step = 2; }
            }
            else if (sub_step == 11)  // 左移（第五个弯）
            {
                sc.Move(0, 0.10, 0);
                if (frame_cnt > 10) { frame_cnt = 0; sub_step = 2; }
            }                                                
            else if (sub_step == 5)  // 侧移远离右墙
            {
                sc.Move(0, 0.10, 0);
                if (frame_cnt > 12) { frame_cnt = 0; sub_step = 2; }
            }
            else if (sub_step == 2)  // 弧线转
            {
                double turn_dir = (turn_step == 1) ? 0.45 : ((turn_step == 0 || turn_step == 4) ? 0.40 : ((turn_step == 2 || turn_step == 3) ? -0.35 : 0.35));

                double side_corr = 0;
                if (filtered_ob_x < 0.35) side_corr = (turn_dir > 0) ? -0.08 : 0.08;
                if (turn_step == 2 || turn_step == 3) {
                    if (filtered_ob_y < 0.25) side_corr = 0.08;
                } else {
                    if (filtered_ob_z < 0.25) side_corr = -0.08;
                }
                
                if (turn_step == 0) side_corr += 0.06;
                if (turn_step == 1) side_corr += 0.06;                
                if (turn_step == 2) side_corr -= 0.06;
                if (turn_step == 3) side_corr -= 0.06;
                if (turn_step == 4) side_corr += 0.06;
                sc.Move(0.06, side_corr, turn_dir);

                static double start_yaw;
                if (frame_cnt == 1) start_yaw = yaw;
                double dyaw = fabs(yaw - start_yaw);
                if (dyaw > M_PI) dyaw = 2 * M_PI - dyaw;

                double target_angle = (turn_step == 1) ? 1.5 : ((turn_step == 3) ? 1.4 : 1.57);
                if (dyaw > target_angle)
                {
                    frame_cnt = 0;
                    sub_step = 3;
                }
            }
            else if (sub_step == 3)  // 前进收尾
            {
                sc.Move(0.08, 0, 0);
                if (frame_cnt > 15)
                {
                    frame_cnt = 0;
                    sub_step = 0;
                    turn_step++;
                }
            }
        }
        break; 
               
        case 2: /* 上下三级台阶 —— 闭环控制版 (前脚上顶后边转边走) */
        {
            static int stair_step = 0;
            static int stair_cnt = 0;
            stair_cnt++;
            double pitch = stateCB.state.imu_state().rpy()[1];
            double roll  = stateCB.state.imu_state().rpy()[0];

            // ====== 全程检测 ArucoTag (10cm marker, DICT_4X4_50, ID=0, 贴在台阶正面) ======
            cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
            cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(undist, dict, corners, ids, params);

            static double aruco_dist  = 999;
            static double aruco_angle = 999;
            static bool   aruco_detected = false;
            static int    aruco_hold = 0;

            bool detected_this_frame = false;
            if (!ids.empty())
            {
                for (size_t i = 0; i < ids.size(); i++)
                {
                    if (ids[i] == 0)
                    {
                        double marker_size_px = cv::norm(corners[i][0] - corners[i][1]);
                        aruco_dist = 0.10 * K.at<double>(0,0) / marker_size_px;
                        std::vector<cv::Vec3d> rvecs, tvecs;
                        cv::aruco::estimatePoseSingleMarkers(corners, 0.10, K, D, rvecs, tvecs);
                        // 用 marker 位置算偏航角: atan2(横向偏移, 纵向距离)
                        // 正值=marker在右, 负值=marker在左
                        aruco_angle = atan2(tvecs[i][0], tvecs[i][2]);
                        detected_this_frame = true;
                        aruco_hold = 30;  // 保持 30 帧 (~0.6s)
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

            // ====== Aruco 方向修正 (分档增益: 角度大时力度大) ======
            double yaw_corr = 0;
            if (aruco_detected && fabs(aruco_angle) > 0.05)  // 死区 ±0.05rad (~3°)
            {
                double k_yaw = (fabs(aruco_angle) > 0.15) ? 0.20 : 0.08;
                yaw_corr = -k_yaw * aruco_angle;              // 负=右转
                yaw_corr = std::max(-0.35, std::min(0.35, yaw_corr)); // 限幅
            }

            // ====== IMU 航向锁定 (ArUco 丢失时兜底) ======
            static double s1_start_yaw = 0;
            double imu_yaw_err = yaw - s1_start_yaw;
            if (imu_yaw_err >  M_PI) imu_yaw_err -= 2.0 * M_PI;
            if (imu_yaw_err < -M_PI) imu_yaw_err += 2.0 * M_PI;
            double imu_corr = -0.45 * imu_yaw_err;              // P 控制拉回, 负号=偏右则左修
            imu_corr = std::max(-0.50, std::min(0.50, imu_corr));

            // 最终修正: ArUco 优先, 丢失时 IMU 兜底
            double heading_corr = aruco_detected ? yaw_corr : imu_corr;

            // ====== 共享状态: odometry + lidar ======
            static double px_start     = 0;    // S1 起始
            static double py_start     = 0;
            static double peak_pitch   = 0;    // 下台阶用
            static int    obx_far_at   = 0;    // ob_x > 1.5 首次帧号
            static int    settle_cnt   = 0;
            static int    prev_step    = -1;
            static bool   early_trigger = true;
            static bool   s5_timed_out = false;  // S5 超时, 下台阶带左偏

            // 刚进入新 step 时重置
            if (stair_step != prev_step)
            {
                px_start    = px;
                py_start    = py;
                peak_pitch  = 0;
                obx_far_at  = 0;
                settle_cnt  = 0;
                prev_step   = stair_step;
                if (stair_step == 1)
                    s1_start_yaw = yaw;   // S1 入口记录航向, S2 共用
            }

            // pitch 峰值跟踪 (下台阶 step 6/7/8 使用)
            if (fabs(pitch) > peak_pitch)
                peak_pitch = fabs(pitch);

            // ====== 全局安全检测: 侧倾过大立即触发恢复 ======
            bool roll_emergency = (fabs(roll) > 0.50);  // >28° 侧倾
            bool in_climb       = (stair_step >= 1 && stair_step <= 3);
            bool in_descend     = (stair_step >= 6 && stair_step <= 8);

            if (roll_emergency && (in_climb || in_descend))
            {
                sc.StopMove();
                cout << "!! ROLL EMERGENCY: roll=" << roll << " at step=" << stair_step << endl;
                stair_cnt = 0;
                stair_step = 10;
                // 跳过本轮后续逻辑
                // (放在 if-else 链最前面检查)
            }
            // ===========================================================

            if (stair_step == 0)  // 直走接近台阶 → ArUco对准 → 爬升
            {
                static int  s0_phase     = 0;  // 0=走近, 1=原地对准
                static int  s0_align_cnt = 0;

                if (s0_phase == 0)  // 走近台阶
                {
                    sc.ClassicWalk(true);
                    // 靠近时降速, 给 yaw_corr 更多修正时间
                    double vx = (isfinite(ob_x) && ob_x < 0.70) ? 0.10 : 0.18;
                    sc.Move(vx, 0, yaw_corr);

                    if (stair_cnt % 10 == 0)
                        cout << "[S0] cnt=" << stair_cnt
                             << " ob_x=" << ob_x << " py=" << py
                             << " vx=" << vx
                             << " yaw_corr=" << yaw_corr << endl;

                    if (isfinite(ob_x) && ob_x < 0.55 && ob_x > 0.1)
                    {
                        // 检测到台阶很近: 如果 ArUco 可见且角度偏了, 先原地对准
                        if (aruco_detected && fabs(aruco_angle) > 0.06)
                        {
                            sc.StopMove();
                            s0_phase     = 1;
                            s0_align_cnt = 0;
                            cout << "[S0→ALIGN] aruco_angle=" << aruco_angle
                                 << " → align first" << endl;
                        }
                        else
                        {
                            sc.StopMove();
                            cout << "[S0→1] ob_x=" << ob_x << " → CLIMB" << endl;
                            stair_cnt   = 0;
                            s0_phase    = 0;
                            stair_step  = 1;
                        }
                    }
                    else if (stair_cnt > 200)
                    {
                        s0_phase = 0;
                        cout << "[S0→1] TIMEOUT → CLIMB" << endl;
                        stair_cnt  = 0;
                        stair_step = 1;
                    }
                }
                else  // s0_phase == 1: 原地对准 yaw
                {
                    sc.ClassicWalk(true);
                    double align_vyaw = (aruco_detected)
                        ? std::max(-0.35, std::min(0.35, -0.30 * aruco_angle))
                        : 0.0;
                    sc.Move(0, 0, align_vyaw);
                    s0_align_cnt++;

                    if (s0_align_cnt % 10 == 0)
                        cout << "[S0-ALIGN] cnt=" << s0_align_cnt
                             << " aruco_angle=" << aruco_angle
                             << " vyaw=" << align_vyaw << endl;

                    // 对准完成 / 超时 / ArUco丢失 → 爬
                    if ((aruco_detected && fabs(aruco_angle) < 0.05)
                        || s0_align_cnt > 50
                        || !aruco_detected)
                    {
                        sc.StopMove();
                        cout << "[S0-ALIGN→1] done → CLIMB" << endl;
                        stair_cnt   = 0;
                        s0_phase    = 0;
                        stair_step  = 1;
                    }
                }
            }
            else if (stair_step == 1)  // ★ 连续爬三层, 多条件 OR 兜底
            {
                static int s1_settle = 0;  // S1 航向稳定计数器
                s1_settle++;

                // 前 6 帧抑制航向修正 (StopMove→ClassicWalk 切换时 IMU yaw 有跳变)
                // 第 6 帧重新记录航向基准
                double s1_hdg = heading_corr;
                if (s1_settle <= 6)
                {
                    s1_hdg = 0;
                    if (s1_settle == 6)
                    {
                        s1_start_yaw = yaw;
                        cout << "[S1] YAW RELOCK: " << s1_start_yaw << endl;
                    }
                }
                // 左脚打滑检测: roll 突然变负(左倾) → 短暂右移压回去
                // 与之前雷达侧向居中的区别: 只在对侧倾有反应, 不平白干扰爬升
                double roll_corr = 0;
                if (s1_settle > 6 && fabs(roll) > 0.10)
                {
                    roll_corr = 0.15 * roll;   // roll<0(左倾)→vy<0(右移)
                    roll_corr = std::max(-0.18, std::min(0.18, roll_corr));
                }

                sc.ClassicWalk(true);
                sc.Move(0.15, roll_corr, s1_hdg);
                // 欧氏距离 (px 和 py 都可能有累积误差, 用 2D 距离)
                double dpx = px - px_start;
                double dpy = py - py_start;
                double d2d = sqrt(dpx*dpx + dpy*dpy);

                // 首次 ob_x > 1.5 时记录帧号
                if (obx_far_at == 0 && isfinite(ob_x) && ob_x > 1.5)
                    obx_far_at = stair_cnt;

                if (stair_cnt % 10 == 0)
                    cout << "[S1] cnt=" << stair_cnt
                         << " d2d=" << d2d
                         << " ob_x=" << ob_x
                         << " roll=" << roll << " rcorr=" << roll_corr
                         << " obx_far_at=" << obx_far_at << endl;

                // ★ 三条件 OR, 任一满足 → 先多走一点再后退 → 然后转弯
                bool A = (d2d > 1.13);                                          // 里程 (多走 ~8cm)
                bool B = (obx_far_at > 0 && stair_cnt > obx_far_at + 158);     // lidar+延时
                bool C = (stair_cnt > 371);                                     // 帧数兜底

                if (A || B || C)
                {
                    cout << "[S1→2] A=" << A << " B=" << B << " C=" << C
                         << " d2d=" << d2d
                         << " obx_far_at=" << obx_far_at
                         << " cnt=" << stair_cnt
                         << " → BACKUP" << endl;
                    stair_cnt = 0;
                    stair_step = 2;  // 先去后退步, 再转
                }
            }
            else if (stair_step == 2)  // 先停稳 (30帧) → 微退 (25帧)
            {
                if (stair_cnt < 30)      // 停 30 帧 (~0.6s) 站稳
                {
                    sc.StopMove();
                }
                else
                {
                    sc.ClassicWalk(true);
                    sc.Move(-0.08, 0, heading_corr);   // 慢速微退, 同时 IMU 锁航向
                }

                if (stair_cnt % 10 == 0)
                    cout << "[S2] cnt=" << stair_cnt
                         << " d2d=" << sqrt(pow(px-px_start,2)+pow(py-py_start,2)) << endl;

                if (stair_cnt > 55)      // 停30帧 + 退25帧 = 55帧
                {
                    sc.StopMove();
                    cout << "[S2→5] → TURN" << endl;
                    stair_cnt = 0;
                    stair_step = 5;
                }
            }
            else if (stair_step == 5)  // ★★★ 边走边左转90°, 后腿自然拖上顶层 ★★★
            {
                static double target_yaw      = 0;
                static bool   turn_inited     = false;
                static double px_turn_start   = 0;
                static double py_turn_start   = 0;

                if (!turn_inited)
                {
                    target_yaw      = yaw + M_PI / 2.0;  // +90° = 左转
                    turn_inited     = true;
                    stair_cnt       = 0;
                    px_turn_start   = px;
                    py_turn_start   = py;
                    cout << "[S5] TURN START  yaw=" << yaw
                         << "  target=" << target_yaw << endl;
                }

                // 1. 角度误差归一化
                double err = target_yaw - yaw;
                if (err >  M_PI) err -= 2.0 * M_PI;
                if (err < -M_PI) err += 2.0 * M_PI;

                // 2. 比例控制
                const double k_p = 0.45;
                double vyaw = k_p * err;
                vyaw = std::max(-0.55, std::min(0.55, vyaw));

                // 3. 前进速度: 极小 vx, 接近原地转, 左后腿有时间侧面抬腿
                double abs_err = fabs(err);
                double vx;
                if      (abs_err > 1.0)  vx = 0.04;  // 大角度: 接近原地转
                else if (abs_err > 0.3)  vx = 0.08;  // 中角度: 慢走慢转
                else                      vx = 0.12;  // 小角度: 收尾

                // 4. 转弯期间前进距离
                double dpx = px - px_turn_start;
                double dpy = py - py_turn_start;
                double d2d_turn = sqrt(dpx*dpx + dpy*dpy);

                // 5. 执行
                sc.ClassicWalk(true);
                sc.Move(vx, 0, vyaw);

                if (stair_cnt % 10 == 0)
                    cout << "[S5] cnt=" << stair_cnt
                         << " err=" << err << " vyaw=" << vyaw
                         << " vx=" << vx << " d2d=" << d2d_turn
                         << " yaw=" << yaw << endl;

                // 6. 退出: 角度到位 + 走了足够远(后腿拖上来了) + 最小时间
                bool angle_ok = (fabs(err) < 0.08);
                bool rear_ok  = (d2d_turn > 0.25);
                bool min_time = (stair_cnt > 45);

                if (angle_ok && rear_ok && min_time)
                {
                    sc.StopMove();
                    cout << "[S5] DONE  err=" << err
                         << " d2d=" << d2d_turn
                         << " → DESCEND" << endl;
                    turn_inited = false;
                    s5_timed_out = false;
                    stair_cnt   = 0;
                    stair_step  = 6;
                }
                else if (stair_cnt > 200)
                {
                    sc.StopMove();
                    cout << "[S5] TIMEOUT  err=" << err
                         << " → DESCEND with drift" << endl;
                    turn_inited = false;
                    s5_timed_out = true;
                    stair_cnt   = 0;
                    stair_step  = 6;
                }
            }
            else if (stair_step == 6)  // 下第一级 (顶层→中层)
            {
                double descend_vyaw = s5_timed_out ? 0.04 : 0;
                sc.ClassicWalk(true);
                sc.Move(0.10, 0, descend_vyaw);

                bool dropped = (peak_pitch > 0.30);
                bool settled = (fabs(pitch) < 0.15 && stair_cnt > 20);

                if (dropped && settled)
                    settle_cnt++;
                else
                    settle_cnt = 0;

                // 快速退出: 走了足够远且前方无台阶遮挡 → 已经在地面
                bool on_ground = (stair_cnt > 30 && (!isfinite(ob_x) || ob_x > 2.0));

                if (settle_cnt > 12 || on_ground || stair_cnt > 250)
                {
                    sc.StopMove();
                    stair_cnt = 0;
                    stair_step = (on_ground) ? 9 : 7;
                }
            }
            else if (stair_step == 7)  // 下第二级 (中层→底层)
            {
                double descend_vyaw = s5_timed_out ? 0.04 : 0;
                sc.ClassicWalk(true);
                sc.Move(0.10, 0, descend_vyaw);

                bool dropped = (peak_pitch > 0.30);
                bool settled = (fabs(pitch) < 0.15 && stair_cnt > 20);

                if (dropped && settled)
                    settle_cnt++;
                else
                    settle_cnt = 0;

                bool on_ground = (stair_cnt > 30 && (!isfinite(ob_x) || ob_x > 2.0));

                if (settle_cnt > 12 || on_ground || stair_cnt > 250)
                {
                    sc.StopMove();
                    stair_cnt = 0;
                    stair_step = (on_ground) ? 9 : 8;
                }
            }
            else if (stair_step == 8)  // 下第三级 (底层→地面)
            {
                double descend_vyaw = s5_timed_out ? 0.04 : 0;
                sc.ClassicWalk(true);
                sc.Move(0.10, 0, descend_vyaw);

                bool dropped = (peak_pitch > 0.30);
                bool settled = (fabs(pitch) < 0.15 && stair_cnt > 20);

                if (dropped && settled)
                    settle_cnt++;
                else
                    settle_cnt = 0;

                bool on_ground = (stair_cnt > 30 && (!isfinite(ob_x) || ob_x > 2.0));

                if (settle_cnt > 12 || on_ground || stair_cnt > 250)
                {
                    sc.StopMove();
                    stair_cnt = 0;
                    stair_step = 9;
                }
            }
            else if (stair_step == 9)  // 下完台阶, 再往前走一段确保 clearance
            {
                sc.ClassicWalk(true);
                sc.Move(0.15, 0, 0);
                if (stair_cnt > 25)
                {
                    sc.StopMove();
                    stair_cnt  = 0;
                    stair_step = 0;
                    Flag_Task  = 6;   // → 下一任务
                }
            }
            else if (stair_step == 10)  // 倾斜恢复
            {
                if (stair_cnt < 30)
                {
                    sc.StopMove();
                }
                else
                {
                    double recover_dir   = (pitch > 0) ? 0.08 : -0.08;
                    double side_recover  = 0;
                    if (fabs(roll) > 0.2)
                        side_recover = (roll > 0) ? -0.08 : 0.08;

                    sc.Move(recover_dir, side_recover, 0);

                    if (fabs(pitch) < 0.1 && fabs(roll) < 0.1 && stair_cnt > 60)
                    {
                        sc.StopMove();
                        stair_cnt  = 0;
                        stair_step = 9;  // 恢复后走一段确保离开台阶区域
                    }
                }
            }
            else
            {
                sc.StopMove();
                Flag_Task = 3;
            }
        }
        break;
        
        case 3: /* 前进直至碰到下一个aruco并右转 */
        {
            // 切换为常规步态
            sc.StaticWalk();
            // 前进直至碰到下一个aruco并右转
            Flag_Task = 4;
        }
        break;

        case 4: /* 左转并走向终点 */
        {
            // 左转并走向终点
            Flag_Task = 5;
        }
        break;

        case 5: /* 跳进终点区域 */
        {
            // if (end_jump_times == 0)
            // {
            //     sc.FrontJump();
            //     end_jump_times++;
            // }
            Flag_Task = 6;
        }

        case 6: /* 终点区 —— stop and celebrate */
            sc.StopMove();
            cout << "Mission complete 🎉\n";
            return 0;
        }

        imshow("Go2 Front Cam", undist);
        if (waitKey(1) == 27)
            break; // ESC to quit
    }
    sc.StopMove();

    return 0;
}
