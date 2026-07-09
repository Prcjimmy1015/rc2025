/**
 * test_task.cpp — 独立测试入口
 *
 * 直接调用 arm_bridge.h 中的函数，不经过完整的 case 状态机流程。
 * 方便调试单个子模块。
 * 
 * 比赛前移除:
 *   rm go2_runner/test_task.cpp
 *   同时从 CMakeLists.txt 删除 test_task 编译目标相关行
 *
 * 用法:
 *   ./build/test_task eth0 --task 1 --marker 1 --gui
 *   ./build/test_task eth0 --task 2 --gui
 *   ./build/test_task eth0 --task 3 --target 1 --gui
 *   ./build/test_task eth0 --Turn LEFT --gui
 *   ./build/test_task eth0 --Turn RIGHT --gui
 *   ./build/test_task eth0 --Align 0.35 --gui       (比值对齐，robot 须在平台上)
 *   ./build/test_task eth0 --Detect1 --marker 1      (仅机械臂检测+比值)
 *   ./build/test_task eth0 --Detect2                  (仅机械臂检测+比值)
 */

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/vui/vui_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <csignal>
#include "app_runtime.h"
#include "globals.h"
#include "params.h"
#include "../arm_task/arm_bridge.h"

using namespace unitree::robot;
using namespace std;
static atomic<bool> g_exit(false);

void sig_handler(int s) { if (s == SIGINT) g_exit = true; }

int main(int ac, char **av)
{
    if (ac < 2) {
        cerr << "Usage: " << av[0] << " <eth_if> [--gui] [--task N] [--marker N] [--target N]\n"
             << "                    [--Turn LEFT|RIGHT] [--Align 0.35] [--Detect1] [--Detect2]\n";
        return -1;
    }
    const char *eth = av[1];
    string mode; float align_ratio = 0.5f;
    int task_num = 0, marker_id = 1, target_platform = 1;
    for (int i = 2; i < ac; ++i) {
        string a = av[i];
        if (a == "--gui") g_enable_gui = true;
        else if (a == "--task" && i+1 < ac) task_num = atoi(av[++i]);
        else if (a == "--marker" && i+1 < ac) marker_id = atoi(av[++i]);
        else if (a == "--target" && i+1 < ac) target_platform = atoi(av[++i]);
        else if (a == "--Turn" && i+1 < ac) { mode = "Turn"; align_ratio = (string(av[++i])=="LEFT")?1:-1; }
        else if (a == "--Align" && i+1 < ac) { mode = "Align"; align_ratio = atof(av[++i]); }
        else if (a == "--Detect1") mode = "Detect1";
        else if (a == "--Detect2") mode = "Detect2";
        else { cerr << "Unknown: " << a << "\n"; return -1; }
    }

    signal(SIGINT, sig_handler);
    ChannelFactory::Instance()->Init(0, eth);

    AppRuntime rt;
    if (!initAppRuntime(rt, eth)) { cerr << "Camera fail\n"; return -1; }
    g_orig_px = px; g_orig_py = py; g_orig_yaw = yaw;
    px0 = px; py0 = py; yaw0 = yaw;

    go2::SportClient &sc = rt.sc;
    go2::ObstaclesAvoidClient &avc = rt.avoid_client;
    cv::VideoCapture &cap = rt.cap;
    go2::VuiClient vc;
    vc.SetTimeout(10.0f); vc.Init();

    // ============================ 执行所选测试 ============================

    if (task_num == 1) {
        cout << "[Test] 执行 Task 1 (marker=" << marker_id << ")\n";
        dogTask1Execute(sc, cap, vc, marker_id, &yaw);
    }
    else if (task_num == 2) {
        cout << "[Test] 执行 Task 2\n";
        dogTask2Execute(sc, cap, vc, &yaw);
    }
    else if (task_num == 3) {
        cout << "[Test] 执行 Task 3 (target=" << target_platform << ")\n";
        dogTask3Execute(sc, cap, vc, target_platform, &yaw);
    }
    else if (mode == "Turn") {
        int dir = (align_ratio >= 0) ? 1 : -1;
        cout << "[Test] 转弯 90° " << (dir>0?"LEFT":"RIGHT") << "\n";
        dogTurn90Degrees(sc, dir, &yaw);
    }
    else if (mode == "Align") {
        cout << "[Test] 比值对齐 target=" << align_ratio << "\n";
        dogAlignToPlatform(sc, cap, align_ratio, &yaw);
    }
    else if (mode == "Detect1") {
        cout << "[Test] Stage 1 detect (marker=" << marker_id << ")\n";
        int class_id; float wx, wz, depth;
        float ratio = armStage1Detect(marker_id, class_id, wx, wz, depth);
        cout << "[Test] ratio=" << ratio << " class_id=" << class_id << "\n";
        cv::Mat arm_view = cv::imread(ARM_ANNOTATED_IMAGE_PATH);
        if (!arm_view.empty()) { cv::imshow("Arm D435", arm_view); cv::waitKey(0); }
    }
    else if (mode == "Detect2") {
        cout << "[Test] Stage 2 detect\n";
        int class_id; float wx, wz, depth;
        float ratio = armStage2Detect(class_id, wx, wz, depth);
        cout << "[Test] ratio=" << ratio << " class_id=" << class_id << "\n";
        cv::Mat arm_view = cv::imread(ARM_ANNOTATED_IMAGE_PATH);
        if (!arm_view.empty()) { cv::imshow("Arm D435", arm_view); cv::waitKey(0); }
    }
    else {
        cerr << "[Test] 未指定测试模式。可用: --task 1|2|3, --Turn, --Align, --Detect1, --Detect2\n";
    }

    // cleanup
    sc.StopMove();
    avc.UseRemoteCommandFromApi(false); avc.SwitchSet(false); avc.Move(0,0,0);
    this_thread::sleep_for(chrono::milliseconds(200));
    sc.SwitchJoystick(true); sc.RecoveryStand();
    this_thread::sleep_for(chrono::milliseconds(500)); sc.BalanceStand();
    cout << "[Test] 退出。\n";
    return 0;
}