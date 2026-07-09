/**
 * test_task.cpp — 独立测试入口
 *
 * 用法:
 *   ./build/test_task eth0 --Turn LEFT --gui
 *   ./build/test_task eth0 --Turn RIGHT --gui
 *   ./build/test_task eth0 --Warn --gui          (警告标志识别 + 动作)
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

void sig_handler(int s) { if (s == SIGINT) exit(0); }

int main(int ac, char **av)
{
    if (ac < 2) {
        cerr << "Usage: " << av[0] << " <eth_if> [--gui] [--Turn LEFT|RIGHT] [--Warn]\n";
        return -1;
    }
    const char *eth = av[1];
    string mode;
    for (int i = 2; i < ac; ++i) {
        string a = av[i];
        if (a == "--gui") g_enable_gui = true;
        else if (a == "--Turn" && i+1 < ac) { mode = "Turn" + string(av[++i]); }
        else if (a == "--Warn") mode = "Warn";
    }

    signal(SIGINT, sig_handler);
    ChannelFactory::Instance()->Init(0, eth);

    if (mode == "Warn")
    {
        // ─── 干净模式：仅 Sport + Vui + 临时摄像头 ───
        go2::SportClient sc;
        sc.SetTimeout(10.0f); sc.Init();

        go2::VuiClient vc;
        vc.SetTimeout(10.0f); vc.Init();

        // 临时打开摄像头 -> 拍摄一帧 -> 立即释放
        cv::VideoCapture cap;
        string gst = string("udpsrc address=230.1.1.1 port=1720 multicast-iface=") + eth +
                     " ! application/x-rtp, media=video, encoding-name=H264 "
                     "! rtph264depay ! avdec_h264 ! videoconvert "
                     "! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1";
        if (!cap.open(gst, cv::CAP_GSTREAMER)) {
            cerr << "[Test] 摄像头打开失败\n";
            return -1;
        }
        cv::Mat dummy; cap.read(dummy); cap.read(dummy);

        cv::Mat frame; cap.read(frame);
        cap.release(); // 立即释放，后续动作执行时 DDS 通道完全空闲

        int wid = dogDetectWarningMarker(frame);
        cout << "[Test] 警告标志: " << wid << endl;
        dogDoAlertAction(sc, vc, wid);

        sc.StopMove();
        sc.SwitchJoystick(true); sc.RecoveryStand();
        this_thread::sleep_for(chrono::milliseconds(500)); sc.BalanceStand();
        cout << "[Test] 退出。\n";
        return 0;
    }

    // ─── Turn 模式：使用完整 AppRuntime ───
    AppRuntime rt;
    if (!initAppRuntime(rt, eth)) { cerr << "Camera fail\n"; return -1; }

    go2::SportClient &sc = rt.sc;
    cv::VideoCapture &cap = rt.cap;
    go2::VuiClient vc;
    vc.SetTimeout(10.0f); vc.Init();

    cv::Mat dummy2; cap.read(dummy2); cap.read(dummy2);
    cout << "[Test] 就绪\n";

    if (mode == "TurnLEFT")  { dogTurn90Degrees(sc, cap, +1); }
    else if (mode == "TurnRIGHT") { dogTurn90Degrees(sc, cap, -1); }
    else { cerr << "[Test] 用法: --Turn LEFT|RIGHT, --Warn\n"; }

    sc.StopMove();
    sc.SwitchJoystick(true); sc.RecoveryStand();
    this_thread::sleep_for(chrono::milliseconds(500)); sc.BalanceStand();
    cout << "[Test] 退出。\n";
    return 0;
}