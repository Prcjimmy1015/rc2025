/**
 * test_task.cpp — 独立测试入口（无摄像头依赖）
 *
 * 用法:
 *   ./build/test_task eth0 --Turn LEFT
 *   ./build/test_task eth0 --Turn RIGHT
 *   ./build/test_task eth0 --Full 0         (完整流程: 左转→伸懒腰→后退→右转)
 *   ./build/test_task eth0 --Full 1         (完整流程: 左转→打招呼→右转)
 *   ./build/test_task eth0 --Full 2         (完整流程: 左转→闪烁前灯→右转)
 */

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/vui/vui_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <csignal>
#include "arm_task/arm_bridge.h"

using namespace unitree::robot;
using namespace std;

void sig_handler(int s) { if (s == SIGINT) exit(0); }

int main(int ac, char **av)
{
    if (ac < 2) {
        cerr << "Usage: " << av[0] << " <eth_if> [--Turn LEFT|RIGHT] [--Full 0|1|2]\n";
        return -1;
    }
    const char *eth = av[1];
    string mode;
    for (int i = 2; i < ac; ++i) {
        string a = av[i];
        if (a == "--Turn" && i+1 < ac) { mode = "Turn" + string(av[++i]); }
        else if (a == "--Full" && i+1 < ac) { mode = "Full" + string(av[++i]); }
    }

    signal(SIGINT, sig_handler);
    ChannelFactory::Instance()->Init(0, eth);

    go2::SportClient sc;
    sc.SetTimeout(10.0f); sc.Init();

    go2::VuiClient vc;
    vc.SetTimeout(10.0f); vc.Init();

    cout << "[Test] 就绪\n";

    if (mode == "Full0" || mode == "Full1" || mode == "Full2")
    {
        int action_id = mode[4] - '0';
        dogFullTaskManual(sc, vc, action_id);
    }
    else if (mode == "TurnLEFT")
    {
        dogTurn90DegreesNoCam(sc, +1);
        sc.StopMove();
        sc.SwitchJoystick(true); sc.RecoveryStand();
        this_thread::sleep_for(chrono::milliseconds(500)); sc.BalanceStand();
    }
    else if (mode == "TurnRIGHT")
    {
        dogTurn90DegreesNoCam(sc, -1);
        sc.StopMove();
        sc.SwitchJoystick(true); sc.RecoveryStand();
        this_thread::sleep_for(chrono::milliseconds(500)); sc.BalanceStand();
    }
    else
    {
        cerr << "[Test] 用法: --Turn LEFT|RIGHT, --Full 0|1|2\n";
        return -1;
    }

    cout << "[Test] 退出。" << endl;
    return 0;
}