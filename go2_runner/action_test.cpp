/**
 * action_test.cpp — 独立动作测试入口（不走 ONNX 推理）
 *
 * 用法:
 *   ./build/action_test eth0 stretch   (伸懒腰)
 *   ./build/action_test eth0 wave      (打招呼)
 *   ./build/action_test eth0 flash     (闪烁前灯)
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

void sig_handler(int s)
{
    if (s == SIGINT)
        exit(0);
}

int main(int ac, char **av)
{
    if (ac < 3)
    {
        cerr << "Usage: " << av[0] << " <eth_if> <stretch|wave|flash>\n";
        return -1;
    }
    const char *eth = av[1];
    string action = av[2];

    signal(SIGINT, sig_handler);
    ChannelFactory::Instance()->Init(0, eth);

    go2::SportClient sc;
    sc.SetTimeout(10.0f);
    sc.Init();

    go2::VuiClient vc;
    vc.SetTimeout(10.0f);
    vc.Init();

    cout << "[ActionTest] 就绪, 动作: " << action << endl;

    if (action == "stretch")
    {
        cout << "[ActionTest] 执行: 伸懒腰" << endl;
        sc.Stretch();
        sleep(4);
    }
    else if (action == "wave")
    {
        cout << "[ActionTest] 执行: 打招呼" << endl;
        sc.Hello();
        sleep(4);
    }
    else if (action == "flash")
    {
        cout << "[ActionTest] 执行: 闪烁前灯三次" << endl;
        for (int i = 0; i < 3; i++)
        {
            cout << "  [flash " << i << "] ON" << endl;
            vc.SetBrightness(255);
            usleep(500000);
            cout << "  [flash " << i << "] OFF" << endl;
            vc.SetBrightness(0);
            usleep(500000);
        }
        vc.SetBrightness(0);
        cout << "[ActionTest] 闪烁完成" << endl;
    }
    else
    {
        cerr << "[ActionTest] 未知动作: " << action << "\n";
        cerr << "  可用: stretch, wave, flash\n";
        return -1;
    }

    sc.StopMove();
    sc.SwitchJoystick(true);
    sc.RecoveryStand();
    this_thread::sleep_for(chrono::milliseconds(500));
    sc.BalanceStand();
    cout << "[ActionTest] 退出。" << endl;
    return 0;
}