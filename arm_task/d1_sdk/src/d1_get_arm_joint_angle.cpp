#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <iostream>
#include <atomic>
#include <chrono>
#include <unistd.h>

#include "msg/PubServoInfo_.hpp"

#define TOPIC_SERVO "current_servo_angle"
#define WAIT_TIMEOUT_MS 3000

using namespace unitree::robot;
using namespace unitree::common;
using namespace unitree_arm::msg::dds_;

std::atomic<bool> g_got_data = false;
double g_joints[7] = {0};

void ServoHandler(const void* msg)
{
    const PubServoInfo_* pm = (const PubServoInfo_*)msg;
    g_joints[0] = pm->servo0_data_();
    g_joints[1] = pm->servo1_data_();
    g_joints[2] = pm->servo2_data_();
    g_joints[3] = pm->servo3_data_();
    g_joints[4] = pm->servo4_data_();
    g_joints[5] = pm->servo5_data_();
    g_joints[6] = pm->servo6_data_();
    g_got_data = true;
}

int main()
{
    ChannelFactory::Instance()->Init(0, "ens37");
    ChannelSubscriber<PubServoInfo_> sub(TOPIC_SERVO);
    sub.InitChannel(ServoHandler);

    auto start = std::chrono::steady_clock::now();
    while (true)
    {
        if (g_got_data) break;

        auto cost = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start
        ).count();
        if (cost >= WAIT_TIMEOUT_MS)
        {
            std::cout << ",,,,,,,\n";
            ChannelFactory::Instance()->Release();
            return 1;
        }
        // 20毫秒休眠，usleep单位微秒
        usleep(20000);
    }

    std::cout << g_joints[0] << ","
              << g_joints[1] << ","
              << g_joints[2] << ","
              << g_joints[3] << ","
              << g_joints[4] << ","
              << g_joints[5] << ","
              << g_joints[6] << std::endl;

    ChannelFactory::Instance()->Release();
    return 0;
}

