#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

int main(int argc, char *argv[])
{
    if(argc != 4)
    {
        printf("用法:%s <id> <angle> <delay>\n", argv[0]);
        printf("示例：%s 0 90.0 1000\n", argv[0]);
        return 1;
    }
    int id = atoi(argv[1]);
    double angle = atof(argv[2]);
    int delay = atoi(argv[3]);
    printf("收到参数：id = %d, angle = %.2f, delay = %d\n", id, angle, delay);
    ChannelFactory::Instance()->Init(0);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    unitree_arm::msg::dds_::ArmString_ msg{};
    char cmd_str[256];
    snprintf(cmd_str, sizeof(cmd_str), "{\"seq\":4,\"address\":1,\"funcode\":1,\"data\":{\"id\":%d,\"angle\":%.1f,\"delay_ms\":%d}}", id, angle, delay);
    msg.data_() = cmd_str;
    publisher.Write(msg);    
    printf("已发送指令：%s\n", cmd_str);
 
    return 0;
}

 
