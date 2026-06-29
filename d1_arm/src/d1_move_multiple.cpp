#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

#define TOPIC "rt/arm_Command"

using namespace unitree::robot;
using namespace unitree::common;

int main(int argc, char *argv[])
{
    if(argc != 8)
    {
        printf("用法:%s <angle0> <angle1> <angle2> <angle3><angle4> <angle5><angle6>\n", argv[0]);
        printf("示例：%s 0 -60 60 0 30 0 0\n", argv[0]);
        return 1;
    }
    double angle0 = atof(argv[1]);
    double angle1 = atof(argv[2]);
    double angle2 = atof(argv[3]);
    double angle3 = atof(argv[4]);
    double angle4 = atof(argv[5]);
    double angle5 = atof(argv[6]);
    double angle6 = atof(argv[7]);
    ChannelFactory::Instance()->Init(0, "ens37");
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    unitree_arm::msg::dds_::ArmString_ msg{};
    char cmd_str[256];
    snprintf(cmd_str, sizeof(cmd_str), "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1,\"angle0\":%.1f,\"angle1\":%.1f,\"angle2\":%.1f,\"angle3\":%.1f,\"angle4\":%.1f,\"angle5\":%.1f,\"angle6\":%.1f}}", angle0, angle1, angle2, angle3, angle4, angle5, angle6);
    msg.data_() = cmd_str;
    publisher.Write(msg);
    printf("已发送指令：%s\n", cmd_str);
 
    return 0;
}
