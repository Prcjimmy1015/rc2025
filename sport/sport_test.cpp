#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/vui/vui_client.hpp>
#include <unistd.h>

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  //argv[1]由终端传入，为机器人连接的网卡名称

  //创建sport client对象
  unitree::robot::go2::SportClient sport_client;
  sport_client.SetTimeout(10.0f);//超时时间
  sport_client.Init();

  //创建vui client对象
  unitree::robot::go2::VuiClient vui_client;
  vui_client.SetTimeout(10.0f);
  vui_client.Init();

  sport_client.Hello(); //打招呼动作
  sleep(4);//延迟4s
  sport_client.Stretch(); //伸懒腰动作
  sleep(4);//延迟4s

  //灯光闪烁三次
  for (int i = 0; i < 3; i++)
  {
    vui_client.SetBrightness(10);
    usleep(400000);
    vui_client.SetBrightness(0);
    usleep(400000);
  }
  vui_client.SetBrightness(0);//恢复中等亮度

  return 0;
}
