#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <csignal>
#include <atomic>
#include <vector>
#include <cmath>
#include "params.h"
#include "globals.h"
#include "utils.h"
#include "aruco_server.h"
#include "app_runtime.h"
#include "cases/case0.h"
#include "cases/case1.h"
#include "cases/case2.h"
#include "cases/case3.h"
#include "cases/case4.h"
using namespace unitree::robot;using namespace cv;using namespace std;
static atomic<bool> g_exit(false);
void sig(int s){if(s==SIGINT){cout<<"\n[SIGINT]\n";g_exit=true;}}

static void t0(go2::SportClient &sc,Mat &f){
static bool once=false;if(!once){cout<<"\n=== V22_CONTINUITY ===\n"<<endl;once=true;}
static int cnt=0;cnt++;
static bool settled=false;static int n_st=0;static double yaw_settle=0;
if(!settled){n_st++;sc.StaticWalk();sc.Euler(0,0.8,0);if(n_st==1)yaw_settle=yaw;
double yd=yaw-yaw_settle;if(yd>M_PI)yd-=2*M_PI;if(yd<-M_PI)yd+=2*M_PI;
double steer=-yd*2.0;steer=max(-0.3,min(0.3,steer));sc.Move(0,0,steer);
if(n_st>=30){settled=true;cout<<"[V22] Settled, go.\n"<<endl;}return;}

sc.Euler(0,0.8,0);
Mat g,b,n;cvtColor(f,g,COLOR_BGR2GRAY);GaussianBlur(g,b,{5,5},0);
threshold(b,n,50,255,THRESH_BINARY_INV);
{Mat k=getStructuringElement(MORPH_RECT,Size(3,3));morphologyEx(n,n,MORPH_OPEN,k);}
int rh=100,roiy=b.rows-rh;if(roiy<0)roiy=0;double e=0;int ci=0,rw=n.cols;vector<int>cc(rw,0);
for(int r=roiy;r<b.rows;++r){const uchar*row=n.ptr(r);for(int x=0;x<rw;++x)if(row[x]){cc[x]++;ci++;}}
static int last_pc=640;
int win_l=max(0,last_pc-300),win_r=min(rw-1,last_pc+300);
int pk=0;for(int x=win_l;x<=win_r;++x)if(cc[x]>pk)pk=cc[x];
int pc=-1;
if(pk>=5){
    double sum_w=0,sum_wx=0;
    for(int x=win_l;x<=win_r;++x)if(cc[x]>pk*0.5){sum_w+=cc[x];sum_wx+=cc[x]*x;}
    if(sum_w>0)pc=(int)(sum_wx/sum_w);
}
if(pc<0){pc=last_pc;}
bool ok=(pk>=5&&ci>=50&&ci<=100000);if(ok)e=pc-640;
if(ok&&ci>5000&&pk>80)last_pc=pc;
if(ok){int cx=max(0,min(1279,pc));int cy=b.rows-rh/2;
circle(f,Point(cx,cy),10,Scalar(0,255,0),-1);line(f,Point(cx,cy+25),Point(cx,cy-25),Scalar(0,255,0),2);}

double pcross=(ci>0)?pk*100.0/ci:999;
bool is_cross=(ci>45000 && pcross<0.25 && abs(e)<400);
bool is_sharp=(abs(e)>400);

if(cnt%15==0){
    const char*tag="NORM";
    if(is_cross)tag="CROSS";
    else if(is_sharp)tag="SHARP";
    else if(!ok)tag="NOLINE";
    printf("[V22] %s err=%.0f ci=%d pk=%d cr=%.2f%%\n",tag,e,ci,pk,pcross);
}

double ly=0;{double _,dy;transformLocal(px,py,yaw,_,ly,dy);}double lc=(ly>0.35)?-0.3:(ly<-0.35)?0.3:0;

static int sharp_burst=0;
static int burst_cooldown=0;
if(is_sharp&&sharp_burst==0&&burst_cooldown==0){sharp_burst=30;}
if(burst_cooldown>0)burst_cooldown--;
if(sharp_burst>0){
    sharp_burst--;
    double s=-e*0.12;s=max(-1.0,min(1.0,s));
    sc.Move(0,0,s);
    if(cnt%15==0)printf("[V22] >> BURST %d/30 s=%.2f\n",30-sharp_burst,s);
    if(sharp_burst==0)burst_cooldown=15;
}else if(is_cross){
    sc.Move(0.15,0,0);
}else{
    if(ok){
        double tg=e/1280.0*60.0*M_PI/180.0;
        double s=-tg*6.0;s=max(-1.0,min(1.0,s));
        double vx=(abs(e)>300)?0.08:0.12;
        double vy=e*0.0006;vy=max(-0.15,min(0.15,vy));
        sc.Move(vx,vy,s);
    }else{
        sc.Move(0,0,0.3);  // 原地左转，搜索线条
    }
}}

int main(int ac,char**av){
if(ac<2){cerr<<"Usage: "<<av[0]<<" <eth_if> [--gui] [--task N]\n";return -1;}
const char*eth=av[1];
for(int i=2;i<ac;++i){string a=av[i];if(a=="--gui")g_enable_gui=true;else if(a=="--task"&&i+1<ac){g_force_task=atoi(av[++i]);if(g_force_task==0){g_case0_skip_init=true;g_case0_second_pass=true;}cout<<"[Config] task:"<<g_force_task<<endl;}else{cerr<<"Unknown: "<<a<<"\n";return -1;}}
signal(SIGINT,sig);ChannelFactory::Instance()->Init(0,eth);AppRuntime rt;if(!initAppRuntime(rt,eth)){cerr<<"Camera fail\n";return -1;}
// 原点固定为开机位置，永不重置
px0=0;py0=0;yaw0=0;thread t(aruco_socket_server,5005);t.detach();cout<<(g_enable_gui?"GUI\n":"Headless\n")<<flush;
go2::SportClient &sc=rt.sc;go2::ObstaclesAvoidClient &avc=rt.avoid_client;VideoCapture &cap=rt.cap;Mat frame,undist;int fc=0;auto t0t=chrono::steady_clock::now();
while(!g_exit){if(!cap.read(frame)||frame.empty())break;fc++;undistort(frame,undist,K,D);if(g_force_task>=0)Flag_Task=g_force_task;
if(g_case0_skip_init){t0(sc,undist);if(g_enable_gui){double fps=fc/chrono::duration<double>(chrono::steady_clock::now()-t0t).count();putText(undist,format("V22 FPS %.1f",fps),{10,30},FONT_HERSHEY_SIMPLEX,1,{0,255,0},2);imshow("Go2",undist);int key=waitKey(1);if(key==27)break;if(key=='r'){double rlx,rly,rdyaw;transformLocal(px,py,yaw,rlx,rly,rdyaw);static int recn=0;printf("\n[RECORD] #%d: lx=%.2f ly=%.2f yaw=%.3f\n\n",++recn,rlx,rly,yaw);}}continue;}
double lx,ly,dyaw;transformLocal(px,py,yaw,lx,ly,dyaw);
switch(Flag_Task){case 0:{int ret=case0_tick(sc,undist,rt.stateCB.state,fc);if(g_force_task<0){if(ret==1){Flag_Task=1;g_case0_second_pass=false;case1_reset_statics();}else if(ret==2){Flag_Task=2;g_case0_second_pass=false;case2_reset();}}break;}
case 1:if(g_force_task<0&&case1_tick(sc,fc,lx,ly,yaw)){Flag_Task=0;g_case0_second_pass=true;case0_reset_statics();}break;
case 2:if(g_force_task<0&&case2_tick(sc))Flag_Task=3;break;
case 3:case 4:case 5:case 6:case 7:case 8:if(g_force_task<0&&case3_tick(sc,lx,ly,dyaw))Flag_Task=9;break;
case 9:if(case4_tick(sc,avc))return 0;break;}
if(g_enable_gui){double fps=fc/chrono::duration<double>(chrono::steady_clock::now()-t0t).count();putText(undist,format("V22 FPS %.1f",fps),{10,30},FONT_HERSHEY_SIMPLEX,1,{0,255,0},2);imshow("Go2",undist);if(waitKey(1)==27)break;}}
sc.StopMove();avc.UseRemoteCommandFromApi(false);avc.SwitchSet(false);avc.Move(0,0,0);this_thread::sleep_for(chrono::milliseconds(200));sc.SwitchJoystick(true);sc.RecoveryStand();this_thread::sleep_for(chrono::milliseconds(500));sc.BalanceStand();cout<<"[Exit] Remote restored.\n";return 0;}