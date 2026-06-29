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

static void t0(go2::SportClient &sc, Mat &f){
static bool once=false;if(!once){cout<<"\n=== V7_SLOW_DEEPER ===\n"<<endl;once=true;}
static int cnt=0;cnt++;
static bool settled=false;static int n_st=0;static double yaw_settle=0;
if(!settled){n_st++;sc.StaticWalk();sc.Euler(0,0.8,0);if(n_st==1)yaw_settle=yaw;
double yd=yaw-yaw_settle;if(yd>M_PI)yd-=2*M_PI;if(yd<-M_PI)yd+=2*M_PI;
double steer=-yd*2.0;steer=max(-0.3,min(0.3,steer));sc.Move(0,0,steer);
if(n_st>=30){settled=true;cout<<"[V7] Settled, go.\n"<<endl;}return;}

Mat g,b,n;cvtColor(f,g,COLOR_BGR2GRAY);GaussianBlur(g,b,{5,5},0);
threshold(b,n,50,255,THRESH_BINARY_INV);
{Mat k=getStructuringElement(MORPH_RECT,Size(3,3));morphologyEx(n,n,MORPH_OPEN,k);}
int rh=100,roiy=b.rows-rh;if(roiy<0)roiy=0;double e=0;int ci=0,rw=n.cols;vector<int>cc(rw,0);
for(int r=roiy;r<b.rows;++r){const uchar*row=n.ptr(r);for(int x=0;x<rw;++x)if(row[x]){cc[x]++;ci++;}}
int pc=-1,pk=0;for(int x=0;x<rw;++x)if(cc[x]>pk){pk=cc[x];pc=x;}
bool ok=(pk>=5&&ci>=50&&ci<=50000);if(ok)e=pc-640;
if(ok){int cx=max(0,min(1279,pc));int cy=b.rows-rh/2;
circle(f,Point(cx,cy),10,Scalar(0,255,0),-1);line(f,Point(cx,cy+25),Point(cx,cy-25),Scalar(0,255,0),2);}
if(cnt%15==0){if(ok)printf("[V7] LINE err=%.0f ci=%d pk=%d\n",e,ci,pk);else printf("[V7] NO LINE ci=%d pk=%d\n",ci,pk);}
double ly=0;{double _,dy;transformLocal(px,py,yaw,_,ly,dy);}double lc=(ly>0.35)?-0.3:(ly<-0.35)?0.3:0;
if(ok&&abs(e)<400&&ci>100){double tg=e/1280.0*60.0*M_PI/180.0;double s=-tg*3.0;s=max(-0.8,min(0.8,s));if(abs(lc)>0.01)s=lc;s=max(-0.8,min(0.8,s));double vx=(abs(e)>300)?0.10:0.15;sc.Move(vx,0,s);if(cnt%15==0)printf("[V7] servo s=%.2f vx=%.2f\n",s,vx);}
else if(ok){double tg=e/1280.0*60.0*M_PI/180.0;if(abs(e)>=400){double s=max(-0.8,min(0.8,-tg*3.0));sc.Move(0,0,s);if(cnt%15==0)printf("[V7] STOP+TURN\n");}else{double s=max(-0.8,min(0.8,-tg*3.0));if(abs(lc)>0.01)s=lc;s=max(-0.8,min(0.8,s));sc.Move(0.12,0,s);}}
else{double s=max(-0.8,min(0.8,lc));sc.Move(0.12,0,s);}}

int main(int ac,char**av){
if(ac<2){cerr<<"Usage: "<<av[0]<<" <eth_if> [--gui] [--task N]\n";return -1;}
const char*eth=av[1];
for(int i=2;i<ac;++i){string a=av[i];if(a=="--gui")g_enable_gui=true;else if(a=="--task"&&i+1<ac){g_force_task=atoi(av[++i]);if(g_force_task==0){g_case0_skip_init=true;g_case0_second_pass=true;}cout<<"[Config] task:"<<g_force_task<<endl;}else{cerr<<"Unknown: "<<a<<"\n";return -1;}}
signal(SIGINT,sig);ChannelFactory::Instance()->Init(0,eth);AppRuntime rt;if(!initAppRuntime(rt,eth)){cerr<<"Camera fail\n";return -1;}
px0=px;py0=py;yaw0=yaw;thread t(aruco_socket_server,5005);t.detach();cout<<(g_enable_gui?"GUI\n":"Headless\n")<<flush;
go2::SportClient &sc=rt.sc;go2::ObstaclesAvoidClient &avc=rt.avoid_client;VideoCapture &cap=rt.cap;Mat frame,undist;int fc=0;auto t0t=chrono::steady_clock::now();
while(!g_exit){if(!cap.read(frame)||frame.empty())break;fc++;undistort(frame,undist,K,D);if(g_force_task>=0)Flag_Task=g_force_task;
if(g_case0_skip_init){t0(sc,undist);if(g_enable_gui){double fps=fc/chrono::duration<double>(chrono::steady_clock::now()-t0t).count();putText(undist,format("V7 FPS %.1f",fps),{10,30},FONT_HERSHEY_SIMPLEX,1,{0,255,0},2);imshow("Go2",undist);if(waitKey(1)==27)break;}continue;}
double lx,ly,dyaw;transformLocal(px,py,yaw,lx,ly,dyaw);
switch(Flag_Task){case 0:{int ret=case0_tick(sc,undist,rt.stateCB.state,fc);if(g_force_task<0){if(ret==1){Flag_Task=1;g_case0_second_pass=false;case1_reset_statics();}else if(ret==2){Flag_Task=2;g_case0_second_pass=false;case2_reset();}}break;}
case 1:if(g_force_task<0&&case1_tick(sc,fc,lx,ly,yaw)){Flag_Task=0;g_case0_second_pass=true;case0_reset_statics();}break;
case 2:if(g_force_task<0&&case2_tick(sc))Flag_Task=3;break;
case 3:case 4:case 5:case 6:case 7:case 8:if(g_force_task<0&&case3_tick(sc,lx,ly,dyaw))Flag_Task=9;break;
case 9:if(case4_tick(sc,avc))return 0;break;}
if(g_enable_gui){double fps=fc/chrono::duration<double>(chrono::steady_clock::now()-t0t).count();putText(undist,format("V7 FPS %.1f",fps),{10,30},FONT_HERSHEY_SIMPLEX,1,{0,255,0},2);imshow("Go2",undist);if(waitKey(1)==27)break;}}
sc.StopMove();avc.UseRemoteCommandFromApi(false);avc.SwitchSet(false);avc.Move(0,0,0);this_thread::sleep_for(chrono::milliseconds(200));sc.SwitchJoystick(true);sc.RecoveryStand();this_thread::sleep_for(chrono::milliseconds(500));sc.BalanceStand();cout<<"[Exit] Remote restored.\n";return 0;}
