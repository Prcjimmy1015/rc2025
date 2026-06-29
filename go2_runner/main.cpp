#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/common/time/time_tool.hpp>
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
#include "visualizer.h"
#include "aruco_server.h"
#include "app_runtime.h"
#include "cases/case0.h"
#include "cases/case1.h"
#include "cases/case2.h"
#include "cases/case3.h"
#include "cases/case4.h"
using namespace unitree::robot; using namespace cv; using namespace std;

static atomic<bool> g_exit_requested(false);
void signalHandler(int sig) { if(sig==SIGINT){cout<<"\n[SIGINT]\n";g_exit_requested=true;} }

static void task0_inline(go2::SportClient &sc, Mat &frame) {
    static bool once=false; if(!once){cout<<"\n***** TASK0_INLINE_ACTIVE *****\n"<<endl;once=true;}
    Mat g,b,n; cvtColor(frame,g,COLOR_BGR2GRAY); GaussianBlur(g,b,{5,5},0);
    threshold(b,n,50,255,THRESH_BINARY_INV);
    {Mat k=getStructuringElement(MORPH_RECT,Size(3,3));morphologyEx(n,n,MORPH_OPEN,k);}
    int rh=40,ys=b.rows-rh;if(ys<0)ys=0;
    double err=0;int cnt=0,rw=n.cols;vector<int>cc(rw,0);
    for(int r=ys;r<b.rows;++r){const uchar*row=n.ptr(r);for(int x=0;x<rw;++x)if(row[x]){cc[x]++;cnt++;}}
    int pc=-1,pk=0;for(int x=0;x<rw;++x)if(cc[x]>pk){pk=cc[x];pc=x;}
    bool ok=(pk>=5&&cnt>=50&&cnt<=50000);if(ok)err=pc-640;
    if(ok){int cx=max(0,min(1279,pc));int cy=b.rows-rh/2;
        circle(frame,Point(cx,cy),10,Scalar(0,255,0),-1);
        line(frame,Point(cx,cy+25),Point(cx,cy-25),Scalar(0,255,0),2);}
    double ly=0;{double _,dy;transformLocal(px,py,yaw,_,ly,dy);}
    double lc=(ly>0.35)?-0.3:(ly<-0.35)?0.3:0;
    if(ok&&abs(err)<400&&cnt>100){static double I=0,Le=0;I+=err;I=max(-50.,min(50.,I));
        double s=-(0.12*err+0.002*I+0.01*(err-Le));Le=err;s=max(-0.8,min(0.8,s));
        if(abs(lc)>0.01)s=lc;s=max(-0.8,min(0.8,s));
        sc.StaticWalk();sc.Euler(0,0.4,0);sc.Move(0.25,0,s);}
    else if(ok){double s=abs(err)<640?max(-0.5,min(0.5,-err*0.003)):0;
        if(abs(lc)>0.01)s=lc;s=max(-0.8,min(0.8,s));
        sc.StaticWalk();sc.Euler(0,0.4,0);sc.Move(0.2,0,s);}
    else{double s=max(-0.8,min(0.8,lc));sc.StaticWalk();sc.Euler(0,0.4,0);sc.Move(0.15,0,s);}
}

static int runMainLoop(AppRuntime &rt) {
    go2::SportClient &sc=rt.sc; go2::ObstaclesAvoidClient &avoid_client=rt.avoid_client;
    VideoCapture &cap=rt.cap; Mat frame,undist; int fc=0; auto t0=chrono::steady_clock::now();
    while(!g_exit_requested){
        if(!cap.read(frame)||frame.empty())break;fc++;undistort(frame,undist,K,D);
        double lx,ly,dyaw;transformLocal(px,py,yaw,lx,ly,dyaw);
        if(g_force_task>=0)Flag_Task=g_force_task;
        if(g_case0_skip_init){task0_inline(sc,undist);
            if(g_enable_gui){double fps=fc/chrono::duration<double>(chrono::steady_clock::now()-t0).count();
                putText(undist,format("TASK0 FPS %.1f",fps),{10,30},FONT_HERSHEY_SIMPLEX,1,{0,255,0},2);
                imshow("Go2 Front Cam",undist);if(waitKey(1)==27)break;}
            if(fc%30==0)cout<<"[TASK0] frame="<<fc<<endl;continue;}
        switch(Flag_Task){
        case 0:{int ret=case0_tick(sc,undist,rt.stateCB.state,fc);
            if(g_force_task<0){if(ret==1){Flag_Task=1;g_case0_second_pass=false;case1_reset_statics();}
            else if(ret==2){Flag_Task=2;g_case0_second_pass=false;case2_reset();}}break;}
        case 1:if(g_force_task<0&&case1_tick(sc,fc,lx,ly,yaw)){Flag_Task=0;g_case0_second_pass=true;case0_reset_statics();}break;
        case 2:if(g_force_task<0&&case2_tick(sc))Flag_Task=3;break;
        case 3:case 4:case 5:case 6:case 7:case 8:if(g_force_task<0&&case3_tick(sc,lx,ly,dyaw))Flag_Task=9;break;
        case 9:if(case4_tick(sc,avoid_client))return 0;break;}
        if(g_enable_gui){double fps=fc/chrono::duration<double>(chrono::steady_clock::now()-t0).count();
            putText(undist,format("FPS %.1f",fps),{10,30},FONT_HERSHEY_SIMPLEX,1,{0,255,0},2);
            imshow("Go2 Front Cam",undist);if(waitKey(1)==27||g_exit_requested)break;}
    }
    sc.StopMove();avoid_client.UseRemoteCommandFromApi(false);avoid_client.SwitchSet(false);
    avoid_client.Move(0,0,0);this_thread::sleep_for(chrono::milliseconds(200));
    sc.SwitchJoystick(true);sc.RecoveryStand();this_thread::sleep_for(chrono::milliseconds(500));
    sc.BalanceStand();cout<<"[Exit] Remote restored.\n";return 0;
}

int main(int argc,char**argv){
    if(argc<2){cerr<<"Usage: "<<argv[0]<<" <eth_if> [--gui] [--task N]\n";return -1;}
    const char*eth_if=argv[1];
    for(int i=2;i<argc;++i){string a=argv[i];
        if(a=="--gui")g_enable_gui=true;
        else if(a=="--task"&&i+1<argc){g_force_task=atoi(argv[++i]);
            if(g_force_task==0){g_case0_skip_init=true;g_case0_second_pass=true;}
            cout<<"[Config] task:"<<g_force_task<<endl;}
        else{cerr<<"Unknown: "<<a<<"\n";return -1;}}
    signal(SIGINT,signalHandler);ChannelFactory::Instance()->Init(0,eth_if);
    AppRuntime rt;if(!initAppRuntime(rt,eth_if)){cerr<<"Camera fail\n";return -1;}
    px0=px;py0=py;yaw0=yaw;thread t(aruco_socket_server,5005);t.detach();
    cout<<(g_enable_gui?"GUI\n":"Headless\n")<<flush;return runMainLoop(rt);
}
