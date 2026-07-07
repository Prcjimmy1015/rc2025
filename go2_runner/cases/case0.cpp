#include "case0.h"
#include "../params.h"
#include "../globals.h"
#include "../utils.h"
#include "../line_follow.h"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
using namespace unitree::robot;
using namespace cv;
using namespace std;

static void task0_loop(go2::SportClient &sc, const Mat &u, double ly)
{
    Mat g,b,n; cvtColor(u,g,COLOR_BGR2GRAY); GaussianBlur(g,b,{5,5},0);
    threshold(b,n,50,255,THRESH_BINARY_INV);
    {Mat k=getStructuringElement(MORPH_RECT,Size(3,3));morphologyEx(n,n,MORPH_OPEN,k);}
    int rh=40,ys=b.rows-rh;if(ys<0)ys=0;
    double e=0;int c=0,rw=n.cols;vector<int>cc(rw,0);
    for(int r=ys;r<b.rows;++r){const uchar*row=n.ptr(r);for(int x=0;x<rw;++x)if(row[x]){cc[x]++;c++;}}
    int pc=-1,pk=0;for(int x=0;x<rw;++x)if(cc[x]>pk){pk=cc[x];pc=x;}
    bool ok=(pk>=5&&c>=50&&c<=50000);if(ok)e=pc-640;

    static int cnt=0;cnt++;
    if(cnt%10==0){
        if(ok)cout<<"[T0] LINE err="<<e<<" c="<<c<<" pk="<<pk<<endl;
        else  cout<<"[T0] NO  LINE c="<<c<<" pk="<<pk<<endl;
    }

    double lc=0;if(ly>0.35)lc=-0.3;else if(ly<-0.35)lc=0.3;

    if(ok&&abs(e)<400&&c>100){
        static double I=0,Le=0;I+=e;I=max(-50.,min(50.,I));
        double s=-(0.12*e+0.002*I+0.01*(e-Le));Le=e;s=max(-0.8,min(0.8,s));
        if(abs(lc)>0.01)s=lc;s=max(-0.8,min(0.8,s));
        sc.StaticWalk();sc.Euler(0,0.4,0);sc.Move(0.25,0,s);
    }else if(ok){
        double s=abs(e)<640?max(-0.5,min(0.5,-e*0.003)):0;
        if(abs(lc)>0.01)s=lc;s=max(-0.8,min(0.8,s));
        sc.StaticWalk();sc.Euler(0,0.4,0);sc.Move(0.2,0,s);
    }else{
        double s=max(-0.5,min(0.5,lc));
        sc.StaticWalk();sc.Euler(0,0.4,0);sc.Move(0.15,0,s);
    }
}

int case0_tick(go2::SportClient &sc, const Mat &undist,
               const unitree_go::msg::dds_::SportModeState_ &state, int fcount)
{
    (void)state;(void)fcount;
    double lx,ly,dyaw;transformLocal(px,py,yaw,lx,ly,dyaw);
    static int st=0;

    static int dbg=0;
    if(dbg++%15==0)printf("[case0] st=%d lx=%.2f px=%.2f py=%.2f\n",st,lx,px,py);
    if(g_case0_skip_init && st<3){st=3;cout<<"=== T0 ACTIVE ==="<<endl;}
    if(st==0){sc.StaticWalk();sc.Euler(0,0,0);sc.Move(0.15,0,0);
        if(lx>=0.2){sc.StopMove();sc.Move(0,0,0);st=1;}return 0;}
    if(st==1){sc.FrontJump();st=2;
        this_thread::sleep_for(chrono::milliseconds(300));px0=px;py0=py;yaw0=yaw;return 0;}
    if(st==2){sc.BalanceStand();this_thread::sleep_for(chrono::milliseconds(800));st=3;return 0;}

    if(g_case0_skip_init){task0_loop(sc,undist,ly);return 0;}

    // 稳定期：BalanceStand 后需要几帧让 StaticWalk 生效
    static int settle_frames = 0;
    if(st==3 && settle_frames < 30){
        settle_frames++;
        sc.StaticWalk(); sc.Euler(0,0,0); sc.Move(0.05, 0, 0);
        if(settle_frames >= 30){ cout << "[case0] Settled, go.\n" << endl; }
        return 0;
    }

    return pureLineFollow(sc,undist,lx,ly,dyaw,fcount,g_case0_second_pass);
}
void case0_reset_statics(){}