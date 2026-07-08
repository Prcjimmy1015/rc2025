#include "case3.h"
#include "../globals.h"
#include "../utils.h"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include <vector>
using namespace unitree::robot;
using namespace cv;
using namespace std;

struct Checkpoint {
    double lx, ly, yaw_target;
    int type;
    bool done;
    const char* name;
};

// 四组实测平均值, yaw 不参与触发 (里程计漂移太大)
static Checkpoint cps[] = {
    {0.20,  1.16,  0,  1, false, "T1"},
    {1.66,  3.83,  0,  1, false, "T2"},
    {-0.88, 3.60,  0,  1, false, "T3"},
    {-0.97, 1.91,  0,  1, false, "T4"},
    {-0.81, 0.98,  0,  2, false, "T5"},
    {g_orig_px, g_orig_py, 0,  3, false, "A2"},
};
static const int N_CPS = sizeof(cps)/sizeof(cps[0]);

static bool settled=false;
static int n_st=0;
static double yaw_settle=0;
static int cnt=0;
static int last_pc=640;
static int sharp_burst=0;
static int burst_cooldown=0;

static int cp_idx=0;
static bool in_cp=false;
static int cp_timer=0;
static bool has_red = false;  // latch: 在 T2 之后看到红点就保持

void case3_reset(){
    settled=false; n_st=0; cnt=0; last_pc=640;
    sharp_burst=0; burst_cooldown=0;
    cp_idx=0; in_cp=false; cp_timer=0;
    has_red=false;
    for(int i=0;i<N_CPS;i++)cps[i].done=false;
}

int case3_tick(go2::SportClient &sc,
               cv::Mat &undist,
               double lx,
               double ly)
{
    static bool once=false;
    if(!once){cout<<"\n=== V22_CASE3 ===\n"<<endl;once=true;}
    cnt++;

    if(!settled){
        n_st++;
        sc.StaticWalk();
        sc.Euler(0,0.8,0);
        if(n_st==1) yaw_settle=yaw;
        double yd=yaw-yaw_settle;
        if(yd>M_PI)yd-=2*M_PI;if(yd<-M_PI)yd+=2*M_PI;
        double steer=-yd*2.0;steer=max(-0.3,min(0.3,steer));
        sc.Move(0,0,steer);
        if(n_st>=30){settled=true;cout<<"[V22] Settled, go.\n"<<endl;}
        return 0;
    }

    sc.Euler(0,0.8,0);

    Mat g,b,n;
    cvtColor(undist,g,COLOR_BGR2GRAY);
    GaussianBlur(g,b,{5,5},0);
    threshold(b,n,50,255,THRESH_BINARY_INV);
    {Mat k=getStructuringElement(MORPH_RECT,Size(3,3));morphologyEx(n,n,MORPH_OPEN,k);}

    int rh=100,roiy=b.rows-rh;if(roiy<0)roiy=0;
    double e=0;int ci=0,rw=n.cols;
    vector<int>cc(rw,0);
    for(int r=roiy;r<b.rows;++r){
        const uchar*row=n.ptr(r);
        for(int x=0;x<rw;++x)if(row[x]){cc[x]++;ci++;}
    }

    int win_l=max(0,last_pc-300),win_r=min(rw-1,last_pc+300);
    int pk=0;for(int x=win_l;x<=win_r;++x)if(cc[x]>pk)pk=cc[x];
    int pc=-1;
    if(pk>=5){
        double sum_w=0,sum_wx=0;
        for(int x=win_l;x<=win_r;++x)
            if(cc[x]>pk*0.5){sum_w+=cc[x];sum_wx+=cc[x]*x;}
        if(sum_w>0)pc=(int)(sum_wx/sum_w);
    }
    if(pc<0)pc=last_pc;
    bool ok=(pk>=5&&ci>=50&&ci<=100000);
    if(ok)e=pc-640;
    if(ok&&ci>5000&&pk>80)last_pc=pc;

    // 红点检测: 只在 T2 完成后 (cp_idx>=2) 才开门, 防止远处误检
    if(!has_red && cp_idx>=2){
        Mat hsv, mask1, mask2;
        cvtColor(undist, hsv, COLOR_BGR2HSV);
        inRange(hsv, Scalar(0, 100, 100), Scalar(10, 255, 255), mask1);
        inRange(hsv, Scalar(170, 100, 100), Scalar(180, 255, 255), mask2);
        Mat mask = mask1 | mask2;
        int red_px = countNonZero(mask);
        if (red_px > 500){
            has_red = true;
            cout << "🔴 RED DETECTED (pixels=" << red_px << ") latch ON" << endl;
        }
    }

    if(!in_cp && cp_idx<N_CPS && !cps[cp_idx].done){
        double dist;
        if(cp_idx == N_CPS-1){
            double wx = g_orig_px - px, wy = g_orig_py - py;
            dist = sqrt(wx*wx + wy*wy);
        }else{
            double dx=lx-cps[cp_idx].lx, dy=ly-cps[cp_idx].ly;
            dist=sqrt(dx*dx+dy*dy);
        }
        // 触发: T3 需要红点, 其余纯距离 <0.30
        bool trigger = false;
        if(strcmp(cps[cp_idx].name, "T3") == 0){
            trigger = (dist < 0.30 && has_red);
        }else{
            trigger = (dist < 0.30);
        }
        if(trigger){
            in_cp=true; cp_timer=0;
            sc.StopMove();
            printf("\n[CP] ARRIVE %s (lx=%.2f ly=%.2f yaw=%.2f) dist=%.2f\n\n",
                   cps[cp_idx].name,lx,ly,yaw,dist);
        }
    }

    if(in_cp){
        cp_timer++;
        if(cps[cp_idx].type==3){
            in_cp=false; cps[cp_idx].done=true; cp_idx++;
            printf("[CP] %s ARRIVED → FINISH\n",cps[cp_idx-1].name);
        }else if(cps[cp_idx].type==2){
            sc.StopMove();
            sc.FrontJump();
            in_cp=false; cps[cp_idx].done=true; cp_idx++;
            printf("[CP] %s JUMP\n",cps[cp_idx-1].name);
        }else{
            sc.Move(0,0,0);
            if(cp_timer%15==0)printf("[CP] %s PAUSE %d/90\n",
                cps[cp_idx].name,cp_timer);
            if(cp_timer>=90){
                in_cp=false; cps[cp_idx].done=true; cp_idx++;
                printf("[CP] %s DONE\n",cps[cp_idx-1].name);
            }
        }
        if(cps[cp_idx-1].type==3) return 1;
        return 0;
    }

    if(cp_idx == N_CPS-1 && !in_cp){
        double wx = g_orig_px - px, wy = g_orig_py - py;
        double wdist = sqrt(wx*wx + wy*wy);
        double target_angle = atan2(wy, wx);
        double heading_err = target_angle - yaw;
        if(heading_err > M_PI) heading_err -= 2*M_PI;
        if(heading_err < -M_PI) heading_err += 2*M_PI;
        double steer = heading_err * 1.5;
        steer = max(-0.3, min(0.3, steer));
        if(cnt%15==0) printf("[A2] dist=%.2f hdg=%.1fdeg steer=%.2f\n",
            wdist, heading_err*180/M_PI, steer);
        sc.StaticWalk(); sc.Euler(0,0,0);
        sc.Move(0.15, 0, steer);
        return 0;
    }

    {
        int h=undist.rows, w=undist.cols;
        rectangle(undist, {0, h-100}, {w, h}, {255,0,0}, 1);
        if(ok && pk>=5){
            int px_val = (int)(640 + e);
            px_val = max(0, min(w-1, px_val));
            circle(undist, {px_val, h-50}, 8, {0,255,0}, 2);
            line(undist, {640, h}, {640, h-100}, {0,255,255}, 1);
        }
        putText(undist, format("err=%.0f ci=%d pk=%d", e, ci, pk),
                {10, 60}, FONT_HERSHEY_SIMPLEX, 0.6, {0,255,0}, 1);
    }

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

    double lc=(ly>0.35)?-0.3:(ly<-0.35)?0.3:0;

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
    }else if(ok){
        double tg=e/1280.0*60.0*M_PI/180.0;
        double s=-tg*6.0;s=max(-1.0,min(1.0,s));
        double vx=(abs(e)>300)?0.08:0.12;
        double vy=e*0.0006;vy=max(-0.15,min(0.15,vy));
        sc.Move(vx,vy,s);
    }else{
        sc.Move(0.1,0,0.3);
    }

    return 0;
}