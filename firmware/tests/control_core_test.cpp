#include <cassert>
#include <cmath>
#include <cstdio>
#include "../ESP32_PID_Balance/control_core.h"
int main(){
 int trims[4]={0,0,0,0},out[4];
 balance::AxisPID r,p;
 // Steady front down: boost physical front pair M1/M3.
 balance::mix(true,1150,0,p.update(-20,0,.01f),trims,out);
 assert(out[0]>1150 && out[2]>1150 && out[1]<1150 && out[3]<1150);
 // Steady right down: boost physical right pair M1/M2.
 balance::mix(true,1150,r.update(-20,0,.01f),0,trims,out);
 assert(out[0]>1150 && out[1]>1150 && out[2]<1150 && out[3]<1150);
 balance::mix(true,1150,r.update(20,0,.01f),0,trims,out);
 assert(out[2]>1150 && out[3]>1150 && out[0]<1150 && out[1]<1150);
 balance::mix(true,1150,0,p.update(20,0,.01f),trims,out);
 assert(out[1]>1150 && out[3]>1150 && out[0]<1150 && out[2]<1150);
 // No corrections while stopped or base throttle is zero, even with trims.
 for(int &t:trims)t=40;
 balance::mix(false,1200,100,100,trims,out);for(int v:out)assert(v==1000);
 balance::mix(true,1000,100,100,trims,out);for(int v:out)assert(v==1000);
 // Bound every possible correction corner and maximum trims.
 for(int base=1000;base<=1300;base+=25)for(int rr=-100;rr<=100;rr+=25)for(int pp=-100;pp<=100;pp+=25){
  balance::mix(true,base,rr,pp,trims,out);for(int v:out)assert(v>=1000 && v<=1400);
 }
 // Rate damping resists motion away from equilibrium.
 balance::AxisPID d;assert(d.update(0,-50,.01f)>0);assert(d.update(0,50,.01f)<0);
 // Saturated error must not wind up integral. Verify bounded I and reset.
 balance::AxisPID i;i.ki=.5f;
 for(int n=0;n<10000;n++)i.update(-1000,0,.01f);
 assert(i.integral==0);assert(i.update(-1000,0,.01f)==100);
 i.update(-1,0,.01f);assert(i.integral>0);i.reset();assert(i.integral==0);
 assert(balance::wrap(359)==-1 && balance::wrap(-359)==1);
 puts("PASS: all four correction directions, stop/minimum gating, output limits, gyro damping, anti-windup, reset and wrap.");
}

