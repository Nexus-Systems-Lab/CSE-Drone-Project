#pragma once
#include <math.h>

namespace balance {
inline float limit(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
inline float wrap(float a){while(a>180)a-=360;while(a<-180)a+=360;return a;}
constexpr int MIN_US=1000, BASE_MAX_US=1300, MOTOR_MAX_US=1400;
constexpr float CORRECTION_MAX=100.0f;
struct AxisPID {
 float kp=1.5f,ki=0.0f,kd=0.05f,integral=0;
 void reset(){integral=0;}
 float update(float angle,float rate,float dt){
  // Target=0 degrees. Derivative on measured gyro rate, not noisy error differences.
  float error=-angle;
  float candidate=limit(integral+error*dt,-50,50);
  float raw=kp*error+ki*candidate-kd*rate;
  if((raw<=CORRECTION_MAX && raw>=-CORRECTION_MAX) ||
     (raw>CORRECTION_MAX && error<0) || (raw<-CORRECTION_MAX && error>0))integral=candidate;
  return limit(kp*error+ki*integral-kd*rate,-CORRECTION_MAX,CORRECTION_MAX);
 }
};
inline void mix(bool enabled,int throttle,float r,float p,const int trims[4],int out[4]){
 // M1 FR, M2 RR, M3 FL, M4 RL.
 // User verified: front down -> pitch negative; right down -> roll negative.
 if(!enabled || throttle<=MIN_US){for(int i=0;i<4;i++)out[i]=MIN_US;return;}
 float command[4]={throttle+r+p+trims[0],throttle+r-p+trims[1],
                   throttle-r+p+trims[2],throttle-r-p+trims[3]};
 for(int i=0;i<4;i++)out[i]=(int)lroundf(limit(command[i],MIN_US,MOTOR_MAX_US));
}
}

