#ifndef __PID_H
#define __PID_H

#include "stdint.h"



typedef struct 
{
  float kp;
	float ki;
	float kd;
	
	float dErrP;
	float dErrI;
	float dErrD;
	
	float  errNow;
	float errOld1;
	float errOld2;
	
	float dCtrOut;
	float dOutMAX;
	float ctrOut;
	float OutMAX;
	
}PID_IncrementType;//增量式

typedef struct 
{
  float kp;
	float ki;
	float kd;
	
	float ErrP;
	float ErrI;
	float ErrD;
	
	float errNow;
	float errOld;
	float errILim;
	float OutMAX;
	
	float ctrOut;

}PID_AbsoluteType;//绝对式 


extern PID_AbsoluteType Chassiswheelpid[4];
extern PID_AbsoluteType YawInner;
extern PID_AbsoluteType YawOuter;
extern PID_AbsoluteType PitchInner;
extern PID_AbsoluteType PitchOuter;
extern PID_AbsoluteType ChassisfollowOutter;
extern PID_AbsoluteType StirMotorOutterPID,StirMotorInnerPID,ShootLPID,ShootRPID;

void PID_IncrementMode(PID_IncrementType *);        //增量式pid函数
void PID_AbsoluteMode(PID_AbsoluteType *);					//位置式pid函数
void DealSpeed (int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);








#endif





