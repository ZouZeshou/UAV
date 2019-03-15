#include "ChassisControl.h"
#include "BSP_usart.h"
#include "stdio.h"
#include "stdint.h"
#include "math.h"
#include "string.h"
#include "stdbool.h"
#include "stdlib.h"
#include "pid.h"
#include "GimbalControl.h"
#include "BSP_can.h"
#include "ramp.h"
#include "Keyboard.h"
#include "DBUS.h"
#include "STMGood.h"
int YawMidPosi = 4750;
ChassisSpeed Chassisdata = {0};
int ChassisWheelDebug = 0;
int ChassisFollowDebug = 0;
int ChassisMode;
float currentparam;
PID_AbsoluteType Chassiswheelpid[4];
PID_AbsoluteType ChassisfollowInner,ChassisfollowOutter;
float ChassisAngle2;
/**
 * @brief Choose Chassis Mode
 * @param None
 * @return None
 * @attention  None
 */
 void ChooseChassisMode(void)
{
	if(RC_Ctl.rc.s1 == 1)
		{
			ChassisMode = ROTATE;
		}
	else
		{
			ChassisMode = FOLLOW;
		}
 }
/**
 * @brief deal with the data for chassis
 * @param None
 * @return None
 * @attention  None
 */
void DealRemotedata(void)
{
	ChassisAngle2 = -(GimbalData.Yawposition - YawinitMidposi) * 0.00076699f + (-Gyroscope2.gz) * 0.191986218f/6000.0f;
	if(fps.DBUS>0)
	{
		if(ChassisMode == FOLLOW)
		{
			Chassisdata.Vx = (int16_t)(((RC_Ctl.rc.ch0-1024)*NormalParameter + KeyMousedata.Vx));
			Chassisdata.Vy = (int16_t)(((RC_Ctl.rc.ch1-1024)*NormalParameter + KeyMousedata.Vy));
		}
		else if(ChassisMode == ROTATE)
		{
			Chassisdata.Vxabs = (int16_t)(((RC_Ctl.rc.ch0-1024)*NormalParameter + KeyMousedata.Vx));
			Chassisdata.Vyabs = (int16_t)(((RC_Ctl.rc.ch1-1024)*NormalParameter + KeyMousedata.Vy));
			Chassisdata.Vx = (int16_t)(Chassisdata.Vyabs * sinf(ChassisAngle2) + Chassisdata.Vxabs * cosf(ChassisAngle2));
			Chassisdata.Vy = (int16_t)(Chassisdata.Vyabs * cosf(ChassisAngle2) - Chassisdata.Vxabs * sinf(ChassisAngle2));
		
		}
	}
}
/**
 * @brief initialize the parameter of chassis
 * @param None
 * @return None
 * @attention  None
 */
void ChassisInit(void)
{
	int i;
	
	ChassisfollowOutter.kp=100;
	ChassisfollowOutter.ki=0;
	ChassisfollowOutter.kd=0;
	ChassisfollowOutter.OutMAX=5000;
	
	ChassisfollowInner.kp=10;
	ChassisfollowInner.ki=0;
	ChassisfollowInner.kd=0;
	ChassisfollowInner.OutMAX=5000;

	for(i = 0;i < 4;i++)
	{
		Chassiswheelpid[i].kp=4;
		Chassiswheelpid[i].ki=0.1;
		Chassiswheelpid[i].kd=0;
	
		Chassiswheelpid[i].errILim=3000;
		Chassiswheelpid[i].OutMAX=10000;	
	}

}

/**
 * @brief deal with the data for chassis follwer
 * @param None
 * @return None
 * @attention  None
 */
void Chassisfollow(void)
{
	if(ChassisFollowDebug == 1)
	{
		ChassisfollowOutter.kp = P;
		ChassisfollowOutter.ki = I;
		ChassisfollowOutter.kd = D;
		ChassisfollowOutter.OutMAX = V1;
		
		ChassisfollowInner.kp = p;
		ChassisfollowInner.ki = i;
		ChassisfollowInner.kd = d;
		ChassisfollowInner.OutMAX = V2;
	}
	if(ChassisMode == FOLLOW)
	{
		//Ë«»·
//		ChassisfollowOutter.errNow =(float)( (YawMidPosi-GimbalData.Yawposition)*0.0439453125f);
//		PID_AbsoluteMode(&ChassisfollowOutter);
//		ChassisfollowInner.errNow = (float)(-ChassisfollowOutter.ctrOut - (-Gyroscope2.gz));
//		PID_AbsoluteMode(&ChassisfollowInner);
//		Chassisdata.Rotate = (int16_t)(-ChassisfollowInner.ctrOut);
		//µ¥»·
		ChassisfollowOutter.errNow =(float)( (YawMidPosi-GimbalData.Yawposition)*0.0439453125f);
		PID_AbsoluteMode(&ChassisfollowOutter);
		Chassisdata.Rotate = ChassisfollowOutter.ctrOut;
	}
	else if(ChassisMode == ROTATE)
	{
		Chassisdata.Rotate = -5000;
	}
}
/**
 * @brief transform the wanted speed to real mecanum speed
 * @param Vx Vy rotate
 * @return None
* @attention  the second step
 */
void MecanumCalculate(void)
{
	float Buffer[4],MaxSpeed;
	int i;
	float Param;
    
    Buffer[0] = Chassisdata.Vx + Chassisdata.Vy + Chassisdata.Rotate;
    Buffer[1] = Chassisdata.Vx - Chassisdata.Vy + Chassisdata.Rotate;
    Buffer[2] = -Chassisdata.Vx + Chassisdata.Vy + Chassisdata.Rotate;
    Buffer[3] = -Chassisdata.Vx - Chassisdata.Vy + Chassisdata.Rotate;
	
	for(i = 0, MaxSpeed = 0; i < 4; i++)
    {
        if(fabs(Buffer[i]) > fabs(MaxSpeed))
        {
            MaxSpeed =fabs(Buffer[i]);
        }
    }
	if(MaxSpeed > MaxWheelSpeed)
    {
        Param =(float) (MaxWheelSpeed /(float)(MaxSpeed));
        Chassisdata.Speed[0] = (int)(Buffer[0] * Param);
        Chassisdata.Speed[1] = (int)(Buffer[1] * Param);
        Chassisdata.Speed[2] = (int)(Buffer[2] * Param);
        Chassisdata.Speed[3] = (int)(Buffer[3] * Param); 
    }
	else
    {
        Chassisdata.Speed[0] = Buffer[0];
        Chassisdata.Speed[1] = Buffer[1];
        Chassisdata.Speed[2] = Buffer[2];
        Chassisdata.Speed[3] = Buffer[3];
    }

 
}

/**
 * @brief caculate the initial current
 * @param None
 * @return None
 * @attention  the third step
 */
void ChassisPid (void)
{
	
	int i;
	if(ChassisWheelDebug == 1)
	{
		for(i = 0;i < 4;i++)
		{
			Chassiswheelpid[i].kp=P;
			Chassiswheelpid[i].ki=I;
			Chassiswheelpid[i].kd=D;
		
			Chassiswheelpid[i].errILim=3000;
			Chassiswheelpid[i].OutMAX=10000;	
		}
	}
	for(i=0;i<4;i++)
	{	
		Chassiswheelpid[i].errNow=(float)(Chassisdata.Speed[i]- Chassisdata.BackSpeed[i]);
		PID_AbsoluteMode(&Chassiswheelpid[i]);
	}
	//current restrict
	Chassisdata.CurrentSum = (float)(fabs(Chassiswheelpid[0].ctrOut) 	\
	+	fabs(Chassiswheelpid[1].ctrOut) + fabs(Chassiswheelpid[2].ctrOut) + fabs(Chassiswheelpid[3].ctrOut));
	
	if(Chassisdata.CurrentSum > MaxCurrentSum)
	{
		currentparam = (MaxCurrentSum/Chassisdata.CurrentSum);
		Chassisdata.Current[0] = (int16_t)(Chassiswheelpid[0].ctrOut * currentparam);
		Chassisdata.Current[1] = (int16_t)(Chassiswheelpid[1].ctrOut * currentparam);
		Chassisdata.Current[2] = (int16_t)(Chassiswheelpid[2].ctrOut * currentparam);
		Chassisdata.Current[3] = (int16_t)(Chassiswheelpid[3].ctrOut * currentparam);
	}
	else
	{
		Chassisdata.Current[0] = (int16_t)(Chassiswheelpid[0].ctrOut);
		Chassisdata.Current[1] = (int16_t)(Chassiswheelpid[1].ctrOut);
		Chassisdata.Current[2] = (int16_t)(Chassiswheelpid[2].ctrOut);
		Chassisdata.Current[3] = (int16_t)(Chassiswheelpid[3].ctrOut);
	}
	
	
}
