#include "GimbalControl.h"
#include "stdint.h"
#include "pid.h"
#include "STMGood.h"
#include "BSP_usart.h"
#include "ChassisControl.h"
#include "ShootControl.h"
#include "DBUS.h"
#include "Keyboard.h"
#include "BSP_can.h"
#include "arm_math.h"
#include "detect.h"
#include "camera.h"
#define USEENCODER 1
#define AUTO 1
#define HAND 0
GimbalMotor GimbalData = {0};
PID_AbsoluteType YawInner;
PID_AbsoluteType YawOutter;
PID_AbsoluteType PitchInner;
PID_AbsoluteType PitchOutter;

PID_AbsoluteType v_YawInner;
PID_AbsoluteType v_YawOutter;
PID_AbsoluteType v_PitchInner;
PID_AbsoluteType v_PitchOutter;
int gimbalmode = 0;
int YawTargetEncoder = 0,PitchTargetEncoder = 0;
int PitchDebug = 0;
int YawDebug = 0;
int v_PitchDebug = 0;
int v_YawDebug = 0;

/**
 * @brief initialize the parameter of Gimbal
 * @param None
 * @return None
 * @attention  None
 */
void GimbalInit (void)
{
	PitchOutter.kp = 50;//30
	PitchOutter.ki = 0;
	PitchOutter.kd = 0;	
	PitchOutter.errILim = 0;
	PitchOutter.OutMAX = 500;//400
	
	PitchInner.kp = 80;//50
	PitchInner.ki = 0;
	PitchInner.kd = 0;
	PitchInner.errILim = 6000;
	PitchInner.OutMAX = 25000;

	YawOutter.kp = 40;//12
	YawOutter.ki = 0;
	YawOutter.kd = 0;	
	YawOutter.errILim = 0;
	YawOutter.OutMAX = 500;
	
	YawInner.kp = 35;//60
	YawInner.ki = 0;
	YawInner.kd = 0;
	YawInner.errILim = 3000;
	YawInner.OutMAX = 25000;
	
	v_PitchOutter.kp = 0;//30
	v_PitchOutter.ki = 0;
	v_PitchOutter.kd = 0;	
	v_PitchOutter.errILim = 0;
	v_PitchOutter.OutMAX = 500;//400
	
	v_PitchInner.kp = 0;//50
	v_PitchInner.ki = 0;
	v_PitchInner.kd = 0;
	v_PitchInner.errILim = 6000;
	v_PitchInner.OutMAX = 25000;

	v_YawOutter.kp = 0;//12
	v_YawOutter.ki = 0;
	v_YawOutter.kd = 0;	
	v_YawOutter.errILim = 0;
	v_YawOutter.OutMAX = 500;
	
	v_YawInner.kp = 0;//60
	v_YawInner.ki = 0;
	v_YawInner.kd = 0;
	v_YawInner.errILim = 3000;
	v_YawInner.OutMAX = 25000;
}

/**
 * @brief switch gimbal mode
 * @param None
 * @return None
 * @attention  None
 */
void switch_gimbal_mode(void)
{
	static int pitch_overborder,yaw_overborder;
	static int use_vision;
	//越界处理
	if(GimbalData.Pitchposition >= GimbalData.PitchMax||GimbalData.Pitchposition <= GimbalData.PitchMin)
	{
		pitch_overborder = 1;
	}
	else
	{
		pitch_overborder = 0;
	}
	if(GimbalData.Yawposition >= GimbalData.YawMax||GimbalData.Yawposition <= GimbalData.YawMin)
	{
		yaw_overborder = 1;
	}
	else
	{
		yaw_overborder = 0;
	}
	
	if(pitch_overborder==1||yaw_overborder==1)
	{
		use_vision = 0;
	}
	else if(RC_Ctl.rc.s1 == 1)
	{
		use_vision =1;
	}
	else
	{
		use_vision =0;
	}
	//选择模式；手动或自动
	if(use_vision==1)
	{
		gimbalmode = AUTO;
		
		GimbalData.PitchTarget1 = GimbalData.Pitchangle;
		GimbalData.YawTarget1 = GimbalData.Yawposition;
	}
	else
	{
		gimbalmode = HAND;
	}
}
/**
 * @brief Calibrate the position of Gimbal Motor
 * @param None
 * @return None
 * @attention  None
 *///1800 2500 2210 
void GimbalCalibration(void)
{
		if(USEENCODER)
		{
			GimbalData.YawTarget1 = GimbalData.YawBacknow;
		}
		else
		{
			GimbalData.YawTarget1 = GimbalData.Yawangle;
		}
		GimbalData.PitchTarget1 = GimbalData.Pitchangle;
		
		GimbalData.PitchMax = 7350;
		GimbalData.PitchMid = 7045;
		GimbalData.PitchMin = 6050;

	if(GimbalData.YawBacknow < 3000)
	{
		GimbalData.YawMax = 1800;
		GimbalData.YawMid = 0;
		GimbalData.YawMin = -3456;
	}
	else
	{
		GimbalData.YawMax = 10000;
		GimbalData.YawMid = 0;
		GimbalData.YawMin = 4735;
	}

		GimbalData.PitchMaxangle = (int)((GimbalData.PitchMax - GimbalData.PitchBacknow) * 0.0439506f);
		GimbalData.PitchMinangle = (int)(( GimbalData.PitchMin - GimbalData.PitchBacknow) * 0.0439506f);

//		GimbalData.YawMaxangle = 90;
//		GimbalData.YawMidangle = 45;
//		GimbalData.YawMinangle = 0;
		

}
/**
 * @brief get the tagetposition from remote and  keyboard,mouse
 * @param None
 * @return None
 * @attention  None
 */
void GetGimbalTarget(void)
{
/********************Deal the remote**************************************************/	
	if(Devicestate[10] == ONLINE)
	{
		if(USEENCODER)
		{
			GimbalData.YawTarget1 -= (float)(((-RC_Ctl.rc.ch2 + 1024)*0.0003f)*22.75f + RC_Ctl.mouse.x * MOUSE_YAW_CONST*22.75f);
		}
		else
		{
			GimbalData.YawTarget1 += (float)(((-RC_Ctl.rc.ch2 + 1024)*0.0003f) + RC_Ctl.mouse.x * MOUSE_YAW_CONST);
		}
		GimbalData.PitchTarget1 += (float)(((RC_Ctl.rc.ch1 - 1024)*0.0001f) + RC_Ctl.mouse.y * MOUSE_PITCH_CONST);
	}
/******************************limit the yawtarget*************************************/	
	if(USEENCODER)
	{
		if(GimbalData.YawTarget1 > GimbalData.YawMax) 
		{
			GimbalData.YawTarget1 = GimbalData.YawMax;
			GimbalData.YawTarget2 = GimbalData.YawMax;
		}
		else if(GimbalData.YawTarget1 < GimbalData.YawMin)
		{
			GimbalData.YawTarget2 = GimbalData.YawMin;
			GimbalData.YawTarget1 = GimbalData.YawMin;
		}
		else
			GimbalData.YawTarget2 = GimbalData.YawTarget1;
	}
	else	
	{
		YawTargetEncoder = (int)(GimbalData.Yawposition - (GimbalData.YawTarget1 - GimbalData.Yawangle)/0.0439506776f);
		if(YawTargetEncoder > GimbalData.YawMax)
		{
			GimbalData.YawTarget2 = GimbalData.YawTarget1 + (YawTargetEncoder - GimbalData.YawMax) * 0.0439506776f;
			GimbalData.YawTarget1 = GimbalData.YawTarget2;
		}
		else if(YawTargetEncoder < GimbalData.YawMin)
		{
			GimbalData.YawTarget2 = GimbalData.YawTarget1 + (YawTargetEncoder - GimbalData.YawMin) * 0.0439506776f;
			GimbalData.YawTarget1 = GimbalData.YawTarget2;
		}
		else
			GimbalData.YawTarget2 = GimbalData.YawTarget1;
	}
/******************************limit the pitch target********************************/	
//	PitchTargetEncoder = (int)(GimbalData.Pitchposition + (GimbalData.PitchTarget1 - GimbalData.Pitchangle)/0.0439506776f);
//	if(PitchTargetEncoder > GimbalData.PitchMax)
//	{
//		GimbalData.PitchTarget2 = GimbalData.PitchTarget1 - (PitchTargetEncoder - GimbalData.PitchMax) * 0.0439506776f;
//	}
//	else if(PitchTargetEncoder < GimbalData.PitchMin)
//	{
//		GimbalData.PitchTarget2 = GimbalData.PitchTarget1 - (PitchTargetEncoder - GimbalData.PitchMin) * 0.0439506776f;
//	}
//	else
//		GimbalData.PitchTarget2 = GimbalData.PitchTarget1;
	
	if(GimbalData.PitchTarget1 > GimbalData.PitchMaxangle )
	{
		GimbalData.PitchTarget2 = GimbalData.PitchMaxangle;
		GimbalData.PitchTarget1 = GimbalData.PitchTarget2;
	}
	else if(GimbalData.PitchTarget1 < GimbalData.PitchMinangle)
	{
		GimbalData.PitchTarget2 = GimbalData.PitchMinangle;
		GimbalData.PitchTarget1 = GimbalData.PitchTarget2;
	}
	else
		GimbalData.PitchTarget2 = GimbalData.PitchTarget1;
}

/**
 * @brief transform the single circular position to a continuous data
 * @param backposition from encoder
 * @return A continuous position data(totalposition)
 * @attention  None
 */
void DealGimbalPosition (void)
{
	
	if(GimbalData.Yawinit)
	{
		if(GimbalData.YawBacknow - GimbalData.YawBackold > 5000)
			GimbalData.Yawcirclecounter--;
		if(GimbalData.YawBacknow - GimbalData.YawBackold < -5000)
			GimbalData.Yawcirclecounter++;
	}
	GimbalData.YawBackold = GimbalData.YawBacknow;
	if(!GimbalData.Yawinit)
		GimbalData.Yawinit = 1;
	
	GimbalData.Yawposition = GimbalData.YawBacknow + GimbalData.Yawcirclecounter*8191;
	GimbalData.Yawpositionold = GimbalData.Yawposition;
	
//	GimbalData.Pitchposition = GimbalData.PitchBacknow;
//有bug	
	if(GimbalData.Pitchinit)
	{
		if(GimbalData.PitchBacknow - GimbalData.PitchBackold > 5000)
			GimbalData.Pitchcirclecounter--;
		if(GimbalData.PitchBacknow - GimbalData.PitchBackold < -5000)
			GimbalData.Pitchcirclecounter++;
	}
	GimbalData.PitchBackold = GimbalData.PitchBacknow;
	if(!GimbalData.Pitchinit)
		GimbalData.Pitchinit = 1;
	
	GimbalData.Pitchposition = GimbalData.PitchBacknow + GimbalData.Pitchcirclecounter*8191;
	GimbalData.Pitchpositionold = GimbalData.Pitchposition;


}
/**
 * @brief PID control of pitchmotor
 * @param None
 * @return None
 * @attention  None
 */
void PitchPID (float *Target)
{
	if(PitchDebug == 1)
	{
		PitchOutter.kp=P;//25
		PitchOutter.ki=I;
		PitchOutter.kd=D;	
		PitchOutter.errILim=0;
		PitchOutter.OutMAX=V1;
		
		PitchInner.kp=p;//20
		PitchInner.ki=i;
		PitchInner.kd=d;
		PitchInner.errILim=3000;
		PitchInner.OutMAX=V2;
	}

	PitchOutter.errNow = (*Target - GimbalData.Pitchangle);//处理成角度值
	PID_AbsoluteMode(&PitchOutter);
	PitchInner.errNow = PitchOutter.ctrOut -  GimbalData.Pitchspeed;//(1/65536*4000)
	PID_AbsoluteMode(&PitchInner);
	GimbalData.PitchCurrent = PitchInner.ctrOut;
}

/**
 * @brief PID control of Yaw motor
 * @param None
 * @return None
 * @attention  None
 */
void YawPID (float *Target)
{
	if(YawDebug == 1)
	{
		YawOutter.kp=P;//15
		YawOutter.ki=I;
		YawOutter.kd=D;	
		YawOutter.errILim=0;
		YawOutter.OutMAX=V1;
		
		YawInner.kp=p;//50
		YawInner.ki=i;
		YawInner.kd=d;
		YawInner.errILim=1000;
		YawInner.OutMAX=V2;
	}
	if(USEENCODER)
	{
		YawOutter.errNow = (*Target - GimbalData.Yawposition) * (-0.0439506f);
	}
	else
	{
		YawOutter.errNow = (*Target - GimbalData.Yawangle);
	}
	PID_AbsoluteMode(&YawOutter);
	YawInner.errNow = YawOutter.ctrOut - GimbalData.YawEncoderspeed*(-5.7608f);
//	YawInner.errNow = YawOutter.ctrOut - GimbalData.Yawspeed;
	PID_AbsoluteMode(&YawInner);
	GimbalData.YawCurrent = (int16_t)(-YawInner.ctrOut);
}
/***********************************auto function**************************************/

/**
 * @brief PID control of pitchmotor
 * @param None
 * @return None
 * @attention  None
 */
void v_PitchPID (float *Target)
{
	if(v_PitchDebug == 1)
	{
		v_PitchOutter.kp=P;//25
		v_PitchOutter.ki=I;
		v_PitchOutter.kd=D;	
		v_PitchOutter.errILim=0;
		v_PitchOutter.OutMAX=V1;
		
		v_PitchInner.kp=p;//20
		v_PitchInner.ki=i;
		v_PitchInner.kd=d;
		v_PitchInner.errILim=3000;
		v_PitchInner.OutMAX=V2;
	}

	v_PitchOutter.errNow = -(*Target - pcParam.pcCenterY.f);//处理成角度值
	PID_AbsoluteMode(&v_PitchOutter);
	v_PitchInner.errNow = v_PitchOutter.ctrOut -  GimbalData.Pitchspeed;//(1/65536*4000)
	PID_AbsoluteMode(&v_PitchInner);
	GimbalData.PitchCurrent = v_PitchInner.ctrOut;
}

/**
 * @brief PID control of Yaw motor
 * @param None
 * @return None
 * @attention  None
 */
void v_YawPID (float *Target)
{
	if(v_YawDebug == 1)
	{
		v_YawOutter.kp=P;//15
		v_YawOutter.ki=I;
		v_YawOutter.kd=D;	
		v_YawOutter.errILim=0;
		v_YawOutter.OutMAX=V1;
		
		v_YawInner.kp=p;//50
		v_YawInner.ki=i;
		v_YawInner.kd=d;
		v_YawInner.errILim=1000;
		v_YawInner.OutMAX=V2;
	}

	v_YawOutter.errNow = -(*Target - pcParam.pcCenterX.f);
	PID_AbsoluteMode(&v_YawOutter);
	v_YawInner.errNow = v_YawOutter.ctrOut - GimbalData.YawEncoderspeed*(-5.7608f);
	PID_AbsoluteMode(&v_YawInner);
	GimbalData.YawCurrent = (int16_t)(-v_YawInner.ctrOut);
}