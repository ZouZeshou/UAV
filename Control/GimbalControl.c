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
#define YAW_USEENCODER 1
#define PIT_USEENCODER 1
#define AUTO 1
#define HAND 0
GimbalMotor GimbalData = {0};
PID_AbsoluteType YawInner;
PID_AbsoluteType YawOuter;
PID_AbsoluteType PitchInner;
PID_AbsoluteType PitchOuter;

PID_AbsoluteType v_YawInner;
PID_AbsoluteType v_YawOuter;
PID_AbsoluteType v_PitchInner;
PID_AbsoluteType v_PitchOuter;
int gimbalmode = 0;
int YawTargetEncoder = 0,PitchTargetEncoder = 0;
int PitchDebug = 0;
int YawDebug = 0;
int v_PitchDebug = 0;
int v_YawDebug = 1;
int use_vision = 0;
int pitch_add = 0;

/**
 * @brief initialize the parameter of Gimbal
 * @param None
 * @return None
 * @attention  None
 */
void GimbalInit (void)
{
	if(PIT_USEENCODER)
	{
		PitchOuter.kp = 20;//30
		PitchOuter.ki = 0;
		PitchOuter.kd = 0;	
		PitchOuter.errILim = 0;
		PitchOuter.OutMAX = 200;//400
		
		PitchInner.kp = 40;//50
		PitchInner.ki = 0.2;
		PitchInner.kd = 0;
		PitchInner.errILim = 3000;
		PitchInner.OutMAX = 8000;
	}
	else
	{
		PitchOuter.kp = 15;//30
		PitchOuter.ki = 0;
		PitchOuter.kd = 0;	
		PitchOuter.errILim = 0;
		PitchOuter.OutMAX = 200;//400
		
		PitchInner.kp = 40;//50
		PitchInner.ki = 0.2;
		PitchInner.kd = 0;
		PitchInner.errILim = 3000;
		PitchInner.OutMAX = 8000;
	}
	YawOuter.kp = 35;//12
	YawOuter.ki = 0;
	YawOuter.kd = 0;	
	YawOuter.errILim = 0;
	YawOuter.OutMAX = 200;
	
	YawInner.kp = 40;//60
	YawInner.ki = 0.1;
	YawInner.kd = 0;
	YawInner.errILim = 6000;
	YawInner.OutMAX = 25000;
	
	v_PitchOuter.kp = 8;//30
	v_PitchOuter.ki = 0;
	v_PitchOuter.kd = 0;	
	v_PitchOuter.errILim = 0;
	v_PitchOuter.OutMAX = 150;//400
	
	v_PitchInner.kp = 40;//50
	v_PitchInner.ki = 0.25;
	v_PitchInner.kd = 0;
	v_PitchInner.errILim = 3000;
	v_PitchInner.OutMAX = 8000;

	v_YawOuter.kp = 6;//12
	v_YawOuter.ki = 0;
	v_YawOuter.kd = 0;	
	v_YawOuter.errILim = 0;
	v_YawOuter.OutMAX = 40;
	
	v_YawInner.kp = 150;//60
	v_YawInner.ki = 0.2;
	v_YawInner.kd = 0;
	v_YawInner.errILim = 6000;
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
	//越界处理
	if(GimbalData.Pitchposition >= GimbalData.PitchMax||GimbalData.Pitchposition<= GimbalData.PitchMin)
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
	
	if(pitch_overborder==1||yaw_overborder==1||pcdata_right==0||catch_target==0)
	{
		use_vision = 0;
	}
	else
	{
		use_vision =1;
	}
	//选择模式；手动或自动
	if(use_vision==1 && (RC_Ctl.rc.s2==2||KeyMousedata.use_vision==1))
	{
		gimbalmode = AUTO;
		GimbalData.PitchTarget1 = GimbalData.Pitchposition;
		GimbalData.YawTarget1 = GimbalData.Yawposition;
	}
	else
	{
		gimbalmode = HAND;
	}
	
	if(RC_Ctl.rc.s2 == 1||KeyMousedata.laser_on == 1)
	{
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
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
	if(YAW_USEENCODER)
	{
		GimbalData.YawTarget1 = GimbalData.YawBacknow;
	}
	else
	{
		GimbalData.YawTarget1 = GimbalData.Yawangle;
	}	
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
	if(PIT_USEENCODER)
	{
		GimbalData.PitchTarget1 = GimbalData.PitchBacknow;
		GimbalData.PitchMax = 30000;
		GimbalData.PitchMid = 10000;
		GimbalData.PitchMin = -7000;
	}
	else
	{
		GimbalData.PitchTarget1 = GimbalData.Pitchangle;
		GimbalData.PitchMaxangle = 25;
		GimbalData.PitchMidangle = 0;
		GimbalData.PitchMinangle = -5;
	}
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
		if(YAW_USEENCODER)
		{
			GimbalData.YawTarget1 -= (float)(((-RC_Ctl.rc.ch2 + 1024)*0.0003f)*22.75f + RC_Ctl.mouse.x *22.75f* MOUSE_YAW_CONST);
		}
		else
		{
			GimbalData.YawTarget1 += (float)(((-RC_Ctl.rc.ch2 + 1024)*0.0003f) + RC_Ctl.mouse.x * MOUSE_YAW_CONST);
		}
		if(PIT_USEENCODER)
		{
			GimbalData.PitchTarget1 += (float)((((RC_Ctl.rc.ch1 - 1024)*0.0001f)*800 + RC_Ctl.mouse.y *200* MOUSE_PITCH_CONST));
		}
		else
		{
			GimbalData.PitchTarget1 += (float)((((RC_Ctl.rc.ch1 - 1024)*0.0001f) + RC_Ctl.mouse.y * MOUSE_PITCH_CONST));
		}
	}
/******************************limit the yawtarget*************************************/	
	if(YAW_USEENCODER)
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
/***********************limit pitch position**********************************/
	if(PIT_USEENCODER)
	{
		if(GimbalData.PitchTarget1 > GimbalData.PitchMax)
		{
			GimbalData.PitchTarget2 = GimbalData.PitchMax;
			GimbalData.PitchTarget1 = GimbalData.PitchMax;
		}
		else if(GimbalData.PitchTarget1 < GimbalData.PitchMin)
		{
			GimbalData.PitchTarget2 = GimbalData.PitchMin;
			GimbalData.PitchTarget1 = GimbalData.PitchMin;
		}
		else
		{
			GimbalData.PitchTarget2 = GimbalData.PitchTarget1;
		}
	}
	else
	{
		if(GimbalData.PitchTarget1 > GimbalData.PitchMaxangle)
		{
			GimbalData.PitchTarget2 = GimbalData.PitchMaxangle;
			GimbalData.PitchTarget1 = GimbalData.PitchMaxangle;
		}
		else if(GimbalData.PitchTarget1 < GimbalData.PitchMinangle)
		{
			GimbalData.PitchTarget2 = GimbalData.PitchMinangle;
			GimbalData.PitchTarget1 = GimbalData.PitchMinangle;
		}
		else
		{
			GimbalData.PitchTarget2 = GimbalData.PitchTarget1;
		}
	}
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
		PitchOuter.kp=A;//25
		PitchOuter.ki=B;
		PitchOuter.kd=C;	
		PitchOuter.errILim=200;
		PitchOuter.OutMAX=E;
		
		PitchInner.kp=a;//20
		PitchInner.ki=b;
		PitchInner.kd=c;
		PitchInner.errILim=3000;
		PitchInner.OutMAX=e;
	}
	if(PIT_USEENCODER)
	{
		PitchOuter.errNow = (*Target - GimbalData.Pitchposition)*0.001220852f;//处理成角度值
	}
	else
	{
		PitchOuter.errNow = (*Target - GimbalData.Pitchangle);
	}
	PID_AbsoluteMode(&PitchOuter);
//	PitchInner.errNow = PitchOuter.ctrOut -  GimbalData.PitchBackspeed*0.166666667f;
	PitchInner.errNow = PitchOuter.ctrOut -  GimbalData.Pitchspeed;
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
		YawOuter.kp=P;//15
		YawOuter.ki=I;
		YawOuter.kd=D;	
		YawOuter.errILim=0;
		YawOuter.OutMAX=V1;
		
		YawInner.kp=p;//50
		YawInner.ki=i;
		YawInner.kd=d;
		YawInner.errILim=6000;
		YawInner.OutMAX=V2;
	}
	if(YAW_USEENCODER)
	{
		YawOuter.errNow = (*Target - GimbalData.Yawposition) * (-0.0439506f);
	}
	else
	{
		YawOuter.errNow = (*Target - GimbalData.Yawangle);
	}
	PID_AbsoluteMode(&YawOuter);
	YawInner.errNow = YawOuter.ctrOut - GimbalData.YawEncoderspeed*(-5.7608f);
//	YawInner.errNow = YawOuter.ctrOut - GimbalData.Yawspeed;
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
		v_PitchOuter.kp=A;//25
		v_PitchOuter.ki=B;
		v_PitchOuter.kd=C;	
		v_PitchOuter.errILim=0;
		v_PitchOuter.OutMAX=E;
		
		v_PitchInner.kp=a;//20
		v_PitchInner.ki=b;
		v_PitchInner.kd=c;
		v_PitchInner.errILim=3000;
		v_PitchInner.OutMAX=e;
	}
	if(GimbalData.Pitchangle > 0)
	{
		pitch_add = (int)(fabs(GimbalData.Pitchangle) * D);
	}
	else
	{
		pitch_add = 0;
	}
	v_PitchOuter.errNow = (*Target - pcParam.pcCenterY.f)*0.05f;//处理成角度值
	PID_AbsoluteMode(&v_PitchOuter);
	v_PitchInner.errNow = v_PitchOuter.ctrOut -  GimbalData.Pitchspeed;//(1/65536*4000)
//		if(v_PitchInner.errNow > 20)
//	{
//		v_PitchInner.errNow = 20;
//	}
//	else if(v_PitchInner.errNow < -20)
//	{
//		v_PitchInner.errNow = -20;
//	}
	PID_AbsoluteMode(&v_PitchInner);
//	GimbalData.PitchCurrent = v_PitchInner.ctrOut + pitch_add;
	GimbalData.PitchCurrent = v_PitchInner.ctrOut;
//	GimbalData.PitchCurrent = v_PitchOuter.ctrOut + pitch_add;//单环
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
		v_YawOuter.kp=P;//15
		v_YawOuter.ki=I;
		v_YawOuter.kd=D;	
		v_YawOuter.errILim=0;
		v_YawOuter.OutMAX=V1;
		
		v_YawInner.kp=p;//50
		v_YawInner.ki=i;
		v_YawInner.kd=d;
		v_YawInner.errILim=3000;
		v_YawInner.OutMAX=V2;
	}

	v_YawOuter.errNow = (*Target - pcParam.pcCenterX.f)*0.05f;
	PID_AbsoluteMode(&v_YawOuter);
//	v_YawInner.errNow = v_YawOuter.ctrOut - GimbalData.YawEncoderspeed*(-5.7608f);
	v_YawInner.errNow = v_YawOuter.ctrOut - GimbalData.Yawspeed;

	PID_AbsoluteMode(&v_YawInner);
	GimbalData.YawCurrent = (int16_t)(-v_YawInner.ctrOut);
}