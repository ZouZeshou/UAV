#include "ShootControl.h"
#include "pid.h"
#include "BSP_usart.h"
#include "DBUS.h"
#include "STMGood.h"
#include "Keyboard.h"
#define STIRADDITION 29487.6 //8191*36/10
int16_t FrictionSpd = 0;
int16_t ShootFrequency = 10;//1000/5/25

PID_AbsoluteType StirMotorOutterPID,StirMotorInnerPID;
PID_AbsoluteType fric_l_pid,fric_r_pid;
StirMotor StirMotorData = {0};
ShootMotor fric_l_data = {0};
ShootMotor fric_r_data = {0};
int StirMotorMode;
int StirUpdateCounter;
int StirMotorDebug = 0;
int fric_debug = 0;
/**
 * @brief initialise the data will be used in shoot motor
 * @param None
 * @return None
 * @attention None
 */
void ShootInit (void)
{
	StirMotorOutterPID.kp = 20;
	StirMotorOutterPID.ki = 0;
	StirMotorOutterPID.kd = 0;
	StirMotorOutterPID.errILim = 1000;
	StirMotorOutterPID.OutMAX = 1250;
	
	StirMotorInnerPID.kp = 30;
	StirMotorInnerPID.ki = 0;
	StirMotorInnerPID.kd = 0;
	StirMotorInnerPID.errILim = 3000 ;
	StirMotorInnerPID.OutMAX = 8000 ;
	
	fric_l_pid.kp = 11;
	fric_l_pid.ki = 0.15f;
	fric_l_pid.kd = 0;
	fric_l_pid.errILim = 6000;
	fric_l_pid.OutMAX = 10000;
	
	fric_r_pid.kp = 11;
	fric_r_pid.ki = 0.15f;
	fric_r_pid.kd = 0;
	fric_r_pid.errILim = 6000;
	fric_r_pid.OutMAX = 10000;
}
/**
 * @brief initialise the data will be used in shoot motor
 * @param None
 * @return None
 * @attention None
 */
void PWMInit(void)
{
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);//buzzer
	HAL_TIM_PWM_Start(&htim3,  TIM_CHANNEL_2); // ctrl imu temperature
	Buzzer_off();
}
/**
 * @brief Transform the discrete position to a continuous position 
 * @param None
 * @return None
 * @attention this function should be operation after the motor had electricity;
 */
void DealStirMotorPosition ()
{
	if(StirMotorData.PositionInit)
	{
		if(StirMotorData.BackPositionNew-StirMotorData.BackPositionOld>5000)
			StirMotorData.CircleCounter--;
		if(StirMotorData.BackPositionNew-StirMotorData.BackPositionOld<-5000)
			StirMotorData.CircleCounter++;
	}
	StirMotorData.BackPositionOld = StirMotorData.BackPositionNew;	
	StirMotorData.TotalPosition = StirMotorData.BackPositionNew + StirMotorData.CircleCounter*8192;
	if(StirMotorData.PositionInit == 0)
		StirMotorData.TargetPosition = StirMotorData.TotalPosition;
	StirMotorData.PositionInit=1;
}
/**
 * @brief update the target position of stirmotor 
 * @param None
 * @return None
 * @attention None
 */
void StirMotorStart (int16_t * ShootFrequency)
{	
		static double position_diff;
		static int jam_count;
		position_diff = StirMotorData.TargetPosition - StirMotorData.TotalPosition;
		if(StirUpdateCounter++ >= *ShootFrequency && position_diff < 3*STIRADDITION)
		{
			HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_2);	
			StirMotorData.TargetPosition += STIRADDITION ;
			StirUpdateCounter = 0;
		}
		if(position_diff >= 3*STIRADDITION)
		{
			if(jam_count++ >= 100)
				StirMotorData.TargetPosition-= position_diff + STIRADDITION;
		}
		else
		{
			jam_count = 0;
		}
}
/**
 * @brief switch for stirmotor and ShootControl
 * @param None
 * @return None
 * @attention for debug
 */
void Switchshoot (void)
{
	static int32_t Friction_ok = 0;
	
	if(RC_Ctl.rc.s2 == 2||KeyMousedata.fric_start)
	{
		FrictionSpd = 9500;
	}
	else
		FrictionSpd = 0;
	
	if(RC_Ctl.rc.s1 == 2||KeyMousedata.stir_start)
	{
		ShootFrequency = 10;
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
		StirMotorStart(&ShootFrequency);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
	}
}
/**
 * @brief  turn on the buzzer 
 * @param None
 * @return None
 * @attention for debug
 */
void Buzzer_on(int16_t arr,int16_t ccr)
{
	BUZZER_ARR = arr;
	BUZZER_CCR = ccr;
}
/**
 * @brief  turn off the buzzer 
 * @param None
 * @return None
 * @attention for debug
 */
void Buzzer_off(void)
{
	BUZZER_CCR =0;
}
/**
 * @brief calculate the final current of frictionMotor by PID controller
 * @param Targetspeed
 * @return None
 * @attention None
 */
void fric_pidcontrol(int16_t targetspeed)
{
	if(fric_debug == 1)
	{
		fric_l_pid.kp = P;
		fric_l_pid.ki = I;
		fric_l_pid.kd = D;
		fric_l_pid.errILim = 0;
		fric_l_pid.OutMAX = V1;
		
		fric_r_pid.kp = P;
		fric_r_pid.ki = I;
		fric_r_pid.kd = D;
		fric_r_pid.errILim = 0;
		fric_r_pid.OutMAX = V1;
	}
	fric_l_pid.errNow = -targetspeed - fric_l_data.BackSpeed;
	PID_AbsoluteMode(&fric_l_pid);
	fric_l_data.Current = (int16_t)(fric_l_pid.ctrOut);
	
	fric_r_pid.errNow = targetspeed - fric_r_data.BackSpeed;
	PID_AbsoluteMode(&fric_r_pid);
	fric_r_data.Current = (int16_t)(fric_r_pid.ctrOut);
}
/**
 * @brief calculate the final current of StirMotor by PID controller
 * @param TargetPosition
 * @return None
 * @attention None
 */
void StirPID (int64_t TargetPosition,int16_t BackSpeed,int16_t BackPosition)
{
	if(StirMotorDebug == 1)
	{
		StirMotorOutterPID.kp = P;//0.3(*36)
		StirMotorOutterPID.ki = I;
		StirMotorOutterPID.kd = D;
		StirMotorOutterPID.errILim = 1000;
		StirMotorOutterPID.OutMAX = V1;//160
		
		StirMotorInnerPID.kp = p;//100
		StirMotorInnerPID.ki = i;
		StirMotorInnerPID.kd = d;
		StirMotorInnerPID.errILim = 3000 ;
		StirMotorInnerPID.OutMAX = V2 ;//8000
	}	
	StirMotorOutterPID.errNow = (TargetPosition - StirMotorData.TotalPosition)*0.001220852f;//angle(∂»)
	PID_AbsoluteMode(&StirMotorOutterPID);
	StirMotorInnerPID.errNow = (StirMotorOutterPID.ctrOut - BackSpeed*0.166666667f);//angle velocity£®∂»/√ø√Î£©
	PID_AbsoluteMode(&StirMotorInnerPID);
	StirMotorData.Current = (int16_t)(StirMotorInnerPID.ctrOut);
}