
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "gpio.h"
#include "dma.h"
#include "usart.h"
#include "main.h"


#include "stdio.h"
#include "stdint.h"
#include "math.h"
#include "string.h"
#include "stdbool.h"

#include "can.h"
#include "BSP_can.h"
#include "ChassisControl.h"
#include "delay.h"
#include "BSP_usart.h"
#include "pid.h"
#include "GimbalControl.h"
#include "ahrs.h"
#include "ShootControl.h"
#include "writtingtask.h"
#include "DBUS.h"
#include "Keyboard.h"
#include "STMGood.h"
#include "detect.h"
#include "ANO_DT.h"
#include "drv_imu.h"
int initmark=0;
int IMU_OK = 0;
int GYRO_OK = 0;
int Stop_GyroJudge = 0;
int GyroscopeState = 0;
int Task_03Init = 0;
extern uint8_t Remotebuffer[18];
/**
 * @brief main control task
 * @param None
 * @return None
 * @attention None
 */
void StartTask02(void const * argument)
{
	for(;;)
  {
		if(IMU_OK)
		{
			taskENTER_CRITICAL();
			if(initmark==0)
			{
				GimbalCalibration();
				initmark=1;
			}
			DealGimbalPosition();
			switch_gimbal_mode();
			GetGimbalTarget();			
			PitchPID(&GimbalData.PitchTarget2);
			YawPID(&GimbalData.YawTarget2);	
	  
			DealKeyMousedata();
			Switchshoot();
			fric_pidcontrol(FrictionSpd);
			StirPID (StirMotorData.TargetPosition,StirMotorData.BackSpeed,StirMotorData.BackPositionNew);
		//ShootControl
	  
			taskEXIT_CRITICAL();
		}
	
    osDelay(5);
  }

}
/**
 * @brief can send task
 * @param None
 * @return None
 * @attention None
 */
void StartTask03(void const * argument)
{
	for(;;)
  {
		if(IMU_OK)
		{
			if(Task_03Init == 0)
			{
				Buzzer_on(300,150);
				xdelay_ms(300);
				Buzzer_off();
				Task_03Init = 1;
			}
				Can1_SendMsg(0x1FF,GimbalData.YawCurrent,GimbalData.PitchCurrent,StirMotorData.Current,0);
				Can2_SendMsg(0x200,fric_l_data.Current,fric_r_data.Current,0,0);
		}
     osDelay(5);
  }

}
/**
 * @brief updata the data of onboard Gyroscope
 * @param None
 * @return None
 * @attention None
 */
void StartTask04(void const * argument)
{
	static int wait = 0;
	for(;;)
  {
		if(wait++ > 2500)
		{
			IMU_OK = 1;
			wait = 3000;
		}
		mpu_get_data(&sensor);
		UpdateIMU(&sensor);
//		mahony_ahrs_update(&sensor,&atti);
		imu_temp_keep();
		osDelay(2);
  }

}
/**
 * @brief LED Toggle
 * @param None
 * @return None
 * @attention None
 */
void StartTask05(void const * argument)
{	
	for(;;)
  {
		if(IMU_OK)
		{
			HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_1);		
			PrintFunction();
			ANO_DT_Data_Exchange();
			osDelay(50);
		}
  }

}
/**
 * @brief detect
 * @param None
 * @return None
 * @attention None
 */
void StartTask06(void const * argument)
{
	for(;;)
  {
		static int buzzer_ontime;
		GetDeviceState();
		DeviceDetect(Devicestate,Offline);
//		if(Devicestate[4]==OFFLINE||Devicestate[5]==OFFLINE||Devicestate[6]==OFFLINE||Devicestate[11]==OFFLINE||Devicestate[12]==OFFLINE)
//		{
//			buzzer_ontime++;
//			if(buzzer_ontime >= 20 && buzzer_ontime <= 40)
//			{
//				Buzzer_on(300,150);	
//			}
//			else if(buzzer_ontime > 40)
//			{
//				buzzer_ontime = 0;
//			}
//		}
//		else
//		{
//			Buzzer_off();
//		}
		osDelay(15);
	}
}
/**
 * @brief 07
 * @param None
 * @return None
 * @attention None
 */
void StartTask07(void const * argument)
{
	static int counter1 = 0,counter2 = 0;
	for(;;)
  {
		
		osDelay(15);
  }

}
/**
 * @brief 调试时需用到的打印数据函数
 * @param None
 * @return None
 * @attention None
 */
void PrintFunction(void)
{

/***************************************************** chassisdebug ********************************************/
//	  printf("work\r\n");
//	  printf("targetspeed %d %d %d %d\r\n",Chassisdata.Speed[0],Chassisdata.Speed[1],Chassisdata.Speed[2],Chassisdata.Speed[3]);
//		printf("BackSpeed %d %d %d %d\r\n",Chassisdata.BackSpeed[0],Chassisdata.BackSpeed[1],Chassisdata.BackSpeed[2],Chassisdata.BackSpeed[3]);
////	  printf("current %d %d %d %d\r\n",Chassisdata.Current[0],Chassisdata.Current[1],Chassisdata.Current[2],Chassisdata.Current[3]);
////	  printf("Remotebuffer %d %d %d %d\r\n",Remotebuffer[0],Remotebuffer[1],Remotebuffer[2],Remotebuffer[3]);
	  printf("channel l%d r%d x%d y%d z%d\r\n",RC_Ctl.mouse.press_l,RC_Ctl.mouse.press_r,RC_Ctl.mouse.x,RC_Ctl.mouse.y,RC_Ctl.mouse.z);
//	  printf("Vx %d Vy %d Rotate %d\r\n",Chassisdata.Vx,Chassisdata.Vy,Chassisdata.Rotate);
//	  printf("error %f %f %f %f\r\n",Chassiswheelpid[0].errNow,Chassiswheelpid[1].errNow,Chassiswheelpid[2].errNow,Chassiswheelpid[3].errNow);
//	  printf("ctrout %f %f %f %f\r\n",Chassiswheelpid[0].ctrOut,Chassiswheelpid[1].ctrOut,Chassiswheelpid[2].ctrOut,Chassiswheelpid[3].ctrOut);
//	  printf("currentparam %f\r\n",currentparam);
//	  printf("currentSum %f\r\n",Chassisdata.CurrentSum);
//	  printf("pid %f %f %f %f\r\n",Chassiswheelpid[0].kp,Chassiswheelpid[1].kp,Chassiswheelpid[2].kp,Chassiswheelpid[3].kp);
//			printf("FoOut err%f out%f\r\n ",ChassisfollowOutter.errNow,ChassisfollowOutter.ctrOut);
//			printf("Foinner err%f out%f\r\n",ChassisfollowInner.errNow,ChassisfollowInner.ctrOut);
	
/*************************************************** Gimbaldebug ***********************************************/
		  printf("/*******************Gimbal******************/ \r\n");
	//	printf("q0%f q1%f q2%f q3%f\r\n",q0,q1,q2,q3);
//printf("fps.Gyro_1%d\r\n",fps.Gyro_1);
//	printf("GYROSTATE %d\r\n",GyroscopeState);
	printf("GYRO angyaw%.3f angpit%.3f angroll%.3f\r\n",Gyroscope.angleyaw,Gyroscope.anglepitch,Gyroscope.angleroll);
//	printf("fpsdbus %d\r\n",fps.DBUS);
//	printf("Yaw %dpittarencoder%d\r\n",YawTargetEncoder,PitchTargetEncoder);
//	printf("GYRO_OK %d\r\n",GYRO_OK);
//	printf("offline %d %d %d %d %d %d %d %d %d %d %d\r\n",Offline[0],Offline[1],Offline[2],Offline[3],
//	Offline[4],Offline[5],Offline[6],Offline[7],Offline[8],Offline[9],Offline[10]);
//	printf("fps %d %d %d %d %d %d %d %d %d %d %d\r\n",fps.Wheel_1,fps.Wheel_2,fps.Wheel_3,fps.Wheel_4,fps.Yaw,
//	fps.Pit,fps.Stir,fps.Gyro_1,fps.Gyro_2,fps.Gyro_3,fps.DBUS);
//	printf("Devicestate %d %d %d %d %d %d %d %d %d %d %d\r\n",Devicestate[0],Devicestate[1],Devicestate[2],Devicestate[3],
//	Devicestate[4],Devicestate[5],Devicestate[6],Devicestate[7],Devicestate[8],Devicestate[9],Devicestate[10]);
//		printf("yawspd %d pitspd %d\r\n",GimbalData.Yawspeed,GimbalData.Pitchspeed);
//		printf("yawencoderspeed %d\r\n",GimbalData.YawEncoderspeed*(-57608)/10000);
//		printf("pitMax %d Min %d\r\n",GimbalData.PitchMaxangle,GimbalData.PitchMinangle);
//		printf("Yawang %.4f pitang %.4f\r\n",GimbalData.Yawangle,GimbalData.Pitchangle);
//	  printf("backpos Yaw %d Pitch %d\r\n",GimbalData.YawBacknow,GimbalData.PitchBacknow);
//		printf("backold Yaw %d Pitch %d\r\n",GimbalData.YawBackold,GimbalData.PitchBackold);
//	  printf("totalpos Yaw %d Pitch %d\r\n",GimbalData.Yawposition,GimbalData.Pitchposition);
//		printf("counter pit %d\r\n",GimbalData.Pitchcirclecounter);
//		printf("Pitinit%d\r\n",GimbalData.Pitchinit);
//	  printf("gyro x %d y %d z %d\r\n",imu_data.gx,imu_data.gy,imu_data.gz);
//	  printf("Yawtaget %f Pittaget %f\r\n",GimbalData.YawTarget,GimbalData.PitchTarget);
//			printf("error yaw%.2f pitch%.2f\r\n",YawOutter.errNow,PitchOutter.errNow);
//			printf("Yaw outter %.2f inner %.2f\r\n",YawOutter.ctrOut,YawInner.ctrOut);
////			printf("Yawcur %d\r\n",GimbalData.YawCurrent); 
//			printf("Pitch outter %.2f inner %.2f\r\n",PitchOutter.ctrOut,PitchInner.ctrOut);
//			printf("Gyro1 gy%d gz%d ang%4.f\r\n",Gyroscope1.gy,Gyroscope1.gz,Gyroscope1.angle);
//			printf("Gyro1State %d\r\n",Gyro1State);
//			printf("Gyro2 gy%d gz%d ang%f\r\n",Gyroscope2.gy,Gyroscope2.gz,Gyroscope2.angle);
//		 printf("PitSpd %d YawSpd %d\r\n",GimbalData.Pitchspeed,GimbalData.Yawspeed);
////  printf("FollowctrOut %f\r\n",ChassisfollowOutter.ctrOut);
////	  printf("GimbalImu %f\r\n",GimbalData.ImuData);
/****************************************************** ShootControldebug ***************************************************/
//	  printf("/*******************Stir******************/ \r\n");
//	  printf(" current %d BackSpeed %d Targetpos %lld\r\n",StirMotorData.Current,StirMotorData.BackSpeed,StirMotorData.TargetPosition);
//	  printf("newpos %d oldpos %d totalpos %lld \r\n",StirMotorData.BackPositionNew,StirMotorData.BackPositionOld,StirMotorData.TotalPosition);
//		printf("StirTar %lld\r\n",StirMotorData.TargetPosition);
//			printf(" Stir %d \r\n",StirMotorData.Current);
//	  printf("PIDInner error %f ctrout %f\r\n",StirMotorInnerPID.errNow,StirMotorInnerPID.ctrOut);
/*************************/
//printf("sinf(%f) %f\r\n",P,sinf(P));
//printf("cosf(%f) %f\r\n",I,cosf(I));
//printf("ChassisAngle %f\r\n",ChassisAngle2);
}
