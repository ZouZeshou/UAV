#include "detect.h"

uint8_t Devicestate[11] = {0};
uint8_t Offline[11] = {0};
//	 Wheel_1;   0
//	 Wheel_2;   1
//	 Wheel_3;   2
//   Wheel_4;		3
//	 Yaw;				4
//	 Pit;				5
//	 Stir;			6
//	 Gyro_1;		7
//	 Gyro_2;		8
//	 Gyro_3;		9
//	 DBUS;      10
/**************zzs_add***************/
/**
 * @brief Judge the state of Gyroscope
 * @param gy,gz,angle,fps
 * @return state of Gyroscope
 * @attention  None
 */
int JudgeGyro(GyroData * Gyro,int  RT_fps)
{
	static float angleold = 0, anglechange = 0;
	static int RT_fpsOld = 0 ,RT_fpsChange = 0;
	anglechange = Gyro->angleyaw - angleold;
	angleold = Gyro->angleyaw;
	RT_fpsChange = RT_fps - RT_fpsOld;
	RT_fpsOld = RT_fps;
	if(RT_fpsChange == 0)
	{
		return GYROOFFLINE;
	}
	else if(abs(Gyro->gy) >= 20)
	{
		return GYROABNORMAL;
	}
	else if(abs(Gyro->gz) >= 20)
	{
		return GYROABNORMAL;
	}
	else if(fabs(anglechange) >= 0.05)
	{
		return GYROABNORMAL;
	}
	else
	{
		return GYRONORMAL;
	}
}
/**
 * @brief Judge the device online or offline
 * @param 
 * @return 
 * @attention  None
 */

void DeviceDetect(uint8_t *state,uint8_t *result)
{
	static int counter[11] = {0};
	for(int i=0;i<11;i++)
	{
		if(state[i] == OFFLINE)
		{
			if(counter[i]++ > 5 && state[i] == OFFLINE)
			{
				result[i] = OFFLINE;
			}
		}
		else
		{
			counter[i] = 0;
			result[i] = ONLINE;
		}
	}
}

void GetDeviceState(void)
{
	
	Devicestate[0] = JudgeDeviceState(fps.Wheel_1,0);
	Devicestate[1] = JudgeDeviceState(fps.Wheel_2,1);
	Devicestate[2] = JudgeDeviceState(fps.Wheel_3,2);
	Devicestate[3] = JudgeDeviceState(fps.Wheel_4,3);
	Devicestate[4] = JudgeDeviceState(fps.Yaw,4);
	Devicestate[5] = JudgeDeviceState(fps.Pit,5);
	Devicestate[6] = JudgeDeviceState(fps.Stir,6);
	Devicestate[7] = JudgeDeviceState(fps.Gyro_1,7);
	Devicestate[8] = JudgeDeviceState(fps.Gyro_2,8);
	Devicestate[9] = JudgeDeviceState(fps.Gyro_3,9);
	Devicestate[10] = JudgeDeviceState(fps.DBUS,10);
}

int JudgeDeviceState(int fps,int i)
{
	static int fpsold[11] = {0};
	static int fpschange[11] = {0};
	fpschange[i] = fps - fpsold[i];
	fpsold[i] = fps;
	if(fpschange[i] == 0)
		return OFFLINE;
	else
		return ONLINE;	
}

