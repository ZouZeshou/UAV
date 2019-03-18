#ifndef __DETECT_H
#define __DETECT_H
#include "BSP_can.h"

#define GYROOFFLINE 1
#define GYROABNORMAL 2
#define GYRONORMAL 3

#define ONLINE 1
#define OFFLINE 0

extern uint8_t Devicestate[13];
extern uint8_t Offline[13];
void GetDeviceState(void);
void DeviceDetect(uint8_t *state,uint8_t *result);
int JudgeGyro(GyroData * Gyro,int RT_fps);
int JudgeDeviceState(int fps,int i);



#endif
