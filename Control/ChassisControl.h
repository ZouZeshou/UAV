#ifndef __CHASSISCONTROL_H
#define __CHASSISCONTROL_H
#include "pid.h"
#include "stdint.h"
//-300 1500 pitch
//6000 8000 Yaw
#define NormalParameter 12.12f
#define MaxWheelSpeed 8000
#define MaxCurrentSum 12000 
#define CHASSISDEBUG 0
#define ACCELARATION 1500
#define PITCHMOTORSETLEFT

#define YAWMIN 0
#define YAW_MID_ANGLE 0
#define YAWMAX 90

#define FOLLOW 1
#define ROTATE 2
#define YawinitMidposi 4715

void Chassisfollow(void);
void DealRemotedata(void);
void MecanumCalculate(void);
void ChassisPid (void);
void ChassisInit(void);
void ChooseChassisMode(void);
typedef struct 
{
	int16_t Vx;
	int16_t Vy;
	int16_t Rotate;
	int16_t Vxabs;
	int16_t Vyabs;
	int16_t BackSpeed[4];
	int Speed[4];
	int Buffspeed[4];
	int16_t Current[4];
	float CurrentSum;

}ChassisSpeed;


extern int ChassisMode;
extern ChassisSpeed Chassisdata;
extern float currentparam;
extern int YawMidPosi;
extern float ChassisAngle2;
extern PID_AbsoluteType Chassiswheelpid[4];
extern PID_AbsoluteType ChassisfollowInner,ChassisfollowOutter;
#endif
