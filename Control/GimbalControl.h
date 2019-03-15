#ifndef __GIMBALCONTROL_H
#define __GIMBALCONTROL_H
#include "stdint.h"
#include "pid.h"

typedef struct
{
	int16_t YawBacknow;
	int16_t YawBackold;
	int32_t Yawposition;
	int32_t Yawpositionold;
	int16_t YawEncoderspeed;
	int16_t Yawspeed;
	float   Yawangle;
	int16_t Yawcirclecounter;
	int16_t Yawinit;
	int16_t YawCurrent;
	float YawTarget;
	float YawTarget1;
	float YawTarget2;
	float YawTarget3;
	float YawTarget4;
	int YawMid;
	int YawMax;
	int YawMin;
	int YawMidangle;
	int YawMaxangle;
	int YawMinangle;
	
	
	int16_t PitchBacknow;
	int16_t PitchBackold;
	int32_t Pitchposition;
	int32_t Pitchpositionold;
	int16_t Pitchspeed;
	float   Pitchangle;
	int32_t Pitchcirclecounter;
	int16_t Pitchinit;
	int16_t PitchCurrent;
	float PitchTarget;
	float PitchTarget1;
	float PitchTarget2;
	float PitchTarget3;
	float PitchTarget4;
	int PitchMid;
	int PitchMax;
	int PitchMin;
	int PitchMidangle;
	int PitchMaxangle;
	int PitchMinangle;
	
	float ImuData;
}GimbalMotor;

extern GimbalMotor GimbalData;
extern PID_AbsoluteType YawInner;
extern PID_AbsoluteType YawOutter;
extern PID_AbsoluteType PitchInner;
extern PID_AbsoluteType PitchOutter;
extern  int YawTargetEncoder ,PitchTargetEncoder ;

void GimbalInit (void);
void GimbalCalibration(void);
void GetGimbalTarget(void);
void DealGimbalPosition(void);
void PitchPID(float *Target);
void YawPID(float *Target);
void GetYawIncrement(void);

#endif
