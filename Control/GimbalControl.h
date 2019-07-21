#ifndef __GIMBALCONTROL_H
#define __GIMBALCONTROL_H
#include "stdint.h"
#include "pid.h"
#include "camera.h"

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
	volatile int16_t YawCurrent;
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
	int16_t PitchBackspeed;
	int32_t Pitchposition;
	int32_t Pitchpositionold;
	int16_t PitchEncoderspeed;
	int16_t Pitchspeed;
	float   Pitchangle;
	int32_t Pitchcirclecounter;
	int16_t Pitchinit;
	volatile int16_t PitchCurrent;
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
extern PID_AbsoluteType YawOuter;
extern PID_AbsoluteType PitchInner;
extern PID_AbsoluteType PitchOuter;
extern PID_AbsoluteType v_YawInner;
extern PID_AbsoluteType v_YawOuter;
extern PID_AbsoluteType v_PitchInner;
extern PID_AbsoluteType v_PitchOuter;
extern PID_AbsoluteType pixel_pid;
extern  int YawTargetEncoder ,PitchTargetEncoder ;
extern int gimbalmode;
extern int use_vision;
extern int pitch_add ;
void GimbalInit (void);
void GimbalCalibration(void);
void GetGimbalTarget_yaw(void);
void GetGimbalTarget_pit(void);
void PitchPID(float *Target);
void DealGimbalPosition(void);
void YawPID(float *Target);
void v_PitchPID (float *Target);
void v_YawPID(float *Target);
void GetYawIncrement(void);
void switch_gimbal_mode(void);
void limit_target(void);
void pixel_to_encoder(float pixel_value,int pixel_center,float* target);

#endif
