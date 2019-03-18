#ifndef __SHOOTCONTROL_H
#define __SHOOTCONTROL_H
#include "tim.h"
#include "stdint.h"

#define BUZZER_CCR             TIM12->CCR1
#define BUZZER_ARR           TIM12->ARR
typedef struct
{
	int PositionInit;
	
	double TargetPosition;
	int64_t CircleCounter;
	int64_t TotalPosition;
	int16_t BackPositionOld;
	int16_t BackPositionNew;
	int16_t BackSpeed;
	int16_t Speed;
	int16_t Current;

}StirMotor;

typedef struct
{
	int16_t BackSpeed;
	int16_t TargetSpeed;
	int16_t Current;
}ShootMotor;

extern StirMotor StirMotorData;
extern ShootMotor fric_l_data,fric_r_data;
extern int16_t FrictionSpd;

void ChooseStirMotorMode(void);
void ShootInit (void);
void PWMInit(void);
void StirPID (int64_t TargetPosition,int16_t BackSpeed,int16_t BackPosition);
void DealStirMotorPosition (void);
void StirMotorStart (void);
void Switchshoot (void);
void fric_pidcontrol(int16_t targetspeed);
void Buzzer_on(int16_t arr,int16_t ccr);
void Buzzer_off(void);
#endif
