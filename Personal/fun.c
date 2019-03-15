/*Include the header file which are the system product automatically*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "gpio.h"
#include "dma.h"
#include "usart.h"
#include "main.h"
#include "can.h"
/*Include the header file which are frome C lib*/
#include "stdio.h"
#include "stdint.h"
#include "math.h"
#include "string.h"
#include "stdbool.h"
#define sin 0.707106781
#define cos 0.707106781
#define R 100..0
//int16_t errnow = 0;
//int16_t errold = 0;
//extern int finalspeed;
//int64_t position = 0;
//double angle = 0;
//double all = 8191 * 36;
//double angle_to_position = 0;
//double countercicle;
//extern PID_AbsoluteType pid;
//void Position_conversion(int16_t Position1){
//	errnow = Position1 - errold;
//	errold = Position1;
//	if(finalspeed>=0)
//	{
//	if(errnow < -300)
//		errnow = 8191 +errnow;
//	}
//	else if(finalspeed<0)
//	{
//	if(errnow > 200)
//		errnow = -8191 + errnow;
//	}
//	position += errnow;
////	countercicle = position/all;
////	angle = countercicle*360;
//	
//}

int16_t PositionChange=0,OldBackPosition=0;
int64_t Distance=0;
extern int64_t D1,D2,D3,D4;
float Xi,Yi,Anglei;
int64_t Position_Addition (int16_t BackPosition,int16_t BackSpeed)//处理里程函数
{
	PositionChange = BackPosition - OldBackPosition;
	OldBackPosition = BackPosition;
		if(BackSpeed>=0)
	{
	if(PositionChange < -300)
		PositionChange = 8191 +PositionChange;
	}
	else if(BackSpeed<0)
	{
	if(PositionChange > 200)
		PositionChange = -8191 + PositionChange;
	}
	Distance += PositionChange;


	return Distance ;

}


/************计算坐标**************/
//void Calculate_Position(int64_t d1,int64_t d2,int64_t d3,int64_t d4)
//{
//	Xi = (D4-D2-D3+D1)*sin/2.0;
//	Yi = (D4-D2+D3-D1)*cos/2.0;
//	Anglei = (D1+D2+D3+D4)/R ;




//}








