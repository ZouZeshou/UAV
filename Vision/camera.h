#ifndef _CAMERA_H_
#define _CAMERA_H_
#include "stdint.h"
#include "FreeRTOS.h"
typedef union{
	int16_t d;
	uint8_t c[2];
}int16uchar;

typedef union{
	float f;
	uint8_t c[4];
}float4uchar;

typedef union{
   float f;
	 unsigned char uc[4];
}float2uchar;


typedef struct {
	 uint8_t isTrue; 
	
	 float pcTargetX;//adjust value
	 float pcTargetY;
	
	 float refer_centerX;
	 float refer_centerY;
	
	 float2uchar pcCenterX;
	 float2uchar pcCenterY;
	 float2uchar pcCenterZ;
	
	 float2uchar pcCompensationX;
	 float2uchar pcCompensationY;
	
	 float CompXout;
	 float CompYout;
	
	 float adjustX;
	 float adjustY;
} pcDataParam;


extern pcDataParam pcParam,pcParamLast;
extern float sbs;
extern int flaglose;

void Vision_IRQ(void);
void pcDataInit(void);
void Vision_Decode(void);
void VisionInit(void);

#endif
