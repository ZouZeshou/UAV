#include "camera.h"
#include "kalman_filter.h"
#include "usart.h"
#include "CRC_Check.h"
#define VISIONDATALENGTH 23
#define REFER_CENTER_X  510                //295  443
#define REFER_CENTER_Y 435

kalman1_state kalmanl;
float data = 0;
int data_len = 0;
float out1 = 0;
int16_t Times = 20000;
uint8_t uart6_buff[50];
int vlostcount;
pcDataParam pcParam,pcParamLast;
extern float IMG_KP,IMG_KI,IMG_KD,IMG_ERRILIM,IMG_MAXOUT;
uint16_t cnt1 = 0;
uint16_t cnt2 = 0;
int flaglose = 1;
int flagshoot = 0;
extern int16_t TmsY;
float sbs = 0;
void pcDataInit(void)
{
	kalman1_init(&kalmanl,data,5e7);
	pcParam.isTrue = 0;
	
	pcParam.pcTargetX=0.f;
	pcParam.pcTargetY=0.f;
	pcParam.adjustX=0;
	pcParam.adjustY=0;

	pcParam.CompXout=0;
	pcParam.CompYout=0;
			
	pcParam.refer_centerX = REFER_CENTER_X;
	pcParam.refer_centerY = REFER_CENTER_Y;
}

void Vision_IRQ(void){
	if(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE) != RESET){
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);		
		HAL_UART_DMAStop(&huart6);
		HAL_UART_Receive_DMA(&huart6,uart6_buff,VISIONDATALENGTH);
	}
}

void Vision_Decode(void)
{
	if(uart6_buff[0]==0xA5&& Verify_CRC16_Check_Sum(uart6_buff,VISIONDATALENGTH))//
	{

//		TmsY = 0;
		
		pcParam.pcCenterX.uc[0] = uart6_buff[1];
		pcParam.pcCenterX.uc[1] = uart6_buff[2];
		pcParam.pcCenterX.uc[2] = uart6_buff[3];
		pcParam.pcCenterX.uc[3] = uart6_buff[4];
				
		pcParam.pcCenterY.uc[0] = uart6_buff[5];
		pcParam.pcCenterY.uc[1] = uart6_buff[6];
		pcParam.pcCenterY.uc[2] = uart6_buff[7];
		pcParam.pcCenterY.uc[3] = uart6_buff[8];
				
		pcParam.pcCenterZ.uc[0] = uart6_buff[9];
		pcParam.pcCenterZ.uc[1] = uart6_buff[10];
		pcParam.pcCenterZ.uc[2] = uart6_buff[11];
		pcParam.pcCenterZ.uc[3] = uart6_buff[12];
				
		pcParam.pcCompensationX.uc[0] = uart6_buff[13];
		pcParam.pcCompensationX.uc[1] = uart6_buff[14];
		pcParam.pcCompensationX.uc[2] = uart6_buff[15];
		pcParam.pcCompensationX.uc[3] = uart6_buff[16];
				
		pcParam.pcCompensationY.uc[0] = uart6_buff[17];
		pcParam.pcCompensationY.uc[1] = uart6_buff[18];
		pcParam.pcCompensationY.uc[2] = uart6_buff[19];
		pcParam.pcCompensationY.uc[3] = uart6_buff[20];		
        
		if((pcParam.pcCenterX.f)<5&&(pcParam.pcCenterY.f)<5)
		{
			cnt1++;
			if(cnt1>=5)
			{
				cnt1 = 6;
				flaglose = 1;
				flagshoot = 0;
			}
			else
			{
				flagshoot = 1;
				pcParam.pcCenterX = pcParamLast.pcCenterX;
				pcParam.pcCenterY = pcParamLast.pcCenterY;
				pcParam.pcCenterZ = pcParamLast.pcCenterZ;
				pcParam.pcCompensationX = pcParamLast.pcCompensationX;
				pcParam.pcCompensationY = pcParamLast.pcCompensationY;
			}
		}
		else
		{
				cnt1 = 0;
				flaglose = 0;
				flagshoot = 1;
				pcParamLast.pcCenterX = pcParam.pcCenterX;
				pcParamLast.pcCenterY = pcParam.pcCenterY;
				pcParamLast.pcCenterZ = pcParam.pcCenterZ;
				pcParamLast.pcCompensationX = pcParam.pcCompensationX;
				pcParamLast.pcCompensationY = pcParam.pcCompensationY;			
		}
		
		if(flaglose == 0)
		{
			pcParam.pcTargetX = pcParam.refer_centerX - pcParam.pcCenterX.f;//
			pcParam.pcTargetY = pcParam.pcCenterY.f - REFER_CENTER_Y +0;
		}
		else
		{
			pcParam.pcTargetX = 0;
			pcParam.pcTargetY = 0;
		}
		if(pcParamLast.pcCompensationX.f>30) pcParamLast.pcCompensationX.f=0;
		data = pcParamLast.pcCompensationX.f;
		out1 = -kalman1_filter(&kalmanl,data)*6.6;
	}
	__HAL_UART_CLEAR_PEFLAG(&huart6);
}


