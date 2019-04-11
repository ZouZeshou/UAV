#include "camera.h"
#include "kalman_filter.h"
#include "usart.h"
#include "CRC_Check.h"
#define VISIONDATALENGTH 23
#define REFER_CENTER_X  640                //295  443
#define REFER_CENTER_Y 360

kalman1_state kalmanl;
float data = 0;
int pcdata_right = 0;
uint8_t uart6_buff[50];
pcDataParam pcParam,pcParamLast;


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
	if(uart6_buff[0]==0xA5&& Verify_CRC16_Check_Sum(uart6_buff,VISIONDATALENGTH))
	{
		pcdata_right = 1;
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
    
	}
	else
	{
		pcdata_right = 0;
	}
	__HAL_UART_CLEAR_PEFLAG(&huart6);
}

void sendSignal(uint8_t signal){
	uint8_t data[5];
	data[0]=0xA6;
	data[1]=signal;
	Append_CRC8_Check_Sum(data,3);
	data[3] = '\r';
	data[4] = '\n';
	HAL_UART_Transmit_IT(&huart6,data,5); 
}

