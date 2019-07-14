#include "camera.h"
#include "kalman_filter.h"
#include "usart.h"
#include "CRC.h"
#include "STMGood.h"
#include "Judge.h"
#include "Keyboard.h"
int sending_to_pc = 0;
int16_t center_x = 420 ;               //295  443
int16_t center_y = 340 ;
kalman1_state kalmanl;
float data = 0;
int pcdata_right = 0;
int catch_target = 0;
int catch_target_counter1 = 0;
int catch_target_counter2 = 0;
int camera_center_debug = 0;
uint8_t uart6_buff[50];
pcDataParam pcParam,pcParamLast;


void pcDataInit(void)
{
	kalman1_init(&kalmanl,data,5e7);
	
	pcParam.pcTargetX=0.f;
	pcParam.pcTargetY=0.f;
	pcParam.adjustX=0;
	pcParam.adjustY=0;

	pcParam.CompXout=0;
	pcParam.CompYout=0;

}

void Vision_IRQ(void){
	if(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE) != RESET){
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);		
		HAL_UART_DMAStop(&huart6);
		HAL_UART_Receive_DMA(&huart6,uart6_buff,23);
	}
}

void Vision_Decode(void)
{
	if(camera_center_debug==1)
	{
		pcParam.refer_centerX = P;
		pcParam.refer_centerY = I;
	}
	else
	{
		pcParam.refer_centerX = center_x;
		pcParam.refer_centerY = center_y;
	}	

	
	if(uart6_buff[0]==0xA5&& Verify_CRC16_Check_Sum(uart6_buff,23))
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
		
    if(pcParam.pcCenterX.f<0||pcParam.pcCenterY.f<0)
		{
			if(catch_target_counter1++ > 30)
			{
				catch_target = 0;
				catch_target_counter1 = 0;
				catch_target_counter2 = 0;
			}
			pcParam.pcCenterX = pcParamLast.pcCenterX;
			pcParam.pcCenterY = pcParamLast.pcCenterY;
			pcParam.pcCenterZ = pcParamLast.pcCenterZ;
			pcParam.pcCompensationX = pcParamLast.pcCompensationX;
			pcParam.pcCompensationY = pcParamLast.pcCompensationY;
		}
		else
		{
			catch_target_counter1 = 0;
			if(catch_target_counter2++ > 10)
			{
				catch_target = 1;
			}
			pcParamLast.pcCenterX = pcParam.pcCenterX;
			pcParamLast.pcCenterY = pcParam.pcCenterY;
			pcParamLast.pcCenterZ = pcParam.pcCenterZ;
			pcParamLast.pcCompensationX = pcParam.pcCompensationX;
			pcParamLast.pcCompensationY = pcParam.pcCompensationY;	
		}
	}
	else
	{
		pcdata_right = 0;
	}
	__HAL_UART_CLEAR_PEFLAG(&huart6);
}

void send_data_to_pc(void)
{
	static uint8_t data[6];
	data[0]=0xA6;
	
	if(Judge_GameRobotState.robot_id == 6)//红色方
	{
		data[1]=1;
	}
	else if(Judge_GameRobotState.robot_id == 16)//蓝色方
	{
		data[1]=0;
	}
	
	if(KeyMousedata.Base_or_robot == 0)//打地面
	{
		data[2]=1;
		center_x = 400 ;               
		center_y = 340 ;
		
	}
	else if(KeyMousedata.Base_or_robot == 1)//打基地
	{
		data[2]=0;
		center_x = 400 ;               
		center_y = 450 ;
	}
	else if(KeyMousedata.Base_or_robot == 2)//打基地护盾
	{
		data[2]=2;
		center_x = 400 ;               
		center_y = 450 ;
	}
	else
		data[2]=1;
//	data[1]= P;
//	data[2]= I;
	Append_CRC8_Check_Sum(data,4);
	data[4] = '\r';
	data[5] = '\n';
//	HAL_UART_Transmit_DMA(&huart6,data,6);
//	HAL_UART_Transmit_IT(&huart6,data,6); 
	sending_to_pc = 1;
	HAL_UART_Transmit(&huart6,data,6,0xff);
	sending_to_pc = 0;
}

