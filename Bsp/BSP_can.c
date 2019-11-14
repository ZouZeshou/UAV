#include "BSP_can.h"
#include "can.h"
#include "stdint.h"
#include "ChassisControl.h"
#include "GimbalControl.h"
#include "ShootControl.h"
#include "ahrs.h"
static wl4data w4d;
static wl2data w2d;
GyroData Gyroscope1 = {0};
GyroData Gyroscope2 = {0};
GyroData Gyroscope = {0};
FPS fps = {0};

/**
 * @brief Enable Can1 and Can2
 * @param None
 * @return None
 * @attention None
 */
void CAN_Enable(void)
{
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	CANFilterStart(&hcan1);
	
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	CANFilterStart(&hcan2);
}


/**
 * @brief interrupt function in IRQ
 * @param None
 * @return None
 * @attention None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	uint8_t RxData1[8],RxData2[8];
	CAN_RxHeaderTypeDef Can1Header,Can2Header;
	if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0,&Can1Header,RxData1 );
		if(Can1Header.StdId==0x201)
		{
			fric_l_data.BackPosition = RxData1[0]<<8|RxData1[1];
			fric_l_data.BackSpeed = RxData1[2]<<8|RxData1[3];
			
			fps.Fric_L++;
		}
		if(Can1Header.StdId==0x202)
		{
			fric_r_data.BackPosition = RxData1[0]<<8|RxData1[1];
			fric_r_data.BackSpeed = RxData1[2]<<8|RxData1[3];
			fps.Fric_R++;
		}
		if(Can1Header.StdId==0x205)
		{
			GimbalData.YawBacknow = RxData1[0]<<8|RxData1[1];
			GimbalData.YawEncoderspeed = (int16_t)((RxData1[2]<<8|RxData1[3]));
			fps.Yaw++;
		}
		if(Can1Header.StdId==0x206)
		{
			GimbalData.PitchBacknow = RxData1[0]<<8|RxData1[1];
			GimbalData.PitchBackspeed = RxData1[2]<<8|RxData1[3];
			
			fps.Pit++;
		}	
		if(Can1Header.StdId==0x207)
		{
			StirMotorData.BackPositionNew = RxData1[0]<<8|RxData1[1];
			StirMotorData.BackSpeed = RxData1[2]<<8|RxData1[3];
			DealStirMotorPosition ();
			fps.Stir++;
		}
		if(Can1Header.StdId==0x402)
		{
			w2d.c[0] = RxData1[0];
			w2d.c[1] = RxData1[1];
			Gyroscope2.gy = w2d.d;
			w2d.c[0] = RxData1[2];
			w2d.c[1] = RxData1[3];
			Gyroscope2.gz = w2d.d;
			w4d.c[0] = RxData1[4];
			w4d.c[1] = RxData1[5];
			w4d.c[2] = RxData1[6];
			w4d.c[3] = RxData1[7];
			Gyroscope2.angleyaw = w4d.f;
			fps.Gyro_2++;
		}
	}
	
	if(hcan->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0,&Can2Header,RxData2 );
		
		
		if(Can2Header.StdId==0x401)
		{
			w2d.c[0] = RxData2[0];
			w2d.c[1] = RxData2[1];
			Gyroscope1.gy = w2d.d;
			w2d.c[0] = RxData2[2];
			w2d.c[1] = RxData2[3];
			Gyroscope1.gz = w2d.d;
			w4d.c[0] = RxData2[4];
			w4d.c[1] = RxData2[5];
			w4d.c[2] = RxData2[6];
			w4d.c[3] = RxData2[7];
			Gyroscope1.angleyaw = w4d.f;
			fps.Gyro_1++;
		}
	}
	

}
/**
 * @brief Enable filter(0 for can1 and 14 for can2)
 * @param None
 * @return None
 * @attention None
 */
void CANFilterStart(CAN_HandleTypeDef *hcan)
{
	CAN_FilterTypeDef filter1;
	CAN_FilterTypeDef filter2;
	if(hcan->Instance == CAN1)
	{
		filter1.FilterActivation=ENABLE;
		filter1.FilterBank=0U;
		filter1.FilterFIFOAssignment=CAN_FILTER_FIFO0;
		filter1.FilterIdHigh=0x0000;
		filter1.FilterIdLow=0x0000;
		filter1.FilterMaskIdHigh=0x0000;
		filter1.FilterMaskIdLow=0x0000;
		filter1.FilterMode=CAN_FILTERMODE_IDMASK;
		filter1.FilterScale=CAN_FILTERSCALE_32BIT;
		filter1.SlaveStartFilterBank=14;
		
		HAL_CAN_ConfigFilter(&hcan1,&filter1);
	}
	if(hcan->Instance == CAN2)
	{
		filter2.FilterActivation=ENABLE;
		filter2.FilterBank=14;
		filter2.FilterFIFOAssignment=CAN_FILTER_FIFO0;
		filter2.FilterIdHigh=0x0000;
		filter2.FilterIdLow=0x0000;
		filter2.FilterMaskIdHigh=0x0000;
		filter2.FilterMaskIdLow=0x0000;
		filter2.FilterMode=CAN_FILTERMODE_IDMASK;
		filter2.FilterScale=CAN_FILTERSCALE_32BIT;
		filter2.SlaveStartFilterBank=14;
		
		HAL_CAN_ConfigFilter(&hcan2,&filter2);
	}
	
}
/**
 * @brief Send the message by Can1
 * @param None
 * @return None
 * @attention None
 */
void Can1_SendMsg(uint32_t id,int16_t current0,int16_t current1,int16_t current2,int16_t current3)
{	
	CAN_TxHeaderTypeDef Txmsg1;
	uint8_t TxData1[8];
	
	Txmsg1.DLC=0x08;
	Txmsg1.IDE=CAN_ID_STD;
	Txmsg1.RTR=CAN_RTR_DATA;
	Txmsg1.StdId=id;
	
	TxData1[0]=(unsigned char)(current0>>8);
	TxData1[1]=(unsigned char)(current0);
	TxData1[2]=(unsigned char)(current1>>8);
	TxData1[3]=(unsigned char)(current1);
	TxData1[4]=(unsigned char)(current2>>8);
	TxData1[5]=(unsigned char)(current2);
	TxData1[6]=(unsigned char)(current3>>8);
	TxData1[7]=(unsigned char)(current3);
	
	HAL_CAN_AddTxMessage(&hcan1,&Txmsg1,TxData1,(uint32_t *)CAN_TX_MAILBOX0);
	
}
/**
 * @brief Send the message by Can2
 * @param None
 * @return None
 * @attention None
 */
void Can2_SendMsg(uint32_t id,int16_t current0,int16_t current1,int16_t current2,int16_t current3)
{	
	CAN_TxHeaderTypeDef Txmsg2;
	uint8_t TxData[8];
	
	Txmsg2.DLC=0x08;
	Txmsg2.IDE=CAN_ID_STD;
	Txmsg2.RTR=CAN_RTR_DATA;
	Txmsg2.StdId=id;
	TxData[0]=(unsigned char)(current0>>8);
	TxData[1]=(unsigned char)(current0);
	TxData[2]=(unsigned char)(current1>>8);
	TxData[3]=(unsigned char)(current1);
	TxData[4]=(unsigned char)(current2>>8);
	TxData[5]=(unsigned char)(current2);
	TxData[6]=(unsigned char)(current3>>8);
	TxData[7]=(unsigned char)(current3);
	
	HAL_CAN_AddTxMessage(&hcan2,&Txmsg2,TxData,(uint32_t *)CAN_TX_MAILBOX0);
	
}
