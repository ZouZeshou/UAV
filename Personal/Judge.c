#include "Judge.h"
#include "BSP_usart.h"
#include "camera.h"
int sending_to_client=0;

uint8_t rbuff[255];
uint8_t HeaderData[20],ReceiveData[255];

int16_t datatype;
int JudgeReceivedNewData = 0;

int JudgeSendFresh = 0;
int imu_receive = 0;

float ToJudgeData[3];
uint8_t ToJudgeMask;
uint8_t JudgeReceivedNewDataSignal[14] = {0};

wl2data w2data;
wl4data w4data;

ext_game_state_t 														Judge_GameState;
ext_game_result_t     											Judge_GameResult;
ext_game_robot_survivors_t 									Judge_GameRobotSurvivors;
ext_event_data_t 														Judge_EventData;
ext_supply_projectile_action_t     					Judge_SupplyProjectileAction;
ext_supply_projectile_booking_t 						Judge_SupplyProjectileBooking;

ext_game_robot_state_t											Judge_GameRobotState;
ext_power_heat_data_t												Judge_PowerHeatData;
ext_game_robot_pos_t												Judge_GameRobotPos;
ext_buff_musk_t															Judge_BuffMusk;
aerial_robot_energy_t												Judge_AerialRobotEnergy;
ext_robot_hurt_t														Judge_RobotHurt;
ext_shoot_data_t  													Judge_ShootData;
ext_student_interactive_header_data_t				Judge_StudentInfoHeader;
client_custom_data_t												Judge_ClientData;
robot_interactive_data_t  									Judge_RobotInterfaceData;

void RobotSendMsgToClient(float data1,float data2,float data3,uint8_t mask)
{
	sending_to_client = 1;
	static uint8_t seq;
	static uint8_t UpData[50];
	if(seq == 255) seq = 0;
	seq++;
	UpData[0] 	= 0xA5;
	w2data.d 	= 19;
	UpData[1] 	= w2data.c[0];
	UpData[2] 	= w2data.c[1];
	UpData[3] 	= seq;
	Append_CRC8_Check_Sum(UpData,5);
	/* cmd_id */
	w2data.d 	= 0x0301;
	UpData[5] 	= w2data.c[0];
	UpData[6] 	= w2data.c[1];
	/* content id */
	w2data.d 	= 0xD180;
	UpData[7] 	= w2data.c[0];
	UpData[8] 	= w2data.c[1];
	/* sender id */
	w2data.d 	= Judge_GameRobotState.robot_id;
//	w2data.d = 0x0006;
	UpData[9] 	= w2data.c[0];
	UpData[10] 	= w2data.c[1];
	/* receiver id */
	if(Judge_GameRobotState.robot_id < 10)
		w2data.d 	= Judge_GameRobotState.robot_id | 0x0100;
	else
		w2data.d 	= (Judge_GameRobotState.robot_id -10) | 0x0110;
//	w2data.d = 0x0106;
	UpData[11] 	= w2data.c[0];
	UpData[12] 	= w2data.c[1];
	
	/* data send */
	w4data.f 	= data1;
	UpData[13] 	= w4data.c[0];
	UpData[14] 	= w4data.c[1];
	UpData[15] 	= w4data.c[2];
	UpData[16] 	= w4data.c[3];
	w4data.f 	= data2;
	UpData[17] 	= w4data.c[0];
	UpData[18] 	= w4data.c[1];
	UpData[19] 	= w4data.c[2];
	UpData[20] 	= w4data.c[3];
	w4data.f 	= data3;
	UpData[21] 	= w4data.c[0];
	UpData[22] 	= w4data.c[1];
	UpData[23] 	= w4data.c[2];
	UpData[24] 	= w4data.c[3];
	UpData[25] 	= mask;
	/* CRC-check */
	Append_CRC16_Check_Sum(UpData,28);
	if(sending_to_pc == 0)
	{
	HAL_UART_Transmit_IT(&huart7,UpData,28);
	}
	sending_to_client = 0;
//	send_by_register(UpData);
//	printf("send to client\r\n");
//	printf("id %d\r\n",Judge_GameRobotState.robot_id);
}

void RobotSendMsgToRobot(uint8_t data_to_send)
{
	static uint8_t seq;
	static uint8_t UpData[50];
	if(seq == 255) seq = 0;
	seq++;
	UpData[0] 	= 0xA5;
	w2data.d 	= 7;
	UpData[1] 	= w2data.c[0];
	UpData[2] 	= w2data.c[1];
	UpData[3] 	= seq;
	Append_CRC8_Check_Sum(UpData,5);
	/* cmd_id */
	w2data.d 	= 0x0301;
	UpData[5] 	= w2data.c[0];
	UpData[6] 	= w2data.c[1];
	/* content id */
	w2data.d 	= 0x0201;
	UpData[7] 	= w2data.c[0];
	UpData[8] 	= w2data.c[1];
	/* sender id */
	w2data.d 	= Judge_GameRobotState.robot_id;
//	w2data.d = 0x0006;
	UpData[9] 	= w2data.c[0];
	UpData[10] 	= w2data.c[1];
	/* receiver id */
	w2data.d 	= Judge_GameRobotState.robot_id + 1;
//	w2data.d = 0x0007;
	UpData[11] 	= w2data.c[0];
	UpData[12] 	= w2data.c[1];
	
	/* data send */
	UpData[13] = data_to_send;
	/* CRC-check */
	Append_CRC16_Check_Sum(UpData,16);
	if(sending_to_client==0 && sending_to_pc == 0)
	{
	HAL_UART_Transmit_IT(&huart7,UpData,16);
	}
//	printf("send to robot\r\n");
//	printf("id %d\r\n",Judge_GameRobotState.robot_id);
}


void JudgeData(uint8_t data)
{
	static int HeaderIndex;
	static int dataIndex;
	static int InfoStartReceive;
	static int16_t datalength;
	static uint8_t packindex;
	if(data == 0xA5)
	{
		HeaderIndex = 1;
		HeaderData[0] = data;
		InfoStartReceive = 0;
	}
	else
	{
		if(HeaderIndex < 5)
		{
			HeaderData[HeaderIndex++] = data;
			if(HeaderIndex == 5 && Verify_CRC8_Check_Sum(HeaderData,5))
				{
				w2data.c[0] = HeaderData[1];
				w2data.c[1] = HeaderData[2];
				datalength = w2data.d;
				packindex = HeaderData[3];
				InfoStartReceive = 1;
				dataIndex = 5;
				memcpy(ReceiveData,HeaderData,5);
				return;
			}
		}
		if(InfoStartReceive)
		{
				if(dataIndex < datalength+9)
				{//9: frame(5)+cmd_id(2)+crc16(2)
					ReceiveData[dataIndex++] = data;
				}
				if(dataIndex == datalength+9)
				{//do the deal once
					InfoStartReceive = 0;
					if(Verify_CRC16_Check_Sum(ReceiveData,datalength+9))
					{
						w2data.c[0] = ReceiveData[5];
						w2data.c[1] = ReceiveData[6];
						datatype = w2data.d;//cmd_id
						JudgeReceivedNewData = 1;
						switch(datatype)
						{
							case 0x0001:
							{
								JudgeReceivedNewDataSignal[0] = 1;
								memcpy(&Judge_GameState,&ReceiveData[7],sizeof(ext_game_state_t));
								break;
							}
							case 0x0002:
							{
								JudgeReceivedNewDataSignal[1] = 1;
								memcpy(&Judge_GameResult,&ReceiveData[7],sizeof(ext_game_result_t));
								break;
							}
							case 0x0003:
							{
								JudgeReceivedNewDataSignal[2] = 1;
								memcpy(&Judge_GameRobotSurvivors,&ReceiveData[7],sizeof(ext_game_robot_survivors_t));
								break;
							}
							case 0x0101:
							{
								JudgeReceivedNewDataSignal[3] = 1;
								memcpy(&Judge_EventData,&ReceiveData[7],sizeof(ext_event_data_t));
								break;
							}
							case 0x0102:
							{
								JudgeReceivedNewDataSignal[4] = 1;
								memcpy(&Judge_SupplyProjectileAction,&ReceiveData[7],sizeof(ext_supply_projectile_action_t));
								break;
							}
							case 0x0103:
							{
								JudgeReceivedNewDataSignal[5] = 1;
								memcpy(&Judge_SupplyProjectileBooking,&ReceiveData[7],sizeof(ext_supply_projectile_booking_t));
								break;
							}
							case 0x0201:
							{
								JudgeReceivedNewDataSignal[6] = 1;
								memcpy(&Judge_GameRobotState,&ReceiveData[7],sizeof(ext_game_robot_state_t));
								break;
							}
							case 0x0202:
							{
								JudgeReceivedNewDataSignal[7] = 1;
								memcpy(&Judge_PowerHeatData,&ReceiveData[7],sizeof(ext_power_heat_data_t));
								break;
							}
							case 0x0203:
							{
								JudgeReceivedNewDataSignal[8] = 1;
								memcpy(&Judge_GameRobotPos,&ReceiveData[7],sizeof(ext_game_robot_pos_t));
								break;
							}
							case 0x0204:
							{
								JudgeReceivedNewDataSignal[9] = 1;
								memcpy(&Judge_BuffMusk,&ReceiveData[7],sizeof(ext_buff_musk_t));
								break;
							}
							case 0x0205:
							{
								JudgeReceivedNewDataSignal[10] = 1;
								memcpy(&Judge_AerialRobotEnergy,&ReceiveData[7],sizeof(aerial_robot_energy_t));
								break;
							}
							case 0x0206:
							{
								JudgeReceivedNewDataSignal[11] = 1;
								memcpy(&Judge_RobotHurt,&ReceiveData[7],sizeof(ext_robot_hurt_t));
								break;
							}
							case 0x0207:
							{
								JudgeReceivedNewDataSignal[12] = 1;
								memcpy(&Judge_ShootData,&ReceiveData[7],sizeof(ext_shoot_data_t));
								break;
							}
							case 0x0301:
							{
								JudgeReceivedNewDataSignal[13] = 1;
								memcpy(&Judge_StudentInfoHeader,&ReceiveData[7],sizeof(ext_student_interactive_header_data_t));
								memcpy(&Judge_RobotInterfaceData,&ReceiveData[7+sizeof(ext_student_interactive_header_data_t)],sizeof(robot_interactive_data_t));
								break;
							}
						}
					}
				}
			
		}
	}
}




