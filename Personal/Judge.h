#ifndef _JUDGE_H_
#define _JUDGE_H_

#include "stm32f4xx_hal.h"
#include "CRC.h"
#include <string.h>
#include "usart.h"
#include "BSP_can.h"


//0x0001	比赛状态数据
typedef __packed struct{ 
	uint8_t game_type : 4; /*1 机甲大师赛   
												  *2  单项赛   
												  *3  ICRA	*/
	uint8_t game_progress : 4;/*0  未开始比赛 
														 *1  准备阶段  
														 *2  自检阶段
														 *3  5s倒计时 
														 *4  对战中
														 *5  比赛结算中  */
	uint16_t stage_remain_time; /* 当前阶段剩余时间 s*/
}ext_game_state_t;

//0x0002	比赛结果数据
typedef __packed struct{ 
	uint8_t winner; /* 0 平局
									 * 1 红方胜利
									 * 2 蓝方胜利 */
}ext_game_result_t;

//0x0003	机器人存活数据
typedef __packed struct{
	uint16_t robot_legion; /* 红方 位0开始 英雄、工程、步兵1、步兵2、步兵3、空中、哨兵、保留 
												  * 蓝方 位8开始 英雄、工程、步兵1、步兵2、步兵3、空中、哨兵、保留 */
}ext_game_robot_survivors_t;

//0x0101	场地事件数据
typedef __packed struct{
	uint32_t event_type; /* 无用 如需要，查手册 */
}ext_event_data_t;

//0x0102	补给站动作标识
typedef __packed struct { 
	uint8_t supply_projectile_id;/* 补给站ID    
															  * 1：1号补给口 
															  * 2：2号补给口 */
	uint8_t supply_robot_id;/* 补弹机器人ID   
													 * 0 ： 无机器人补弹 
													 * 1 ： 红英雄
													 * 2 ： 红工程
													 * 3/4/5 ： 红步兵
													 * 11 ： 蓝英雄
													 * 12 ： 蓝工程
													 * 13/14/15 ： 蓝步兵		*/
	uint8_t supply_projectile_step;/* 出弹口开闭状态 
																  * 0：关闭
																  * 1：子弹准备中
																  * 2：子弹下落 */
	uint8_t supply_projectile_num;  /* 补弹数量  50/100/150/200 */
} ext_supply_projectile_action_t;

//0x0103	请求补给站补弹 对抗赛未开放
typedef __packed struct {
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_num; 
} ext_supply_projectile_booking_t;

//0x0201  比赛机器人状态
typedef __packed struct {
	uint8_t robot_id;/* 机器人ID 
									  * 1： 红英雄
									  * 2： 红工程
									  * 3/4/5：红步兵
									  * 6： 红空中
									  * 7： 红哨兵
										  * 11：蓝英雄
										  * 12：蓝工程
										  * 13/14/15：蓝步兵		
										  * 16  蓝空中
										  * 17  蓝哨兵*/
	uint8_t robot_level; /* 机器人等级 1/2/3*/
	uint16_t remain_HP; /* 机器人剩余血量 */
	uint16_t max_HP; /* 机器人上限血量 */
	uint16_t shooter_heat0_cooling_rate; /* 机器人17mm枪口每秒冷却值 */
	uint16_t shooter_heat0_cooling_limit; /* 机器人17mm枪口热量上限 */
	uint16_t shooter_heat1_cooling_rate;	/* 机器人42mm枪口每秒冷却值 */
	uint16_t shooter_heat1_cooling_limit; /* 机器人42mm枪口热量上限 */
	uint8_t mains_power_gimbal_output : 1; /* 云台电源输出情况 */
	uint8_t mains_power_chassis_output : 1; /* 底盘电源输出情况 */
	uint8_t mains_power_shooter_output : 1; /* 发射摩擦轮电源输出情况 */
} ext_game_robot_state_t;

//0x0202 实时功率热量数据
typedef __packed struct {
	uint16_t chassis_volt; /* 底盘输出电压 mV */
	uint16_t chassis_current; /* 底盘输出电流 mA*/
	float chassis_power; /* 底盘输出功率 W */
	uint16_t chassis_power_buffer; /* 底盘功率缓冲 J */
	uint16_t shooter_heat0; /* 17mm枪口热量 */
	uint16_t shooter_heat1; /* 42mm枪口热量 */
} ext_power_heat_data_t;

//0x0203 机器人位置以及枪口朝向角度
typedef __packed struct {
	float x; 
	float y; 
	float z; 
	float yaw; 
} ext_game_robot_pos_t;

//0x0204 机器人增益
typedef __packed struct { 
	uint8_t power_rune_buff; /* bit0  补血状态 
													  * bit1  热量冷却加倍
													  * bit2  防御加成
													  * bit3  攻击加成 
													  * 其余保留 */
}ext_buff_musk_t;

//0x0205 空中机器人能量状态 只有空中机器人能够接收
typedef __packed struct {
	uint8_t energy_point; /* 积累的能量 */
	uint8_t attack_time; /* 剩余可攻击时间 */
}aerial_robot_energy_t;

//0x0206 伤害状态
typedef __packed struct {
	uint8_t armor_id : 4; /* 装甲伤害时为装甲ID 其余为0 */
	uint8_t hurt_type : 4; /* 0 装甲伤害 
												  * 1 模块离线扣血 
												  * 2 枪口超热量扣血 
												  * 3 底盘超功率扣血 */
} ext_robot_hurt_t;

//0x0207 实时射击信息
typedef __packed struct {
	uint8_t bullet_type; /* 子弹类型 17mm为1 42mm为2*/
	uint8_t bullet_freq; /* 射频 */
	float bullet_speed; /* 射速 */
} ext_shoot_data_t;

//0x0301 机器人间交互数据
typedef __packed struct { 
	uint16_t data_cmd_id; /* 数据段的内容ID */
	uint16_t send_ID; /* 发送者的ID 若红1， 则ID为 1*/
	uint16_t receiver_ID; /* 接受者的ID */
}ext_student_interactive_header_data_t;

//客户端信息 内容ID 0xD180
/*发送客户端信息时，内容ID为0xD180 
										发送者ID 为发送的机器人ID 
										接受者的ID 为机器人对应客户端的ID*/
typedef __packed struct{
	float data1; /* 自定义浮点数据 1 */ 
	float data2; /* 自定义浮点数据 2 */
	float data3; /* 自定义浮点数据 3 */
	uint8_t masks; /* 自定义八位数据 位0-5 分别控制数据面板的六个指示灯 */
} client_custom_data_t;


//机器人之间通信数据 
/* 发送机器人之间通信信息时， 内容ID 为0x0200-0x02FF 
															发送者ID 为发送机器人的ID 
															接受者ID 为接受机器人的ID 
															数据段的字节数要小于113 */
typedef __packed struct
{
	uint8_t data[112];
} robot_interactive_data_t;


extern ext_game_state_t 														Judge_GameState;
extern ext_game_result_t     												Judge_GameResult;
extern ext_game_robot_survivors_t 									Judge_GameRobotSurvivors;

extern ext_event_data_t 														Judge_EventData;
extern ext_supply_projectile_action_t     					Judge_SupplyProjectileAction;
extern ext_supply_projectile_booking_t 							Judge_SupplyProjectileBooking;

extern ext_game_robot_state_t												Judge_GameRobotState;
extern ext_power_heat_data_t												Judge_PowerHeatData;
extern ext_game_robot_pos_t													Judge_GameRobotPos;
extern ext_buff_musk_t															Judge_BuffMusk;
extern aerial_robot_energy_t												Judge_AerialRobotEnergy;
extern ext_robot_hurt_t															Judge_RobotHurt;
extern ext_shoot_data_t  														Judge_ShootData;
extern ext_student_interactive_header_data_t				Judge_StudentInfoHeader;
extern client_custom_data_t													Judge_ClientData;
extern robot_interactive_data_t  										Judge_RobotInterfaceData;

extern float ToJudgeData[3];
extern uint8_t ToJudgeMask;
extern uint8_t JudgeReceivedNewDataSignal[14];

void RobotSendMsgToClient(float data1,float data2,float data3,uint8_t mask);
void JudgeData(uint8_t data);
void RobotSendMsgToRobot(uint8_t data_to_send);

/***********************************************************************************************/
/***********************************************************************************************/
/***********************************************************************************************/
/***********************************************************************************************/
/***********************************************************************************************/

extern int imu_receive;
extern int JudgeSendFresh;


#endif
