/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef __AHRS_H__
#define __AHRS_H__

#ifdef AHRS_H_GLOBAL
    #define AHRS_H_EXTERN 
#else
    #define AHRS_H_EXTERN extern
#endif

#include "AHRS_MiddleWare.h"
struct ahrs_sensor
{
  float ax;
  float ay;
  float az;

  float wx;
  float wy;
  float wz;

  float mx;
  float my;
  float mz;
};

struct attitude
{
  float roll;
  float pitch;
  float yaw;
};

float invSqrt(float x);

void madgwick_ahrs_update(struct ahrs_sensor *sensor, struct attitude *atti);
void madgwick_ahrs_updateIMU(struct ahrs_sensor *sensor, struct attitude *atti);

void mahony_ahrs_update(struct ahrs_sensor *sensor, struct attitude *atti);
void mahony_ahrs_updateIMU(struct ahrs_sensor *sensor, struct attitude *atti);

/*******************************************************************************************/
/**
  * @brief          根据加速度的数据，磁力计的数据进行四元数初始化
  * @author         luopin
  * @param[in]      需要初始化的四元数数组
  * @param[in]      用于初始化的加速度计,(x,y,z)不为空 单位 m/s2 
  * @param[in]      用于初始化的磁力计计,(x,y,z)不为空 单位 uT
  * @retval         返回空
  */
extern void AHRS_init(float quat[4], const float accel[3], const float mag[3]);

/**
  * @brief          根据陀螺仪的数据，加速度的数据，磁力计的数据进行四元数更新
  * @author         luopin
  * @param[in]      需要更新的四元数数组
  * @param[in]      更新定时时间，固定定时调用，例如1000Hz，传入的数据为0.001f,
  * @param[in]      用于更新的陀螺仪数据,数组顺序(x,y,z) 单位 rad
  * @param[in]      用于初始化的加速度数据,数组顺序(x,y,z) 单位 m/s2 
  * @param[in]      用于初始化的磁力计数据,数组顺序(x,y,z) 单位 uT
  * @retval         返回空
  */
extern unsigned char AHRS_update(float quat[4], const float timing_time, const float gyro[3], const float accel[3], const float mag[3]);

/**
  * @brief          根据四元数大小计算对应的欧拉角偏航yaw
  * @author         luopin
  * @param[in]      四元数数组，不为NULL
  * @retval         返回的偏航角yaw 单位 rad
  */
extern float get_yaw(const float quat[4]);

/**
  * @brief          根据四元数大小计算对应的欧拉角俯仰角 pitch
  * @author         luopin
  * @param[in]      四元数数组，不为NULL
  * @retval         返回的俯仰角 pitch 单位 rad
  */
extern float get_pitch(const float quat[4]);
/**
  * @brief          根据四元数大小计算对应的欧拉角横滚角 roll
  * @author         luopin
  * @param[in]      四元数数组，不为NULL
  * @retval         返回的横滚角 roll 单位 rad
  */
extern float get_roll(const float quat[4]);

/**
  * @brief          根据四元数大小计算对应的欧拉角yaw，pitch，roll
  * @author         luopin
  * @param[in]      四元数数组，不为NULL
  * @param[in]      返回的偏航角yaw 单位 rad
  * @param[in]      返回的俯仰角pitch  单位 rad
  * @param[in]      返回的横滚角roll 单位 rad
  */
extern void get_angle(const float quat[4], float *yaw, float *pitch, float *roll);
/**
  * @brief          返回当前的重力加速度
  * @author         luopin
  * @param[in]      空
  * @retval         返回重力加速度 单位 m/s2
  */

extern float get_carrier_gravity(void);
#endif // __AHRS_H__
