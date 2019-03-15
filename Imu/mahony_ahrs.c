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

#include "math.h"
#include "stdint.h"
#include "ahrs.h"
#include "BSP_can.h"
#include "GimbalControl.h"
//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq 500.0f        // sample frequency in Hz
#define twoKpDef (2.0f * 0.5f)   // 2 * proportional gain
#define twoKiDef (2.0f * 0.005f) // 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions
static volatile float twoKp = twoKpDef;                                           // 2 * proportional gain (Kp)
static volatile float twoKi = twoKiDef;                                           // 2 * integral gain (Ki)
static volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                 // quaternion of sensor frame relative to auxiliary frame
static volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;


//this function takes 56.8us.(168M)
void mahony_ahrs_update(struct ahrs_sensor *sensor, struct attitude *atti)
{
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  gx = sensor->wx;
  gy = sensor->wy;
  gz = sensor->wz;
  ax = sensor->ax;
  ay = sensor->ay;
  az = sensor->az;
  mx = sensor->mx;
  my = sensor->my;
  mz = sensor->mz;
  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
  {
    mahony_ahrs_updateIMU(sensor, atti);
    return;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // Estimated direction of gravity and magnetic field
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // Compute and apply integral feedback if enabled
    if (twoKi > 0.0f)
    {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx; // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else
    {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  atti->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // roll     -pi----pi
  atti->pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                                // pitch    -pi/2----pi/2
  atti->yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3;  // yaw      -pi----pi
	//zzsadd
	static int cnt1,cnt2,cnt3;
	static float imulast1,imunow1,imu_first1, imulast2,imunow2,imu_first2, imulast3,imunow3,imu_first3;
	static int inittic;
	static int imu_init_ok;
	imunow1 = atti->roll;
	imunow2 = atti->yaw;
	imunow3 = atti->pitch;
	if(imunow1 - imulast1> 330) cnt1--;
	if(imunow1 - imulast1 < -330) cnt1++;
	if(imunow2 - imulast2> 330) cnt2--;
	if(imunow2 - imulast2 < -330) cnt2++;
	if(imunow3 - imulast3> 160) cnt3--;
	if(imunow3 - imulast3 < -160) cnt3++;
	imulast1 = imunow1;
	imulast2 = imunow2;
	imulast3 = imunow3;
	if(!imu_init_ok )
	{
		imu_first1= imunow1 + cnt1*360.0;
		imu_first2= imunow2 + cnt2*360.0;
		imu_first3= imunow3 + cnt3*180.0;
		inittic++;
		if(inittic > 1000) 
			imu_init_ok  = 1; 
	}
Gyroscope.angleroll  = imunow1 + cnt1*360.0 - imu_first1;
Gyroscope.angleyaw =  imunow2 + cnt2*360.0 - imu_first2;
Gyroscope.anglepitch =  imunow3 + cnt3*180.0 - imu_first3;
}

void mahony_ahrs_updateIMU(struct ahrs_sensor *sensor, struct attitude *atti)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  gx = sensor->wx;
  gy = sensor->wy;
  gz = sensor->wz;
  ax = sensor->ax;
  ay = sensor->ay;
  az = sensor->az;
  mx = sensor->mx;
  my = sensor->my;
  mz = sensor->mz;
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (twoKi > 0.0f)
    {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx; // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else
    {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  atti->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // roll     -pi----pi
  atti->pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;                                // pitch    -pi/2----pi/2
  atti->yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3;  // yaw      -pi----pi
	//zzsadd
	static int cnt1,cnt2,cnt3;
	static float imulast1,imunow1,imu_first1, imulast2,imunow2,imu_first2, imulast3,imunow3,imu_first3;
	static int inittic;
	static int imu_init_ok;
	imunow1 = atti->roll;
	imunow2 = atti->yaw;
	imunow3 = atti->pitch;
	if(imunow1 - imulast1> 330) cnt1--;
	if(imunow1 - imulast1 < -330) cnt1++;
	if(imunow2 - imulast2> 330) cnt2--;
	if(imunow2 - imulast2 < -330) cnt2++;
	if(imunow3 - imulast3> 160) cnt3--;
	if(imunow3 - imulast3 < -160) cnt3++;
	imulast1 = imunow1;
	imulast2 = imunow2;
	imulast3 = imunow3;
	if(!imu_init_ok )
	{
		imu_first1= imunow1 + cnt1*360.0;
		imu_first2= imunow2 + cnt2*360.0;
		imu_first3= imunow3 + cnt3*180.0;
		inittic++;
		if(inittic > 1000) 
			imu_init_ok  = 1; 
	}
Gyroscope.angleroll  = imunow1 + cnt1*360.0 - imu_first1;
Gyroscope.angleyaw =  imunow2 + cnt2*360.0 - imu_first2;
Gyroscope.anglepitch =  imunow3 + cnt3*180.0 - imu_first3;
	
GimbalData.Pitchangle = Gyroscope.anglepitch;
GimbalData.Yawangle = Gyroscope.angleyaw;
//
}
/**
  * @brief     Fast inverse square-root, to calculate 1/Sqrt(x)
               sizeof(long) must be 4 bytes.
  * @param[in] input:x
  * @retval    1/Sqrt(x)
  */
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}
