#include "drv_imu.h"
#include "BSP_can.h"
#include "arm_math.h"
#include "GimbalControl.h"
static float INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static float INS_accel[3] = {0.0f, 0.0f, 0.0f};
static float INS_mag[3] = {0.0f, 0.0f, 0.0f};

static float INS_Angle[3] = {0.0f, 0.0f, 0.0f};      //欧拉角 单位 rad
static float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //四元数

static float accel_filter[3] = {0.0f, 0.0f, 0.0f};//low pass filter
static float gyro_filter[3] = {0.0f, 0.0f, 0.0f};
static float mag_filter[3] = {0.0f, 0.0f, 0.0f};
static float filter1[3][3] = {0.0f};
static float filter2[3][3] = {0.0f};
static float filter3[3][3] = {0.0f};
static uint8_t upcount1 = 0;
static uint8_t upcount2 = 0;
static uint8_t upcount3 = 0;
void UpdateIMU(struct ahrs_sensor *sensor)
{
	static uint8_t updata_count = 0;
	static int cnt1,cnt2,cnt3;
	static float imulast1,imunow1,imu_first1, imulast2,imunow2,imu_first2, imulast3,imunow3,imu_first3;
	static int inittic;
	static int imu_init_ok;
	INS_gyro[0] = sensor->wx;
	INS_gyro[1] = sensor->wy;
	INS_gyro[2] = sensor->wz;
	
	INS_accel[0] = sensor->ax;
	INS_accel[1] = sensor->ay;
	INS_accel[2] = sensor->az;
	
	INS_mag[0] = sensor->mx;
	INS_mag[1] = sensor->my;
	INS_mag[2] = sensor->mz;
	
	if(updata_count == 0)
	{
		AHRS_init(INS_quat, INS_accel, INS_mag);
    get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);
		updata_count++;
	}
	else
	{
//  	LFP_Filter(INS_gyro,gyro_filter,filter1,&upcount1);
		LFP_Filter(INS_accel,accel_filter,filter2,&upcount2);
//		LFP_Filter(INS_mag,mag_filter,filter3,&upcount3);
		AHRS_update(INS_quat,0.002f,INS_gyro,accel_filter,INS_mag);
		get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);
	}
	
	//zzsadd
	imunow1 = INS_Angle[2] * 57.3f;
	imunow2 = INS_Angle[0] * 57.3f;
	imunow3 = INS_Angle[1] * 57.3f;
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
		if(inittic > 1500) 
			imu_init_ok  = 1; 
	}
	Gyroscope.angleroll  = imunow1 + cnt1*360.0 - imu_first1;
	Gyroscope.angleyaw =  imunow2 + cnt2*360.0 - imu_first2;
	Gyroscope.anglepitch =  imunow3 + cnt3*180.0 - imu_first3;
	
	Gyroscope.gx = (int16_t)(INS_gyro[0]*57.3f);
	Gyroscope.gy = (int16_t)(INS_gyro[1]*57.3f);
	Gyroscope.gz = (int16_t)(INS_gyro[2]*57.3f);
	Gyroscope.ax = (int16_t)(accel_filter[0]*400.0f);
	Gyroscope.ay = (int16_t)(accel_filter[1]*400.0f);
	Gyroscope.az = (int16_t)(accel_filter[2]*400.0f);
	Gyroscope.mx = (int16_t)(INS_mag[0]*400.0f);
	Gyroscope.my = (int16_t)(INS_mag[1]*400.0f);
	Gyroscope.mz = (int16_t)(INS_mag[2]*400.0f);
	
	GimbalData.Yawspeed = Gyroscope.gz;
	GimbalData.Pitchspeed = Gyroscope.gy;
	GimbalData.Pitchangle = Gyroscope.anglepitch;
	GimbalData.Yawangle = Gyroscope.angleyaw;

}
void LFP_Filter(float *data_get,float *data_back,float (*filter)[3],uint8_t *updata_count)
{
	static const float filter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
	
	if(*updata_count == 0)
	{
		filter[0][0] = filter[1][0] = filter[2][0] = data_get[0];
    filter[0][1] = filter[1][1] = filter[2][1] = data_get[1];
    filter[0][2] = filter[1][2] = filter[2][2] = data_get[2];
		*updata_count = 1;
	}
	else
	{
		filter[0][0] = filter[1][0];
    filter[1][0] = filter[2][0];
    filter[2][0] = filter[1][0] * filter_num[0] + filter[0][0] * filter_num[1] + data_get[0] * filter_num[2];

    filter[0][1] = filter[1][1];
    filter[1][1] = filter[2][1];
    filter[2][1] = filter[1][1] * filter_num[0] + filter[0][1] * filter_num[1] + data_get[1] * filter_num[2];

    filter[0][2] = filter[1][2];
    filter[1][2] = filter[2][2];
    filter[2][2] = filter[1][2] * filter_num[0] + filter[0][2] * filter_num[1] + data_get[2] * filter_num[2];
	}
	data_back[0] = filter[2][0];
	data_back[1] = filter[2][1];
	data_back[2] = filter[2][2];
}