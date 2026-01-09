#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "MahonyAHRS.h"
#include "Serial.h"
#include <math.h>

int16_t AX, AY, AZ, GX, GY, GZ;
float ax, ay, az, gx, gy, gz;

extern volatile float q0, q1, q2, q3;

float pitch, roll, yaw;
int16_t pitch_int, roll_int, yaw_int;

float dt = 0.002;

float ax_bias = 0, ay_bias = 0, az_bias = 0;
float gx_bias = 0, gy_bias = 0, gz_bias = 0;

uint16_t calib_count = 0;
uint8_t calib_flag = 0;

void quat2euler(float q0, float q1, float q2, float q3, float* phi, float* theta, float* psi);

int main(void)
{
	OLED_Init();
	MPU6050_Init();
	Serial_Init();
	
	while (1)
	{
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		
		if (!calib_flag) 
		{
			calib_count++;
			
			gx_bias += GX / 16.4f * (3.14159f / 180.0f);
			gy_bias += GY / 16.4f * (3.14159f / 180.0f);
			gz_bias += GZ / 16.4f * (3.14159f / 180.0f);
		
			if (calib_count >= 100) 
			{
				gx_bias = gx_bias / 100.0f;
				gy_bias = gy_bias / 100.0f;
				gz_bias = gz_bias / 100.0f;
				
				calib_flag = 1;
			}
			continue;
		}
		
		ax = AX / 2048.0f;
		ay = AY / 2048.0f;
		az = AZ / 2048.0f;
		
		gx = (GX / 16.4f) * (3.14159f / 180.0f) - gx_bias;
		gy = (GY / 16.4f) * (3.14159f / 180.0f) - gy_bias;
		gz = (GZ / 16.4f) * (3.14159f / 180.0f) - gz_bias;
		
		MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		
		quat2euler(q0, q1, q2, q3, &roll, &pitch, &yaw); 
		
		pitch_int = (int16_t)(pitch * 10);
        roll_int = (int16_t)(roll * 10);
        yaw_int = (int16_t)(yaw * 10);
		
		Serial_TxPacket[0] = (pitch_int >> 8) & 0xFF;
		Serial_TxPacket[1] = pitch_int & 0xFF;
		Serial_TxPacket[2] = (roll_int >> 8) & 0xFF;
		Serial_TxPacket[3] = roll_int & 0xFF;
		Serial_TxPacket[4] = (yaw_int >> 8) & 0xFF;
		Serial_TxPacket[5] = yaw_int & 0xFF;

		Serial_SendPacket();
		
		Delay_ms(2);
	}
}

void quat2euler(float q0,float q1,float q2,float q3, float* phi, float* theta, float* psi) {
    float R[3][3];
    R[0][0] = 1 - 2 * (q2 * q2 + q3 * q3);
    R[0][1] = 2 * (q1 * q2 - q0 * q3);
    R[0][2] = 2 * (q0 * q2 + q1 * q3);
    R[1][0] = 2 * (q1 * q2 + q0 * q3);
    R[1][1] = 1 - 2 * (q1 * q1 + q3 * q3);
    R[1][2] = 2 * (q2 * q3 - q0 * q1);
    R[2][0] = 2 * (q1 * q3 - q0 * q2);
    R[2][1] = 2 * (q0 * q1 + q2 * q3);
    R[2][2] = 1 - 2 * (q1 * q1 + q2 * q2);

    *phi = atan2(R[2][1], R[2][2]);
    *theta = asin(-R[2][0]);
    *psi = atan2(R[1][0], R[0][0]);
	
	*phi = *phi * 180.0f / 3.14159265f;
    *theta = *theta * 180.0f / 3.14159265f;
    *psi = *psi * 180.0f / 3.14159265f;
}
