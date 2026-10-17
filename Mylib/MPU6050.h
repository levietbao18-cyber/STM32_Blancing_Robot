#ifndef MPU6050_H
#define MPU6050_H

#if __has_include("stm32f1xx_hal.h")
#include "stm32f1xx_hal.h"
#elif __has_include("stm32c0xx_hal.h")
#include "stm32c0xx_hal.h"
#elif __has_include("stm32g4xx_hal.h")
#include "stm32g4xx_hal.h"
#endif

typedef struct{
	float Q_angle;
	float Q_bias;
	float R_measure;

	float angle;
	float bias;
	float rate;

	float P[2][2];
}Kalman_t;

typedef struct{
	I2C_HandleTypeDef *hi2c;
	uint8_t address;

	float Ax,Ay,Az;
	float Gx,Gy,Gz;

	float Gx_offset,Gy_offset,Gz_offset;
    float Pitch;
    float Roll;

	Kalman_t KalmanPitch;
	Kalman_t KalmanRoll;


}MPU6050_TypeDef;


void MPU6050_Init(MPU6050_TypeDef *MPU,I2C_HandleTypeDef *hi2c,uint8_t address);
void MPU6050_Calibrate(MPU6050_TypeDef *MyMpu,uint16_t samples);
void MPU6050_ReadAll(MPU6050_TypeDef *MyMPU,float dt);
float Kalman_GetAngle(Kalman_t *kalman,float newAngle,float newRate, float dt);

#endif
