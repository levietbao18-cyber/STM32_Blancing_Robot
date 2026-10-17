#include"MPU6050.h"
#include <math.h>
#define MPU6050_Who_Am_I		0x75

#define MPU6050_SMPLRT_DIV		0x19
#define MPU6050_CONFIG			0X1A
#define MPU6050_GYRO_CONFIG 	0X1B
#define MPU6050_ACCEL_CONFIG	0X1C
#define MPU6050_FIF0_EN			0X23

#define MPU6050_ACCEL_OUT		0X3B
#define MPU6050_GYRO_OUT		0X43

#define MPU6050_USER_CTRL		0X6A
#define MPU6050_PWR_MGMT_1		0X6B
#define MPU6050_PWR_MGMT_2		0X6C

#define RAD_TO_DEG              57.29577951308232f

const float AFS_SEL_0=				16384.0f;
const float FS_SEL_1=				65.5f;

static void Kalman_Init(Kalman_t *Kalman) {
    Kalman->Q_angle = 0.001f;
    Kalman->Q_bias = 0.003f;
    Kalman->R_measure = 0.03f;
    Kalman->angle = 0.0f;
    Kalman->bias = 0.0f;
    Kalman->rate = 0.0f;
    Kalman->P[0][0] = 0.0f;
    Kalman->P[0][1] = 0.0f;
    Kalman->P[1][0] = 0.0f;
    Kalman->P[1][1] = 0.0f;
}
void MPU6050_Init(MPU6050_TypeDef *MyMPU,I2C_HandleTypeDef *hi2c,uint8_t address){
	MyMPU->hi2c=hi2c;
	MyMPU->address=address;
	 MyMPU->Gx_offset      = 0.0f;
	 MyMPU->Gy_offset      = 0.0f;
	 MyMPU->Gz_offset      = 0.0f;

    Kalman_Init(&MyMPU->KalmanPitch);
    Kalman_Init(&MyMPU->KalmanRoll);

	uint8_t address_check;
	uint8_t data;
	HAL_I2C_Mem_Read(MyMPU->hi2c, MyMPU->address<<1, MPU6050_Who_Am_I, 1, &address_check,1, 10);
	if(address_check ==0x68){
		data=0x00;
		HAL_I2C_Mem_Write(MyMPU->hi2c, MyMPU->address<<1, MPU6050_PWR_MGMT_1,1, &data, 1, 10);

		data=0x00;
		HAL_I2C_Mem_Write(MyMPU->hi2c, MyMPU->address<<1, MPU6050_SMPLRT_DIV,1, &data, 1, 10);

		data=0x08;
		HAL_I2C_Mem_Write(MyMPU->hi2c, MyMPU->address<<1, MPU6050_GYRO_CONFIG,1, &data, 1, 10);

		data=0x00;
		HAL_I2C_Mem_Write(MyMPU->hi2c, MyMPU->address<<1, MPU6050_ACCEL_CONFIG,1, &data, 1, 10);

		data=0x00;
		HAL_I2C_Mem_Write(MyMPU->hi2c, MyMPU->address<<1, MPU6050_CONFIG, 1,&data, 1, 10);
	}
}

float Kalman_GetAngle(Kalman_t *Kalman,float newAngle, float newRate,float dt){

	Kalman->rate=newRate-Kalman->bias;
	Kalman->angle+=dt*Kalman->rate;


	Kalman->P[0][0]+=dt*(dt*Kalman->P[1][1]-Kalman->P[0][1]-Kalman->P[1][0]+Kalman->Q_angle);
	Kalman->P[0][1]-=dt*Kalman->P[1][1];
	Kalman->P[1][0]-=dt*Kalman->P[1][1];
	Kalman->P[1][1]+=Kalman->Q_bias*dt;

	float S=Kalman->P[0][0] +Kalman->R_measure;

	float K[2];
	K[0]=Kalman->P[0][0]/S;
	K[1]=Kalman->P[1][0]/S;

	float y=newAngle-Kalman->angle;

	Kalman->angle+=K[0]*y;
	Kalman->bias+=K[1]*y;

	float P00_temp=Kalman->P[0][0];
	float P01_temp=Kalman->P[0][1];

	Kalman->P[0][0]-=K[0]*P00_temp;
	Kalman->P[0][1]-=K[0]*P01_temp;
	Kalman->P[1][0]-=K[1]*P00_temp;
	Kalman->P[1][1]-=K[1]*P01_temp;

	return Kalman->angle;
}
 void MPU6050_Calibrate(MPU6050_TypeDef *MyMPU,uint16_t samples){
	uint8_t data[6];
	float sum_Gx=0,sum_Gy=0,sum_Gz=0;


	uint16_t i;
	for(i=0;i<samples;i++){
		if(HAL_I2C_Mem_Read(MyMPU->hi2c, MyMPU->address<<1,MPU6050_GYRO_OUT, 1, data, 6, 100)==HAL_OK){
			int16_t Gx_raw=(int16_t)(data[0]<<8 | data[1]);
			int16_t Gy_raw=(int16_t)(data[2]<<8 | data[3]);
			int16_t Gz_raw=(int16_t)(data[4]<<8 | data[5]);

			sum_Gx+=(float)Gx_raw/FS_SEL_1;
			sum_Gy+=(float)Gy_raw/FS_SEL_1;
			sum_Gz+=(float)Gz_raw/FS_SEL_1;
		}
		 HAL_Delay(3);
	}
	MyMPU->Gx_offset=sum_Gx/samples;
	MyMPU->Gy_offset=sum_Gy/samples;
	MyMPU->Gz_offset=sum_Gz/samples;
}
void MPU6050_ReadAll(MPU6050_TypeDef *MyMPU,float dt){
	uint8_t data[14];

	HAL_I2C_Mem_Read(MyMPU->hi2c, MyMPU->address<<1, MPU6050_ACCEL_OUT, 1, data, 14, 100);


	int16_t AX_raw = (int16_t)(data[0]<<8 | data[1]);
	int16_t AY_raw = (int16_t)(data[2]<<8 | data[3]);
	int16_t AZ_raw = (int16_t)(data[4]<<8 | data[5]);
	MyMPU->Ax=(float)AX_raw/AFS_SEL_0;
	MyMPU->Ay=(float)AY_raw/AFS_SEL_0;
	MyMPU->Az=(float)AZ_raw/AFS_SEL_0;

	int16_t GX_raw = (int16_t)(data[8]<<8 | data[9]);
	int16_t GY_raw = (int16_t)(data[10]<<8 | data[11]);
	int16_t GZ_raw = (int16_t)(data[12]<<8 | data[13]);
	MyMPU->Gx=(float)GX_raw/FS_SEL_1-MyMPU->Gx_offset;
	MyMPU->Gy=(float)GY_raw/FS_SEL_1-MyMPU->Gy_offset;
	MyMPU->Gz=(float)GZ_raw/FS_SEL_1-MyMPU->Gz_offset;

	float pitch= atan2(MyMPU->Ax,sqrt(MyMPU->Ay*MyMPU->Ay+MyMPU->Az*MyMPU->Az))*RAD_TO_DEG;
	float roll=atan2(MyMPU->Ay,sqrt(MyMPU->Ax*MyMPU->Ax+MyMPU->Az*MyMPU->Az))*RAD_TO_DEG;


	MyMPU->Roll=Kalman_GetAngle(&MyMPU->KalmanRoll, roll, MyMPU->Gx, dt);
	MyMPU->Pitch=Kalman_GetAngle(&MyMPU->KalmanPitch, pitch, -MyMPU->Gy, dt);

}
