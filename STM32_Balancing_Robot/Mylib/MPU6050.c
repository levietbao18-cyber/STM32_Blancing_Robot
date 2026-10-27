#include"MPU6050.h"
#include <math.h>
#include <string.h>
#define REG_SMPLRT_DIV      0x19U
#define REG_CONFIG          0x1AU
#define REG_GYRO_CONFIG     0x1BU
#define REG_ACCEL_CONFIG    0x1CU
#define REG_ACCEL_OUT       0x3BU   /* 14 byte: Accel(6) + Temp(2) + Gyro(6) */
#define REG_PWR_MGMT_1      0x6BU
#define REG_WHO_AM_I        0x75U

/* ---- Giá trị cấu hình ---- */
#define WHO_AM_I_EXPECTED   0x68U   /* module clone có thể trả giá trị khác -> chỉnh ở đây */
#define PWR1_DEVICE_RESET   0x80U
#define PWR1_CLK_PLL_GYRO_X 0x01U   /* SLEEP=0, CLKSEL=1: PLL theo gyro X (datasheet khuyến nghị) */
#define GYRO_FS_500DPS      0x08U
#define ACCEL_FS_2G         0x00U
#define DLPF_CFG            0x02U   /* ~94 Hz, trễ ~3 ms (0x03 trễ ~4.8 ms) */

#define ACCEL_LSB_PER_G     16384.0f
#define GYRO_LSB_PER_DPS    65.5f

#define REVERSED_ACCEL_LSB_PER_G 	1/16384.0f
#define REVERSED_GYRO_LSB_PER_DPS 	1/65.5f

#define RAD_TO_DEG_F        57.29577951308232f

#define I2C_TIMEOUT_MS      3U
#define RESET_DELAY_MS      100U
#define PLL_DELAY_MS        10U
#define CALIB_DELAY_MS      3U



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
static MPU6050_StatusTypdef MPU6050_WriteReg(MPU6050_TypeDef *MPU,uint8_t reg, uint8_t value){
	return (HAL_I2C_Mem_Write(MPU->hi2c, MPU->address<<1, reg, I2C_MEMADD_SIZE_8BIT,&value, 1, I2C_TIMEOUT_MS)==HAL_OK)  ?MPU_OK	: MPU_ERROR_COMMON;

}
MPU6050_StatusTypdef MPU6050_Init(MPU6050_TypeDef *MPU,I2C_HandleTypeDef *hi2c,uint8_t address){
	memset(MPU,0,sizeof(*MPU));
	MPU->hi2c=hi2c;
	MPU->address=address;

    Kalman_Init(&MPU->KalmanPitch);
    Kalman_Init(&MPU->KalmanRoll);

	uint8_t address_check;
	if(HAL_I2C_Mem_Read(MPU->hi2c, MPU->address<<1, REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &address_check,1, 10)!=HAL_OK) return MPU_ERROR_COMMON;
	if(address_check !=WHO_AM_I_EXPECTED) return MPU_ERROR_ADR;

	if(MPU6050_WriteReg(MPU, REG_PWR_MGMT_1, PWR1_DEVICE_RESET)!=MPU_OK) return MPU_ERROR_COMMON;

	if(MPU6050_WriteReg(MPU, REG_PWR_MGMT_1, PWR1_CLK_PLL_GYRO_X)!=MPU_OK) return MPU_ERROR_COMMON;

	static const uint8_t cfg[][2]={
			{REG_SMPLRT_DIV, 0x00}, // 1K HZ when DLPF is enable
			{REG_GYRO_CONFIG, GYRO_FS_500DPS},
			{REG_ACCEL_CONFIG, ACCEL_FS_2G},
			{REG_CONFIG, DLPF_CFG}
	};
	for(int i=0;i<sizeof(cfg)/sizeof(cfg[0]);i++){
		if(MPU6050_WriteReg(MPU, cfg[i][0], cfg[i][1])!=MPU_OK) return MPU_ERROR_COMMON;
	}
	return MPU_OK;
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
 MPU6050_StatusTypdef MPU6050_Calibrate(MPU6050_TypeDef *MPU,uint16_t samples){
	if(samples==0U) return MPU_ERROR_CALIB;
	uint8_t data[14];
	float sum_Gx=0,sum_Gy=0,sum_Gz=0;
	float sum_Ax=0,sum_Ay=0,sum_Az=0;
	uint16_t success_samples=0;;
	uint16_t i;
	for(i=0;i<samples;i++){
		if(HAL_I2C_Mem_Read(MPU->hi2c, MPU->address<<1,REG_ACCEL_OUT, 1, data, 14, I2C_TIMEOUT_MS)!=HAL_OK) return MPU_ERROR_COMMON;

			int16_t Ax_raw = (int16_t)(data[0]<<8 | data[1]);
			int16_t Ay_raw = (int16_t)(data[2]<<8 | data[3]);
			int16_t Az_raw = (int16_t)(data[4]<<8 | data[5]);

			int16_t Gx_raw = (int16_t)(data[8]<<8 | data[9]);
			int16_t Gy_raw = (int16_t)(data[10]<<8 | data[11]);
			int16_t Gz_raw = (int16_t)(data[12]<<8 | data[13]);

			sum_Gx+=(float)Gx_raw*REVERSED_GYRO_LSB_PER_DPS;
			sum_Gy+=(float)Gy_raw*REVERSED_GYRO_LSB_PER_DPS;
			sum_Gz+=(float)Gz_raw*REVERSED_GYRO_LSB_PER_DPS;

			sum_Ax+=(float)Ax_raw*REVERSED_ACCEL_LSB_PER_G;
			sum_Ay+=(float)Ay_raw*REVERSED_ACCEL_LSB_PER_G;
			sum_Az+=(float)Az_raw*REVERSED_ACCEL_LSB_PER_G;

			success_samples++;
		 HAL_Delay(3);
	}
	if(success_samples<samples/2U) return MPU_ERROR_CALIB;
	const float inv =1/(float)success_samples;
	MPU->Gx_offset=sum_Gx*inv;
	MPU->Gy_offset=sum_Gy*inv;
	MPU->Gz_offset=sum_Gz*inv;

	const float pitch0=atan2f(sum_Ax*inv,sqrtf(sum_Ay*sum_Ay*inv*inv+sum_Az*sum_Az*inv*inv))*RAD_TO_DEG_F;

	MPU->Pitch=pitch0;
	MPU->KalmanPitch.angle=pitch0;
	return MPU_OK;
}
MPU6050_StatusTypdef MPU6050_ReadAll(MPU6050_TypeDef *MPU,float dt){
	uint8_t data[14]={0};

	if(HAL_I2C_Mem_Read(MPU->hi2c, MPU->address<<1, REG_ACCEL_OUT, 1, data, 14, 5)!=HAL_OK) return MPU_ERROR_COMMON;


	int16_t AX_raw = (int16_t)(data[0]<<8 | data[1]);
	int16_t AY_raw = (int16_t)(data[2]<<8 | data[3]);
	int16_t AZ_raw = (int16_t)(data[4]<<8 | data[5]);
	MPU->Ax=(float)AX_raw*REVERSED_ACCEL_LSB_PER_G ;
	MPU->Ay=(float)AY_raw*REVERSED_ACCEL_LSB_PER_G ;
	MPU->Az=(float)AZ_raw*REVERSED_ACCEL_LSB_PER_G ;

	int16_t GX_raw = (int16_t)(data[8]<<8 | data[9]);
	int16_t GY_raw = (int16_t)(data[10]<<8 | data[11]);
	int16_t GZ_raw = (int16_t)(data[12]<<8 | data[13]);
	MPU->Gx=(float)GX_raw*REVERSED_GYRO_LSB_PER_DPS -MPU->Gx_offset;
	MPU->Gy=(float)GY_raw*REVERSED_GYRO_LSB_PER_DPS -MPU->Gy_offset;
	MPU->Gz=(float)GZ_raw*REVERSED_GYRO_LSB_PER_DPS -MPU->Gz_offset;

	float pitch_accel= atan2f(MPU->Ax,sqrtf(MPU->Ay*MPU->Ay+MPU->Az*MPU->Az))*RAD_TO_DEG_F;
	//float roll_accel=atan2f(MPU->Ay,sqrtf(MPU->Ax*MPU->Ax+MPU->Az*MPU->Az))*RAD_TO_DEG;


	//MPU->Roll=Kalman_GetAngle(&MPU->KalmanRoll, roll_accel, MPU->Gx, dt);
	MPU->Pitch=Kalman_GetAngle(&MPU->KalmanPitch, pitch_accel, -MPU->Gy, dt);
	return MPU_OK;
}
void MPU6050_RecoverBus(MPU6050_TypeDef *MPU){
	HAL_I2C_DeInit(MPU->hi2c);
	HAL_I2C_Init(MPU->hi2c);
}
