#ifndef PID_H
#define PID_H

#if __has_include("stm32f1xx_hal.h")
#include "stm32f1xx_hal.h"
#elif __has_include("stm32c0xx_hal.h")
#include "stm32c0xx_hal.h"
#elif __has_include("stm32g4xx_hal.h")
#include "stm32g4xx_hal.h"
#endif

typedef struct{
	float Kp,Ki,Kd;

	float out_max;
	float out_min;

	float pre_error;
	float intergral;

}PID_t;
void PID_init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max);
float PID_compute(PID_t *pid,float setPoint, float currentValue, float dt);
#endif
