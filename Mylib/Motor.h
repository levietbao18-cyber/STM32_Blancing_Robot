#ifndef MOTOR_H
#define MOTOR_H

#if __has_include("stm32f1xx_hal.h")
#include "stm32f1xx_hal.h"
#elif __has_include("stm32c0xx_hal.h")
#include "stm32c0xx_hal.h"
#elif __has_include("stm32g4xx_hal.h")
#include "stm32g4xx_hal.h"
#endif
typedef struct{
	GPIO_TypeDef *PortIN1;
	uint16_t	PinIN1;

	GPIO_TypeDef *PortIN2;
	uint16_t PinIN2;

	TIM_HandleTypeDef *htim;
	uint32_t TIM_Channel;

	uint8_t is_reversed;
	uint16_t MAX_PWM;
}Motor_t;
void Motor_Init(Motor_t *motor,GPIO_TypeDef *PortIn1,uint16_t PinIn1,
								GPIO_TypeDef *PortIn2,uint16_t PinIn2,
								TIM_HandleTypeDef *htim,uint32_t channel,
								uint8_t is_reversed);
void Motor_Setspeed(Motor_t *motor, float speed);

#endif
