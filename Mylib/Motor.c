#include "Motor.h"

void Motor_Init(Motor_t *motor,GPIO_TypeDef *PortIn1,uint16_t PinIn1,
								GPIO_TypeDef *PortIn2,uint16_t PinIn2,
								TIM_HandleTypeDef *htim,uint32_t channel,
								uint8_t is_reversed){
	motor->PortIN1=PortIn1;
	motor->PortIN2=PortIn2;
	motor->PinIN1=PinIn1;
	motor->PinIN2=PinIn2;

	motor->htim=htim;
	motor->TIM_Channel=channel;

	motor->is_reversed=is_reversed;
	motor->MAX_PWM=motor->htim->Instance->ARR;

}
void Motor_Setspeed(Motor_t *motor,float speed){
	if(motor->is_reversed) speed=-speed;

	if(speed==0.0f){
		HAL_GPIO_WritePin(motor->PortIN1, motor->PinIN1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->PortIN2, motor->PinIN2, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(motor->htim,motor->TIM_Channel,0);
	}
	else if(speed>0){
		if(speed>motor->MAX_PWM) speed=motor->MAX_PWM;
		HAL_GPIO_WritePin(motor->PortIN1, motor->PinIN1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor->PortIN2, motor->PinIN2, GPIO_PIN_RESET);

	}
	else{
		speed=-speed;
		if(speed>motor->MAX_PWM) speed=motor->MAX_PWM;
		HAL_GPIO_WritePin(motor->PortIN1, motor->PinIN1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor->PortIN2, motor->PinIN2, GPIO_PIN_SET);
	}
	__HAL_TIM_SET_COMPARE(motor->htim,motor->TIM_Channel,(uint16_t)speed);



}
