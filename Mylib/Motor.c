#include "Motor.h"
#include <math.h>
#if defined(MOTOR_USE_SLOW_DECAY)
/* ====================================================================
 * TRIỂN KHAI CHO CHẾ ĐỘ SLOW DECAY
 * ==================================================================== */
void Motor_Init(Motor_t *motor,
                GPIO_TypeDef *PortDir, uint16_t PinDir,
                TIM_HandleTypeDef *htim, uint32_t channel,
                uint8_t is_reversed, uint16_t deadband)
{
    motor->PortDir     = PortDir;
    motor->PinDir      = PinDir;
    motor->htim        = htim;
    motor->TIM_Channel = channel;
    motor->is_reversed = is_reversed;
    motor->max_pwm    = motor->htim->Instance->ARR;
    motor->deadband    = deadband;
}

void Motor_Setspeed(Motor_t *motor, float speed)
{
	if (motor->is_reversed) speed = -speed;

    if (fabsf(speed) <1.0f) {

        HAL_GPIO_WritePin(motor->PortDir, motor->PinDir, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->TIM_Channel, 0);
        return;
    }

    float abs_speed=fabsf(speed);
    if (abs_speed > (float)motor->max_pwm) {
    	abs_speed = (float)motor->max_pwm;
    }

    uint16_t pwm_out=motor->deadband+(uint16_t)((abs_speed/(float)motor->max_pwm)*(float)(motor->max_pwm-motor->deadband));

    if (speed > 0) {

        if (pwm_out > motor->max_pwm) pwm_out = motor->max_pwm;

        HAL_GPIO_WritePin(motor->PortDir, motor->PinDir, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->TIM_Channel, pwm_out);
    }
    else if(speed<0){


        if (pwm_out> motor->max_pwm) pwm_out = motor->max_pwm;

        HAL_GPIO_WritePin(motor->PortDir, motor->PinDir, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->TIM_Channel, motor->max_pwm-pwm_out);
    }
}

#else

void Motor_Init(Motor_t *motor,
                GPIO_TypeDef *PortIn1, uint16_t PinIn1,
                GPIO_TypeDef *PortIn2, uint16_t PinIn2,
                TIM_HandleTypeDef *htim, uint32_t channel,
                uint8_t is_reversed, uint16_t deadband)
{
    motor->PortIN1     = PortIn1;
    motor->PortIN2     = PortIn2;
    motor->PinIN1      = PinIn1;
    motor->PinIN2      = PinIn2;
    motor->htim        = htim;
    motor->TIM_Channel = channel;
    motor->is_reversed = is_reversed;
    motor->max_pwm     = motor->htim->Instance->ARR;
    motor->deadband    = deadband;
}

void Motor_Setspeed(Motor_t *motor, float speed)
{
    if (motor->is_reversed) speed = -speed;

    if (fabsf(speed) < 1.0f) {
        HAL_GPIO_WritePin(motor->PortIN1, motor->PinIN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->PortIN2, motor->PinIN2, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->TIM_Channel, 0);
        return;
    }
    float abs_speed=fabsf(speed);
    if(abs_speed>motor->max_pwm)
    	abs_speed=motor->max_pwm;
    uint16_t pwm_out=motor->deadband+(uint16_t)((abs_speed/(float)motor->max_pwm)*(float)(motor->max_pwm-motor->deadband));
    if (speed > 0) {
        if (pwm_out > motor->max_pwm) pwm_out = motor->max_pwm;

        HAL_GPIO_WritePin(motor->PortIN1, motor->PinIN1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->PortIN2, motor->PinIN2, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->TIM_Channel, pwm_out);
    }
    else if(speed<0){
        if (pwm_out > motor->max_pwm) pwm_out = motor->max_pwm;

        HAL_GPIO_WritePin(motor->PortIN1, motor->PinIN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->PortIN2, motor->PinIN2, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->TIM_Channel, pwm_out);
    }
}

#endif
