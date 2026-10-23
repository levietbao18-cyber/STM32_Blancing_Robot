#ifndef PID_H
#define PID_H
#include<stdint.h>


typedef struct{
	float Kp,Ki,Kd;

	float out_max;
	float out_min;

	float pre_error;
	float integral;

}PID_t;
void PID_init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max);
void PID_reset(PID_t *pid);
float PID_compute(PID_t *pid,float setPoint, float currentValue, float dt);
float PID_Compute_WithRate(PID_t *pid, float setPoint, float currentValue,float rate, float dt);
#endif
