#include "PID.h"
#include <stddef.h>
void PID_init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max){
	pid->Kd=kd;
	pid->Kp=kp;
	pid->Ki=ki;
	pid->out_max=out_max;
	pid->out_min=out_min;
	pid->integral=0.0f;
	pid->pre_error=0.0f;
	pid->has_pre_error=0;
}

void PID_reset(PID_t *pid){
	pid->pre_error=0;
	pid->integral=0;
	pid->has_pre_error=0;
}
float PID_compute(PID_t *pid,float setPoint, float currentValue, float dt){
	if (pid == NULL || dt <= 0.0f) return 0.0f;
	float error =setPoint-currentValue;

	float P_out=error*pid->Kp;


	float I_out=0.0f;
	if(pid->Ki!=0){
		pid->integral+=error*dt;
		I_out=pid->Ki*pid->integral;
		if(I_out>pid->out_max){
			I_out=pid->out_max;
			pid->integral=pid->out_max/pid->Ki;
		}
		else if(I_out<pid->out_min){
			I_out=pid->out_min;
			pid->integral=pid->out_min/pid->Ki;
		}
	}

	float D_out=0.0f;
	if(pid->has_pre_error!=0){
		D_out=pid->Kd*(error-pid->pre_error)/dt;
		pid->pre_error=error;
	}
	else {
		pid->has_pre_error=1;
	}

	pid->pre_error=error;
	float output= P_out+I_out+D_out;
	if(output >pid->out_max) output=pid->out_max;
	else if(output<pid->out_min) output=pid->out_min;
	return output;
}
float PID_compute_WithRate(PID_t *pid, float setPoint, float currentValue,float rate, float dt ){
	if (pid == NULL || dt <= 0.0f) return 0.0f;
	float error =setPoint-currentValue;

	float P_out=error*pid->Kp;

	float I_out=0.0f;
	if(pid->Ki!=0){
		pid->integral+=error*dt;
		I_out=pid->Ki*pid->integral;
		if(I_out>pid->out_max){
			I_out=pid->out_max;
			pid->integral=pid->out_max/pid->Ki;
		}
		else if(I_out<pid->out_min){
			I_out=pid->out_min;
			pid->integral=pid->out_min/pid->Ki;
		}
	}

	float D_out=0.0f;
	D_out=-pid->Kd*rate;


	float output= P_out+I_out+D_out;
	if(output >pid->out_max) output=pid->out_max;
	else if(output<pid->out_min) output=pid->out_min;
	return output;
}
