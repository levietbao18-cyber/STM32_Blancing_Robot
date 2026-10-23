#include "PID.h"
void PID_init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max){
	pid->Kd=kd;
	pid->Kp=kp;
	pid->Ki=ki;
	pid->out_max=out_max;
	pid->out_min=out_min;
	pid->intergral=0.0f;
	pid->pre_error=0.0f;
}
float PID_compute(PID_t *pid,float setPoint, float currentValue, float dt){
	float error =setPoint-currentValue;

	float P_out=error*pid->Kp;

	pid->intergral+=error*dt;
	if(pid->intergral>pid->out_max) pid->intergral=pid->out_max;
	else if(pid->intergral<pid->out_min) pid->intergral=pid->out_min;

	float I_out=pid->Ki*pid->intergral;
	float D_out=0.0f;
	if(dt>0.0f){
		D_out=pid->Kd*(error-pid->pre_error)/dt;
	}
	pid->pre_error=error;
	float Output= P_out+I_out+D_out;
	if(Output >pid->out_max) Output=pid->out_max;
	else if(Output<pid->out_min) Output=pid->out_min;
	return Output;
}
