#include "PID.h"
#include <stddef.h>
void PID_init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max){
	if(pid==NULL) return;
	pid->Kd=kd;
	pid->Kp=kp;
	pid->Ki=ki;
	pid->out_max=out_max;
	pid->out_min=out_min;
	pid->integral=0.0f;
	pid->pre_error=0.0f;
}
void PID_Reset(PID_t *pid){

    if (pid == NULL) return;
    pid->integral=0;
    pid->pre_error=0;

}
float PID_compute(PID_t *pid,float setPoint, float currentValue, float dt){
	if(pid==NULL || dt<=0.0f) return 0.0f;
	float error =setPoint-currentValue;

	float P_out=error*pid->Kp;

	pid->integral+=pid->Ki*error*dt;
	if(pid->integral>pid->out_max) pid->integral=pid->out_max;
	else if(pid->integral<pid->out_min) pid->integral=pid->out_min;
	float I_out=pid->integral;


	float D_out=pid->Kd*(error-pid->pre_error)/dt;
	pid->pre_error=error;

	float output= P_out+I_out+D_out;

	if(output >pid->out_max) output=pid->out_max;
	else if(output<pid->out_min) output=pid->out_min;

	return output;
}
float PID_Compute_WithRate(PID_t *pid, float setPoint, float currentValue, float currentRate, float dt){
	if(pid==NULL || dt<=0.0f) return 0.0f;

	float error =setPoint-currentValue;

	float P_out=error*pid->Kp;

	pid->integral+=pid->Ki*error*dt;
	if(pid->integral>pid->out_max) pid->integral=pid->out_max;
	else if(pid->integral<pid->out_min) pid->integral=pid->out_min;
	float I_out=pid->integral;


	// currentRate (deg/s) vốn đã là đạo hàm góc (dtheta/dt), không chia cho dt nữa
	float D_out = -pid->Kd * currentRate;
	pid->pre_error=error;

	float output= P_out+I_out+D_out;

	if(output >pid->out_max) output=pid->out_max;
	else if(output<pid->out_min) output=pid->out_min;

	return output;
}
