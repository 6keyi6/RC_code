#include "pid.h"

void abs_limit(float *a, float ABS_MAX)
{
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}


//增量式PID参数清零
void IncrementPID_Reset(incrementalpid_t *pid_t)
{
	pid_t->Target = 0;
	pid_t->Measured = 0;
	pid_t->err = 0;
	pid_t->err_last = 0;
	pid_t->err_beforeLast = 0;	
	pid_t->d_out = 0;
	pid_t->i_out = 0;
	pid_t->p_out = 0;
	pid_t->pwm = 0; 
}

//位置式PID参数清零
void PositionPID_Reset(positionpid_t *pid_t)
{
	pid_t->Target = 0;
	pid_t->Measured = 0;
	pid_t->err = 0;
	pid_t->err_last = 0;
	pid_t->d_out = 0;
	pid_t->i_out = 0;
	pid_t->p_out = 0;
	pid_t->integral_err = 0;
	pid_t->pwm = 0; 
}
//增量PID_参数重置
void IncrementalPID_paraReset(incrementalpid_t *pid_t, float kp, float ki, float kd, uint32_t MaxOutput, uint32_t IntegralLimit)
{
	pid_t->Target = 0;
	pid_t->Measured = 0;
	pid_t->err = 0;
	pid_t->err_last = 0;
	pid_t->err_beforeLast = 0;
	pid_t->Kp = kp;
	pid_t->Ki = ki;
	pid_t->Kd = kd;
	pid_t->MaxOutput = MaxOutput;
	pid_t->IntegralLimit = IntegralLimit;
	pid_t->pwm = 0; 			
}
void PositionPID_paraReset(positionpid_t *pid_t, float kp, float ki, float kd, uint32_t MaxOutput, uint32_t IntegralLimit){
	pid_t->Target = 0;
	pid_t->Measured = 0;
	pid_t->MaxOutput = MaxOutput;
	pid_t->IntegralLimit = IntegralLimit;
	pid_t->err = 0;
	pid_t->err_last = 0;
	pid_t->integral_err = 0;
	pid_t->Kp = kp;
	pid_t->Ki = ki;
	pid_t->Kd = kd;
	pid_t->pwm = 0; 			
}

// 位置
float Position_PID(positionpid_t *pid_t, float target, float measured) {
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;

	pid_t->err = pid_t->Target - pid_t->Measured;
	if(pid_t->err<5 && pid_t->err> -5)
	{
		pid_t->pwm = 0.0f;
		return pid_t->pwm;
	}
	pid_t->integral_err += pid_t->err;
	pid_t->p_out = pid_t->Kp*pid_t->err;
	pid_t->i_out = pid_t->Ki*pid_t->integral_err;
	pid_t->d_out = pid_t->Kd*(pid_t->err - pid_t->err_last);
	//限幅
	abs_limit(&pid_t->i_out, pid_t->IntegralLimit);
	pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
	abs_limit(&pid_t->pwm, pid_t->MaxOutput);
	pid_t->err_last = pid_t->err;
	return pid_t->pwm;
}
// 增量
float Incremental_PID(incrementalpid_t *pid_t, float target, float measured) {

	pid_t->Target = target;
	pid_t->Measured = measured;
	pid_t->err = pid_t->Target - pid_t->Measured;
	pid_t->p_out = pid_t->Kp*(pid_t->err - pid_t->err_last);
	pid_t->i_out = pid_t->Ki*pid_t->err;
	pid_t->d_out = pid_t->Kd*(pid_t->err - 2.0f*pid_t->err_last + pid_t->err_beforeLast);
	//积分限幅
	abs_limit(&pid_t->i_out, pid_t->IntegralLimit);
	pid_t->pwm += (pid_t->p_out + pid_t->i_out + pid_t->d_out);
	//输出限幅
	abs_limit(&pid_t->pwm, pid_t->MaxOutput);
	pid_t->err_beforeLast = pid_t->err_last;
	pid_t->err_last = pid_t->err;
	return pid_t->pwm;
}