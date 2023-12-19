#include "pid.h"
#include "stm32f4xx.h"
#include "math.h"



void abs_limit(float *a, float ABS_MAX)
{
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}


/* 角度Pid时，在更新tar和cur之后紧接着调用, 处理完再进行PID计算*/
float Handle_Angle8191_PID_Over_Zero(float tar, float cur)
{
	if(tar - cur > 4096)    //4096 ：半圈机械角度
	{
		cur += 8192;        //8191,8192无所谓了，四舍五入
	}
	else if(tar - cur < -4096)
	{
		cur = cur - 8192;
	}
	else
	{
		//*cur = *cur;
		// do nothing
	}
	return cur;
}
/* 角度Pid时，在更新tar和cur之后紧接着调用, 处理完再进行PID计算*/
float Handle_Angle360_PID_Over_Zero(float tar, float cur)
{
	if(tar - cur > 180)    //4096 ：半圈机械角度
	{
		cur += 360;        //8191,8192无所谓了，四舍五入
	}
	else if(tar - cur < -180)
	{
		cur = cur - 360;
	}
	else
	{
		//*cur = *cur;
		// do nothing
	}
	return cur;
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
//增量
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

//位置式
float Position_PID(positionpid_t *pid_t, float target, float measured) {
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;
	//电机编码器过零处理
  pid_t->Measured = Handle_Angle8191_PID_Over_Zero(	(pid_t->Target),(pid_t->Measured));	
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
	pid_t->pwm = (pid_t->p_out +pid_t->i_out + pid_t->d_out);	
	abs_limit(&pid_t->pwm, pid_t->MaxOutput);
	pid_t->err_last = pid_t->err;
	
	return pid_t->pwm;
}

//位置式
float Shoot_Position_PID(positionpid_t *pid_t, float target, float measured) {
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;
	pid_t->err = pid_t->Target - pid_t->Measured;
	
	pid_t->integral_err += pid_t->err;
	
	pid_t->p_out = pid_t->Kp*pid_t->err;
	pid_t->i_out = pid_t->Ki*pid_t->integral_err;
	pid_t->d_out = pid_t->Kd*(pid_t->err - pid_t->err_last);
	//限幅
	abs_limit(&pid_t->i_out, pid_t->IntegralLimit);
	
	pid_t->pwm = (pid_t->p_out +pid_t->i_out + pid_t->d_out);	
	
	abs_limit(&pid_t->pwm, pid_t->MaxOutput);
	
	pid_t->err_last = pid_t->err;
	
	return pid_t->pwm;
}

/***************************************以下为专属PID**************************************************************/
//YAW轴控制 PID 位置
float YAW_Position_PID(positionpid_t *pid_t, float target, float measured) {
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;

	pid_t->err = pid_t->Target - pid_t->Measured;

//	if(pid_t->err<5 && pid_t->err> -5)
//	{
//		pid_t->pwm = 0.0f;
//		return pid_t->pwm;
//	}
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

//YAW轴控制 PID 增量
float YAW_Incremental_PID(incrementalpid_t *pid_t, float target, float measured) {

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

//PICTH轴控制 PID 外环
float PICTH_Position_OUT_PID(positionpid_t *pid_t, float target, float measured) {
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;
//	//分区赛步兵需要过零处理
//  pid_t->Measured = Handle_Angle8191_PID_Over_Zero(pid_t->Target,pid_t->Measured);
	pid_t->err = pid_t->Target - pid_t->Measured;

//	if(pid_t->err<5 && pid_t->err> -5)
//	{
//		pid_t->pwm = 0.0f;
//		return pid_t->pwm;
//	}
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
//PICTH轴控制 PID 内环
float PICTH_Position_IN_PID(positionpid_t *pid_t, float target, float measured) {
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;
//	//分区赛步兵需要过零处理
//  pid_t->Measured = Handle_Angle8191_PID_Over_Zero(pid_t->Target,pid_t->Measured);
	pid_t->err = pid_t->Target - pid_t->Measured;


	pid_t->integral_err += pid_t->err;
	
	pid_t->p_out = pid_t->Kp*pid_t->err;
	pid_t->i_out = pid_t->Ki*pid_t->integral_err;
	pid_t->d_out = pid_t->Kd*(pid_t->err - pid_t->err_last);

	//限幅
	abs_limit(&pid_t->i_out, pid_t->IntegralLimit);
    if (fabs(pid_t->err) >= pid_t->I_SeparThresh)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }
	
	//pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
	abs_limit(&pid_t->pwm, pid_t->MaxOutput);
	pid_t->err_last = pid_t->err;
	
	return pid_t->pwm;
}
float YAW_Vision_OUT_PID(positionpid_t *pid_t, float target, float measured) 
{
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;

	pid_t->err = pid_t->Target - pid_t->Measured;


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

float YAW_Vision_IN_PID(positionpid_t *pid_t, float target, float measured) 
{
		pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;

	pid_t->err = pid_t->Target - pid_t->Measured;


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


//底盘跟随 位置
float Follow_PTZ_Position_PID(positionpid_t *pid_t, float target, float measured) {
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;
	//电机编码器过零处理
  	pid_t->Measured = Handle_Angle8191_PID_Over_Zero(	(pid_t->Target),(pid_t->Measured));	
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
//底盘跟随 增量
float Follow_PTZ_Incremental_PID(incrementalpid_t *pid_t, float target, float measured) {

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
//底盘跟随 增量
float Follow_PTZ_Incremental_PID_t(incrementalpid_t *pid_t, float target, float measured) {

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

//YAW轴控制 PID 位置
float YAW_Centre_Position_PID(positionpid_t *pid_t, float target, float measured) {
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;
	//电机编码器过零处理
  pid_t->Measured = Handle_Angle8191_PID_Over_Zero(	(pid_t->Target),(pid_t->Measured));	
	pid_t->err = pid_t->Target - pid_t->Measured;

	if(pid_t->err<15 && pid_t->err> -15)
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

//YAW轴控制 PID 增量
float YAW_Centre_Incremental_PID(incrementalpid_t *pid_t, float target, float measured) {

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


float RUD_OUT_PID(positionpid_t *pid_t, float target, float measured) {
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;

	pid_t->err = pid_t->Target - pid_t->Measured;

//	if(pid_t->err<5 && pid_t->err> -5)
//	{
//		pid_t->pwm = 0.0f;
//		return pid_t->pwm;
//	}
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
float RUD_IN_PID(positionpid_t *pid_t, float target, float measured) {
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;

	pid_t->err = pid_t->Target - pid_t->Measured;

//	if(pid_t->err<5 && pid_t->err> -5)
//	{
//		pid_t->pwm = 0.0f;
//		return pid_t->pwm;
//	}
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
//位置式PID算法，对偏差值进行累加积分。 陀螺仪温度控制
float imu_Position_PID(imu_positionpid_t *pid_t, float target, float measured)
{
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。
    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);
    pid_t->err_last = pid_t->err;
	
    return pid_t->pwm;
}



//底盘跟随 位置
float Follow_OUT_PID(positionpid_t *pid_t, float target, float measured) {
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;
//	//电机编码器过零处理
//  	pid_t->Measured = Handle_Angle8191_PID_Over_Zero(	(pid_t->Target),(pid_t->Measured));	
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
//底盘跟随 增量
float Follow_IN_PID(incrementalpid_t *pid_t, float target, float measured) {

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

float RUD_SPIN_OUT_PID(positionpid_t *pid_t, float target, float measured)
{
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;

	pid_t->err = pid_t->Target - pid_t->Measured;

//	if(pid_t->err<5 && pid_t->err> -5)
//	{
//		pid_t->pwm = 0.0f;
//		return pid_t->pwm;
//	}
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
float RUD_SPIN_IN_PID(positionpid_t *pid_t, float target, float measured)
{
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;

	pid_t->err = pid_t->Target - pid_t->Measured;

//	if(pid_t->err<5 && pid_t->err> -5)
//	{
//		pid_t->pwm = 0.0f;
//		return pid_t->pwm;
//	}
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

float SPIN_IN_PID(incrementalpid_t *pid_t, float target, float measured)
{
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
