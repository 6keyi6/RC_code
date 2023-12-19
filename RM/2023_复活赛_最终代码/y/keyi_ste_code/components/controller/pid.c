/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include "main.h"
#include "Math.h"
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


#define ULONG_MAX 0xffffffffUL
float VarSpeed_I_A = ULONG_MAX; /*!< 变速积分 A，需为正数。*/
float VarSpeed_I_B = ULONG_MAX; /*!< 变速积分 B，需为正数， */

void abs_limit_PID(float *a, float ABS_MAX)
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
		
//PID参数初始化函数
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

//普通PID
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
	abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit);
	pid_t->pwm = (pid_t->p_out +pid_t->i_out + pid_t->d_out);	
	abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);
	pid_t->err_last = pid_t->err;
	
	return pid_t->pwm;
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
	abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit);
	
	pid_t->pwm += (pid_t->p_out + pid_t->i_out + pid_t->d_out);
	
	//输出限幅
	abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);
	
	pid_t->err_beforeLast = pid_t->err_last;
	pid_t->err_last = pid_t->err;

	return pid_t->pwm;
}





float YAW_OUT_PID(positionpid_t *pid_t, float target, float measured)
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
	abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit);
	pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
	abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);
	pid_t->err_last = pid_t->err;
	
	return pid_t->pwm;
}
float YAW_IN_PID(positionpid_t *pid_t, float target, float measured)
{
		pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;

	pid_t->err = pid_t->Target - pid_t->Measured;


	pid_t->integral_err += pid_t->err;
	
	pid_t->p_out = pid_t->Kp*pid_t->err;
	pid_t->i_out = pid_t->Ki*pid_t->integral_err;
	pid_t->d_out = pid_t->Kd*(pid_t->err - pid_t->err_last);

	//限幅
	abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit);
	pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
	abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);
	pid_t->err_last = pid_t->err;
	
	return pid_t->pwm;	
}

//*************************************************************************
float PositionPID_Cal(positionpid_t *pid_t, float target, float measured)
{
	 pid_t->Target = (float)target;
	 pid_t->Measured = (float)measured;

	 pid_t->err = pid_t->Target - pid_t->Measured;
		 //死区
	if(fabs(pid_t->err) < 5)
	{
		 pid_t->pwm =0;
		return pid_t->pwm;
	}
	 pid_t->p_out = pid_t->Kp*pid_t->err;
	 
	
    float I_VarSpeedf = 0;
		if (fabs(pid_t->err) <= VarSpeed_I_B)
	  I_VarSpeedf = 1;
	  else if (fabs(pid_t->err) <= (VarSpeed_I_A + VarSpeed_I_B))
	 	I_VarSpeedf = (VarSpeed_I_A - (fabs(pid_t->err)) + VarSpeed_I_B) / VarSpeed_I_A;
		
		 if(pid_t->Ki!= 0){
    pid_t->integral_err += I_VarSpeedf * pid_t->err * pid_t->dt;
    /*Constrain*/
    abs_limit_PID(&pid_t->integral_err,  pid_t->IntegralLimit /pid_t->i_out );
  }
  else{
    pid_t->integral_err = 0;
  }
	
	  if (fabs(pid_t->err) < pid_t->I_SeparThresh)
  { 
    pid_t->i_out = pid_t->Ki * pid_t->integral_err;
    /*Constarin*/
    abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit);
  }
  else{
    /*Clear*/
    pid_t->i_out = 0;
  }
	
//	float d_err = 0;
//	if (D_of_Current)
//		d_err = (Current - pre_Current) / dt;
//	else
//		d_err = (Error - pre_error) / dt;


//	D_Term = Kd * d_err;

//	pre_error = Error;

  pid_t->pwm = pid_t->p_out + pid_t->i_out ;
  
  /* Constarin */
  abs_limit_PID(&pid_t->pwm,pid_t->MaxOutput);	
	  return pid_t->pwm;
}


//PICTH轴控制 PID 外环
float PICTH_OUT_PID(positionpid_t *pid_t, float target, float measured) {
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;

	pid_t->err = pid_t->Target - pid_t->Measured;

	pid_t->integral_err += pid_t->err;
	
	pid_t->p_out = pid_t->Kp*pid_t->err;
	pid_t->i_out = pid_t->Ki*pid_t->integral_err;
	pid_t->d_out = pid_t->Kd*(pid_t->err - pid_t->err_last);

	//限幅
	abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit);

	pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
	abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);
	pid_t->err_last = pid_t->err;
	
	return pid_t->pwm;
}

float PICTH_IN_PID(positionpid_t *pid_t, float target, float measured) {
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;

	pid_t->err = pid_t->Target - pid_t->Measured;


	pid_t->integral_err += pid_t->err;
	
	pid_t->p_out = pid_t->Kp*pid_t->err;
	pid_t->i_out = pid_t->Ki*pid_t->integral_err;
	pid_t->d_out = pid_t->Kd*(pid_t->err - pid_t->err_last);

	//限幅
	abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit);
    if (fabs(pid_t->err) >= pid_t->I_SeparThresh)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }
	
	//pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
	abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);
	pid_t->err_last = pid_t->err;
	
	return pid_t->pwm;
}




//底盘跟随 位置
float Follow_OUT_PID(positionpid_t *pid_t, float target, float measured) {
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
	abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit);
	pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
	abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);
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
	abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit);
	pid_t->pwm += (pid_t->p_out + pid_t->i_out + pid_t->d_out);
	//输出限幅
	abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);
	pid_t->err_beforeLast = pid_t->err_last;
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
	abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit);
	
	pid_t->pwm = (pid_t->p_out +pid_t->i_out + pid_t->d_out);	
	
	abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);
	
	pid_t->err_last = pid_t->err;
	
	return pid_t->pwm;
}


float YAW_Vision_OUT_PID(positionpid_t *pid_t, float target, float measured) 
{
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;
	pid_t->err = pid_t->Target - pid_t->Measured;

//	pid_t->integral_err += pid_t->err;
	
	pid_t->p_out = pid_t->Kp*pid_t->err;
	pid_t->i_out += pid_t->Ki * pid_t->err;
	pid_t->d_out = pid_t->Kd*(pid_t->err - pid_t->err_last);

	//限幅
	abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit);
	pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
	abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);
	pid_t->err_last = pid_t->err;
	
	return pid_t->pwm;
}

float YAW_Vision_IN_PID(positionpid_t *pid_t, float target, float measured) 
{
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;


    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    if (fabs(pid_t->err) >= pid_t->I_SeparThresh)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //输出限幅
    abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

float PICTH_Vis_OUT_PID(positionpid_t *pid_t, float target, float measured) {
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
//    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
//    pid_t->d_out = KalmanFilter(&Cloud_PITCHODKalman, pid_t->d_out);
    //积分限幅
    abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;	
}

//PICTH轴控制 PID 内环
float PICTH_Vis_IN_PID(positionpid_t *pid_t, float target, float measured) {

//    pid_t->Target = (float)target;
//    pid_t->Measured = (float)measured;
//    pid_t->err = pid_t->Target - pid_t->Measured;


//    pid_t->p_out = pid_t->Kp * pid_t->err;
//    pid_t->i_out += pid_t->Ki * pid_t->err;
//    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
//    //积分限幅
//    abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

//    if (fabs(pid_t->err) >= pid_t->I_SeparThresh)
//    {
//        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
//    }
//    else
//    {
//        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
//    }

//    //输出限幅
//    abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);

//    pid_t->err_last = pid_t->err;
//    return pid_t->pwm;

    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;


    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    if (fabs(pid_t->err) >= pid_t->I_SeparThresh)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //输出限幅
    abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
		

}



/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
