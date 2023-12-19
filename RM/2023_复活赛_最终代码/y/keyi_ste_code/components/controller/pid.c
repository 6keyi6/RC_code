/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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
float VarSpeed_I_A = ULONG_MAX; /*!< ���ٻ��� A����Ϊ������*/
float VarSpeed_I_B = ULONG_MAX; /*!< ���ٻ��� B����Ϊ������ */

void abs_limit_PID(float *a, float ABS_MAX)
{
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}		
/* �Ƕ�Pidʱ���ڸ���tar��cur֮������ŵ���, �������ٽ���PID����*/
float Handle_Angle8191_PID_Over_Zero(float tar, float cur)
{
	if(tar - cur > 4096)    //4096 ����Ȧ��е�Ƕ�
	{
		cur += 8192;        //8191,8192����ν�ˣ���������
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
//����ʽPID��������
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

//λ��ʽPID��������
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
		
//PID������ʼ������
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

//��ͨPID
float Position_PID(positionpid_t *pid_t, float target, float measured) {
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;
	//������������㴦��
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
	//�޷�
	abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit);
	pid_t->pwm = (pid_t->p_out +pid_t->i_out + pid_t->d_out);	
	abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);
	pid_t->err_last = pid_t->err;
	
	return pid_t->pwm;
}
//����
float Incremental_PID(incrementalpid_t *pid_t, float target, float measured) {
	pid_t->Target = target;
	pid_t->Measured = measured;
	pid_t->err = pid_t->Target - pid_t->Measured;
		
	pid_t->p_out = pid_t->Kp*(pid_t->err - pid_t->err_last);
	pid_t->i_out = pid_t->Ki*pid_t->err;
	pid_t->d_out = pid_t->Kd*(pid_t->err - 2.0f*pid_t->err_last + pid_t->err_beforeLast);
	
	//�����޷�
	abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit);
	
	pid_t->pwm += (pid_t->p_out + pid_t->i_out + pid_t->d_out);
	
	//����޷�
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

	//�޷�
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

	//�޷�
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
		 //����
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


//PICTH����� PID �⻷
float PICTH_OUT_PID(positionpid_t *pid_t, float target, float measured) {
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;

	pid_t->err = pid_t->Target - pid_t->Measured;

	pid_t->integral_err += pid_t->err;
	
	pid_t->p_out = pid_t->Kp*pid_t->err;
	pid_t->i_out = pid_t->Ki*pid_t->integral_err;
	pid_t->d_out = pid_t->Kd*(pid_t->err - pid_t->err_last);

	//�޷�
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

	//�޷�
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




//���̸��� λ��
float Follow_OUT_PID(positionpid_t *pid_t, float target, float measured) {
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;
	//������������㴦��
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
	//�޷�
	abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit);
	pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
	abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);
	pid_t->err_last = pid_t->err;
	return pid_t->pwm;
}
//���̸��� ����
float Follow_IN_PID(incrementalpid_t *pid_t, float target, float measured) {

	pid_t->Target = target;
	pid_t->Measured = measured;
	pid_t->err = pid_t->Target - pid_t->Measured;
	pid_t->p_out = pid_t->Kp*(pid_t->err - pid_t->err_last);
	pid_t->i_out = pid_t->Ki*pid_t->err;
	pid_t->d_out = pid_t->Kd*(pid_t->err - 2.0f*pid_t->err_last + pid_t->err_beforeLast);
	//�����޷�
	abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit);
	pid_t->pwm += (pid_t->p_out + pid_t->i_out + pid_t->d_out);
	//����޷�
	abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);
	pid_t->err_beforeLast = pid_t->err_last;
	pid_t->err_last = pid_t->err;
	return pid_t->pwm;
}




//λ��ʽ
float Shoot_Position_PID(positionpid_t *pid_t, float target, float measured) {
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;
	pid_t->err = pid_t->Target - pid_t->Measured;
	
	pid_t->integral_err += pid_t->err;
	
	pid_t->p_out = pid_t->Kp*pid_t->err;
	pid_t->i_out = pid_t->Ki*pid_t->integral_err;
	pid_t->d_out = pid_t->Kd*(pid_t->err - pid_t->err_last);
	//�޷�
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

	//�޷�
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
    //�����޷�
    abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit); //ȡ������������޷���

    if (fabs(pid_t->err) >= pid_t->I_SeparThresh)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //����޷�
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
    //�����޷�
    abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit); //ȡ������������޷���

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //����޷�
    abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;	
}

//PICTH����� PID �ڻ�
float PICTH_Vis_IN_PID(positionpid_t *pid_t, float target, float measured) {

//    pid_t->Target = (float)target;
//    pid_t->Measured = (float)measured;
//    pid_t->err = pid_t->Target - pid_t->Measured;


//    pid_t->p_out = pid_t->Kp * pid_t->err;
//    pid_t->i_out += pid_t->Ki * pid_t->err;
//    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
//    //�����޷�
//    abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit); //ȡ������������޷���

//    if (fabs(pid_t->err) >= pid_t->I_SeparThresh)
//    {
//        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
//    }
//    else
//    {
//        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
//    }

//    //����޷�
//    abs_limit_PID(&pid_t->pwm, pid_t->MaxOutput);

//    pid_t->err_last = pid_t->err;
//    return pid_t->pwm;

    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;


    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //�����޷�
    abs_limit_PID(&pid_t->i_out, pid_t->IntegralLimit); //ȡ������������޷���

    if (fabs(pid_t->err) >= pid_t->I_SeparThresh)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //����޷�
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
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION:��ͨPID
  *                 PID_DELTA: ���PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid������
  * @param[in]      max_iout: pid���������
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
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
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
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ��
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
