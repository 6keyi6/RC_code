#include "Chassis_Control.h"
#include "Math.h"
#include "math.h"
#include "arm_math.h"
#include "Remote_Control.h"
//PID ��������
positionpid_t M2006_OUT_PID[4];
positionpid_t M2006_IN_PID[4];
positionpid_t M3508_OUT_PID[4];
incrementalpid_t M3508_IN_PID[4];
//ת���ֲ�������
RUD_Param_t RUD_Param[4];
float RUD_InitAngle[4] = {0.00f,0.00f,0.00f,0.00f};
float RUN_totalAngle[4];
float RUN_speed[4];
float DRIVE_speed[4];

//PID������
M3508_motor_t M3508_motor[4] ={0};
M6020_RUD_motor_t M6020_RUD_motor[4] = {0};

/**
  * @brief  ����PID��ʼ��
  * @param  void
  * @retval None
  */
void Chassis_PID_Init(void)
{
		for(int i=0;i<4;i++)
		{
			PositionPID_paraReset(&M2006_OUT_PID[i],-150.f,0,0,16000,20000);
			PositionPID_paraReset(&M2006_IN_PID[i],20.f,0.,0,16000,20000);
			
			//PositionPID_paraReset(&M3508_OUT_PID[i],1.4f,0.f,0.f,16000,8000);
			IncrementalPID_paraReset(&M3508_IN_PID[i],10.f,0.5f,0.f,16000,5000);
		}
		PositionPID_paraReset(&M2006_OUT_PID[3],-100.f,0,0,16000,20000);
			PositionPID_paraReset(&M2006_IN_PID[3],15.f,0.,0,16000,20000);
    /*--- ת���ֲ���У׼����ʼ��----------------------------------------------------------------*/
    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        RUD_Param[i].Init_angle = RUD_Param[i].Total_angle = RUD_InitAngle[i];

        RUD_Param[i].Turns_cnt = 0;
        RUD_Param[i].TarTurns_cnt = 0;
        RUD_Param[i].Turns_flag = 0;
        RUD_Param[i].Target_angle = 0;
    }		
}

/**
  * @brief  �����ܿ���
  * @param  void
  * @retval None
  */
int angle[4];
void Chassis_control(void)
{
	  //��ȡ������ֵ
		for(int i = 0;i < 4;i++)
		{
			 RUN_speed[i] = M2006moto[i].speed_rpm;          //�����ֲ����ٶ�
			 DRIVE_speed[i] = M3508moto[i].speed_rpm;        //�����ֲ����ٶ�
		}	
		angle[0] = AbsEncoderData[2].realAngle-102400;
		angle[1] = AbsEncoderData[3].realAngle-102400;
		angle[2] = AbsEncoderData[0].realAngle-102400;
		angle[3] = AbsEncoderData[1].realAngle-102400;
		RUN_totalAngle[0] =angle[0]/45.511111111;//�����ֲ����Ƕ�
//		RUN_totalAngle[0] =(AbsEncoderData[2].realAngle)/16348*360;//�����ֲ����Ƕ�
		RUN_totalAngle[1] =angle[1]/45.511111111;//�����ֲ����Ƕ�
		RUN_totalAngle[2] =angle[2]/45.511111111;//�����ֲ����Ƕ�
		RUN_totalAngle[3] =angle[3]/45.511111111;//�����ֲ����Ƕ�
		if(Robot_Receive.chassismode != CHAS_LockMode)
		{
			 Process(Expt.Target_Vx*20,Expt.Target_Vy*20,-Expt.Target_Vw*20);
		}
		else 
	  {
			 Process(0,0,0);//������ס			    
	  }
}

float Ramp_Vy,Ramp_Vx,Ramp_Vw;;
float ACCCCC_VAL1 = 17.0f, DECCCCC_VAL = 25.0f;
int16_t Cal_Speed[4];
void Process(float Vx, float Vy, float Vw)
{


		if(Robot_Receive.chassismode == CHAS_DisableMode||DevicesGet_State(DR16_MONITOR) == Off_line)//--- ����
		{
			  Vx = Vy = Vw  = 0.0f;
				for(int i = 0;i < 4;i++)
				{
					PositionPID_Reset(&M2006_OUT_PID[i]);
					PositionPID_Reset(&M2006_IN_PID[i]);
					PositionPID_Reset(&M3508_OUT_PID[i]);
					IncrementPID_Reset(&M3508_IN_PID[i]);
				}	
	     	CAN_0x1FFSend(&hcan1,0,0,0,0);//ת����
	      CAN_0x200Send(&hcan1,0,0,0,0);//������				
			   return;
		}

			//--- ת���ֵ���CAN�������������ٶ�Ϊ0����ֹ���̷�ת
//			if(DevicesGet_State(0xF<<7))
//			{
//					Vx = Vy = Vw = 0;
//			}
			
			//--- ʹ���ٶ�б��
//					Chassis_Drv_Slow(&Ramp_Vx, Vx, 40.0f, ACCCCC_VAL1, DECCCCC_VAL);
//					Chassis_Drv_Slow(&Ramp_Vy, Vy, 40.0f, ACCCCC_VAL1, DECCCCC_VAL);
			

//		  //ң��û������ʱ�����Ӵ��е�����Ǹ�����غ�
//			if(Vx == 0 && Vy == 0 && Vw == 0)
//		  {
//			   Vx = Vy = Vw =0;
//		  }

			/* ���ֽ��� --------------------------*/		
		   Rudder_Solve(Vx, Vy, Vw, Cal_Speed);
	
			//-- PID ����				
			for(uint8_t i = 0 ; i < 4 ; i++)
			{		
				 M3508_motor[i].DriInResult = Incremental_PID(&M3508_IN_PID[i],Cal_Speed[i],DRIVE_speed[i]);
				 M6020_RUD_motor[i].RudOutResult = Position_PID(&M2006_OUT_PID[i],RUD_Param[i].Target_angle,RUN_totalAngle[i] );
				 M6020_RUD_motor[i].RudInResult = Position_PID(&M2006_IN_PID[i],M6020_RUD_motor[i].RudOutResult,RUN_speed[i]);
			}

			//-- ��������
			CAN_0x1FFSend(&hcan1,M6020_RUD_motor[0].RudInResult,M6020_RUD_motor[1].RudInResult,M6020_RUD_motor[2].RudInResult,M6020_RUD_motor[3].RudInResult);
//			CAN_0x1FFSend(&hcan1,M6020_RUD_motor[0].RudInResult,0,0,0);
			CAN_0x200Send(&hcan1,M3508_motor[0].DriInResult,M3508_motor[1].DriInResult,M3508_motor[2].DriInResult,M3508_motor[3].DriInResult);
					
}


/**
 * @brief      ���ֽ���
 * @param[in]  None
 * @retval     None
 */
float VxVy_Coe;
float VxVy_Limit = 8000; //�ٶ����� 
float Vw_Limit = 8000; //�ٶ����� 
void Rudder_Solve(int16_t Vx, int16_t Vy, int16_t Vw, int16_t *cal_speed)
{
    float Param = 1.0f;
    float MaxSpeed = 0.0f;

    float const theta = atan(1.0/1.0);
    static uint8_t compare_flag = true;
    static uint16_t No_move_cnt = 0;
		  
    VxVy_Coe = 1.0f;
///* �ٶ����� ---------------------------------------------------------------------------------------*/  
//		Constrain(&Vx, (int16_t)(-VxVy_Limit * VxVy_Coe), (int16_t)(VxVy_Limit * VxVy_Coe));
//		Constrain(&Vy, (int16_t)(-VxVy_Limit * VxVy_Coe), (int16_t)(VxVy_Limit * VxVy_Coe));
//		Constrain(&Vw, (int16_t)(-Vw_Limit), (int16_t)(Vw_Limit));

	
    /* �����ٶȽ���ת���ֽǶ� ---------------------------------------------------------------------------*/
    RudAngle_Calc(Vx, Vy, Vw);//��ȡĿ��ֵ�����ֵ�޹�

		
		if(Vx == 0 && Vy == 0)
    {
        if(abs(Vw) < 70) //---IMU��Ư�����������ٶ�
        {
            if(No_move_cnt < 500/* 500 */) //--- ��ֹ��1000ms�ڼ䲻������̸���
            {
                No_move_cnt++;
                Vw = 0;
            }
            else
            {}
        }
        
    }
    else
    {
        No_move_cnt = 0;
    }
		
		/* ������ �ٶȽ��� ---------------------------------------------------------------------------------*/
    cal_speed[RF_201] = -sqrt(pow(Vx - Vw*arm_sin_f32(theta),2) + pow(Vy + Vw*arm_cos_f32(theta),2));//����x��y����
    cal_speed[LF_202] =  sqrt(pow(Vx - Vw*arm_sin_f32(theta),2) + pow(Vy - Vw*arm_cos_f32(theta),2));
    cal_speed[LB_203] =  sqrt(pow(Vx + Vw*arm_sin_f32(theta),2) + pow(Vy - Vw*arm_cos_f32(theta),2));
    cal_speed[RB_204] = -sqrt(pow(Vx + Vw*arm_sin_f32(theta),2) + pow(Vy + Vw*arm_cos_f32(theta),2));
		

    // Ѱ������ٶ�
	for (uint8_t i = 0; i < 4; i++)
	{
		if (abs(cal_speed[i]) > MaxSpeed)
		{
			MaxSpeed = abs(cal_speed[i]);
		}
	}

	// �ٶȷ���  
	if (MaxSpeed > VxVy_Limit)
	{
		Param = (float)VxVy_Limit / MaxSpeed;
	}

	
  cal_speed[RF_201] *= Param;
	cal_speed[LF_202] *= Param;
	cal_speed[LB_203] *= Param;
	cal_speed[RB_204] *= Param;
	
/* ����ת���ֽǶ�Ŀ��ֵ ----------------------------------------------------------------------------*/
    RUDTargetAngle_Calc(RF_205, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(LF_206, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(LB_207, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(RB_208, RUD_NOT_RESET, RUD_NOT_OPSI);
	
	    for(uint8_t i = 0 ; i < 4 ; i++)
			{
					//--- ת���� �ӻ���ת
					RUD_Param[i].Target_angle = Turn_InferiorArc(i, RUD_Param[i].Target_angle, RUN_totalAngle[i]);
			}
}

/**
 * @brief      �����ٶȽ����Ƕ�
 * @param[in]  None
 * @retval     None
 */
uint8_t No_move_flag = false;
uint8_t spin_flag;
uint16_t Brake_cnt;//--- ɲ��
uint8_t Move_flag;//��־λ
float Radius = 1.0f;  // Բ�ľ�
void RudAngle_Calc(int16_t Vx, int16_t Vy, int16_t Vw)
{
    float const theta = atan(1.0/1.0);

    static uint16_t No_move_cnt = 0;

    static float last_vx = 0, last_vy = 0, last_vw = 0;
    if(Vx == 0 && Vy == 0)
    {
        No_move_flag = true;
        if(abs(Vw) < 70) //---IMU��Ư�����������ٶ�
        {
            if(No_move_cnt < 500) //--- ��ֹ��1000ms�ڼ䲻������̸���
            {
                No_move_cnt++;
                Vw = 0;
            }
            else
            {}
        }
    }
    else
    {
        spin_flag = false;
        No_move_flag = false;
        No_move_cnt = 0;
    }

 if(Vx == 0 && Vy == 0 && Vw == 0)
    {
        Move_flag = false;

        if(Brake_cnt < 500)
        {
            Brake_cnt++;

            //--- ����һ֡���Ǿ�ֹ��ʱ�����һ��Vw���ٶȣ�Ϊ�˲�����Ŀ��Ƕ���һ������Ķ���
            last_vw = (spin_flag == true ? 50 : 0);

            //--- ʹ����һ�ε�Ŀ���ٶ���������һ�ε�Ŀ��Ƕ�
            RUD_Param[RF_205].Target_angle = atan2(last_vx - last_vw*(Radius*arm_sin_f32(theta)),last_vy + last_vw*Radius*arm_cos_f32(theta))*(180/PI);
            RUD_Param[LF_206].Target_angle = atan2(last_vx - last_vw*(Radius*arm_sin_f32(theta)),last_vy - last_vw*Radius*arm_cos_f32(theta))*(180/PI);
            RUD_Param[LB_207].Target_angle = atan2(last_vx + last_vw*(Radius*arm_sin_f32(theta)),last_vy - last_vw*Radius*arm_cos_f32(theta))*(180/PI);
            RUD_Param[RB_208].Target_angle = atan2(last_vx + last_vw*(Radius*arm_sin_f32(theta)),last_vy + last_vw*Radius*arm_cos_f32(theta))*(180/PI);
        }
        else
        {
            spin_flag = false;
            //--- 45�ȹ���
            RUD_Param[RF_205].Init_angle = RUD_InitAngle[RF_205]-45;
            RUD_Param[LF_206].Init_angle = RUD_InitAngle[LF_206]+45;
            RUD_Param[LB_207].Init_angle = RUD_InitAngle[LB_207]-45;
            RUD_Param[RB_208].Init_angle = RUD_InitAngle[RB_208]+45;

            //--- Ŀ��Ƕȹ���
            for(uint8_t i = 0 ; i < 4 ; i++)
            {
                RUD_Param[i].Target_angle = 0;
            }
        }

    }
    else
    {
        if(No_move_flag != true)
        {
            Brake_cnt = 0;
        }
        Move_flag = true;

        //--- ���45�ȹ���
        RUD_Param[RF_205].Init_angle = RUD_InitAngle[RF_205];
        RUD_Param[LF_206].Init_angle = RUD_InitAngle[LF_206];
        RUD_Param[LB_207].Init_angle = RUD_InitAngle[LB_207];
        RUD_Param[RB_208].Init_angle = RUD_InitAngle[RB_208];

        //--- ��Ŀ���ٶȵ�ʱ��Ž��ж��ֽ���ļ���
        RUD_Param[RF_205].Target_angle = atan2(Vx - Vw*(Radius*arm_sin_f32(theta)),Vy + Vw*Radius*arm_cos_f32(theta))*(180/PI);
        RUD_Param[LF_206].Target_angle = atan2(Vx - Vw*(Radius*arm_sin_f32(theta)),Vy - Vw*Radius*arm_cos_f32(theta))*(180/PI);
        RUD_Param[LB_207].Target_angle = atan2(Vx + Vw*(Radius*arm_sin_f32(theta)),Vy - Vw*Radius*arm_cos_f32(theta))*(180/PI);
        RUD_Param[RB_208].Target_angle = atan2(Vx + Vw*(Radius*arm_sin_f32(theta)),Vy + Vw*Radius*arm_cos_f32(theta))*(180/PI);

        if(abs(Vw)>100)
        {
            spin_flag = true;
        }

        //--- ��Ŀ���ٶȵ�ʱ��ʹ����һ�νǶ�����������Ϊ����ģʽ��IMU��ֹ��˲��������΢��Vw�ٶ�
        last_vx = Vx;
        last_vy = 0;
        last_vw = 0;

    }
}

float error_;
void RUDTargetAngle_Calc(int8_t motor_num , int8_t reset , uint8_t opposite)
{
    float Cur_Target = RUD_Param[motor_num].Target_angle;

    float Pre_TempTarget[4] = {RUD_Param[0].Init_angle, RUD_Param[1].Init_angle, RUD_Param[2].Init_angle, RUD_Param[3].Init_angle};

    static float temp_pre = 0;  //--- �����ӻ���ת����һ��δ�����㴦���Ŀ��Ƕ�

    
    /*--------------------------------*/
    //--- ��Ŀ��Ƕ��뵱ǰ�ǶȻ��㵽ͬһ��������
    RUD_Param[motor_num].TarTurns_cnt = (int32_t)(RUN_totalAngle[motor_num]/180.0f) - (int32_t)(RUN_totalAngle[motor_num]/360.0f);

    RUD_Param[motor_num].Target_angle = RUD_Param[motor_num].TarTurns_cnt*360.0f + Cur_Target + RUD_Param[motor_num].Init_angle;

    temp_pre = Cur_Target;

    //---- ��ǰĿ��ǶȺ͵�ǰ�Ƕȵ����
    error_ = RUD_Param[motor_num].Target_angle - RUN_totalAngle[motor_num];

    //--- ����޷�

    AngleLimit(&error_);

    //--- ����Ƕ�������90�����ٶȷ��򲢽�Ŀ��Ƕȵ���180��
    if(my_fabs(error_) > 90.0f && (Move_flag == true || No_move_flag == false || Brake_cnt < 500))
    {
        //--- Ŀ��ֵ���Ӱ������
        RUD_Param[motor_num].Target_angle += 180.0f;

        temp_pre += 180.0f;

        //--- �����ַ�ת
        Cal_Speed[motor_num] = -Cal_Speed[motor_num];
        //--- ȷ��Ŀ��ǶȺ͵�ǰ�Ƕȴ���ͬһ������
        if(RUD_Param[motor_num].Target_angle > RUD_Param[motor_num].TarTurns_cnt*360.0f + 180.0f)
        {
            RUD_Param[motor_num].Target_angle -= 360.0f;
        }
        else if(RUD_Param[motor_num].Target_angle < RUD_Param[motor_num].TarTurns_cnt*360.0f - 180.0f)
        {
            RUD_Param[motor_num].Target_angle += 360.0f;
        }
    }
    /*--------------------------------*/
    Pre_TempTarget[motor_num] = RUD_Param[motor_num].Target_angle;

    RUD_Param[motor_num].PreTar_angle = temp_pre;  //--- �����ӻ���ת����һ��δ�����㴦���Ŀ��Ƕ�
}

/**
 * @brief      ת����Ŀ��Ƕȼ���
 * @param[in]  motor_num
 * @param[in]  reset
 * @param[in]  opposite
 * @retval     None
 */

void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0;
	
	recursiveTimes++;
	
	if(recursiveTimes < 100)
	{
		if(*angle > 180.0f)
		{
			*angle -= 360.0f;
			AngleLimit(angle);
		}
		else if(*angle < -180.0f)
		{
			*angle += 360.0f;
			AngleLimit(angle);
		}
	}
	recursiveTimes--;
}

/**
 * @brief      ������Сƫ�ʹת���ֱ����ӻ���ת
 * @param[in]  target
 * @param[in]  current
 * @retval     target(�Ƕ���)
 */
float Turn_InferiorArc(uint8_t motor, float target, float current)
{
    float Error = target - current;

    if(Error > 180.0f)
	{
		return (target - 360.0f);
	}
	else if(Error < -180.0f)
	{
		return (target + 360.0f);
	}
	else
	{
		return target;
	}
}


void Constrain(int16_t *val, int16_t min, int16_t max)
{
    if (*val <= min)
    {
        *val =  min;
    }
    else if(*val >= max)
    {
        *val =  max;
    }
}


float my_fabs(float num) {
    if (num < 0)
        return -num;
    else
        return num;
}

/**
 * @brief      �����ٶ�б��
 * @param[in]  rec, target, slow_Inc
 * @retval     None
 */
void Chassis_Drv_Slow(float *rec , float target , float slow_Inc, float Accval, float DecVal)
{

    if(my_fabs(*rec) - my_fabs(target) < 0)//����ʱ
    {
        if(my_fabs(*rec) > 10)
        {
            slow_Inc = slow_Inc * Accval;//�ٶ���������ʱ������5��
        }
    }
    
    if(my_fabs(*rec) - my_fabs(target) > 0)
    {
        slow_Inc = slow_Inc * DecVal;//����ʱ�Ŵ�15��
    }
    if(my_fabs(*rec - target) < slow_Inc)
    {
        *rec = target;
    }
    else 
    {
        if((*rec) > target) (*rec) -= slow_Inc;
        if((*rec) < target) (*rec) += slow_Inc;
    }
}

