/**
 * @file Chassis_control.c
 * @author keyi (hzjqq66@163.com)
 * @brief 
 * @version 1.1
 * @date 2022-09-9
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Robot_control.h"
#include "Chassis_control.h"
#include "M3508_Motor.h"
#include "can_control.h"
#include "math.h"
#include "Math.h"
#include "arm_math.h"
#include "RM_JudgeSystem.h"
#include "Chassis_Power.h"
#include "Devices_Monitor.h"
#include "DEV_SuperCapacitor.h"
/********************************PID����**********************************************/
positionpid_t Class_OUT[4] = {0};//������PID����
incrementalpid_t Class_IN[4] = {0};
positionpid_t M3508_Chassis_Follow_PTZ_Ppid;//���̸���PID����
incrementalpid_t M3508_Chassis_Follow_PTZ_Ipid_t[4];
/********************************PID����**********************************************/
M3508_motor_t M3508_motor = {0};//�Զ���ṹ��
/********************************�û��������**********************************************/
int i = 0,j = 0;
float angle_diff;//��ԽǶȲ�
float Radius = 1.0f;  // Բ�ľ�
//float RUD_InitAngle[4] = {134.48f, -162.74f, 133.78f, 109.34f};
float RUD_InitAngle[4] = {90.09f, 332.49f,151.98f, 327.5f};
//90.09  -27.51 151.98 -30.50
//91.19f, -31.77,151.41f, -31.07f
float VxVy_Limit = 5000; //�ٶ����� 
float Vw_Limit;
uint8_t Move_flag;//��־λ
int16_t temp_Vw;
RUD_Param_t RUD_Param[4];
float RUN_totalAngle[4];
float RUN_speed[4];
float DRIVE_speed[4];
float cal_speed[4] = {0};//������
int16_t Cal_Speed[4];
CHAS_CtrlMode_e Chassis_Mode;//����ģʽ
int16_t drv_tempcurrent[4];
/********************************�û��������**********************************************/
positionpid_t M6020_OUT[4];
positionpid_t M6020_IN[4];
positionpid_t Follow_OUT_pid;
incrementalpid_t Follow_IN_pid;

/**
  * @brief  ����PID��ʼ��
  * @param  void
  * @retval None
  */
void Chassis_PID_Init(void)
{
		for(i=0;i<4;i++)
		{
			//����
			PositionPID_paraReset(&M6020_OUT[i],17.f,0,0,10000,20000);
			PositionPID_paraReset(&M6020_IN[i],100.f,0.,0,10000,20000);
			//ԭ���� 1.3    10 0.8
			PositionPID_paraReset(&Class_OUT[i],1.1f,0.f,0.f,16000,8000);
			IncrementalPID_paraReset(&Class_IN[i],10.f,0.5f,0.f,16000,5000);
		}	

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
void Set_RudMaxOut(uint16_t maxout)
{
    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        M6020_IN[i].MaxOutput = maxout;
    }
}
/**
  * @brief  �����ܿ���
  * @param  void
  * @retval None
  */
float tempV[3];
void Chassis_control(void)
{
		//��ȡ������ֵ
		for(i = 0;i < 4;i++)
		{
			 RUN_totalAngle[i] = M6020moto_chassis[i].total_angle / 22.7527f;/////////////����
			 RUN_speed[i] = M6020moto_chassis[i].speed_rpm;
			 DRIVE_speed[i] = M3508moto_chassis[i].speed_rpm;
		}
//		//�������� 0�� 1��
		 if(Robot_Receive.Write_Msg[Cap_Ctrl] == 1)//��������
		 {
		   SupCap_SupplySwitch(1);			 		
//			 tempV[2] = -tempV[2]*2;
		 }
		 else
		 {
		   SupCap_SupplySwitch(0);			 
		 }

		 if(Robot_Receive.chassismode != CHAS_LockMode)
		 {
				 Process(tempV[0],tempV[1],tempV[2]);
		 }
		 else 
		 {
				 Process(0,0,0);//������ס			    
		 }
}


/**
 * @brief      
 * @param[in]  None
 * @retval     None
 */
float temp_spin;
float spin_foloow_speed = 0;
float Ramp_Vy,Ramp_Vx,Ramp_Vw;;
float ACCCCC_VAL1 = 17.0f, DECCCCC_VAL = 25.0f;/*10.0f*/
uint8_t PreChassMode;
uint16_t ramp_count = 0;
float Vw_temp = 0;
void Process(float Vx, float Vy, float Vw)
{
		 //--- ���ݹ������õ�������ٶ�
	   Set_MaxSpeed();

		if(Robot_Receive.chassismode == CHAS_DisableMode || DevicesGet_State(COMMU_0X340_MONITOR) == Off_line)//--- ����
		{
			  Vx = Vy = Vw  = 0.0f;
				for(int i = 0;i < 4;i++)
				{
					PositionPID_Reset(&M6020_OUT[i]);
					PositionPID_Reset(&M6020_IN[i]);
					PositionPID_Reset(&Class_OUT[i]);
					IncrementPID_Reset(&Class_IN[i]);
				}	
	     	CAN_senddata_6020(&hcan2,0,0,0,0);//ת����
	      CAN_senddata(&hcan1,0,0,0,0);//������				
				PreChassMode = CHAS_DisableMode;
			  return;
		}

			//--- ת���ֵ���CAN�������������ٶ�Ϊ0����ֹ���̷�ת
			if(DevicesGet_State(0xF<<7))
			{
					Vx = Vy = Vw = 0;
			}
			
			//--- ��С����ģʽʹ���ٶ�б��
			if(Robot_Receive.chassismode != CHAS_SpinMode)
			{
					Chassis_Drv_Slow(&Ramp_Vx, Vx, 40.0f, ACCCCC_VAL1, DECCCCC_VAL);
					Chassis_Drv_Slow(&Ramp_Vy, Vy, 40.0f, ACCCCC_VAL1, DECCCCC_VAL);
			}
			else
			{
					Ramp_Vx = CHAS_Max_Get_Power()<80?Vx*1.f:Vx;
					Ramp_Vy = CHAS_Max_Get_Power()<80?Vy*1.f:Vy;
			}

		  //ң��û������ʱ�����Ӵ��е�����Ǹ�����غ�
			if(Vx == 0 && Vy == 0 && fabs(Vw) < 70)
		  {
			   Vx = Vy = Vw =0;
		  }

			/* ���ֽ��� --------------------------*/		
		   Rudder_Solve(Ramp_Vx, Ramp_Vy, Vw, Cal_Speed);
		
			//-- PID ����				
			for(uint8_t i = 0 ; i < 4 ; i++)
			{		
				 M3508_motor.NotFollow_I_result[i] = Follow_IN_PID(&Class_IN[i],Cal_Speed[i],DRIVE_speed[i]);
				 drv_tempcurrent[i] = M3508_motor.NotFollow_I_result[i];
				 M6020_RUD_motor[i].RUD_OUT = RUD_OUT_PID(&M6020_OUT[i],RUD_Param[i].Target_angle ,RUN_totalAngle[i] );
				 M6020_RUD_motor[i].RUD_IN = RUD_IN_PID(&M6020_IN[i],M6020_RUD_motor[i].RUD_OUT,RUN_speed[i]);
			}

			
			//-- ��������
			CHAS_Power_Limit(drv_tempcurrent,4);		
			for(uint8_t i = 0 ; i < 4 ; i++)
			{
					M3508_motor.NotFollow_I_result[i] = drv_tempcurrent[i];
			}
			
			//-- ��������
			CAN_senddata(&hcan1,M3508_motor.NotFollow_I_result[0],M3508_motor.NotFollow_I_result[1],M3508_motor.NotFollow_I_result[2],M3508_motor.NotFollow_I_result[3]);
			CAN_senddata_6020(&hcan2,M6020_RUD_motor[0].RUD_IN,M6020_RUD_motor[1].RUD_IN,M6020_RUD_motor[2].RUD_IN,M6020_RUD_motor[3].RUD_IN);
					
}



/**
 * @brief      �����ٶȽ����Ƕ�
 * @param[in]  None
 * @retval     None
 */
uint8_t No_move_flag = false;
uint8_t spin_flag;
uint16_t Brake_cnt;//--- ɲ��
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

float error_;
float kkl;
void RUDTargetAngle_Calc(int8_t motor_num , int8_t reset , uint8_t opposite)
{
    float Cur_Target = RUD_Param[motor_num].Target_angle;
//    float Pre_Target[4] = {RUD_Param[0].Init_angle, RUD_Param[1].Init_angle, RUD_Param[2].Init_angle, RUD_Param[3].Init_angle};
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
    if(fabs(error_) > 90.0f && (Move_flag == true || No_move_flag == false || Brake_cnt < 500))
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
/**
 * @brief      ���ֽ���
 * @param[in]  None
 * @retval     None
 */
float VxVy_Coe;
void Rudder_Solve(int16_t Vx, int16_t Vy, int16_t Vw, int16_t *cal_speed)
{
    float Param = 1.0f;
    float MaxSpeed = 0.0f;

    float const theta = atan(1.0/1.0);
    static uint8_t compare_flag = true;
    static uint16_t No_move_cnt = 0;
		  
	  temp_Vw = Vw;
    VxVy_Coe = 1.0f;
#if USE_RM_Referee	
/* �ٶ����� ---------------------------------------------------------------------------------------*/  
		Constrain(&Vx, (int16_t)(-VxVy_Limit * VxVy_Coe), (int16_t)(VxVy_Limit * VxVy_Coe));
		Constrain(&Vy, (int16_t)(-VxVy_Limit * VxVy_Coe), (int16_t)(VxVy_Limit * VxVy_Coe));
		Constrain(&Vw, (int16_t)(-Vw_Limit), (int16_t)(Vw_Limit));
#else
	
#endif
	
	  if(Robot_Receive.chassismode == CHAS_followMode)
		{
				for(i = 0; i < 4; i++)
				{
					temp_Vw = Follow_OUT_PID(&Class_OUT[i],0,temp_Vw);
				}
//				if(abs(Vw) < 70)
//				{
//					temp_Vw = 0;
//				}
	  }
		 else if(Robot_Receive.chassismode == CHAS_SpinMode)
		{
       temp_Vw = Vw;
	  }
    /* �����ٶȽ���ת���ֽǶ� ---------------------------------------------------------------------------*/
    RudAngle_Calc(Vx, Vy, temp_Vw);//��ȡĿ��ֵ�����ֵ�޹�

		
		if(Vx == 0 && Vy == 0)
    {
        if(abs(temp_Vw) < 70) //---IMU��Ư�����������ٶ�
        {
            if(No_move_cnt < 500/* 500 */) //--- ��ֹ��1000ms�ڼ䲻������̸���
            {
                No_move_cnt++;
                temp_Vw = 0;
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
    cal_speed[RF_201] = -sqrt(pow(Vx - temp_Vw*arm_sin_f32(theta),2) + pow(Vy + temp_Vw*arm_cos_f32(theta),2));//����x��y����
    cal_speed[LF_202] =  sqrt(pow(Vx - temp_Vw*arm_sin_f32(theta),2) + pow(Vy - temp_Vw*arm_cos_f32(theta),2));
    cal_speed[LB_203] =  sqrt(pow(Vx + temp_Vw*arm_sin_f32(theta),2) + pow(Vy - temp_Vw*arm_cos_f32(theta),2));
    cal_speed[RB_204] = -sqrt(pow(Vx + temp_Vw*arm_sin_f32(theta),2) + pow(Vy + temp_Vw*arm_cos_f32(theta),2));
		
//   if(1)
//    {
//        cal_speed[RF_201] *= 0.3f;
//        cal_speed[LF_202] *= 0.3f;
//    }		
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
//	
	    for(uint8_t i = 0 ; i < 4 ; i++)
			{
					//--- ת���� �ӻ���ת
					RUD_Param[i].Target_angle = Turn_InferiorArc(i, RUD_Param[i].Target_angle, RUN_totalAngle[i]);
			}
}



/**
 * @brief      ��������ٶ�����
 * @param[in]  None
 * @retval     None
 */
void Set_MaxSpeed(void)
{
//	#if USE_RM_Referee
	if(SendData.is_cap_output == 0 )
	{
		if(CHAS_Max_Get_Power() < 60)
		{
			VxVy_Limit = 4700.f;
			Vw_Limit = 4700.f;
			Set_RudMaxOut(12000);
		}
		else if(CHAS_Max_Get_Power() == 60)
		{
			VxVy_Limit = 5500.f;
			Vw_Limit = 5500.f;
			Set_RudMaxOut(13000);
		}
		else if(CHAS_Max_Get_Power() == 70)
		{
			VxVy_Limit = 5800.f;
			Vw_Limit = 5800.f;
			Set_RudMaxOut(15500);
		}	
		else if(CHAS_Max_Get_Power() == 80)
		{
			VxVy_Limit = 7000.f;
			Vw_Limit = 7000.f;
			Set_RudMaxOut(15000);
		}			
		else if(CHAS_Max_Get_Power() > 80)
		{
			VxVy_Limit = 8000.f;
			Vw_Limit = 8000.f;
			Set_RudMaxOut(29999);
		}	
		
		if(CHAS_Max_Get_Power() == 65535)
		{
			VxVy_Limit = 8000.f;
			Vw_Limit = 8000.f;			
		}
  }
	
	else 
	{
		 //--- ��������
		 VxVy_Limit = 9400.f;  
		 Vw_Limit = 9400.f;
		 Set_RudMaxOut(13000);
	}
//	#else
//	#endif
}


/**
 * @brief      �����ٶ�б��
 * @param[in]  rec, target, slow_Inc
 * @retval     None
 */
void Chassis_Drv_Slow(float *rec , float target , float slow_Inc, float Accval, float DecVal)
{

    if(fabs(*rec) - fabs(target) < 0)//����ʱ
    {
        if(fabs(*rec) > 10)
        {
            slow_Inc = slow_Inc * Accval;//�ٶ���������ʱ������5��
        }
    }
    
    if(fabs(*rec) - fabs(target) > 0)
    {
        slow_Inc = slow_Inc * DecVal;//����ʱ�Ŵ�15��
    }
    if(fabs(*rec - target) < slow_Inc)
    {
        *rec = target;
    }
    else 
    {
        if((*rec) > target) (*rec) -= slow_Inc;
        if((*rec) < target) (*rec) += slow_Inc;
    }
}



/**
  * @brief      б�º���,ʹĿ�����ֵ������������ֵ
  * @param[in]  �����������,��ǰ���,�仯�ٶ�(Խ��Խ��)
  * @retval     ��ǰ���
  * @attention  
  */
float RAMP_Output(float final, float now, float ramp)
{
	float buffer = 0;

	buffer = final - now;

	if (buffer > 0)
	{
		if (buffer > ramp)
		{
			now += ramp;
		}
		else
		{
			now += buffer;
		}
	}
	else
	{
		if (buffer < -ramp)
		{
			now += -ramp;
		}
		else
		{
			now += buffer;
		}
	}

	return now;
}


/**
  * @brief  ȫ��ʽ
  * @param  void
  * @retval None
  */

void Omnidirectional_Formula(float *Vx, float *Vy)
{
		float RadRaw = 0.0f;
		float temp_Vx = 0.0f;
		float angle = (M6020s_Yaw.realAngle-(700)) / M6020_mAngleRatio ;//��е�Ƕ�ƫ�� ��λΪ����
		RadRaw = angle * DEG_TO_RAD;                           //����ƫ��
		temp_Vx = *Vx;
		*Vx = *Vx * cos(RadRaw) - *Vy * sin(RadRaw);
		*Vy = *Vy * cos(RadRaw) + temp_Vx * sin(RadRaw);
}
/**
 * @brief      ��ȡ���е�����ز���ϵͳ��Ϣ
 * @param[in]  None
 * @retval     Referee data
 */
float CHAS_Get_Power(void)
{
  	return ext_power_heat_data.data.chassis_power;//����ʵʱ����
}
uint16_t CHAS_Max_Get_Power(void)
{
	return ext_game_robot_state.data.chassis_power_limit;// ���������
}
uint16_t CHAS_Get_PowerBuffer(void)
{
	return ext_power_heat_data.data.chassis_power_buffer; // ���̻��幦��
}
