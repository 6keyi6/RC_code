/**
 * @file M6020_Motor.c
 * @author keyi (hzjqq66@163.com)
 * @brief 
 * @version 1.1
 * @date 2022-09-9
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "cmath"
#include "Cloud_control.h"
#include "Remote_Control.h"
#include "DJI_IMU.h"
#include "can_control.h"
#include "M6020_Motor.h"
#include "Robot_control.h"
#include "INS_task.h"
#include "Control_Vision.h"
#include "Devices_Monitor.h"
#include "DR16_control.h"
#include "kalman_Filter.h"
//--- �ٶȵ�λת��תΪ��е�Ƕ�
#define turnangle 0
#define turnAngle 1
#define ENCODER_ANGLE_RATIO 22.7527f 



/*************************�û��Զ���**************************************/
YAWVision_t YAWVision;
PITVision_t PITVision;
uint8_t ModeTurn = 0;
float YawRCCtrl_Coe;//���̿�����ϵ��
uint8_t Self_aiming_logo = 0;//���鿪����־λ
uint8_t misslogo = 0;//���鶪ʧ��־λ
float miss_yaw,miss_pit;//��ʧĿ��ǰһ�εĲ���ֵ
float RC_YAW,RC_PIT;//�Ӿ���ʧĿ��ʱ���ƶ˵�����
uint8_t init_mode = true;
extKalman_t Kalman_CHASFollow_Speed;

first_order_filter_type_t first_order_filter_BIG_YAW;
fp32 num_YAW[1] = {10};

first_order_filter_type_t first_order_filter_BIG_PIT;
fp32 num_PIT[1] = {10};
/*************************�û��Զ���**************************************/

/*************************PID�ṹ��**************************************/

/********************���**********************/
positionpid_t Yaw_Vision_Out_Pid;//YAW�����⻷PID����
positionpid_t Yaw_Vision_IN_Pid;//YAW�����ڻ�PID����
positionpid_t PIT_Vision_Out_Pid;//PIT�����⻷PID����
positionpid_t PIT_Vision_IN_Pid;//PIT�����ڻ�PID����
/********************���**********************/

/********************����**********************/
positionpid_t Yaw_Vision_Out_Pid_t;//YAW�������⻷PID����
positionpid_t Yaw_Vision_IN_Pid_t;//YAW�������ڻ�PID����
positionpid_t PIT_Vision_Out_Pid_t;//PIT�������⻷PID����
positionpid_t PIT_Vision_IN_Pid_t;//PIT�������ڻ�PID����
/********************����**********************/

/********************С����**********************/
positionpid_t Yaw_Vision_Out_Pid_spin;//YAW��С�����⻷PID����
positionpid_t Yaw_Vision_IN_Pid_spin;//YAW��С�����ڻ�PID����
positionpid_t PIT_Vision_Out_Pid_spin;//PIT��С�����⻷PID����
positionpid_t PIT_Vision_IN_Pid_spin;//PIT��С�����ڻ�PID����
/********************С����**********************/

/********************��ͨ**********************/
 positionpid_t M6020_Yaw_OUT;//YAW��PID����
 positionpid_t M6020_Yaw_IN;//YAW��PID����
 positionpid_t M6020_PIT_OUT;//PIT��PID����
 positionpid_t M6020_PIT_IN;//PIT��PID����
/********************��ͨ**********************/

/*************************PID�ṹ��**************************************/

/**
  * @brief  ��̨��PID��ʼ��
  * @param  void
  * @retval None
  */
void Cloud_PID_Init(void)
{
  YAW_PID_Init();
	PITCH_PID_Init();
    //--- ���̸���ģʽ��˫��ģʽ���ڻ�����ֵ�˲�
  KalmanCreate(&Kalman_CHASFollow_Speed, 1, 20);	
}
/**
  * @brief  �����ǿ���YAW����PID��ʼ��
  * @param  void
  * @retval None
  */
void YAW_PID_Init(void)
{
	  PositionPID_paraReset(&M6020_Yaw_OUT,0.7,0,0,29000,20000);
	  PositionPID_paraReset(&M6020_Yaw_IN,280.f,0.f,0,29000,20000);
	
	  /***********************����PID��ʼ��*****************************/
	  PositionPID_paraReset(&Yaw_Vision_Out_Pid,0.5f,0,0,29000,20000);	  //���	 
	  PositionPID_paraReset(&Yaw_Vision_IN_Pid,250.f,6.f,0,29000,20000);
	  Yaw_Vision_IN_Pid.I_SeparThresh = 200;
		
		PositionPID_paraReset(&Yaw_Vision_Out_Pid_t,0.6f,0,0,29000,20000);  //����
	  PositionPID_paraReset(&Yaw_Vision_IN_Pid_t,300.f,2.f,0,29000,20000);
	  Yaw_Vision_IN_Pid_t.I_SeparThresh = 200;
	
		PositionPID_paraReset(&Yaw_Vision_Out_Pid_spin,0.5f,0,0,29000,20000);  
	  PositionPID_paraReset(&Yaw_Vision_IN_Pid_spin,250.f,8.f,0,29000,20000);//С����
	  Yaw_Vision_IN_Pid_spin.I_SeparThresh = 200;
		/***********************����PID��ʼ��*****************************/
}

/**
  * @brief  ��̨PICTH��Ƕȿ��Ƴ�ʼ��
  * @param  void
  * @retval None
  */
void PITCH_PID_Init(void)
{	
       //1 140
			 PositionPID_paraReset(&M6020_PIT_OUT,1.f,0.0f,0.0f,29000,10000);
			 M6020_PIT_OUT.I_SeparThresh = 0;
			 PositionPID_paraReset(&M6020_PIT_IN,180,0.0f,0.f,29990,10000); //150
			 M6020_PIT_IN.I_SeparThresh = 100;
	
		 /***********************����PID��ʼ��*****************************/
			 PositionPID_paraReset(&PIT_Vision_Out_Pid,0.6f,0.0f,0.0f,29000,20000);//���
			 PIT_Vision_Out_Pid.I_SeparThresh = 0;
			 PositionPID_paraReset(&PIT_Vision_IN_Pid,180,1.f,0.f,29990,20000); 
			 PIT_Vision_IN_Pid.I_SeparThresh = 300;

			 PositionPID_paraReset(&PIT_Vision_Out_Pid_t,0.5f,0.0f,0.0f,29000,10000);//����
			 PIT_Vision_Out_Pid_t.I_SeparThresh = 0;
			 PositionPID_paraReset(&PIT_Vision_IN_Pid_t,200,5.f,0.f,29990,20000); 
			 PIT_Vision_IN_Pid_t.I_SeparThresh = 300;
	
			 PositionPID_paraReset(&PIT_Vision_Out_Pid_spin,0.6f,0.0f,0.0f,29000,10000);//С����
			 PIT_Vision_Out_Pid_spin.I_SeparThresh = 0;
			 PositionPID_paraReset(&PIT_Vision_IN_Pid_spin,180,8.f,0.f,29990,20000); 
			 PIT_Vision_IN_Pid_spin.I_SeparThresh = 300;
}
/**
  * @brief  ��̨�ܿ��ƺ���
  * @param  void
  * @retval None
  */
int init_cnt = 2500;
void Cloud_control(void)
{
	Get_CloudMeasured();//��ȡ����ֵ
	Vision_Angle_Update();//����Ƕȸ���
    //--- �ȴ�IMU��ʼ�����
    if(init_mode)
    {
        if(DJIC_IMU.yaw != 0 || DJIC_IMU.pitch != 0)
        {
            if(--init_cnt != 0) //--- �ȴ�IMU�ȶ� 800*2ms�����ȶ�ʱ��+500*2ms������ʱ��
            {
              Set_InitAngle();
            }
						else
						{
							init_mode = false;
							Chassis_start = true;//�������������̱�־λ
						}
        }
        return;
    }	
		
		
	if(DevicesGet_State(DR16_MONITOR) == Off_line || Gimbal_Mode == Gimbal_DisableMode)//--- ���� & ʧ��
	{ 
			Set_InitAngle();//���ó�ʼĿ��Ƕ� ��ը
		  TargetAngle_Update(0,0);//��ȡң����������
			PositionPID_Reset(&M6020_Yaw_IN);//��ͨPID
			PositionPID_Reset(&M6020_PIT_IN);

			PositionPID_Reset(&Yaw_Vision_Out_Pid);//���
			PositionPID_Reset(&Yaw_Vision_IN_Pid);
			PositionPID_Reset(&PIT_Vision_Out_Pid);
			PositionPID_Reset(&PIT_Vision_IN_Pid);
			
			PositionPID_Reset(&Yaw_Vision_Out_Pid_t);//Ԥ������
			PositionPID_Reset(&Yaw_Vision_IN_Pid_t);
			PositionPID_Reset(&PIT_Vision_Out_Pid_t);
			PositionPID_Reset(&PIT_Vision_IN_Pid_t);
			
			PositionPID_Reset(&Yaw_Vision_Out_Pid_spin);//С����
			PositionPID_Reset(&Yaw_Vision_IN_Pid_spin);
			PositionPID_Reset(&PIT_Vision_Out_Pid_spin);
			PositionPID_Reset(&PIT_Vision_IN_Pid_spin);
		
		  Self_aiming_logo = 0;//�����־λ����
			misslogo = 0;//��ʧ��־λ����
			return;
	} 

	  //��ʹ�õ���C��ʱ
	if(ModeTurn != 2)//��ȡ��ʼ�Ƕ�
	{
		M6020_motor.remote_control_YAW = DJIC_IMU.total_yaw  * (turnAngle?ENCODER_ANGLE_RATIO:1);
		PITCH_Use.RC_PICTH = DJIC_IMU.pitch * (turnAngle?ENCODER_ANGLE_RATIO:1);
		ModeTurn = 2;
	}
	
		if(DevicesGet_State(GIMBAL_YAW_MONITOR) == Off_line)//---����������ң������ֵ���ܸ���
		{
			TargetAngle_Update(0,-DR16_Get_ExptPit());//��ȡң����������
		}
		else if(DevicesGet_State(GIMBAL_PIT_MONITOR) == Off_line)
		{
			TargetAngle_Update(-DR16_Get_ExptYaw(),0);//��ȡң����������
		}
		else
		{
	    TargetAngle_Update(-DR16_Get_ExptYaw(),-DR16_Get_ExptPit());//��ȡң����������
		}
	
	switch(Gimbal_Mode)
	{  		
	  case Gimbal_PCMode:// PC����(����)
		{ 
					if(VisionData.RawData.mode == 0 || (VisionData.RawData.yaw_angle == 0 || VisionData.RawData.pitch_angle == 0 ))//��ʧĿ��
					 { 
						 misslogo = 1; 
					 }
						else if(VisionData.RawData.mode == 1)
					 {
						 misslogo = 0;
					 }
					 Cloud_YAWIMUVisionPID();
					 Cloud_PITIMUVisionPID();
					 //--- ���͵���
					 CAN_senddata_6020(&hcan1,YAWVision.Vision_IN ,0,0,PITVision.Vision_IN);
					 
					//--- �������ʱ����Ŀǰ�Ƕ�
					M6020_motor.remote_control_YAW = DJIC_IMU.total_yaw  * (turnAngle?ENCODER_ANGLE_RATIO:1);
					PITCH_Use.RC_PICTH =  DJIC_IMU.pitch* (turnAngle?ENCODER_ANGLE_RATIO:1);	 
    break;
		}
		case Gimbal_NormalMode:// ��ͨ��תģʽ
		{
				 IMU_control_YAW();
				 IMU_control_PITCH();
			   //--- ���͵���
				 CAN_senddata_6020(&hcan1,M6020_motor.YAW_I_result,0,0,PITCH_Use.PICTH_IN);
			
	       miss_yaw = DJIC_IMU.total_yaw  * (turnAngle?ENCODER_ANGLE_RATIO:1);
				 miss_pit = DJIC_IMU.pitch* (turnAngle?ENCODER_ANGLE_RATIO:1);
			break;
		}
	}
}

/**
  * @brief  ��ȡ��̨����ֵ
  * @param  void
  * @retval None
  */
void Get_CloudMeasured(void)
{
		M6020_motor.YAW_OUT_Current =DJIC_IMU.total_yaw  * (turnAngle?ENCODER_ANGLE_RATIO:1);
	  M6020_motor.YAW_IN_Current = DJIC_IMU.Gyro_z;
	  PITCH_Use.PICTH_OUT_Current = DJIC_IMU.pitch * (turnAngle?ENCODER_ANGLE_RATIO:1);
	  PITCH_Use.PICTH_IN_Current = DJIC_IMU.Gyro_y;	
}

/**���ó�ʼĿ��Ƕ�
  * @brief  
  * @param  void
  * @retval None
  */
void Set_InitAngle(void)
{
    M6020_motor.remote_control_YAW = DJIC_IMU.total_yaw * (turnAngle?ENCODER_ANGLE_RATIO:1);
		PITCH_Use.RC_PICTH = DJIC_IMU.pitch * (turnAngle?ENCODER_ANGLE_RATIO:1);
	  //��ʼ����Ƕ�
	  YAWVision.IMUVisionErr = DJIC_IMU.total_yaw  * (turnAngle?ENCODER_ANGLE_RATIO:1);
		PITVision.IMUVisionErr = DJIC_IMU.pitch * (turnAngle?ENCODER_ANGLE_RATIO:1);
}
/**Ŀ��Ƕȸ���
  * @brief  
  * @param  void
  * @retval None
  */
void TargetAngle_Update(float Yawparam, float Pitparam)
{
	  M6020_motor.remote_control_YAW +=  Yawparam * (turnAngle?ENCODER_ANGLE_RATIO:1) * 0.5f;
		PITCH_Use.RC_PICTH += Pitparam * (turnAngle?ENCODER_ANGLE_RATIO:1);
	
		Pit_AngleLimit();//pit�Ƕ�����
}




/**
 * @brief      ����ң��ģʽ�µ���̨����̵ķ���Ƕ�
 * @param[in]  Switch
 * @param[in]  yawparam
 * @retval     None
 */
float xxxx = 0.0015; //0.05
float aaaa = 0.03f; //5
float bbbb = 180.0f;
void CHAS_SeparateLimit(bool Switch, float *yawparam)
{
	static float Separate_diff = 0;
  if(Ctrl_Source != CTRL_RC || Chassis_Mode != CHAS_followMode)
	{
		return;
	}
	Separate_diff = M6020moto_chassis[0].total_angle / 22.7525f;
	if(Separate_diff > 300)
	{
			Separate_diff -= 360.0f;
	}
	else if(Separate_diff < -300)
	{
			Separate_diff += 360.0f;
	}
    if(fabs(Separate_diff) > 65.0f && *yawparam != 0)
    {
        YawRCCtrl_Coe -= (aaaa / bbbb);

        if(fabs(YawRCCtrl_Coe) < aaaa / (bbbb-10.0f))
        {
            YawRCCtrl_Coe = 0;
        }
    }
    else
    {
        YawRCCtrl_Coe = xxxx;
    }
}
/***********************PID����**********************************/
/**
  * @brief  �����ǿ���YAW
  * @param  void
  * @retval None
  */
void IMU_control_YAW(void)
{ 
			M6020_motor.YAW_P_result = YAW_OUT_PID(&M6020_Yaw_OUT,M6020_motor.remote_control_YAW,M6020_motor.YAW_OUT_Current);//�ǶȻ�
			M6020_motor.YAW_I_result = YAW_IN_PID(&M6020_Yaw_IN,M6020_motor.YAW_P_result,M6020_motor.YAW_IN_Current);//�ٶȻ�
}
/**
  * @brief  �Ӿ�YAW������PID����
  * @param  void
  * @retval None
  */
void Cloud_YAWIMUVisionPID(void)
{
	  if( Vision_Mode == Vision_BIGWindmill)//--- ���ģʽ
		{
        YAWVision.Vision_OUT = YAW_Vision_OUT_PID(&Yaw_Vision_Out_Pid,YAWVision.IMUVisionErr,M6020_motor.YAW_OUT_Current);
	      YAWVision.Vision_IN = YAW_Vision_IN_PID(&Yaw_Vision_IN_Pid,YAWVision.Vision_OUT,M6020_motor.YAW_IN_Current);
		}
		else if(Vision_Mode == Vision_Forcast)//--- Ԥ������
		{
        YAWVision.Vision_OUT = YAW_Vision_OUT_PID(&Yaw_Vision_Out_Pid_t,YAWVision.IMUVisionErr,M6020_motor.YAW_OUT_Current);
	      YAWVision.Vision_IN = YAW_Vision_IN_PID(&Yaw_Vision_IN_Pid_t,YAWVision.Vision_OUT,M6020_motor.YAW_IN_Current);			
		}
		else if(Vision_Mode == Vision_Top)//--- С����
		{
			 YAWVision.Vision_OUT = YAW_Vision_OUT_PID(&Yaw_Vision_Out_Pid_spin,YAWVision.IMUVisionErr,M6020_motor.YAW_OUT_Current);
	     YAWVision.Vision_IN = YAW_Vision_IN_PID(&Yaw_Vision_IN_Pid_spin,YAWVision.Vision_OUT,M6020_motor.YAW_IN_Current);			
		}
		else if(Vision_Mode == Vision_Outpost)//ǰ��վ
		{
			 YAWVision.Vision_OUT = YAW_Vision_OUT_PID(&Yaw_Vision_Out_Pid_spin,YAWVision.IMUVisionErr,M6020_motor.YAW_OUT_Current);
	     YAWVision.Vision_IN = YAW_Vision_IN_PID(&Yaw_Vision_IN_Pid_spin,YAWVision.Vision_OUT,M6020_motor.YAW_IN_Current);		
		}
		else //--- Ԥ������
		{
       YAWVision.Vision_OUT = YAW_Vision_OUT_PID(&Yaw_Vision_Out_Pid_t,YAWVision.IMUVisionErr,M6020_motor.YAW_OUT_Current);
	     YAWVision.Vision_IN = YAW_Vision_IN_PID(&Yaw_Vision_IN_Pid_t,YAWVision.Vision_OUT,M6020_motor.YAW_IN_Current);			
		}
}

/**
  * @brief  �����ǿ���PICTH
  * @param  void
  * @retval None
  */
void IMU_control_PITCH(void)
{
	PITCH_Use.PICTH_OUT = PICTH_OUT_PID(&M6020_PIT_OUT,PITCH_Use.RC_PICTH,PITCH_Use.PICTH_OUT_Current);//�ǶȻ�
	PITCH_Use.PICTH_IN = PICTH_IN_PID(&M6020_PIT_IN,PITCH_Use.PICTH_OUT,PITCH_Use.PICTH_IN_Current);//�ٶȻ�
}
/**
  * @brief  �Ӿ�PIT������PID����
  * @param  void
  * @retval None
  */
void Cloud_PITIMUVisionPID(void)
{
//			if(PITVision.IMUVisionErr > 4778)
//			{
//				PITVision.IMUVisionErr = 4778;
//			}
//			if(PITVision.IMUVisionErr < 3496)
//			{
//				PITVision.IMUVisionErr = 3496;
//			}
			Pit_AngleLimit2();
		  if(Vision_Mode == Vision_BIGWindmill)//--- ���ģʽ
			{
			  PITVision.Vision_OUT = PICTH_Vis_OUT_PID(&PIT_Vision_Out_Pid,PITVision.IMUVisionErr,PITCH_Use.PICTH_OUT_Current);
		    PITVision.Vision_IN = PICTH_Vis_IN_PID(&PIT_Vision_IN_Pid,PITVision.Vision_OUT,PITCH_Use.PICTH_IN_Current);
			}
			else if(Vision_Mode == Vision_Forcast)//--- Ԥ������
			{
				PITVision.Vision_OUT = PICTH_Vis_OUT_PID(&PIT_Vision_Out_Pid_t,PITVision.IMUVisionErr,PITCH_Use.PICTH_OUT_Current);
				PITVision.Vision_IN = PICTH_Vis_IN_PID(&PIT_Vision_IN_Pid_t,PITVision.Vision_OUT,PITCH_Use.PICTH_IN_Current);			
			}
			else if(Vision_Mode == Vision_Top)//--- С����
			{
				PITVision.Vision_OUT = PICTH_Vis_OUT_PID(&PIT_Vision_Out_Pid_spin,PITVision.IMUVisionErr,PITCH_Use.PICTH_OUT_Current);
				PITVision.Vision_IN = PICTH_Vis_IN_PID(&PIT_Vision_IN_Pid_spin,PITVision.Vision_OUT,PITCH_Use.PICTH_IN_Current);		
			}
			else if(Vision_Mode == Vision_Outpost)//--- ǰ��վ
			{
				PITVision.Vision_OUT = PICTH_Vis_OUT_PID(&PIT_Vision_Out_Pid_spin,PITVision.IMUVisionErr,PITCH_Use.PICTH_OUT_Current);
				PITVision.Vision_IN = PICTH_Vis_IN_PID(&PIT_Vision_IN_Pid_spin,PITVision.Vision_OUT,PITCH_Use.PICTH_IN_Current);		
			}
			else//--- Ԥ������
			{
				PITVision.Vision_OUT = PICTH_Vis_OUT_PID(&PIT_Vision_Out_Pid_t,PITVision.IMUVisionErr,PITCH_Use.PICTH_OUT_Current);
				PITVision.Vision_IN = PICTH_Vis_IN_PID(&PIT_Vision_IN_Pid_t,PITVision.Vision_OUT,PITCH_Use.PICTH_IN_Current);			
			}			
}
/***********************PID����**********************************/
/**
 * @brief  	   ����Ƕȸ���
 * @param[in]  keyi
 * @retval 	   None
 */
void Vision_Angle_Update(void)
{
			if(misslogo == 0)//û�ж�ʧĿ��֮ǰ  ��������
			{   
					YAWVision.IMUVisionErr = M6020_motor.YAW_OUT_Current + vision_yaw_angle * 22.7527f * 0.5f;
					PITVision.IMUVisionErr = PITCH_Use.PICTH_OUT_Current + vision_pit_angle * 22.7527f;

					miss_yaw = YAWVision.IMUVisionErr;
					miss_pit = PITVision.IMUVisionErr;
					RC_YAW = RC_PIT = 0;
			}
			else	if(misslogo == 1 && Gimbal_Mode == Gimbal_PCMode)  //��ʧĿ��ʱ�����ֵ�ǰ�Ƕ� ���Խ���ң�ؿ���
			{
				if(Ctrl_Source == CTRL_PC)
				{
						//���ֵ��Ϊ��̨��������ֵ
						RC_YAW += -(float)Get_MouseX() * 0.8f;
						RC_PIT += (float)Get_MouseY() * 0.3f;
				}
				else
				{
				    RC_YAW += -Get_CH0() * 0.02f;
				    RC_PIT += -Get_CH1() * 0.02f;
				}
				
				if(RC_PIT > 1000)
					RC_PIT = 1000;
				if(RC_PIT < -1000)
					RC_PIT = -1000;
				YAWVision.IMUVisionErr = RC_YAW + miss_yaw;
				PITVision.IMUVisionErr = RC_PIT + miss_pit;
			}	
	    else
      {
		    RC_YAW = RC_PIT = 0;
		  }			
	
}




/**
 * @brief      Pit����Ƕ��޷�
 * @param[in]  None
 * @retval     None
 */
float Limit_Min;
float Limit_Max;
void Pit_AngleLimit(void)
{
		//--- ������ģʽ������ֵ��Ϊ�ο� ӳ�䵽IMU�ǶȽ��нǶ�����
		Limit_Min = DJIC_IMU.total_pitch *ENCODER_ANGLE_RATIO - fabs(Get_PitDeviateL());
		Limit_Max = DJIC_IMU.total_pitch *ENCODER_ANGLE_RATIO + fabs(Get_PitDeviateH());

    if(PITCH_Use.RC_PICTH > Limit_Max)
    {
        PITCH_Use.RC_PICTH = Limit_Max;
    }
    else if(PITCH_Use.RC_PICTH < Limit_Min)
    {
        PITCH_Use.RC_PICTH = Limit_Min;
    }
}

void Pit_AngleLimit2(void)
{
		//--- ������ģʽ������ֵ��Ϊ�ο� ӳ�䵽IMU�ǶȽ��нǶ�����
		Limit_Min = DJIC_IMU.total_pitch *ENCODER_ANGLE_RATIO - fabs(Get_PitDeviateL1());
		Limit_Max = DJIC_IMU.total_pitch *ENCODER_ANGLE_RATIO + fabs(Get_PitDeviateH1());

    if(PITVision.IMUVisionErr > Limit_Max)
    {
        PITVision.IMUVisionErr = Limit_Max;
    }
    else if(PITVision.IMUVisionErr < Limit_Min)
    {
        PITVision.IMUVisionErr = Limit_Min;
    }
}
/**
 * @brief      ��ȡPit���ƫ���޷�ֵ�ĽǶ�
 * @param[in]  None
 * @retval     deviate encoder
 */
float Get_PitDeviateH(void)
{
    return ((M6020moto_chassis[3].angle>GIMBAL_PIT_MAX)? 0 : M6020moto_chassis[3].angle - GIMBAL_PIT_MAX);
}
float Get_PitDeviateL(void)
{
    return ((M6020moto_chassis[3].angle<GIMBAL_PIT_MIN)? 0 : M6020moto_chassis[3].angle - GIMBAL_PIT_MIN);
}


float Get_PitDeviateH1(void)
{
    return ((M6020moto_chassis[3].angle>GIMBAL_PIT_MAX1)? 0 : M6020moto_chassis[3].angle - GIMBAL_PIT_MAX1);
}
float Get_PitDeviateL1(void)
{
    return ((M6020moto_chassis[3].angle<GIMBAL_PIT_MIN1)? 0 : M6020moto_chassis[3].angle - GIMBAL_PIT_MIN1);
}
/**
 * @brief  	   ���������
 * @param[in]  state
 * @retval 	   None
 */
void Laser_Ctrl(bool state)
{
	HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, (GPIO_PinState)state);
}

float last_in;
float T = 0.002;
float forwardfeed(float in)
{
	float out,last_in;
	out = (in - last_in)/T + in;
	last_in = in;
	return out;
}
     
