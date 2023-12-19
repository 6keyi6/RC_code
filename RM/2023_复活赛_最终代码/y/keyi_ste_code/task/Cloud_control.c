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
//--- 速度单位转不转为机械角度
#define turnangle 0
#define turnAngle 1
#define ENCODER_ANGLE_RATIO 22.7527f 



/*************************用户自定义**************************************/
YAWVision_t YAWVision;
PITVision_t PITVision;
uint8_t ModeTurn = 0;
float YawRCCtrl_Coe;//底盘控制量系数
uint8_t Self_aiming_logo = 0;//自瞄开启标志位
uint8_t misslogo = 0;//自瞄丢失标志位
float miss_yaw,miss_pit;//丢失目标前一次的测量值
float RC_YAW,RC_PIT;//视觉丢失目标时控制端的输入
uint8_t init_mode = true;
extKalman_t Kalman_CHASFollow_Speed;

first_order_filter_type_t first_order_filter_BIG_YAW;
fp32 num_YAW[1] = {10};

first_order_filter_type_t first_order_filter_BIG_PIT;
fp32 num_PIT[1] = {10};
/*************************用户自定义**************************************/

/*************************PID结构体**************************************/

/********************打符**********************/
positionpid_t Yaw_Vision_Out_Pid;//YAW轴打符外环PID参数
positionpid_t Yaw_Vision_IN_Pid;//YAW轴打符内环PID参数
positionpid_t PIT_Vision_Out_Pid;//PIT轴打符外环PID参数
positionpid_t PIT_Vision_IN_Pid;//PIT轴打符内环PID参数
/********************打符**********************/

/********************自瞄**********************/
positionpid_t Yaw_Vision_Out_Pid_t;//YAW轴自瞄外环PID参数
positionpid_t Yaw_Vision_IN_Pid_t;//YAW轴自瞄内环PID参数
positionpid_t PIT_Vision_Out_Pid_t;//PIT轴自瞄外环PID参数
positionpid_t PIT_Vision_IN_Pid_t;//PIT轴自瞄内环PID参数
/********************自瞄**********************/

/********************小陀螺**********************/
positionpid_t Yaw_Vision_Out_Pid_spin;//YAW轴小陀螺外环PID参数
positionpid_t Yaw_Vision_IN_Pid_spin;//YAW轴小陀螺内环PID参数
positionpid_t PIT_Vision_Out_Pid_spin;//PIT轴小陀螺外环PID参数
positionpid_t PIT_Vision_IN_Pid_spin;//PIT轴小陀螺内环PID参数
/********************小陀螺**********************/

/********************普通**********************/
 positionpid_t M6020_Yaw_OUT;//YAW轴PID参数
 positionpid_t M6020_Yaw_IN;//YAW轴PID参数
 positionpid_t M6020_PIT_OUT;//PIT轴PID参数
 positionpid_t M6020_PIT_IN;//PIT轴PID参数
/********************普通**********************/

/*************************PID结构体**************************************/

/**
  * @brief  云台总PID初始化
  * @param  void
  * @retval None
  */
void Cloud_PID_Init(void)
{
  YAW_PID_Init();
	PITCH_PID_Init();
    //--- 底盘跟随模式的双环模式的内环测量值滤波
  KalmanCreate(&Kalman_CHASFollow_Speed, 1, 20);	
}
/**
  * @brief  陀螺仪控制YAW轴电机PID初始化
  * @param  void
  * @retval None
  */
void YAW_PID_Init(void)
{
	  PositionPID_paraReset(&M6020_Yaw_OUT,0.7,0,0,29000,20000);
	  PositionPID_paraReset(&M6020_Yaw_IN,280.f,0.f,0,29000,20000);
	
	  /***********************自瞄PID初始化*****************************/
	  PositionPID_paraReset(&Yaw_Vision_Out_Pid,0.5f,0,0,29000,20000);	  //打符	 
	  PositionPID_paraReset(&Yaw_Vision_IN_Pid,250.f,6.f,0,29000,20000);
	  Yaw_Vision_IN_Pid.I_SeparThresh = 200;
		
		PositionPID_paraReset(&Yaw_Vision_Out_Pid_t,0.6f,0,0,29000,20000);  //自瞄
	  PositionPID_paraReset(&Yaw_Vision_IN_Pid_t,300.f,2.f,0,29000,20000);
	  Yaw_Vision_IN_Pid_t.I_SeparThresh = 200;
	
		PositionPID_paraReset(&Yaw_Vision_Out_Pid_spin,0.5f,0,0,29000,20000);  
	  PositionPID_paraReset(&Yaw_Vision_IN_Pid_spin,250.f,8.f,0,29000,20000);//小陀螺
	  Yaw_Vision_IN_Pid_spin.I_SeparThresh = 200;
		/***********************自瞄PID初始化*****************************/
}

/**
  * @brief  云台PICTH轴角度控制初始化
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
	
		 /***********************自瞄PID初始化*****************************/
			 PositionPID_paraReset(&PIT_Vision_Out_Pid,0.6f,0.0f,0.0f,29000,20000);//打符
			 PIT_Vision_Out_Pid.I_SeparThresh = 0;
			 PositionPID_paraReset(&PIT_Vision_IN_Pid,180,1.f,0.f,29990,20000); 
			 PIT_Vision_IN_Pid.I_SeparThresh = 300;

			 PositionPID_paraReset(&PIT_Vision_Out_Pid_t,0.5f,0.0f,0.0f,29000,10000);//自瞄
			 PIT_Vision_Out_Pid_t.I_SeparThresh = 0;
			 PositionPID_paraReset(&PIT_Vision_IN_Pid_t,200,5.f,0.f,29990,20000); 
			 PIT_Vision_IN_Pid_t.I_SeparThresh = 300;
	
			 PositionPID_paraReset(&PIT_Vision_Out_Pid_spin,0.6f,0.0f,0.0f,29000,10000);//小陀螺
			 PIT_Vision_Out_Pid_spin.I_SeparThresh = 0;
			 PositionPID_paraReset(&PIT_Vision_IN_Pid_spin,180,8.f,0.f,29990,20000); 
			 PIT_Vision_IN_Pid_spin.I_SeparThresh = 300;
}
/**
  * @brief  云台总控制函数
  * @param  void
  * @retval None
  */
int init_cnt = 2500;
void Cloud_control(void)
{
	Get_CloudMeasured();//获取测量值
	Vision_Angle_Update();//自瞄角度更新
    //--- 等待IMU初始化完毕
    if(init_mode)
    {
        if(DJIC_IMU.yaw != 0 || DJIC_IMU.pitch != 0)
        {
            if(--init_cnt != 0) //--- 等待IMU稳定 800*2ms数据稳定时间+500*2ms误差采样时间
            {
              Set_InitAngle();
            }
						else
						{
							init_mode = false;
							Chassis_start = true;//传输数据至底盘标志位
						}
        }
        return;
    }	
		
		
	if(DevicesGet_State(DR16_MONITOR) == Off_line || Gimbal_Mode == Gimbal_DisableMode)//--- 离线 & 失能
	{ 
			Set_InitAngle();//设置初始目标角度 防炸
		  TargetAngle_Update(0,0);//获取遥控器输入量
			PositionPID_Reset(&M6020_Yaw_IN);//普通PID
			PositionPID_Reset(&M6020_PIT_IN);

			PositionPID_Reset(&Yaw_Vision_Out_Pid);//打符
			PositionPID_Reset(&Yaw_Vision_IN_Pid);
			PositionPID_Reset(&PIT_Vision_Out_Pid);
			PositionPID_Reset(&PIT_Vision_IN_Pid);
			
			PositionPID_Reset(&Yaw_Vision_Out_Pid_t);//预测自瞄
			PositionPID_Reset(&Yaw_Vision_IN_Pid_t);
			PositionPID_Reset(&PIT_Vision_Out_Pid_t);
			PositionPID_Reset(&PIT_Vision_IN_Pid_t);
			
			PositionPID_Reset(&Yaw_Vision_Out_Pid_spin);//小陀螺
			PositionPID_Reset(&Yaw_Vision_IN_Pid_spin);
			PositionPID_Reset(&PIT_Vision_Out_Pid_spin);
			PositionPID_Reset(&PIT_Vision_IN_Pid_spin);
		
		  Self_aiming_logo = 0;//自瞄标志位清零
			misslogo = 0;//丢失标志位清零
			return;
	} 

	  //当使用的是C板时
	if(ModeTurn != 2)//获取初始角度
	{
		M6020_motor.remote_control_YAW = DJIC_IMU.total_yaw  * (turnAngle?ENCODER_ANGLE_RATIO:1);
		PITCH_Use.RC_PICTH = DJIC_IMU.pitch * (turnAngle?ENCODER_ANGLE_RATIO:1);
		ModeTurn = 2;
	}
	
		if(DevicesGet_State(GIMBAL_YAW_MONITOR) == Off_line)//---如果电机离线遥控器的值不能更新
		{
			TargetAngle_Update(0,-DR16_Get_ExptPit());//获取遥控器输入量
		}
		else if(DevicesGet_State(GIMBAL_PIT_MONITOR) == Off_line)
		{
			TargetAngle_Update(-DR16_Get_ExptYaw(),0);//获取遥控器输入量
		}
		else
		{
	    TargetAngle_Update(-DR16_Get_ExptYaw(),-DR16_Get_ExptPit());//获取遥控器输入量
		}
	
	switch(Gimbal_Mode)
	{  		
	  case Gimbal_PCMode:// PC控制(自瞄)
		{ 
					if(VisionData.RawData.mode == 0 || (VisionData.RawData.yaw_angle == 0 || VisionData.RawData.pitch_angle == 0 ))//丢失目标
					 { 
						 misslogo = 1; 
					 }
						else if(VisionData.RawData.mode == 1)
					 {
						 misslogo = 0;
					 }
					 Cloud_YAWIMUVisionPID();
					 Cloud_PITIMUVisionPID();
					 //--- 发送电流
					 CAN_senddata_6020(&hcan1,YAWVision.Vision_IN ,0,0,PITVision.Vision_IN);
					 
					//--- 自瞄结束时更新目前角度
					M6020_motor.remote_control_YAW = DJIC_IMU.total_yaw  * (turnAngle?ENCODER_ANGLE_RATIO:1);
					PITCH_Use.RC_PICTH =  DJIC_IMU.pitch* (turnAngle?ENCODER_ANGLE_RATIO:1);	 
    break;
		}
		case Gimbal_NormalMode:// 普通运转模式
		{
				 IMU_control_YAW();
				 IMU_control_PITCH();
			   //--- 发送电流
				 CAN_senddata_6020(&hcan1,M6020_motor.YAW_I_result,0,0,PITCH_Use.PICTH_IN);
			
	       miss_yaw = DJIC_IMU.total_yaw  * (turnAngle?ENCODER_ANGLE_RATIO:1);
				 miss_pit = DJIC_IMU.pitch* (turnAngle?ENCODER_ANGLE_RATIO:1);
			break;
		}
	}
}

/**
  * @brief  获取云台测量值
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

/**设置初始目标角度
  * @brief  
  * @param  void
  * @retval None
  */
void Set_InitAngle(void)
{
    M6020_motor.remote_control_YAW = DJIC_IMU.total_yaw * (turnAngle?ENCODER_ANGLE_RATIO:1);
		PITCH_Use.RC_PICTH = DJIC_IMU.pitch * (turnAngle?ENCODER_ANGLE_RATIO:1);
	  //初始自瞄角度
	  YAWVision.IMUVisionErr = DJIC_IMU.total_yaw  * (turnAngle?ENCODER_ANGLE_RATIO:1);
		PITVision.IMUVisionErr = DJIC_IMU.pitch * (turnAngle?ENCODER_ANGLE_RATIO:1);
}
/**目标角度更新
  * @brief  
  * @param  void
  * @retval None
  */
void TargetAngle_Update(float Yawparam, float Pitparam)
{
	  M6020_motor.remote_control_YAW +=  Yawparam * (turnAngle?ENCODER_ANGLE_RATIO:1) * 0.5f;
		PITCH_Use.RC_PICTH += Pitparam * (turnAngle?ENCODER_ANGLE_RATIO:1);
	
		Pit_AngleLimit();//pit角度限制
}




/**
 * @brief      限制遥控模式下的云台与底盘的分离角度
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
/***********************PID计算**********************************/
/**
  * @brief  陀螺仪控制YAW
  * @param  void
  * @retval None
  */
void IMU_control_YAW(void)
{ 
			M6020_motor.YAW_P_result = YAW_OUT_PID(&M6020_Yaw_OUT,M6020_motor.remote_control_YAW,M6020_motor.YAW_OUT_Current);//角度环
			M6020_motor.YAW_I_result = YAW_IN_PID(&M6020_Yaw_IN,M6020_motor.YAW_P_result,M6020_motor.YAW_IN_Current);//速度环
}
/**
  * @brief  视觉YAW轴自瞄PID计算
  * @param  void
  * @retval None
  */
void Cloud_YAWIMUVisionPID(void)
{
	  if( Vision_Mode == Vision_BIGWindmill)//--- 打符模式
		{
        YAWVision.Vision_OUT = YAW_Vision_OUT_PID(&Yaw_Vision_Out_Pid,YAWVision.IMUVisionErr,M6020_motor.YAW_OUT_Current);
	      YAWVision.Vision_IN = YAW_Vision_IN_PID(&Yaw_Vision_IN_Pid,YAWVision.Vision_OUT,M6020_motor.YAW_IN_Current);
		}
		else if(Vision_Mode == Vision_Forcast)//--- 预测自瞄
		{
        YAWVision.Vision_OUT = YAW_Vision_OUT_PID(&Yaw_Vision_Out_Pid_t,YAWVision.IMUVisionErr,M6020_motor.YAW_OUT_Current);
	      YAWVision.Vision_IN = YAW_Vision_IN_PID(&Yaw_Vision_IN_Pid_t,YAWVision.Vision_OUT,M6020_motor.YAW_IN_Current);			
		}
		else if(Vision_Mode == Vision_Top)//--- 小陀螺
		{
			 YAWVision.Vision_OUT = YAW_Vision_OUT_PID(&Yaw_Vision_Out_Pid_spin,YAWVision.IMUVisionErr,M6020_motor.YAW_OUT_Current);
	     YAWVision.Vision_IN = YAW_Vision_IN_PID(&Yaw_Vision_IN_Pid_spin,YAWVision.Vision_OUT,M6020_motor.YAW_IN_Current);			
		}
		else if(Vision_Mode == Vision_Outpost)//前哨站
		{
			 YAWVision.Vision_OUT = YAW_Vision_OUT_PID(&Yaw_Vision_Out_Pid_spin,YAWVision.IMUVisionErr,M6020_motor.YAW_OUT_Current);
	     YAWVision.Vision_IN = YAW_Vision_IN_PID(&Yaw_Vision_IN_Pid_spin,YAWVision.Vision_OUT,M6020_motor.YAW_IN_Current);		
		}
		else //--- 预测自瞄
		{
       YAWVision.Vision_OUT = YAW_Vision_OUT_PID(&Yaw_Vision_Out_Pid_t,YAWVision.IMUVisionErr,M6020_motor.YAW_OUT_Current);
	     YAWVision.Vision_IN = YAW_Vision_IN_PID(&Yaw_Vision_IN_Pid_t,YAWVision.Vision_OUT,M6020_motor.YAW_IN_Current);			
		}
}

/**
  * @brief  陀螺仪控制PICTH
  * @param  void
  * @retval None
  */
void IMU_control_PITCH(void)
{
	PITCH_Use.PICTH_OUT = PICTH_OUT_PID(&M6020_PIT_OUT,PITCH_Use.RC_PICTH,PITCH_Use.PICTH_OUT_Current);//角度环
	PITCH_Use.PICTH_IN = PICTH_IN_PID(&M6020_PIT_IN,PITCH_Use.PICTH_OUT,PITCH_Use.PICTH_IN_Current);//速度环
}
/**
  * @brief  视觉PIT轴自瞄PID计算
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
		  if(Vision_Mode == Vision_BIGWindmill)//--- 打符模式
			{
			  PITVision.Vision_OUT = PICTH_Vis_OUT_PID(&PIT_Vision_Out_Pid,PITVision.IMUVisionErr,PITCH_Use.PICTH_OUT_Current);
		    PITVision.Vision_IN = PICTH_Vis_IN_PID(&PIT_Vision_IN_Pid,PITVision.Vision_OUT,PITCH_Use.PICTH_IN_Current);
			}
			else if(Vision_Mode == Vision_Forcast)//--- 预测自瞄
			{
				PITVision.Vision_OUT = PICTH_Vis_OUT_PID(&PIT_Vision_Out_Pid_t,PITVision.IMUVisionErr,PITCH_Use.PICTH_OUT_Current);
				PITVision.Vision_IN = PICTH_Vis_IN_PID(&PIT_Vision_IN_Pid_t,PITVision.Vision_OUT,PITCH_Use.PICTH_IN_Current);			
			}
			else if(Vision_Mode == Vision_Top)//--- 小陀螺
			{
				PITVision.Vision_OUT = PICTH_Vis_OUT_PID(&PIT_Vision_Out_Pid_spin,PITVision.IMUVisionErr,PITCH_Use.PICTH_OUT_Current);
				PITVision.Vision_IN = PICTH_Vis_IN_PID(&PIT_Vision_IN_Pid_spin,PITVision.Vision_OUT,PITCH_Use.PICTH_IN_Current);		
			}
			else if(Vision_Mode == Vision_Outpost)//--- 前哨站
			{
				PITVision.Vision_OUT = PICTH_Vis_OUT_PID(&PIT_Vision_Out_Pid_spin,PITVision.IMUVisionErr,PITCH_Use.PICTH_OUT_Current);
				PITVision.Vision_IN = PICTH_Vis_IN_PID(&PIT_Vision_IN_Pid_spin,PITVision.Vision_OUT,PITCH_Use.PICTH_IN_Current);		
			}
			else//--- 预测自瞄
			{
				PITVision.Vision_OUT = PICTH_Vis_OUT_PID(&PIT_Vision_Out_Pid_t,PITVision.IMUVisionErr,PITCH_Use.PICTH_OUT_Current);
				PITVision.Vision_IN = PICTH_Vis_IN_PID(&PIT_Vision_IN_Pid_t,PITVision.Vision_OUT,PITCH_Use.PICTH_IN_Current);			
			}			
}
/***********************PID计算**********************************/
/**
 * @brief  	   自瞄角度更新
 * @param[in]  keyi
 * @retval 	   None
 */
void Vision_Angle_Update(void)
{
			if(misslogo == 0)//没有丢失目标之前  持续更新
			{   
					YAWVision.IMUVisionErr = M6020_motor.YAW_OUT_Current + vision_yaw_angle * 22.7527f * 0.5f;
					PITVision.IMUVisionErr = PITCH_Use.PICTH_OUT_Current + vision_pit_angle * 22.7527f;

					miss_yaw = YAWVision.IMUVisionErr;
					miss_pit = PITVision.IMUVisionErr;
					RC_YAW = RC_PIT = 0;
			}
			else	if(misslogo == 1 && Gimbal_Mode == Gimbal_PCMode)  //丢失目标时，保持当前角度 可以进行遥控控制
			{
				if(Ctrl_Source == CTRL_PC)
				{
						//鼠标值作为云台控制输入值
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
 * @brief      Pit电机角度限幅
 * @param[in]  None
 * @retval     None
 */
float Limit_Min;
float Limit_Max;
void Pit_AngleLimit(void)
{
		//--- 陀螺仪模式以码盘值作为参考 映射到IMU角度进行角度限制
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
		//--- 陀螺仪模式以码盘值作为参考 映射到IMU角度进行角度限制
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
 * @brief      获取Pit电机偏离限幅值的角度
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
 * @brief  	   激光红点控制
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
     
