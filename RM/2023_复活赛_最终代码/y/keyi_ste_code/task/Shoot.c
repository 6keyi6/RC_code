/**
 * @file shoot.c
 * @author keyi (hzjqq66@163.com)
 * @brief
 * @version 1.1
 * @date 2022-09-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
 /*
 *  拨盘   2006
    摩擦轮 3508 *2
 */
#include "shoot.h"
#include "can_control.h"
#include "Remote_Control.h"
#include "M2006_Motor.h"
#include "Devices_Monitor.h"
#include "Control_Vision.h"
#include "tim.h"
#include "DR16_control.h"
//--- 射频
#define SHOOT_FREQ(x)  (uint16_t)(500/(x))
// --- 单次卡弹时间上限
#define Limit_StuckDuration 120
// --- 拨盘单个齿轮的角度
#define GearTeeth_Angle     32764 //(8191*36/9)
//--- 反转齿轮数
#define RollBack_Num    2
#define Reload_Dir      1
// --- 摩擦轮安装方式 正置 1 倒置 -1
WorldTime_RxTypedef shootUnit_WorldTime;
Fric_CtrlMode_e Fric_Mode;//摩擦轮模式
//PID参数
incrementalpid_t FrictL_PID;
incrementalpid_t FrictR_PID;
positionpid_t M2006_Dial_Ppid;
positionpid_t M2006_Dial_Ipid;

M2006_motor_t M2006_motor = {0};//自定义结构体数据
Frict_motor_t Frict_motor = {0};//0是L 1是R
Reload_t Reload;          /*<! 拨盘参数 */
uint8_t Onebullet_heat = 10; /*<! 单个子弹的热量(小弹丸) */
uint16_t Shoot_Freq;        /*<! 射频 */
uint16_t Mshoot_Cnt,Ashoot_Cnt; /* <! 拨弹延时 */
int needShoot = 0;         //需要射击的颗数
uint8_t ContLaunch;       /*<! 连发标志位 */
uint8_t Rage_Mode;        /*<! 狂暴模式,无视热量限制 */
bool Mag_Switch;          /*<! 弹仓开关标志位 */
float Fric_SpeedOffset[4] = {0}; /*<! 摩擦轮速度波动补偿值 */

int waiting_time;//热量限制等待时间
int get_time;//热量限制时间
// --- 摩擦轮转速
int16_t FricSpeed_15 = 4120;//4120
int16_t FricSpeed_18 = 4630;//4630
int16_t FricSpeed_22 = 4700;
int16_t FricSpeed_30 = 7050;//7050
int16_t FricSpeed_Unload = 500;
/**
  * @brief  拨盘PID初始化
  * @param  void
  * @retval None
  */
void Dial_PID_Init(void)
{
		PositionPID_paraReset(&M2006_Dial_Ppid,0.4f,0.0f,1.1f,10000,1000);
	  PositionPID_paraReset(&M2006_Dial_Ipid,2.87f,0.008f,0.0f,9500,1000);
	  Reload.CriticalVal = 1500;
}
/**
  * @brief  摩擦轮PID初始化
  * @param  void
  * @retval None
  */
void Frict_PID_Init(void)
{   
		IncrementalPID_paraReset(&FrictL_PID,20.f,0.5f,0.f,15000,10000);
    IncrementalPID_paraReset(&FrictR_PID,20.f,0.5f,0.0f,15000,10000);
}

/**
  * @brief  射击总控制
  * @param  void
  * @retval None
  */
void Shoot_control(void)
{	
	     static uint16_t Mag_off_cnt = 1;
			 Fric_processing();//摩擦轮
	     AttackMode_Ctrl();//射击模式处理
	     Dial_PID_calc();//拨盘
       FiringRate(&Shoot.SpeedLimit);//发送给视觉子弹初速度
	
    //--- 防止操作手忘记关弹仓
    if(Ctrl_Source == CTRL_PC)
    {
//        if(abs(DR16_Get_ExptVx()) > 50 || abs(DR16_Get_ExptVy()) > 50)
//        {
//            Mag_off_cnt++;
//        }
//        else
//        {
//            Mag_off_cnt = 1;
//        }

//        if((Mag_off_cnt%=500) == 0)
//        {
//            Mag_Ctrl(OFF); //--- PC模式运动状态下1s关一次
//        }
    }
    else if(Ctrl_Source == CTRL_RC)
    {
        if(Chassis_Mode != CHAS_DisableMode && Chassis_Mode != CHAS_LockMode)
        {
            Mag_Ctrl(OFF);
        }
    }	
			 CAN_senddata(&hcan2,Frict_motor.Frict_OUTL,Frict_motor.Frict_OUTR,M2006_motor.Dial_I_result,0);
}
	

void Fric_processing(void)
{
   	Frict_motor.Frict_CurL = M3508_Friction[0].speed_rpm;
	  Frict_motor.Frict_CurR = M3508_Friction[1].speed_rpm;
	 
	  //--- 射速自调整
	  //FricSpeed_Adapt();
  
    if(Ctrl_Source == CTRL_PC && Attack_Mode != Attack_Disable)
		{
			  if(Vision_Mode == Vision_BIGWindmill)
				{
					  Laser_Ctrl(0);   //--- 打符不开激光
				}
				else 
				{
					  Laser_Ctrl(1);   
				}
				
				switch(ShootGet_SpeedLimit())//根据裁判系统设置发射速度 
				{
					case 15:
						   Fric_Mode = Fric_15m_s;
               Frict_motor.RC_FrictL = FricSpeed_15 + Fric_SpeedOffset[0];
					     Frict_motor.RC_FrictR = -(FricSpeed_15 + Fric_SpeedOffset[0]);
					break;
        case 18:
               Fric_Mode = Fric_18m_s;
               Frict_motor.RC_FrictL = FricSpeed_18 + Fric_SpeedOffset[1];
					     Frict_motor.RC_FrictR = -(FricSpeed_18 + Fric_SpeedOffset[1]);           
            break;
        case 22:
               Fric_Mode = Fric_22m_s;
               Frict_motor.RC_FrictL = FricSpeed_22 + Fric_SpeedOffset[2];
					     Frict_motor.RC_FrictR = -(FricSpeed_22 + Fric_SpeedOffset[2]);           
            break;
        case 30:
        case 75:
               Fric_Mode = Fric_30m_s;
               Frict_motor.RC_FrictL = (FricSpeed_30 + Fric_SpeedOffset[3]);
					     Frict_motor.RC_FrictR = -(FricSpeed_30 + Fric_SpeedOffset[3]);           
            break;				
					default:
               Fric_Mode = Fric_15m_s;
               Frict_motor.RC_FrictL = FricSpeed_15 + Fric_SpeedOffset[0];
					     Frict_motor.RC_FrictR = -(FricSpeed_15 + Fric_SpeedOffset[0]);
            break;					
				}
		}
		else if(Ctrl_Source == CTRL_RC)
    {
				switch(Fric_Mode)
				{
					case Fric_15m_s:
               Frict_motor.RC_FrictL = FricSpeed_15 + Fric_SpeedOffset[0];
					     Frict_motor.RC_FrictR = -(FricSpeed_15 + Fric_SpeedOffset[0]);		
					     Laser_Ctrl(ON);
               break;	
					case Fric_18m_s:
               Frict_motor.RC_FrictL = FricSpeed_18 + Fric_SpeedOffset[1];
					     Frict_motor.RC_FrictR = -(FricSpeed_18 + Fric_SpeedOffset[1]);     
               Laser_Ctrl(ON);					
						   break;
          case Fric_22m_s:
               Frict_motor.RC_FrictL = FricSpeed_22 + Fric_SpeedOffset[2];
					     Frict_motor.RC_FrictR = -(FricSpeed_22 + Fric_SpeedOffset[2]); 
               Laser_Ctrl(ON);					
               break;						
	        case Fric_30m_s:
               Frict_motor.RC_FrictL = (FricSpeed_30 + Fric_SpeedOffset[3]);
					     Frict_motor.RC_FrictR = -(FricSpeed_30 + Fric_SpeedOffset[3]);  
					     Laser_Ctrl(ON);
					     break;
					case Fric_Unload:
						   Frict_motor.RC_FrictL = FricSpeed_Unload;
					     Frict_motor.RC_FrictR = -FricSpeed_Unload;
               Laser_Ctrl(ON);					
						   break;
          case Fric_Disable:
               Frict_motor.RC_FrictL = 0;
					     Frict_motor.RC_FrictR = 0;		
					     break;
				  default:
               break;
				}
		}
		else
    {
          Frict_motor.RC_FrictL = 0;
					Frict_motor.RC_FrictR = 0;
			    Laser_Ctrl(OFF);
    }
		
		Frict_motor.RC_FrictL = Frict_motor.RC_FrictL > 8000 ? 8000 : Frict_motor.RC_FrictL;
		Frict_motor.RC_FrictR = Frict_motor.RC_FrictR > 8000 ? 8000 : Frict_motor.RC_FrictR;
		
    //--- 摩擦轮PID计算
    Frict_motor.Frict_OUTL = Incremental_PID(&FrictL_PID,Frict_motor.RC_FrictL,Frict_motor.Frict_CurL);
	  Frict_motor.Frict_OUTR = Incremental_PID(&FrictR_PID,Frict_motor.RC_FrictR,Frict_motor.Frict_CurR);
}
/**
 * @brief  	   摩擦轮转速调整 (Test ing)
 * @param[in]  枪管反馈速度 射速区间 递减值 递增值
 * @note       在转速调整的基础上加入温控处理，因为经过测试发现温度越高射速也越高
 * @note       优先测15 18 30，22几乎用不到
 * @retval 	   None
 */
float Pre_GunSpeed;//上一次转速
float min_speed[4] = {13.4f, 16.8f, 19.0f, 27.8f};
float max_speed[4] = {14.5f, 17.3f, 21.4f, 29.3f};
float sub_val[4] = {30, 55, 20, 45};//参数待调整
float add_val[4] = {25, 20, 40, 55};//参数待调整
void FricSpeed_Adapt(void)
{
    static uint8_t speederr_flag = 0;	
	  uint8_t fricmode = 0;//对应数组0-4

    if(Fric_Mode == Fric_Unload || Fric_Mode == Fric_Disable)
    {
        return;
    }	
		fricmode = Fric_Mode -2;

    //--- 判断射速是否更新 因为裁判系统反馈的射速是float类型的 相邻两次的数据几乎不会相等
    if(Get_GunSpeed() != Pre_GunSpeed)
    {
	        //----- 射速波动调整 -----//
        if(Get_GunSpeed() < min_speed[fricmode] && Get_GunSpeed() > 10.0f)
        {
            speederr_flag = 1;
        }		
        else if(Get_GunSpeed() >= min_speed[fricmode] && Get_GunSpeed() <= max_speed[fricmode])
        {
            speederr_flag = 0;
        }
				
        if(speederr_flag == 1) //--- 射速偏低
        {
            speederr_flag = 0;
            Fric_SpeedOffset[fricmode] += add_val[fricmode];
        }
        if(Get_GunSpeed() > max_speed[fricmode]) //--- 射速偏高
        {
            Fric_SpeedOffset[fricmode] -= sub_val[fricmode];
        }			
				
		}			
		
		Pre_GunSpeed = Get_GunSpeed();
}
/**
 * @brief  	   射击模式处理
 * @param[in]  None
 * @retval 	   None
 */
uint8_t biubiubiu; // 一次吐n颗
uint8_t biubiubiu_flag;
uint8_t kkllmm = 0;
uint8_t shoot_100 = false;
uint8_t biubiubiu_flag_100ms;
void AttackMode_Ctrl(void)
{
    Mshoot_Cnt++;
    Ashoot_Cnt++;	
    if(Attack_Mode == Attack_Disable)
    {
        Mshoot_Cnt = 0;
        Ashoot_Cnt = 0;
        return;
    }	
    switch(Attack_Mode)
    {
    case Attack_15:
    case Attack_18:
    case Attack_22:
    case Attack_30:
        Ashoot_Cnt = 0;	

        if(Rage_Mode == true )
        {
            Shoot_Freq = 20; //--- 换血模式解除热量限制
        }		
				else if(Vision_Mode == Vision_BIGWindmill && Shoot_Freq >= 10)
				{
					  //--- 打符连发射频给小点
            Shoot_Freq = 6;
				}
				
	
        if(Shoot_Freq == 0)
        {
            Set_ReloadNum(0);
        }				
        else if((Mshoot_Cnt %= SHOOT_FREQ(Shoot_Freq)) == 0 && ContLaunch == true && Shoot_Freq > 0)
        {
             Set_ReloadNum(1);
				}
		 break;
		case Attack_Auto:
			   Mshoot_Cnt = 0;

        if(VisionData.RawData.whether_Fire == false || Gimbal_Mode != Gimbal_PCMode)
        {}
			
        if(VisionData.RawData.whether_Fire != false)//自动开火指令
        {	
            if(biubiubiu_flag == false)
            {
                  biubiubiu_flag = true;
                  biubiubiu = 1;
	
            }
        }				
        else
        {}
									
					
				if(shoot_100 == true)
				{
					biubiubiu_flag_100ms++;
					if(biubiubiu_flag_100ms == 200)
					{
						shoot_100 = false;
					}
				}

        if (VisionData.RawData.mode!= false/*&& DevicesGet_State(VISION_MONITOR) != Off_line*/)
        {
            if(biubiubiu_flag == true && biubiubiu > 0 && shoot_100 == false)
            {
                if(Shoot_Freq >= 2)
                {
                    Set_ReloadNum(1);
                    biubiubiu--;
									  shoot_100 = true;
									  biubiubiu_flag_100ms = 0;
                }
            }
            else
            {
                biubiubiu_flag = false;
            }
        }    

				
//			 if (Ashoot_Cnt == 50  && VisionData.RawData.mode!= false)
//        {
//            if(biubiubiu_flag == true && biubiubiu > 0 )
//            {
//                if(Shoot_Freq >= 2)
//                {
//                    Set_ReloadNum(1);
//                    biubiubiu--;
//                }
//            }
//            else
//            {
//                biubiubiu_flag = false;
//            }
//						Ashoot_Cnt = 0;
//        }  
				
			break;
case Attack_Unload:
	     if(Shoot_Freq == 0)
        {
            Set_ReloadNum(0);
        }		
        else if((Mshoot_Cnt%=SHOOT_FREQ(Shoot_Freq)) == 0 && ContLaunch == true)
        {
            Set_ReloadNum(1);
        }
        break;		
	  }		

#if USE_RM_Referee


#else
    Reload.CanShoot_Num = 10;
#endif	
}

/**
  * @brief  拨盘PID计算
  * @param  void
  * @retval None
  */
//减速比 1：36
float total_angle;
uint8_t CanShoot_Limit = 4; //--- 可发射数阈值，避免因偷打超热量
float hhjjk;
void Dial_PID_calc(void)
{	
		 //测量值
		 M2006_motor.pos_temp = M2006_Reload.total_angle;
		 M2006_motor.IN_CurSpeed = M2006_Reload.speed_rpm;
	
    // --- 判断发射系统是否正常工作
//#if USE_RM_Referee
//    if(Get_ShooterPower() == OFF)
//    {
//        Reload.Num = 0;
//        return;
//    }
//#endif

    if(Attack_Mode == Attack_Disable || DevicesGet_State(RELOAD_MONITOR) == Off_line)
    {
        //--- 以当前位置为启动位置
        Reload_Reset();
        Reload.Began = false;
        Reload.Num = 0;
        return;
    }	
	
        if(Shoot_Freq <= 0)//新加大于20 判断
        {
            Reload.Num = 0;
            return;
        }
//    //--- 先执行上次被激活的卡弹任务
    if(Reload.Stuck_Duration > Limit_StuckDuration)
    {
        if(Judge_ReloadStall() == true) //--- 扭矩过大，避免执行反转任务时也卡弹
        {
            Reload.Stuck_Duration++;
            //--- 卡弹时间为 200~300，正转，需改变方向
            if(((int)(Reload.Stuck_Duration / Limit_StuckDuration)) % 2 == 0 && Reload.Motor_Direction == 1)
            {
                Reload.Re_TargetAngle += GearTeeth_Angle * RollBack_Num;
                Reload.Motor_Direction = 0; //--- 进行正转反堵
            }
            else if(((int)(Reload.Stuck_Duration / Limit_StuckDuration)) % 2 == 1 && Reload.Motor_Direction == 0)
            {
                Reload.Re_TargetAngle -= GearTeeth_Angle * RollBack_Num;
                Reload.Motor_Direction = 1; //--- 进行反转
            }
        }

        //---- 已完成卡弹反转任务
        if(abs(M2006_Reload.total_angle - Reload.Re_TargetAngle) <= GearTeeth_Angle)
        {
            Reload.Stuck_Flag = false;
            Reload.Re_TargetAngle = 0;
            Reload.Stuck_Duration = 0;
        }
        else
        {
				  	M2006_motor.Dial_P_result = Shoot_Position_PID(&M2006_Dial_Ppid,Reload.Re_TargetAngle ,M2006_motor.pos_temp);
       		  M2006_motor.Dial_I_result = Shoot_Position_PID(&M2006_Dial_Ipid,M2006_motor.Dial_P_result,M2006_motor.IN_CurSpeed);
            return;
        }
    }	
	
		
    //--- 有发射弹丸需求
    if(Reload.Num > 0)
    {		
        //--- 射频不满足发送需求
        if(Shoot_Freq <= 0)//新加大于20 判断
        {
            Reload.Num = 0;
            return;
        }

        // --- 新任务未开始执行
        if(Reload.Began == false)
        {
            // --- 重置准备进入状态,
            Reload.Began = true;
//            Reload_Reset();
            Reload.Init_Angle = M2006_Reload.total_angle;
            Reload.Target_Angle += Reload.Num * GearTeeth_Angle * Reload_Dir;
        }

        // --- 拨弹任务完成
        if(abs(M2006_Reload.total_angle - Reload.Init_Angle) >= Reload.Num * GearTeeth_Angle - 8191)
        {
            Reload.Num = 0;
            Reload.Began = false;
            Reload.Stuck_Flag = false;
            Reload.Re_TargetAngle = 0;
            Reload.Launched_Num = Reload.Num;
            Reload.Stuck_Duration = 0;
        }
        else
        {
            // --- 更新剩余待拨动的弹丸
            Reload.Launched_Num = abs(M2006_Reload.total_angle - Reload.Init_Angle) / GearTeeth_Angle;
        }

        // --- 卡弹检测
        if(Judge_ReloadStall() == true)
        {
            Reload.Stuck_Duration++;
            if(Reload.Stuck_Duration > Limit_StuckDuration)
            {
                Reload.Stuck_Flag = true; // --- 激活卡弹任务，下一次开始反转
                Reload.Motor_Direction = 1; // --- 开始执行首次反转任务
                Reload.Re_TargetAngle = M2006_Reload.total_angle - GearTeeth_Angle * 2;
            }
        }
        else
        {
            if(Reload.Stuck_Duration > 0)
            {
                Reload.Stuck_Duration -= 10;
            }
            else
            {
                Reload.Stuck_Duration = 0;
            }
        }
    }
    else
    {
        Reload.Num = 0;
    }		

		M2006_motor.Dial_P_result = Shoot_Position_PID(&M2006_Dial_Ppid,Reload.Target_Angle,M2006_motor.pos_temp);
		M2006_motor.Dial_I_result = Shoot_Position_PID(&M2006_Dial_Ipid,M2006_motor.Dial_P_result,M2006_motor.IN_CurSpeed);
		
}


/**
 * @brief  	   设置弹丸装填数
 * @param[in]  num
 * @retval 	   None 
 */
void Set_ReloadNum(uint16_t num)
{
    //--- 防止数据异常
    if(num <= 0)
    {
        Reload.Began = false;
        Reload.Stuck_Flag = false;
        Reload.Re_TargetAngle = 0;
        Reload.Stuck_Duration = 0;
        Reload_Reset();
        return;
    }

    if(Reload.Stuck_Flag != true)
    {
        Reload.Began = false;
//        Reload_Reset();
        Reload.Num = num;
			   if(Shoot_Freq <= 0)//新加大于20 判断
        {
            Reload.Num = 0;  
        }
    }
		

}

/**
 * @brief  	   拨盘重置
 * @param[in]  None
 * @retval 	   None
 */
void Reload_Reset(void)
{   
	  M2006_motor.Dial_I_result = 0;

    Reload.Target_Angle = M2006_Reload.total_angle;
}

/**
 * @brief  	   判断拨盘电机是否处于卡弹临界值
 * @param[in]  None
 * @retval 	   state
 */
bool Judge_ReloadStall(void)
{
    return(abs(M2006_Reload.speed_rpm) < Reload.CriticalVal);

}

/**
 * @brief  	   舵机控制
 * @param[in]  
 * @retval 	   None
 */

uint16_t mag_off = 4700;
uint16_t mag_on = 1800;

uint16_t mag_on_L = 4200;
uint16_t mag_off_L = 1300;
void Mag_Ctrl(uint8_t state)
{
    if (state == 1)
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, mag_on);//左		
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, mag_on_L);//右

		Mag_Switch = 1;
	}
	else
	{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, mag_off);//左		
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, mag_off_L);//右

		Mag_Switch = 0;
	}	
}

/**
 * @brief  	   获取裁判系统相关信息
 * @param[in]  None
 * @retval 	   shoot power state
 */
uint16_t ShootGet_SpeedLimit(void)//射速限制
{
	return Shoot.SpeedLimit; 
}
float Get_GunSpeed(void)//当前射速
{
  return Shoot.BulletSpeed;
}
uint16_t Get_CoolingLimit(void)//热量限制
{
	return cooling_limit;
}
uint16_t Get_GunHeat(void)//当前热量
{
	return cooling_heat;
}
uint16_t Get_CoolingRate(void)//冷却速率
{
	return cooling_rate;
}
