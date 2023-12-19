/**
 ------------------------------------------------------------------------------
 * @file    DR16_control.c
 * @author  keyi
 * @brief   DR16 用户控制
 * @version V0.1
 * @date    2023-2-01
 * @copyright Copyright (c) 2022
 ------------------------------------------------------------------------------
 */
#include "DR16_control.h"
#include "Remote_Control.h"
#include "Shoot.h"
#include "control_Vision.h"
#include "Chassis_control.h"
#include "can_control.h"
#include "cmsis_armcc.h"
#include "shoot.h"
#include "Devices_Monitor.h"
Expt_t Expt;

//--- 键鼠模式下的小陀螺移动VX VY输出值
#define MoveValue 150
#define FollowMoveValue 660
/**
 * @brief      拨杆模式更新
 * @param[in]  None
 * @retval     None
 */
uint8_t Fric_flag;
uint8_t Fric_flag_test = 0;
uint8_t Fric_flag_test1 = 0;
int test_shoot = 0;
int PC_shoot = 0;
uint8_t Mag_flag = 1;
uint16_t Reset_cnt;
uint8_t Robot_Reset_flag = true;
uint8_t ResetFlag = false;   /*<! 重启标志位 */
uint8_t Reflash_UI;  /*<! UI刷新 */
uint8_t Diagonal_Mode = false;//对角模式标志位 

uint8_t ChassisResetFlag = false;   /*<! 底盘重启标志位 */
void DR16_LeverMode_Update(void)
 {
	 switch(Get_S1_L())
	 {
		  case Lever_UP: // --- 左上 -----------------------------------------------
				   switch(Get_S2_R())
					 {
							case Lever_UP: // --- PC端控制
									if (Ctrl_Source != CTRL_PC)
									{
										Set_GimbalMode(Gimbal_NormalMode);//普通模式
										Set_ChassisMode(CHAS_followMode);//跟随模式
										Set_AttackMode(Attack_Disable);//开启摩擦轮
										Set_VisionMode(Vision_Forcast);//开启预测
                    Mag_Ctrl(OFF);//关闭弹仓 ......
									}
							     CtrlSource_Switch(CTRL_PC);//控制源改为PC
						  break;/* 左上-右上 END ------------------------------------------*/
					 
							case Lever_MID:
									 if(Ctrl_Source != CTRL_RC)
									 {
										 CtrlSource_Switch(CTRL_RC);
										 Set_AttackMode(Attack_Disable);//关闭摩擦轮
									 }
									 Set_ChassisMode(CHAS_followMode);//设置为跟随模式
									 Set_GimbalMode(Gimbal_PCMode);// 设置为PC控制(自瞄)
									 Set_VisionMode(Vision_Forcast);//开启预测
									 
									 if(Get_DW() <= -630 && Fric_flag_test == 0) //--- 拨轮打上1s开关摩擦轮
									 {
										  Attack_Mode == Attack_Disable? Fric_flag++ : Fric_flag--;
										  if(Fric_flag > 60 || Fric_flag == 0 )
											{
												Fric_flag_test = 1;
											} 
									 }
									else if(Get_DW() >= -130 && Fric_flag_test == 1)
									{		
										 Fric_flag_test = 0;
									}	

									 if(Fric_flag > 60)
									 {
										 //获取裁判系统设置的最大射速
										  switch(ShootGet_SpeedLimit())
										  {
													case 15: Set_AttackMode(Attack_15);
															break;
													case 18: Set_AttackMode(Attack_18);
															break;
													case 22: 
													case 25: Set_AttackMode(Attack_22); /*ICRA是25 陪练用*/
															break;
													case 30:
													case 75: Set_AttackMode(Attack_30);
															break;
													default: Set_AttackMode(Attack_15); 
															break;												
											}
									 }
								 else if(Fric_flag == 0)
									{
										 Set_AttackMode(Attack_Disable);
									}	
									
									 //发射
										if(Get_DW() > 0 && Get_DW() < 330 && test_shoot == 0 && Shoot_Freq > 0) //--- 单发射
										{
                       test_shoot = 1;
                       ContLaunch = false;
                       Set_ReloadNum(1);											
										}
										else if (Get_DW() == 0) 
										{									 
												test_shoot = 0;
										}
											
										if(Get_DW() >= 630)//连发模式
										{
											  ContLaunch = true;
										}					
										else
										{
												ContLaunch = false;
										}
										
								
							break;  /* 左上-右中 END ------------------------------------------*/
										
							case Lever_DOWN:
									 if(Ctrl_Source != CTRL_RC)
									 {
										 CtrlSource_Switch(CTRL_RC);
										 Set_AttackMode(Attack_Disable);//关闭摩擦轮
									 }
									 
									 Set_ChassisMode(CHAS_LockMode); // 锁住底盘
									 Set_GimbalMode(Gimbal_PCMode);// PC控制(自瞄)
									 
									 //--- 普通自瞄或预测自瞄
										if(Get_CH3() <= -650)  // LY打下小陀螺
										{
												Set_VisionMode(Vision_Top);
										}
										else
										{
											  Set_VisionMode(Vision_BIGWindmill);
										}
							
									 if(Get_DW() <= -630 && Fric_flag_test1 == 0) //--- 拨轮打上1s开关摩擦轮
									 {
											 Attack_Mode == Attack_Disable? Fric_flag++ : Fric_flag--;
										  if(Fric_flag > 60 || Fric_flag == 0 )
											{
												Fric_flag_test1 = 1;
											} 										  
									 }
									else if(Get_DW() >= -130 && Fric_flag_test1 == 1)
									{		
										 Fric_flag_test1 = 0;								
									}		
									
											 if(Fric_flag > 60)
											 {
												 //获取裁判系统设置的最大射速
													switch(ShootGet_SpeedLimit())
													{
															case 15: Set_AttackMode(Attack_15);
																	break;
															case 18: Set_AttackMode(Attack_18);
																	break;
															case 22: 
															case 25:Set_AttackMode(Attack_22); /*ICRA是25 陪练用*/
																	break;
															case 30:
															case 75: Set_AttackMode(Attack_30);
																	break;
															default: Set_AttackMode(Attack_15); 
																	break;												
													}
											 }
											 else if(Fric_flag == 0)
												{
													 Set_AttackMode(Attack_Disable);
												}					
										
											//发射
									if(Get_CH2() < 0 && Get_CH2() > -110 && test_shoot == 0 && Shoot_Freq > 0) //--- 单发射
									{						
										  test_shoot = 1;
									    ContLaunch = false;
										  Set_ReloadNum(1);
									}
									else if (Get_CH2() == 0) 
									{
											test_shoot = 0;
									}
									
									if(Get_CH2() <= -630)//连发模式
									{
										  ContLaunch = true;
									}
									else 
									{
										  ContLaunch = false;
									}
								
								if(Get_DW() >= 630) //--- 自动开火
								{
										Set_AttackMode(Attack_Auto);
								}
								else
								{
										if(Attack_Mode== Attack_Auto)
										{
												Set_AttackMode(Attack_Disable);
										}
								}		
                   if(Get_CH3() == 660)
									 {
										 minimum_HP();		//给视觉发送最低血量的敌方机器人							
									 }
									 else
									 {
										 minimum_HP_clean();
									 }
                   
									 break; /* 左上-右下 END ------------------------------------------*/
	        }
					 break;  // 左上 END ---------------------------------------------------
					
			     case Lever_MID:  // --- 左中 ----------------------------------------------		
								 CtrlSource_Switch(CTRL_RC);//更改控制源
								 Set_GimbalMode(Gimbal_NormalMode);// 云台普通运转模式
					 
									switch(Get_S2_R())
									{
										case Lever_UP:
										Set_ChassisMode(CHAS_followMode);	//设置为跟随模式
										Set_GimbalMode(Gimbal_NormalMode);// 云台普通运转模式
										
										 //获取裁判系统设置的最大射速
											switch(ShootGet_SpeedLimit())
											{
													case 15: Set_AttackMode(Attack_15);
															break;
													case 18: Set_AttackMode(Attack_18);
															break;
													case 22: 
													case 25:Set_AttackMode(Attack_22); /*ICRA是25 陪练用*/
															break;
													case 30:
													case 75: Set_AttackMode(Attack_30);
															break;
													default: Set_AttackMode(Attack_15); 
															break;												
											}
										
										//发射...........
										if(Get_DW() > 0 && Get_DW() < 330 && test_shoot == 0 && Shoot_Freq > 0) //--- 单发射
										{
												 test_shoot = 1;
									       ContLaunch = false;
										     Set_ReloadNum(1);
												
										}
										else if (Get_DW() == 0) 
										{
												test_shoot = 0;
										}
											
										if(Get_DW() >= 630)//连发模式
									  {
										  ContLaunch = true;
									  }
									 else 
									 {
										  ContLaunch = false;
									  }
										break;// --- 左中 右上 END----------------------------------------------		
										
										case Lever_MID:
											   Set_GimbalMode(Gimbal_NormalMode);// 云台普通运转模式
										     Set_AttackMode(Attack_Disable);//关闭摩擦轮
					
										if(Get_DW() <= -50) //--- 拨上小陀螺
										{
												Set_ChassisMode(CHAS_SpinMode);//开启小陀螺
												if(Get_DW() <= -650)
												{
                              //开启超电
													    ChassisCap_Ctrl(ON);
												}
												else
												{
                             //关闭超电
													    ChassisCap_Ctrl(OFF);
												}
										}			
										else
										{
                        //拨盘归零时进行云台回中
											  Set_ChassisMode(CHAS_followMode);// 云台普通运转模式(要进行特殊处理)
										}		
										
										if(Chassis_Mode == CHAS_followMode)
										{
												if(Get_DW() >= 650) //--- 拨下电容放电
												{
													  ChassisCap_Ctrl(ON);
												}
												else
												{
													  ChassisCap_Ctrl(OFF);
												}
										}	
										 Diagonal_Mode = false;//解除对角模式
                     break;		// --- 左中 右中 END----------------------------------------------				
										
										case Lever_DOWN:
											   Set_ChassisMode(CHAS_LockMode);// 锁住底盘
										     Set_GimbalMode(Gimbal_NormalMode);// 云台普通运转模式
										
													if(Get_DW() < -100) //--- 拨上退弹
													{
															Set_AttackMode(Attack_Unload);//退弹模式
															if(Get_DW() <= -600)
															{
                                    ContLaunch = true;/*<! 连发标志位 */
															}
															else
															{
																	  ContLaunch = false;
															}
													}
													else
													{
															Set_AttackMode(Attack_Disable);//关闭摩擦轮
													}	

                          // --- 拨下开关弹仓 	...........				
                          if(Get_DW() > 600 && Mag_flag == 1) 			
													{			
														  Mag_flag = 0;		
                              if(Mag_Switch == ON)
															{
																 Mag_Ctrl(OFF);
															}			
                              else if(Mag_Switch == OFF)
															{
															   Mag_Ctrl(ON);		
															}																  
													}						
                          else if(Get_DW() == 0)
													{	
                              Mag_flag = 1;														
													}														
										break;
									}							
			break;		// 左中 END ---------------------------------------------------
									
			case Lever_DOWN: // --- 左下 ----------------------------------------------
				   CtrlSource_Switch(CTRL_OFF);//关闭机器
			     //机器人失能
           Robot_Disable();
			     Fric_flag = 0;//摩擦轮拨盘计数清零
			     Laser_Ctrl(0);//关闭红外
			        // Robot Reset
					if(Get_DW() == -660 && Robot_Reset_flag == true)
					{
		            Reset_cnt++;
		           
		            if(Reset_cnt == 100)//--- 拨轮打上2s重启
		            {
		                Robot_Reset();
								    ResetFlag = true;//发送给底盘标志位
									  Robot_Reset_flag = false;
		            }
							
					}
					else if(Get_DW() == 0)
					{
		            Reset_cnt = 0;
						    Robot_Reset_flag = true;
						    ResetFlag = false;//发送给底盘标志位
					}


		      if(Get_DW() >= 600)//拨盘重启底盘
					{
					   ChassisResetFlag = true;
					}
					else if(Get_DW() == 0)
					{
						ChassisResetFlag = false;
					}
					
					Diagonal_Mode = false;//解除对角模式
					break;  // 左下 END ---------------------------------------------------
          default:
          break;
    }
	 
		if(Get_S1_L() != Lever_UP)//不是左上
		{
			 Fric_flag = 0;//摩擦轮拨盘计数清零
		}
		else if(Get_S1_L() == Lever_UP && Get_S2_R() == Lever_UP)//是左上但不是有中和右下
		{
			Fric_flag = 0;//摩擦轮拨盘计数清零
		}
		
}
 
void Robot_Reset(void)
{   
    //--- 芯片复位
    __set_FAULTMASK(1);    //关闭所有中断
    HAL_NVIC_SystemReset();//复位
}


 /**
 * @brief      控制源更新
 * @param[in]  None
 * @retval     None
 */
 void DR16_CTRLSource_Update(void)
 {
	 switch(Ctrl_Source)
	 {
		 case CTRL_RC:  //--- 遥控模式
		      DR16RCCtrl_Update();
     break;
		 
		 case CTRL_PC:   //--- 键鼠模式
		      DR16PCCtrl_Update();
		 break;

     case CTRL_OFF:  //---全局失能		
		      Robot_Disable();
		 break;
	 }
 }
 
 /**
 * @brief  	  机器人控制源转换
 * @param[in]	mode
 * @retval 	  None
 */
 void CtrlSource_Switch(DR16Status_Typedef mode)
 {
	     //--- 发生模式跳变
    if (Ctrl_Source != mode)
		{
			  // M6020 Yaw Reset
			///待实现
		}
		Ctrl_Source = mode;
 }
 
 /**
 * @brief      RC控制模式 - 对外设置目标值
 * @param[in]  None
 * @retval     None
 */
 void DR16RCCtrl_Update(void)
 {
	 switch(Chassis_Mode)/////--- 
	 {
		 case CHAS_followMode:
		 case CHAS_ReFollowMode:

			    Expt.Target_Yaw = Get_CH0() *0.002;  //--- Yaw	
		      Expt.Target_Pit = Get_CH1() *0.3/660; //--- Pit
		 
          Expt.Target_Vx = Get_CH2();
		      Expt.Target_Vy = Get_CH3();
		      Expt.Target_Vw = angle_diff;
			    break;
		 
		 case CHAS_SpinMode:
			 
			    Expt.Target_Yaw = Get_CH0() *0.002;  //--- Yaw	
		      Expt.Target_Pit = Get_CH1() *0.3/660; //--- Pit
		 
          Expt.Target_Vx = Get_CH2();
		      Expt.Target_Vy = Get_CH3();
//          Omnidirectional_Formula(&Expt.Target_Vx,&Expt.Target_Vy);//全向公式	  	
          Expt.Target_Vw = angle_diff;
		 
			    break;
		 
		 case CHAS_LockMode:
     case CHAS_DisableMode:
			 
			    Expt.Target_Yaw = Get_CH0() * 0.002;  //--- Yaw
		      Expt.Target_Pit = Get_CH1() *0.3/660; //--- Pit			 
		 
		      Expt.Target_Vx = Expt.Target_Vy = Expt.Target_Vw = 0;
		 
			    break;
		 
		 case CHAS_NotfollowMode:
			 
			    Expt.Target_Yaw = 0;
		      Expt.Target_Pit = Get_CH1() *0.3/660;
		 
          Expt.Target_Vx = Get_CH2();
		      Expt.Target_Vy = Get_CH3();			 
		      Expt.Target_Vw = Get_CH0();
			    break;
	 }

		if(Get_CH3() == -660)//防止满速向后走
		{
			Expt.Target_Vy = 0;
		}
		 if(DevicesGet_State(DR16_MONITOR) == Off_line)
		 {
			 Expt.Target_Yaw = 0;
		   Expt.Target_Pit = 0;
			 Expt.Target_Vx = 0;
			 Expt.Target_Vy = 0;
			 Expt.Target_Vw = 0;
		 }

 }
 /**
 * @brief      PC控制模式
 * @param[in]  None
 * @retval     None
 */
float Chassis_VxVy_Limit;//底盘最大速度限制
float slop_temp = 5.0f;
uint8_t spin_flag;
uint8_t  Notfollow_flag;
uint8_t fric = 0;
uint8_t mag_flag = 1;
uint8_t test_flag = 0; 
uint8_t Vision_Top_flag = 1;
uint8_t Vision_Top_flag1 = 1;
 uint8_t Vision_Top_outpost = 1;
void DR16PCCtrl_Update(void)
{
	  //鼠标值作为云台控制输入值
		Expt.Target_Yaw = (float)Get_MouseX() * 0.05f;
	  Expt.Target_Pit = -(float)Get_MouseY() * 0.01f;
		  
	  Expt.Target_Vw = angle_diff;

	  //进行低通滤波（原）
//    Expt.Target_Yaw = 
//	  Expt.Target_Pit	   
	
    if(Chassis_Mode == CHAS_SpinMode)
		{
			if(GetKeyMouseAction(KEY_W,KeyAction_PRESS))//判断键位是否按下
			{
					Expt.Target_Vy = (Chassis_Mode == CHAS_ReFollowMode?-MoveValue:MoveValue);
					Expt.Target_Vy *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//按CTRL键减速
			}
			else if(GetKeyMouseAction(KEY_S,KeyAction_PRESS))//判断键位是否按下
			{
					Expt.Target_Vy = (Chassis_Mode == CHAS_ReFollowMode?-MoveValue:MoveValue);
					Expt.Target_Vy *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//按CTRL键减速
					Expt.Target_Vy *= -1; 
			}
			else
			{
					Expt.Target_Vy = 0;
			}
			
			if(GetKeyMouseAction(KEY_A,KeyAction_PRESS))//判断键位是否按下
			{
					Expt.Target_Vx = (Chassis_Mode == CHAS_ReFollowMode?-MoveValue:MoveValue);
				
					Expt.Target_Vx *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//按CTRL键减速
					Expt.Target_Vx *= -1; 
			}
			else if(GetKeyMouseAction(KEY_D,KeyAction_PRESS))//判断键位是否按下
			{
					Expt.Target_Vx = (Chassis_Mode == CHAS_ReFollowMode?-MoveValue:MoveValue);
					Expt.Target_Vx *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//按CTRL键减速
			}
			else
			{
					Expt.Target_Vx = 0;
			}
	  }
    else
		{
			if(ReF_Flag == true)
			{
				if(GetKeyMouseAction(KEY_W,KeyAction_PRESS))//判断键位是否按下
				{
						Expt.Target_Vy = (Chassis_Mode == CHAS_ReFollowMode?-FollowMoveValue:FollowMoveValue);
						Expt.Target_Vy *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//按CTRL键减速
				}
				else if(GetKeyMouseAction(KEY_S,KeyAction_PRESS))//判断键位是否按下
				{
						Expt.Target_Vy = (Chassis_Mode == CHAS_ReFollowMode?-FollowMoveValue:FollowMoveValue);
						Expt.Target_Vy *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//按CTRL键减速
						Expt.Target_Vy *= -1; 
				}
				else
				{
						Expt.Target_Vy = 0;
				}
				
				if(GetKeyMouseAction(KEY_A,KeyAction_PRESS))//判断键位是否按下
				{
						Expt.Target_Vx = (Chassis_Mode == CHAS_ReFollowMode?-FollowMoveValue:FollowMoveValue);
					
						Expt.Target_Vx *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//按CTRL键减速
						Expt.Target_Vx *= -1; 
				}
				else if(GetKeyMouseAction(KEY_D,KeyAction_PRESS))//判断键位是否按下
				{
						Expt.Target_Vx = (Chassis_Mode == CHAS_ReFollowMode?-FollowMoveValue:FollowMoveValue);
						Expt.Target_Vx *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//按CTRL键减速
				}
				else
				{
						Expt.Target_Vx = 0;
				}
		  }
			else
			{
					if(GetKeyMouseAction(KEY_W,KeyAction_PRESS))//判断键位是否按下
					{
							Chassis_VxVy_Limit = (Chassis_Mode == CHAS_ReFollowMode?-660:660);
							Chassis_VxVy_Limit *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//按CTRL键减速
							Chassis_Drv_Slow(&Expt.Target_Vy, Chassis_VxVy_Limit, slop_temp);//底盘速度斜坡
					}
					else if(GetKeyMouseAction(KEY_S,KeyAction_PRESS))//判断键位是否按下
					{
							Chassis_VxVy_Limit = (Chassis_Mode == CHAS_ReFollowMode?-660:660);
							Chassis_VxVy_Limit *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//按CTRL键减速
							Chassis_Drv_Slow(&Expt.Target_Vy, -Chassis_VxVy_Limit, slop_temp);//底盘速度斜坡
					}
					else
					{
							Expt.Target_Vy = 0;
					}
					
					if(GetKeyMouseAction(KEY_A,KeyAction_PRESS))//判断键位是否按下
					{
							Chassis_VxVy_Limit = (Chassis_Mode == CHAS_ReFollowMode?-660:660);
						
							Chassis_VxVy_Limit *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//按CTRL键减速

							Chassis_Drv_Slow(&Expt.Target_Vx, -Chassis_VxVy_Limit, slop_temp);//底盘速度斜坡
					}
					else if(GetKeyMouseAction(KEY_D,KeyAction_PRESS))//判断键位是否按下
					{
							Chassis_VxVy_Limit = (Chassis_Mode == CHAS_ReFollowMode?-660:660);
							Chassis_VxVy_Limit *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//按CTRL键减速
							Chassis_Drv_Slow(&Expt.Target_Vx, Chassis_VxVy_Limit, slop_temp);//底盘速度斜坡
					}
					else
					{
							Expt.Target_Vx = 0;
					}		
  		}
  	}
		

    /*--- CTRL+组合键 -----------------------------------------------------------------------------------------------------------*/
    
    //---------------------------- CTRL 组合键 END ------------------------------------------------------		
		if(GetKeyMouseAction(KEY_Z,KeyAction_PRESS)) /*<! 长按Z 打符模式 */
		{
		   Set_VisionMode(Vision_BIGWindmill);
			 Set_ChassisMode(CHAS_LockMode); // 锁住底盘
		}
		else                               
		{
	     Set_VisionMode(Vision_Forcast);  //预测
			 Set_ChassisMode(CHAS_followMode); // 跟随
		}
		
		/*<! 自动射击 + 击打小陀螺模式 */  //---新加 长按V
    if(GetKeyMouseAction(KEY_V,KeyAction_PRESS))
		{
			 Vision_Top_flag = 0;
			 Set_VisionMode(Vision_Top); /*<! 击打小陀螺模式 */
			 Set_AttackMode(Attack_Auto);/*-- 自动击打--*/
		}
		else if(remote_control.keyBoard.key_code != 16384 && Vision_Top_flag == 0)
		{
       Vision_Top_flag = 1;
			 Set_VisionMode(Vision_Forcast);//预测
			 if(Attack_Mode == Attack_Auto)
			 {
				 Set_AttackMode(Attack_15);
			 }			
		}

			/*<! 自动射击 + 击打前哨站模式 */  //---新加 长按X
    if(GetKeyMouseAction(KEY_G,KeyAction_PRESS))
		{
			 Vision_Top_outpost = 0;
			 Set_VisionMode(Vision_Outpost); /*<! 击打前哨站模式 */
			 Set_AttackMode(Attack_Auto);/*-- 自动击打--*/
		}
		else if(remote_control.keyBoard.key_code != 1024 && Vision_Top_outpost == 0)
		{
       Vision_Top_outpost = 1;
			 Set_VisionMode(Vision_Forcast);//预测
			 if(Attack_Mode == Attack_Auto)
			 {
				 Set_AttackMode(Attack_15);
			 }			
		}	
		
		//4.18改
		if(GetKeyMouseAction(KEY_SHIFT,KeyAction_PRESS))  //--- 按SHIFT 小陀螺
		{
			  Set_ChassisMode(CHAS_SpinMode);
			  spin_flag = true;
		}
		else
		{
        if(spin_flag == true)
        {
            Set_ChassisMode(CHAS_followMode);
            spin_flag = false;
        }			
		}


		
//		if(GetKeyMouseAction(KEY_Z,KeyAction_LONG_PRESS) &&GetKeyMouseAction(KEY_X,KeyAction_LONG_PRESS) && GetKeyMouseAction(KEY_C,KeyAction_LONG_PRESS))  //--- 长按 Z+X+C 无跟随模式
//		{
//			  Set_ChassisMode(CHAS_NotfollowMode);///无跟随
//			  Notfollow_flag = true;
//		}
//		else
//		{
//         if(Notfollow_flag == true)
//        {
//            Set_ChassisMode(CHAS_followMode);
//            Notfollow_flag = false;
//        }		
//		}		

		if(GetKeyMouseAction(KEY_Q,KeyAction_LONG_PRESS))  /*<! Q 暴走模式，无视枪管热量 */
    {
        Rage_Mode = 1;
    }
    else
    {
        Rage_Mode = 0;
    }		
		
		
    if(GetKeyMouseAction(KEY_C,KeyAction_LONG_PRESS))  /*<! 长按C 电容开启 */
    {
        ChassisCap_Ctrl(ON);
    }
    else
    {
        ChassisCap_Ctrl(OFF); 
    }		
		
		//上坡模式 	
	  //..........
		

		
    if(GetKeyMouseAction(MOUSE_Left,KeyAction_CLICK) && Reload.Began == false && Shoot_Freq > 0)  /*<! 发射弹丸 */  //新改
    {
        ContLaunch = false;
        Set_ReloadNum(1);
    }
    else if(GetKeyMouseAction(MOUSE_Left,KeyAction_LONG_PRESS) && Shoot_Freq > 0)
    {
        ContLaunch = true;
    }
    else
    {
        ContLaunch = false;
    }	


	 if(GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))/*<! 按ctrl + R 弹仓关 */
		{
			if(GetKeyMouseAction(KEY_R,KeyAction_PRESS))
			{
				 Mag_Ctrl(OFF);
			}
		}
		else if(GetKeyMouseAction(KEY_R,KeyAction_PRESS)) /*<! 长R 弹仓开关 */
		{
			  Mag_Ctrl(ON);
		}
		
		
	 if(GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))/*<! 按 ctrl + E 关弹摩擦轮 */
		{
			if(GetKeyMouseAction(KEY_E,KeyAction_PRESS))
			{
				Set_AttackMode(Attack_Disable);
			}
		}
		else if(GetKeyMouseAction(KEY_E,KeyAction_PRESS)) /*<! 按E 弹摩擦轮开 */
		{
			  Set_AttackMode(Attack_15);
		}
		
		
		if(GetKeyMouseAction(MOUSE_Right,KeyAction_PRESS))  /*<! 按 鼠标右键 自瞄开启 */
		{
				Set_GimbalMode(Gimbal_PCMode);
		}
		else
		{
				Set_GimbalMode(Gimbal_NormalMode);
		}		

			
		
    /*<! UI 刷新 */		
    if(GetKeyMouseAction(KEY_B,KeyAction_PRESS)) /*<! UI 刷新 */
    {
        Reflash_UI = true;
    }
    else
    {
        Reflash_UI = false;
    }


//		if(GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))/*<! 按 ctrl + F 取消45度对角模式 */
//		{
//			if(GetKeyMouseAction(KEY_F,KeyAction_PRESS))
//			{
//         Diagonal_Mode = false;
//			}
//		}
//		else if(GetKeyMouseAction(KEY_F,KeyAction_PRESS)) /*<! 按F对角模式 */
//		{
//         Diagonal_Mode = true;
//		}
		
//	  /*<! 打符Pit手动调整 */
//    if()
//		{
//			
//		}
		
}


/**
 * @brief      获取对外输出数据
 * @param[in]  None
 * @retval     export data
 */
float DR16_Get_ExptYaw(void)
{
    return Expt.Target_Yaw;
}
float DR16_Get_ExptPit(void)
{
    return Expt.Target_Pit;
}
float DR16_Get_ExptVx(void)
{
    return Expt.Target_Vx;
}
float DR16_Get_ExptVy(void)
{
    return Expt.Target_Vy;
}
float DR16_Get_ExptVw(void)
{
    return Expt.Target_Vw;
}

