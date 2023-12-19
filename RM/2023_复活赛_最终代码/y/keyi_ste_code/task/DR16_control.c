/**
 ------------------------------------------------------------------------------
 * @file    DR16_control.c
 * @author  keyi
 * @brief   DR16 �û�����
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

//--- ����ģʽ�µ�С�����ƶ�VX VY���ֵ
#define MoveValue 150
#define FollowMoveValue 660
/**
 * @brief      ����ģʽ����
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
uint8_t ResetFlag = false;   /*<! ������־λ */
uint8_t Reflash_UI;  /*<! UIˢ�� */
uint8_t Diagonal_Mode = false;//�Խ�ģʽ��־λ 

uint8_t ChassisResetFlag = false;   /*<! ����������־λ */
void DR16_LeverMode_Update(void)
 {
	 switch(Get_S1_L())
	 {
		  case Lever_UP: // --- ���� -----------------------------------------------
				   switch(Get_S2_R())
					 {
							case Lever_UP: // --- PC�˿���
									if (Ctrl_Source != CTRL_PC)
									{
										Set_GimbalMode(Gimbal_NormalMode);//��ͨģʽ
										Set_ChassisMode(CHAS_followMode);//����ģʽ
										Set_AttackMode(Attack_Disable);//����Ħ����
										Set_VisionMode(Vision_Forcast);//����Ԥ��
                    Mag_Ctrl(OFF);//�رյ��� ......
									}
							     CtrlSource_Switch(CTRL_PC);//����Դ��ΪPC
						  break;/* ����-���� END ------------------------------------------*/
					 
							case Lever_MID:
									 if(Ctrl_Source != CTRL_RC)
									 {
										 CtrlSource_Switch(CTRL_RC);
										 Set_AttackMode(Attack_Disable);//�ر�Ħ����
									 }
									 Set_ChassisMode(CHAS_followMode);//����Ϊ����ģʽ
									 Set_GimbalMode(Gimbal_PCMode);// ����ΪPC����(����)
									 Set_VisionMode(Vision_Forcast);//����Ԥ��
									 
									 if(Get_DW() <= -630 && Fric_flag_test == 0) //--- ���ִ���1s����Ħ����
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
										 //��ȡ����ϵͳ���õ��������
										  switch(ShootGet_SpeedLimit())
										  {
													case 15: Set_AttackMode(Attack_15);
															break;
													case 18: Set_AttackMode(Attack_18);
															break;
													case 22: 
													case 25: Set_AttackMode(Attack_22); /*ICRA��25 ������*/
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
									
									 //����
										if(Get_DW() > 0 && Get_DW() < 330 && test_shoot == 0 && Shoot_Freq > 0) //--- ������
										{
                       test_shoot = 1;
                       ContLaunch = false;
                       Set_ReloadNum(1);											
										}
										else if (Get_DW() == 0) 
										{									 
												test_shoot = 0;
										}
											
										if(Get_DW() >= 630)//����ģʽ
										{
											  ContLaunch = true;
										}					
										else
										{
												ContLaunch = false;
										}
										
								
							break;  /* ����-���� END ------------------------------------------*/
										
							case Lever_DOWN:
									 if(Ctrl_Source != CTRL_RC)
									 {
										 CtrlSource_Switch(CTRL_RC);
										 Set_AttackMode(Attack_Disable);//�ر�Ħ����
									 }
									 
									 Set_ChassisMode(CHAS_LockMode); // ��ס����
									 Set_GimbalMode(Gimbal_PCMode);// PC����(����)
									 
									 //--- ��ͨ�����Ԥ������
										if(Get_CH3() <= -650)  // LY����С����
										{
												Set_VisionMode(Vision_Top);
										}
										else
										{
											  Set_VisionMode(Vision_BIGWindmill);
										}
							
									 if(Get_DW() <= -630 && Fric_flag_test1 == 0) //--- ���ִ���1s����Ħ����
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
												 //��ȡ����ϵͳ���õ��������
													switch(ShootGet_SpeedLimit())
													{
															case 15: Set_AttackMode(Attack_15);
																	break;
															case 18: Set_AttackMode(Attack_18);
																	break;
															case 22: 
															case 25:Set_AttackMode(Attack_22); /*ICRA��25 ������*/
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
										
											//����
									if(Get_CH2() < 0 && Get_CH2() > -110 && test_shoot == 0 && Shoot_Freq > 0) //--- ������
									{						
										  test_shoot = 1;
									    ContLaunch = false;
										  Set_ReloadNum(1);
									}
									else if (Get_CH2() == 0) 
									{
											test_shoot = 0;
									}
									
									if(Get_CH2() <= -630)//����ģʽ
									{
										  ContLaunch = true;
									}
									else 
									{
										  ContLaunch = false;
									}
								
								if(Get_DW() >= 630) //--- �Զ�����
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
										 minimum_HP();		//���Ӿ��������Ѫ���ĵз�������							
									 }
									 else
									 {
										 minimum_HP_clean();
									 }
                   
									 break; /* ����-���� END ------------------------------------------*/
	        }
					 break;  // ���� END ---------------------------------------------------
					
			     case Lever_MID:  // --- ���� ----------------------------------------------		
								 CtrlSource_Switch(CTRL_RC);//���Ŀ���Դ
								 Set_GimbalMode(Gimbal_NormalMode);// ��̨��ͨ��תģʽ
					 
									switch(Get_S2_R())
									{
										case Lever_UP:
										Set_ChassisMode(CHAS_followMode);	//����Ϊ����ģʽ
										Set_GimbalMode(Gimbal_NormalMode);// ��̨��ͨ��תģʽ
										
										 //��ȡ����ϵͳ���õ��������
											switch(ShootGet_SpeedLimit())
											{
													case 15: Set_AttackMode(Attack_15);
															break;
													case 18: Set_AttackMode(Attack_18);
															break;
													case 22: 
													case 25:Set_AttackMode(Attack_22); /*ICRA��25 ������*/
															break;
													case 30:
													case 75: Set_AttackMode(Attack_30);
															break;
													default: Set_AttackMode(Attack_15); 
															break;												
											}
										
										//����...........
										if(Get_DW() > 0 && Get_DW() < 330 && test_shoot == 0 && Shoot_Freq > 0) //--- ������
										{
												 test_shoot = 1;
									       ContLaunch = false;
										     Set_ReloadNum(1);
												
										}
										else if (Get_DW() == 0) 
										{
												test_shoot = 0;
										}
											
										if(Get_DW() >= 630)//����ģʽ
									  {
										  ContLaunch = true;
									  }
									 else 
									 {
										  ContLaunch = false;
									  }
										break;// --- ���� ���� END----------------------------------------------		
										
										case Lever_MID:
											   Set_GimbalMode(Gimbal_NormalMode);// ��̨��ͨ��תģʽ
										     Set_AttackMode(Attack_Disable);//�ر�Ħ����
					
										if(Get_DW() <= -50) //--- ����С����
										{
												Set_ChassisMode(CHAS_SpinMode);//����С����
												if(Get_DW() <= -650)
												{
                              //��������
													    ChassisCap_Ctrl(ON);
												}
												else
												{
                             //�رճ���
													    ChassisCap_Ctrl(OFF);
												}
										}			
										else
										{
                        //���̹���ʱ������̨����
											  Set_ChassisMode(CHAS_followMode);// ��̨��ͨ��תģʽ(Ҫ�������⴦��)
										}		
										
										if(Chassis_Mode == CHAS_followMode)
										{
												if(Get_DW() >= 650) //--- ���µ��ݷŵ�
												{
													  ChassisCap_Ctrl(ON);
												}
												else
												{
													  ChassisCap_Ctrl(OFF);
												}
										}	
										 Diagonal_Mode = false;//����Խ�ģʽ
                     break;		// --- ���� ���� END----------------------------------------------				
										
										case Lever_DOWN:
											   Set_ChassisMode(CHAS_LockMode);// ��ס����
										     Set_GimbalMode(Gimbal_NormalMode);// ��̨��ͨ��תģʽ
										
													if(Get_DW() < -100) //--- �����˵�
													{
															Set_AttackMode(Attack_Unload);//�˵�ģʽ
															if(Get_DW() <= -600)
															{
                                    ContLaunch = true;/*<! ������־λ */
															}
															else
															{
																	  ContLaunch = false;
															}
													}
													else
													{
															Set_AttackMode(Attack_Disable);//�ر�Ħ����
													}	

                          // --- ���¿��ص��� 	...........				
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
			break;		// ���� END ---------------------------------------------------
									
			case Lever_DOWN: // --- ���� ----------------------------------------------
				   CtrlSource_Switch(CTRL_OFF);//�رջ���
			     //������ʧ��
           Robot_Disable();
			     Fric_flag = 0;//Ħ���ֲ��̼�������
			     Laser_Ctrl(0);//�رպ���
			        // Robot Reset
					if(Get_DW() == -660 && Robot_Reset_flag == true)
					{
		            Reset_cnt++;
		           
		            if(Reset_cnt == 100)//--- ���ִ���2s����
		            {
		                Robot_Reset();
								    ResetFlag = true;//���͸����̱�־λ
									  Robot_Reset_flag = false;
		            }
							
					}
					else if(Get_DW() == 0)
					{
		            Reset_cnt = 0;
						    Robot_Reset_flag = true;
						    ResetFlag = false;//���͸����̱�־λ
					}


		      if(Get_DW() >= 600)//������������
					{
					   ChassisResetFlag = true;
					}
					else if(Get_DW() == 0)
					{
						ChassisResetFlag = false;
					}
					
					Diagonal_Mode = false;//����Խ�ģʽ
					break;  // ���� END ---------------------------------------------------
          default:
          break;
    }
	 
		if(Get_S1_L() != Lever_UP)//��������
		{
			 Fric_flag = 0;//Ħ���ֲ��̼�������
		}
		else if(Get_S1_L() == Lever_UP && Get_S2_R() == Lever_UP)//�����ϵ��������к�����
		{
			Fric_flag = 0;//Ħ���ֲ��̼�������
		}
		
}
 
void Robot_Reset(void)
{   
    //--- оƬ��λ
    __set_FAULTMASK(1);    //�ر������ж�
    HAL_NVIC_SystemReset();//��λ
}


 /**
 * @brief      ����Դ����
 * @param[in]  None
 * @retval     None
 */
 void DR16_CTRLSource_Update(void)
 {
	 switch(Ctrl_Source)
	 {
		 case CTRL_RC:  //--- ң��ģʽ
		      DR16RCCtrl_Update();
     break;
		 
		 case CTRL_PC:   //--- ����ģʽ
		      DR16PCCtrl_Update();
		 break;

     case CTRL_OFF:  //---ȫ��ʧ��		
		      Robot_Disable();
		 break;
	 }
 }
 
 /**
 * @brief  	  �����˿���Դת��
 * @param[in]	mode
 * @retval 	  None
 */
 void CtrlSource_Switch(DR16Status_Typedef mode)
 {
	     //--- ����ģʽ����
    if (Ctrl_Source != mode)
		{
			  // M6020 Yaw Reset
			///��ʵ��
		}
		Ctrl_Source = mode;
 }
 
 /**
 * @brief      RC����ģʽ - ��������Ŀ��ֵ
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
//          Omnidirectional_Formula(&Expt.Target_Vx,&Expt.Target_Vy);//ȫ��ʽ	  	
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

		if(Get_CH3() == -660)//��ֹ���������
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
 * @brief      PC����ģʽ
 * @param[in]  None
 * @retval     None
 */
float Chassis_VxVy_Limit;//��������ٶ�����
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
	  //���ֵ��Ϊ��̨��������ֵ
		Expt.Target_Yaw = (float)Get_MouseX() * 0.05f;
	  Expt.Target_Pit = -(float)Get_MouseY() * 0.01f;
		  
	  Expt.Target_Vw = angle_diff;

	  //���е�ͨ�˲���ԭ��
//    Expt.Target_Yaw = 
//	  Expt.Target_Pit	   
	
    if(Chassis_Mode == CHAS_SpinMode)
		{
			if(GetKeyMouseAction(KEY_W,KeyAction_PRESS))//�жϼ�λ�Ƿ���
			{
					Expt.Target_Vy = (Chassis_Mode == CHAS_ReFollowMode?-MoveValue:MoveValue);
					Expt.Target_Vy *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//��CTRL������
			}
			else if(GetKeyMouseAction(KEY_S,KeyAction_PRESS))//�жϼ�λ�Ƿ���
			{
					Expt.Target_Vy = (Chassis_Mode == CHAS_ReFollowMode?-MoveValue:MoveValue);
					Expt.Target_Vy *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//��CTRL������
					Expt.Target_Vy *= -1; 
			}
			else
			{
					Expt.Target_Vy = 0;
			}
			
			if(GetKeyMouseAction(KEY_A,KeyAction_PRESS))//�жϼ�λ�Ƿ���
			{
					Expt.Target_Vx = (Chassis_Mode == CHAS_ReFollowMode?-MoveValue:MoveValue);
				
					Expt.Target_Vx *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//��CTRL������
					Expt.Target_Vx *= -1; 
			}
			else if(GetKeyMouseAction(KEY_D,KeyAction_PRESS))//�жϼ�λ�Ƿ���
			{
					Expt.Target_Vx = (Chassis_Mode == CHAS_ReFollowMode?-MoveValue:MoveValue);
					Expt.Target_Vx *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//��CTRL������
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
				if(GetKeyMouseAction(KEY_W,KeyAction_PRESS))//�жϼ�λ�Ƿ���
				{
						Expt.Target_Vy = (Chassis_Mode == CHAS_ReFollowMode?-FollowMoveValue:FollowMoveValue);
						Expt.Target_Vy *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//��CTRL������
				}
				else if(GetKeyMouseAction(KEY_S,KeyAction_PRESS))//�жϼ�λ�Ƿ���
				{
						Expt.Target_Vy = (Chassis_Mode == CHAS_ReFollowMode?-FollowMoveValue:FollowMoveValue);
						Expt.Target_Vy *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//��CTRL������
						Expt.Target_Vy *= -1; 
				}
				else
				{
						Expt.Target_Vy = 0;
				}
				
				if(GetKeyMouseAction(KEY_A,KeyAction_PRESS))//�жϼ�λ�Ƿ���
				{
						Expt.Target_Vx = (Chassis_Mode == CHAS_ReFollowMode?-FollowMoveValue:FollowMoveValue);
					
						Expt.Target_Vx *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//��CTRL������
						Expt.Target_Vx *= -1; 
				}
				else if(GetKeyMouseAction(KEY_D,KeyAction_PRESS))//�жϼ�λ�Ƿ���
				{
						Expt.Target_Vx = (Chassis_Mode == CHAS_ReFollowMode?-FollowMoveValue:FollowMoveValue);
						Expt.Target_Vx *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//��CTRL������
				}
				else
				{
						Expt.Target_Vx = 0;
				}
		  }
			else
			{
					if(GetKeyMouseAction(KEY_W,KeyAction_PRESS))//�жϼ�λ�Ƿ���
					{
							Chassis_VxVy_Limit = (Chassis_Mode == CHAS_ReFollowMode?-660:660);
							Chassis_VxVy_Limit *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//��CTRL������
							Chassis_Drv_Slow(&Expt.Target_Vy, Chassis_VxVy_Limit, slop_temp);//�����ٶ�б��
					}
					else if(GetKeyMouseAction(KEY_S,KeyAction_PRESS))//�жϼ�λ�Ƿ���
					{
							Chassis_VxVy_Limit = (Chassis_Mode == CHAS_ReFollowMode?-660:660);
							Chassis_VxVy_Limit *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//��CTRL������
							Chassis_Drv_Slow(&Expt.Target_Vy, -Chassis_VxVy_Limit, slop_temp);//�����ٶ�б��
					}
					else
					{
							Expt.Target_Vy = 0;
					}
					
					if(GetKeyMouseAction(KEY_A,KeyAction_PRESS))//�жϼ�λ�Ƿ���
					{
							Chassis_VxVy_Limit = (Chassis_Mode == CHAS_ReFollowMode?-660:660);
						
							Chassis_VxVy_Limit *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//��CTRL������

							Chassis_Drv_Slow(&Expt.Target_Vx, -Chassis_VxVy_Limit, slop_temp);//�����ٶ�б��
					}
					else if(GetKeyMouseAction(KEY_D,KeyAction_PRESS))//�жϼ�λ�Ƿ���
					{
							Chassis_VxVy_Limit = (Chassis_Mode == CHAS_ReFollowMode?-660:660);
							Chassis_VxVy_Limit *= ((GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))?0.1f:1.0f);//��CTRL������
							Chassis_Drv_Slow(&Expt.Target_Vx, Chassis_VxVy_Limit, slop_temp);//�����ٶ�б��
					}
					else
					{
							Expt.Target_Vx = 0;
					}		
  		}
  	}
		

    /*--- CTRL+��ϼ� -----------------------------------------------------------------------------------------------------------*/
    
    //---------------------------- CTRL ��ϼ� END ------------------------------------------------------		
		if(GetKeyMouseAction(KEY_Z,KeyAction_PRESS)) /*<! ����Z ���ģʽ */
		{
		   Set_VisionMode(Vision_BIGWindmill);
			 Set_ChassisMode(CHAS_LockMode); // ��ס����
		}
		else                               
		{
	     Set_VisionMode(Vision_Forcast);  //Ԥ��
			 Set_ChassisMode(CHAS_followMode); // ����
		}
		
		/*<! �Զ���� + ����С����ģʽ */  //---�¼� ����V
    if(GetKeyMouseAction(KEY_V,KeyAction_PRESS))
		{
			 Vision_Top_flag = 0;
			 Set_VisionMode(Vision_Top); /*<! ����С����ģʽ */
			 Set_AttackMode(Attack_Auto);/*-- �Զ�����--*/
		}
		else if(remote_control.keyBoard.key_code != 16384 && Vision_Top_flag == 0)
		{
       Vision_Top_flag = 1;
			 Set_VisionMode(Vision_Forcast);//Ԥ��
			 if(Attack_Mode == Attack_Auto)
			 {
				 Set_AttackMode(Attack_15);
			 }			
		}

			/*<! �Զ���� + ����ǰ��վģʽ */  //---�¼� ����X
    if(GetKeyMouseAction(KEY_G,KeyAction_PRESS))
		{
			 Vision_Top_outpost = 0;
			 Set_VisionMode(Vision_Outpost); /*<! ����ǰ��վģʽ */
			 Set_AttackMode(Attack_Auto);/*-- �Զ�����--*/
		}
		else if(remote_control.keyBoard.key_code != 1024 && Vision_Top_outpost == 0)
		{
       Vision_Top_outpost = 1;
			 Set_VisionMode(Vision_Forcast);//Ԥ��
			 if(Attack_Mode == Attack_Auto)
			 {
				 Set_AttackMode(Attack_15);
			 }			
		}	
		
		//4.18��
		if(GetKeyMouseAction(KEY_SHIFT,KeyAction_PRESS))  //--- ��SHIFT С����
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


		
//		if(GetKeyMouseAction(KEY_Z,KeyAction_LONG_PRESS) &&GetKeyMouseAction(KEY_X,KeyAction_LONG_PRESS) && GetKeyMouseAction(KEY_C,KeyAction_LONG_PRESS))  //--- ���� Z+X+C �޸���ģʽ
//		{
//			  Set_ChassisMode(CHAS_NotfollowMode);///�޸���
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

		if(GetKeyMouseAction(KEY_Q,KeyAction_LONG_PRESS))  /*<! Q ����ģʽ������ǹ������ */
    {
        Rage_Mode = 1;
    }
    else
    {
        Rage_Mode = 0;
    }		
		
		
    if(GetKeyMouseAction(KEY_C,KeyAction_LONG_PRESS))  /*<! ����C ���ݿ��� */
    {
        ChassisCap_Ctrl(ON);
    }
    else
    {
        ChassisCap_Ctrl(OFF); 
    }		
		
		//����ģʽ 	
	  //..........
		

		
    if(GetKeyMouseAction(MOUSE_Left,KeyAction_CLICK) && Reload.Began == false && Shoot_Freq > 0)  /*<! ���䵯�� */  //�¸�
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


	 if(GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))/*<! ��ctrl + R ���ֹ� */
		{
			if(GetKeyMouseAction(KEY_R,KeyAction_PRESS))
			{
				 Mag_Ctrl(OFF);
			}
		}
		else if(GetKeyMouseAction(KEY_R,KeyAction_PRESS)) /*<! ��R ���ֿ��� */
		{
			  Mag_Ctrl(ON);
		}
		
		
	 if(GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))/*<! �� ctrl + E �ص�Ħ���� */
		{
			if(GetKeyMouseAction(KEY_E,KeyAction_PRESS))
			{
				Set_AttackMode(Attack_Disable);
			}
		}
		else if(GetKeyMouseAction(KEY_E,KeyAction_PRESS)) /*<! ��E ��Ħ���ֿ� */
		{
			  Set_AttackMode(Attack_15);
		}
		
		
		if(GetKeyMouseAction(MOUSE_Right,KeyAction_PRESS))  /*<! �� ����Ҽ� ���鿪�� */
		{
				Set_GimbalMode(Gimbal_PCMode);
		}
		else
		{
				Set_GimbalMode(Gimbal_NormalMode);
		}		

			
		
    /*<! UI ˢ�� */		
    if(GetKeyMouseAction(KEY_B,KeyAction_PRESS)) /*<! UI ˢ�� */
    {
        Reflash_UI = true;
    }
    else
    {
        Reflash_UI = false;
    }


//		if(GetKeyMouseAction(KEY_CTRL,KeyAction_PRESS))/*<! �� ctrl + F ȡ��45�ȶԽ�ģʽ */
//		{
//			if(GetKeyMouseAction(KEY_F,KeyAction_PRESS))
//			{
//         Diagonal_Mode = false;
//			}
//		}
//		else if(GetKeyMouseAction(KEY_F,KeyAction_PRESS)) /*<! ��F�Խ�ģʽ */
//		{
//         Diagonal_Mode = true;
//		}
		
//	  /*<! ���Pit�ֶ����� */
//    if()
//		{
//			
//		}
		
}


/**
 * @brief      ��ȡ�����������
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

