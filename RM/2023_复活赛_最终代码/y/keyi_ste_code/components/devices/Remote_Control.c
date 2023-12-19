/**
  ******************************************************************************
  * @file    Remote_Control.c
  * @author  DJI 
  * @version V1.0.0
  * @date    2015/11/15
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "Remote_Control.h"
#include "math.h"
#include "Devices_Monitor.h"
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOF
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOE
uint16_t TIM_COUNT[2];

RC_Type remote_control;
uint32_t  Latest_Remote_Control_Pack_Time = 0;
uint32_t  LED_Flash_Timer_remote_control = 0;

DR16Status_Typedef Ctrl_Source;
DR16_Export_Data_t DR16_Export_Data;

/*******************************************************************************************
  * @Func		void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
  * @Brief  DR16接收机协议解码程序
  * @Param		RC_Type* rc　存储遥控器数据的结构体　　uint8_t* buff　用于解码的缓存
  * @Retval		None
  * @Date    
 *******************************************************************************************/
void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
{
//	rc->ch1 = (*buff | *(buff+1) << 8) & 0x07FF;	offset  = 1024
	rc->ch1 = (buff[0] | buff[1]<<8) & 0x07FF;
	rc->ch1 -= 1024;
	rc->ch2 = (buff[1]>>3 | buff[2]<<5 ) & 0x07FF;
	rc->ch2 -= 1024;
	rc->ch3 = (buff[2]>>6 | buff[3]<<2 | buff[4]<<10) & 0x07FF;
	rc->ch3 -= 1024;
	rc->ch4 = (buff[4]>>1 | buff[5]<<7) & 0x07FF;		
	rc->ch4 -= 1024;
	
	rc->switch_left = ( (buff[5] >> 4)& 0x000C ) >> 2;
	rc->switch_right =  (buff[5] >> 4)& 0x0003 ;
	
	rc->mouse.x = buff[6] | (buff[7] << 8);	//x axis
	rc->mouse.y = buff[8] | (buff[9] << 8);
	rc->mouse.z = buff[10]| (buff[11] << 8);
	
	rc->mouse.press_left 	= buff[12];	// is pressed?
	rc->mouse.press_right = buff[13];
	
	rc->keyBoard.key_code = buff[14] | buff[15] << 8; //key borad code
		 //your control code ….
  rc->ch4_DW = (buff[16] | (buff[17] << 8)) & 0x07FF;
	if (rc->ch4_DW <= 20 && rc->ch4_DW >= -20)
      rc->ch4_DW = 0;
		 
  rc->ch4_DW -= 1024;
	
	if (rc->ch1 <= 20 && rc->ch1 >= -20)
      rc->ch1= 0;	
	if (rc->ch2 <= 20 && rc->ch2 >= -20)
      rc->ch2 = 0;
	if (rc->ch3 <= 20 && rc->ch3 >= -20)
      rc->ch3 = 0;
	if (rc->ch4 <= 20 && rc->ch4 >= -20)
      rc->ch4 = 0;	
	Latest_Remote_Control_Pack_Time = HAL_GetTick();
	
	if(Latest_Remote_Control_Pack_Time - LED_Flash_Timer_remote_control>500){
			
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			
			LED_Flash_Timer_remote_control = Latest_Remote_Control_Pack_Time;
		
			
	}
   KeyMouseFlag_Update();// 键盘标志位更新 
}


/**
  * @Data    2021/3/30
  * @brief   键盘标志位更新 
  * @param   void
  * @retval  void
  */
void KeyMouseFlag_Update(void)
{
    uint32_t KeyMouse = (uint32_t)remote_control.keyBoard.key_code | remote_control.mouse.press_left << 16 | remote_control.mouse.press_right << 17; // 把键盘鼠标的标志位合并。

    for (int Index = 0; Index < RC_Frame_Lentgh; Index++) //遍历全部键位，更新他们的状态。
    {
        if (KeyMouse & (1 << Index)) //判断第index位是否为1。
        {
            DR16_Export_Data.KeyMouse.PressTime[Index]++;
            if (DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_Press) //满足按下的时间，视为按下
            {
                DR16_Export_Data.KeyMouse.Press_Flag |= 1 << Index; //设置该键的标志位为1
            }

            if (DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_LongPress) //长按判断
            {

                DR16_Export_Data.KeyMouse.Long_Press_Flag |= 1 << Index; //设置长按标志位
            }
        }
        else
        {
            if ((DR16_Export_Data.KeyMouse.PressTime[Index] > TIME_KeyMouse_Press) && (DR16_Export_Data.KeyMouse.PressTime[Index] < TIME_KeyMouse_LongPress)) //时间处于两者之间，为单击。
            {
                DR16_Export_Data.KeyMouse.Click_Press_Flag |= 1 << Index; //设置单击标志位
            }
            else
            {
                DR16_Export_Data.KeyMouse.Click_Press_Flag &= ~(1 << Index); //取反操作，将该键的标志位设为0
            }

            //已经松开，将按下标志位置空。
            DR16_Export_Data.KeyMouse.Press_Flag &= ~(1 << Index);
            DR16_Export_Data.KeyMouse.Long_Press_Flag &= ~(1 << Index);
            DR16_Export_Data.KeyMouse.PressTime[Index] = 0;
        }
    }
}

/**
	* @brief  获取鼠标键盘某个键当前的动作
  * @param	键值  动作
  * @retval 返回键鼠动作状态  0 没有该动作 1 有该动作
  */
bool GetKeyMouseAction(KeyList_e KeyMouse, KeyAction_e Action)
{
    uint8_t action = 0;
    switch (Action)
    {
    case KeyAction_CLICK: //单击

        action = ((DR16_Export_Data.KeyMouse.Click_Press_Flag >> KeyMouse) & 1);
        break;
    case KeyAction_PRESS: //按下
        action = ((DR16_Export_Data.KeyMouse.Press_Flag >> KeyMouse) & 1);
        break;
    case KeyAction_LONG_PRESS: //长按
        action = ((DR16_Export_Data.KeyMouse.Long_Press_Flag >> KeyMouse) & 1);
        break;
    default:
        action = 0;
        break;
    }
    return action;
}


/**
 * @brief      以下Getxxx函数功能都是获得数据包中的Get后面数据的值。
 * @param[in]  None
 * @return     Get后面的数据的值
 */

int16_t Get_CH0(void)//--- RX
{
	return remote_control.ch1;
}
int16_t Get_CH1(void)//--- RY
{
	return remote_control.ch2;
}
int16_t Get_CH2(void)//--- LX
{
	return remote_control.ch3;
}
int16_t Get_CH3(void)//--- LY
{
	return remote_control.ch4;
}
int16_t Get_DW(void)
{
	return remote_control.ch4_DW;
}
uint8_t Get_S1_L(void)
{
	return remote_control.switch_left;
}
uint8_t Get_S2_R(void)
{
	return remote_control.switch_right;
}
int16_t Get_MouseX(void)
{
	return remote_control.mouse.x;
}
int16_t Get_MouseY(void)
{
	return remote_control.mouse.y;
}
int16_t Get_MouseZ(void)
{
	return remote_control.mouse.z;
}
uint8_t Get_Mouse_keyL(void)
{
	return remote_control.mouse.press_left;
}
uint8_t Get_Mouse_keyR(void)
{
	return remote_control.mouse.press_right;
}
uint16_t Get_Mouse_key(void)
{
	return remote_control.keyBoard.key_code;
}























extern uint16_t TIM_COUNT[];
int16_t HighTime;


/*******************************************************************************************
  * @Func		void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
  * @Brief  PWM接收机脉宽计算
  * @Param		TIM_HandleTypeDef *htim 用于测量PWM脉宽的定时器。
  * @Retval		None
  * @Date    
 *******************************************************************************************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	HighTime = (TIM_COUNT[1] - TIM_COUNT[0])>0?(TIM_COUNT[1] - TIM_COUNT[0]):((TIM_COUNT[1] - TIM_COUNT[0])+10000);
	
	Latest_Remote_Control_Pack_Time = HAL_GetTick();
			
  remote_control.ch4 = (HighTime - 4000)*660/4000;
	
	if(Latest_Remote_Control_Pack_Time - LED_Flash_Timer_remote_control>500){
			
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);		
			LED_Flash_Timer_remote_control = Latest_Remote_Control_Pack_Time;
					
	}
	
}
