/**
  ******************************************************************************
  * @file    Remote_Control.h
  * @author  Ginger
  * @version V1.0.0
  * @date    2015/11/14
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
	
#ifndef __RC__
#define __RC__
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "Robot_Control.h"
#define RC_Frame_Lentgh		18//键盘鼠标总和：18个键。
#define TIME_KeyMouse_Press 1 //超过该时间视为 按下。
//在两者之间视为 单击
#define TIME_KeyMouse_LongPress 20 //超过该时间视为 长按

/**
  @brief 连接状态
*/
#ifndef __DR16Status_DEFINED
#define __DR16Status_DEFINED
typedef enum  
{
  CTRL_OFF,    //关闭机器
  CTRL_RC,    //遥控模式
  CTRL_PC    //键鼠模式
}DR16Status_Typedef;
#endif

extern DR16Status_Typedef Ctrl_Source;
typedef enum
{
    //与DR16_Export_data.KeyMouse 的flag位一一对应
    KEY_W = 0,
    KEY_S = 1,
    KEY_A,
    KEY_D,
    KEY_SHIFT,
    KEY_CTRL,
    KEY_Q,
    KEY_E,
    KEY_R,
    KEY_F,
    KEY_G,
    KEY_Z,
    KEY_X,
    KEY_C,
    KEY_V,
    KEY_B,
    MOUSE_Left,
    MOUSE_Right
} KeyList_e;
typedef enum
{
    KeyAction_CLICK,
    KeyAction_PRESS,
    KeyAction_LONG_PRESS
} KeyAction_e; //鼠标键盘（键）事件类型。

/**
 * @brief 手柄上面两挡位开关状态
 */
enum 
{
  Lever_NONE = 0,
  Lever_UP = 1,
  Lever_MID = 3,
  Lever_DOWN = 2,
};

typedef struct
{
    struct
    {
        float x;
        float y;
    } mouse;

    struct
    {

        uint32_t Press_Flag;                //键鼠按下标志
        uint32_t Click_Press_Flag;          //键鼠单击标志
        uint32_t Long_Press_Flag;           //键鼠长按标志
        uint8_t PressTime[RC_Frame_Lentgh]; //键鼠按下持续时间
    } KeyMouse;                             //鼠标的对外输出。

    struct
    {
        float Forward_Back_Value; //Vx
        float Omega_Value;        //自旋值。
        float Left_Right_Value;   //Vy
        float Pitch_Value;
        float Yaw_Value;
        float Dial_Wheel; //拨轮
    } Robot_TargetValue;  //遥控计算比例后的运动速度
    uint16_t infoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} DR16_Export_Data_t;         //供其他文件使用的输出数据。


typedef struct {
	int16_t ch1;	//each ch value from -364 -- +364
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	int16_t ch4_DW;
	uint8_t switch_left;	//3 value
	uint8_t switch_right;
	
	struct {
		int16_t x;
		int16_t y;
		int16_t z;
	
		uint8_t press_left;
		uint8_t press_right;
	}mouse;
	
	struct {
		uint16_t key_code;
/**********************************************************************************
   * 键盘通道:15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
   *          V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
************************************************************************************/

	}keyBoard;
	

}RC_Type;
extern RC_Type remote_control;

//遥控器对外输出参数
typedef struct 
{
	float Target_Vx,Target_Vy,Target_Vw;
}Expt_t;
extern Expt_t Expt;
extern uint32_t  Latest_Remote_Control_Pack_Time ;
void Callback_RC_Handle(RC_Type* rc, uint8_t* buff);


void KeyMouseFlag_Update(void);
int GetKeyMouseAction(KeyList_e KeyMouse, KeyAction_e Action);
int16_t Get_CH0(void);
int16_t Get_CH1(void);
int16_t Get_CH2(void);
int16_t Get_CH3(void);
int16_t  Get_DW(void);
uint8_t Get_S1_L(void);
uint8_t Get_S2_R(void);
int16_t Get_MouseX(void);
int16_t Get_MouseY(void);
int16_t Get_MouseZ(void);
uint8_t Get_Mouse_keyL(void);
uint8_t Get_Mouse_keyR(void);
uint16_t Get_Mouse_key(void);
void DR16_LeverMode_Update(void);
/**
 * @brief  	  设置底盘运作模式
 * @param[in]	mode
 * @retval 	  None
 */
void Set_ChassisMode(CHAS_CtrlMode_e mode);
 void DR16RCCtrl_Update(void);

#endif


