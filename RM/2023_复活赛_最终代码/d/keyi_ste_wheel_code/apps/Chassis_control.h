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
#ifndef __CHIASSIS_CONTROL_H
#define __CHIASSIS_CONTROL_H
#include "pid.h"

#include "M6020_Motor.h"
#define ROBOT_ID 2
#define KEYI2022_INFANTRY 1 //22分区赛麦轮
#define KEYI2022_INFANTRY_SWERVE 2 //22分区赛舵轮

#define RUD_OPSI       1
#define RUD_NOT_OPSI   0
#define RUD_RESET      1
#define RUD_NOT_RESET  0

#ifndef PI
  #define PI               3.14159265358979f
#endif

#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define M6020_mAngleRatio 22.7527f //机械角度与真实角度的比率

#define SpinSpeedRampInit \
    {                     \
        0,                \
            20,           \
            0,            \
            0,            \
    }


/* --- 底盘控制模式 ------------------------------------------------------------*/
typedef enum 
{
	CHAS_DisableMode,	    // 失能模式
    CHAS_followMode ,	    // 跟随模式
	CHAS_ReFollowMode,      // 反向跟随
	CHAS_SpinMode ,	        // 小陀螺模式
    CHAS_sinSpinMode,       // 变速小陀螺
	CHAS_AutoTrackMode,     // 自动追踪模式
	CHAS_LockMode,		    // 锁住底盘
	CHAS_NotfollowMode,	    // 无跟随模式

	CHAS_SupplyMode,	    // 补给模式
	CHAS_SwingMode,		    // 扭腰模式
	CHAS_InitMode,          // 初始化阶段
}CHAS_CtrlMode_e;
extern CHAS_CtrlMode_e Chassis_Mode;

/* 底盘运动 */
typedef struct
{
    float targetXRaw;         //底盘x轴原始数据
    float targetYRaw;         //底盘y轴原始数据
    float targetZRaw;         //底盘z轴原始数据
    float LpfAttFactor;       //底盘滤波系数
    float targetXLPF;         //底盘x轴滤波后数据
    float targetYLPF;         //底盘y轴滤波后数据
    float targetZLPF;         //底盘z轴滤波后数据
    float speedLimit;         //底盘速度限制
    uint16_t OmegaLimit;      //底盘速度限制
    float Trace_Distance;     //跟随装甲板距离
    float FollowtargetYawRaw; //底盘目标Yaw轴跟随云台原始数据
    float FollowtargetYawLPF; //底盘Yaw轴跟随云台滤波后数据

    float SpeedChange_Factor;  //速度改变因子
    float SpeedLimit_Factor;   //限速因子
    uint8_t mode;              //底盘控制模式
    uint8_t swingFlag;         //扭腰标志位
    float spinSpeed;           //自旋速度
    float swingSpeed;          //扭腰速度
    uint8_t PowerOverflowFlag; //超功率标志位
} Chassis_t;

/* --- 转向轮电机相关参数 -------------------------------------------------------*/
typedef struct 
{
    float Init_angle;   // 初始化校准角度
    float Target_angle; // 目标角度
    float PreTar_angle; // 前一次目标角度
    float Total_angle;  // 当前总角度
    int32_t Turns_cnt;
    int32_t TarTurns_cnt;
    int32_t Turns_flag;
}RUD_Param_t;

/* --- 底盘转向电机 ID --------------------------------------------------------*/
enum CHAS_RudMororID_e
{
    RF_205 = 0,
    LF_206 = 1,
    LB_207 = 2,
    RB_208 = 3
};
/* --- 底盘驱动电机 ID --------------------------------------------------------*/
enum CHAS_DrvMotorID_e
{
    RF_201 = 0,
    LF_202 = 1,
    LB_203 = 2,
    RB_204 = 3
};
extern RUD_Param_t RUD_Param[4]; /*<! 转向轮相关参数 */

extern positionpid_t M3508_Chassis_Follow_PTZ_Ppid;//跟随云台电机PID参数
extern incrementalpid_t M3508_Chassis_Follow_PTZ_Ipid;//跟随云台电机PID参数
extern incrementalpid_t M3508_Chassis_Follow_PTZ_Ipid_t[4];//跟随云台电机PID参数
extern incrementalpid_t M3508_CHAS_NotFollow_Ipid[4];// 无跟随PID参数
extern incrementalpid_t M3508_CHAS_Spin_Ipid[4];//小陀螺PID参数

extern positionpid_t M6020_OUT[4];
extern positionpid_t M6020_IN[4];


extern float RUN_totalAngle[4];
extern float RUN_speed[4];
extern float cal_speed[4];


extern positionpid_t Class_OUT[4];
extern incrementalpid_t Class_IN[4];
extern incrementalpid_t Class_SPIN_IN[4];
extern int16_t Cal_Speed[4];

extern float tempV[3];
/**
  * @brief  底盘总控制
  * @param  void
  * @retval None
  */
void Chassis_control(void);
/**
  * @brief  底盘PID初始化
  * @param  void
  * @retval None
  */
void Chassis_PID_Init(void);

/**
  * @brief  底盘跟随PID计算
  * @param  void
  * @retval None
  */
void Chassis_Follow_PTZ_cal(void);
///**
//  * @brief  3508总电流发送
//  * @param  void
//  * @retval None
//  */
//void M3508_All_send(void);
/**
  * @brief  底盘无跟随PID计算
  * @param  void
  * @retval None
  */
void Chassis_NotFollow_cal(void);
/**
  * @brief  小陀螺PID计算
  * @param  void
  * @retval None
  */
void Spin_cal(void);

/**
  * @brief  全向公式
  * @param  void
  * @retval None
  */
void Omnidirectional_Formula(float *Vx, float *Vy);

/**
 * @brief      底盘速度斜坡
 * @param[in]  rec, target, slow_Inc
 * @retval     None
 */
void Chassis_Drv_Slow(float *rec , float target , float slow_Inc, float Accval, float DecVal);


/*********************以下是舵轮函数*****************************/
/**
 * @brief      舵轮解算
 * @param[in]  None
 * @retval     None
 */
void Rudder_Solve(int16_t Vx, int16_t Vy, int16_t Vw, int16_t *cal_speed);
/**
 * @brief      三轴速度解算舵角度
 * @param[in]  None
 * @retval     None
 */
void RudAngle_Calc(int16_t Vx, int16_t Vy, int16_t Vw);
/**
 * @brief      转向轮当前总角度计算
 * @param[in]  num
 * @param[in]  motor_num
 * @param[in]  reset
 * @param[in]  opposite
 * @retval     None
 */
void RUDTotalAngle_Calc(M6020s_t* rudder_motor , int8_t motor_num , int8_t reset , uint8_t opposite);

/**
 * @brief      转向轮目标角度计算
 * @param[in]  motor_num
 * @param[in]  reset
 * @param[in]  opposite
 * @retval     None
 */

void AngleLimit(float *angle);

/**
 * @brief      计算最小偏差，使转向轮保持劣弧旋转
 * @param[in]  target
 * @param[in]  current
 * @retval     target(角度制)
 */
float Turn_InferiorArc(uint8_t motor, float target, float current);

void RUDTargetAngle_Calc(int8_t motor_num , int8_t reset , uint8_t opposite);


void Process(float Vx, float Vy, float Vw);




/**
 * @brief      获取裁判底盘相关裁判系统信息
 * @param[in]  None
 * @retval     Referee data
 */
float CHAS_Get_Power(void);
uint16_t CHAS_Max_Get_Power(void);
uint16_t CHAS_Get_PowerBuffer(void);


void follow_ctr(float Vx,float Vy,float Vw);


/**
 * @brief      设置最大速度限制
 * @param[in]  None
 * @retval     None
 */
void Set_MaxSpeed(void);


void Constrain(int16_t *val, int16_t min, int16_t max);


/**
  * @brief      斜坡函数,使目标输出值缓慢等于期望值
  * @param[in]  期望最终输出,当前输出,变化速度(越大越快)
  * @retval     当前输出
  * @attention  
  */
float RAMP_Output(float final, float now, float ramp);
extern positionpid_t Follow_OUT_pid;
extern incrementalpid_t Follow_IN_pid;

extern int16_t temp_Vw; 
extern float DRIVE_speed[4];
#endif
