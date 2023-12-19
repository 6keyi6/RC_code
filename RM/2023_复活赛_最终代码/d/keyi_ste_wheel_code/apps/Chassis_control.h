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
#define KEYI2022_INFANTRY 1 //22����������
#define KEYI2022_INFANTRY_SWERVE 2 //22����������

#define RUD_OPSI       1
#define RUD_NOT_OPSI   0
#define RUD_RESET      1
#define RUD_NOT_RESET  0

#ifndef PI
  #define PI               3.14159265358979f
#endif

#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define M6020_mAngleRatio 22.7527f //��е�Ƕ�����ʵ�Ƕȵı���

#define SpinSpeedRampInit \
    {                     \
        0,                \
            20,           \
            0,            \
            0,            \
    }


/* --- ���̿���ģʽ ------------------------------------------------------------*/
typedef enum 
{
	CHAS_DisableMode,	    // ʧ��ģʽ
    CHAS_followMode ,	    // ����ģʽ
	CHAS_ReFollowMode,      // �������
	CHAS_SpinMode ,	        // С����ģʽ
    CHAS_sinSpinMode,       // ����С����
	CHAS_AutoTrackMode,     // �Զ�׷��ģʽ
	CHAS_LockMode,		    // ��ס����
	CHAS_NotfollowMode,	    // �޸���ģʽ

	CHAS_SupplyMode,	    // ����ģʽ
	CHAS_SwingMode,		    // Ť��ģʽ
	CHAS_InitMode,          // ��ʼ���׶�
}CHAS_CtrlMode_e;
extern CHAS_CtrlMode_e Chassis_Mode;

/* �����˶� */
typedef struct
{
    float targetXRaw;         //����x��ԭʼ����
    float targetYRaw;         //����y��ԭʼ����
    float targetZRaw;         //����z��ԭʼ����
    float LpfAttFactor;       //�����˲�ϵ��
    float targetXLPF;         //����x���˲�������
    float targetYLPF;         //����y���˲�������
    float targetZLPF;         //����z���˲�������
    float speedLimit;         //�����ٶ�����
    uint16_t OmegaLimit;      //�����ٶ�����
    float Trace_Distance;     //����װ�װ����
    float FollowtargetYawRaw; //����Ŀ��Yaw�������̨ԭʼ����
    float FollowtargetYawLPF; //����Yaw�������̨�˲�������

    float SpeedChange_Factor;  //�ٶȸı�����
    float SpeedLimit_Factor;   //��������
    uint8_t mode;              //���̿���ģʽ
    uint8_t swingFlag;         //Ť����־λ
    float spinSpeed;           //�����ٶ�
    float swingSpeed;          //Ť���ٶ�
    uint8_t PowerOverflowFlag; //�����ʱ�־λ
} Chassis_t;

/* --- ת���ֵ����ز��� -------------------------------------------------------*/
typedef struct 
{
    float Init_angle;   // ��ʼ��У׼�Ƕ�
    float Target_angle; // Ŀ��Ƕ�
    float PreTar_angle; // ǰһ��Ŀ��Ƕ�
    float Total_angle;  // ��ǰ�ܽǶ�
    int32_t Turns_cnt;
    int32_t TarTurns_cnt;
    int32_t Turns_flag;
}RUD_Param_t;

/* --- ����ת���� ID --------------------------------------------------------*/
enum CHAS_RudMororID_e
{
    RF_205 = 0,
    LF_206 = 1,
    LB_207 = 2,
    RB_208 = 3
};
/* --- ����������� ID --------------------------------------------------------*/
enum CHAS_DrvMotorID_e
{
    RF_201 = 0,
    LF_202 = 1,
    LB_203 = 2,
    RB_204 = 3
};
extern RUD_Param_t RUD_Param[4]; /*<! ת������ز��� */

extern positionpid_t M3508_Chassis_Follow_PTZ_Ppid;//������̨���PID����
extern incrementalpid_t M3508_Chassis_Follow_PTZ_Ipid;//������̨���PID����
extern incrementalpid_t M3508_Chassis_Follow_PTZ_Ipid_t[4];//������̨���PID����
extern incrementalpid_t M3508_CHAS_NotFollow_Ipid[4];// �޸���PID����
extern incrementalpid_t M3508_CHAS_Spin_Ipid[4];//С����PID����

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
  * @brief  �����ܿ���
  * @param  void
  * @retval None
  */
void Chassis_control(void);
/**
  * @brief  ����PID��ʼ��
  * @param  void
  * @retval None
  */
void Chassis_PID_Init(void);

/**
  * @brief  ���̸���PID����
  * @param  void
  * @retval None
  */
void Chassis_Follow_PTZ_cal(void);
///**
//  * @brief  3508�ܵ�������
//  * @param  void
//  * @retval None
//  */
//void M3508_All_send(void);
/**
  * @brief  �����޸���PID����
  * @param  void
  * @retval None
  */
void Chassis_NotFollow_cal(void);
/**
  * @brief  С����PID����
  * @param  void
  * @retval None
  */
void Spin_cal(void);

/**
  * @brief  ȫ��ʽ
  * @param  void
  * @retval None
  */
void Omnidirectional_Formula(float *Vx, float *Vy);

/**
 * @brief      �����ٶ�б��
 * @param[in]  rec, target, slow_Inc
 * @retval     None
 */
void Chassis_Drv_Slow(float *rec , float target , float slow_Inc, float Accval, float DecVal);


/*********************�����Ƕ��ֺ���*****************************/
/**
 * @brief      ���ֽ���
 * @param[in]  None
 * @retval     None
 */
void Rudder_Solve(int16_t Vx, int16_t Vy, int16_t Vw, int16_t *cal_speed);
/**
 * @brief      �����ٶȽ����Ƕ�
 * @param[in]  None
 * @retval     None
 */
void RudAngle_Calc(int16_t Vx, int16_t Vy, int16_t Vw);
/**
 * @brief      ת���ֵ�ǰ�ܽǶȼ���
 * @param[in]  num
 * @param[in]  motor_num
 * @param[in]  reset
 * @param[in]  opposite
 * @retval     None
 */
void RUDTotalAngle_Calc(M6020s_t* rudder_motor , int8_t motor_num , int8_t reset , uint8_t opposite);

/**
 * @brief      ת����Ŀ��Ƕȼ���
 * @param[in]  motor_num
 * @param[in]  reset
 * @param[in]  opposite
 * @retval     None
 */

void AngleLimit(float *angle);

/**
 * @brief      ������Сƫ�ʹת���ֱ����ӻ���ת
 * @param[in]  target
 * @param[in]  current
 * @retval     target(�Ƕ���)
 */
float Turn_InferiorArc(uint8_t motor, float target, float current);

void RUDTargetAngle_Calc(int8_t motor_num , int8_t reset , uint8_t opposite);


void Process(float Vx, float Vy, float Vw);




/**
 * @brief      ��ȡ���е�����ز���ϵͳ��Ϣ
 * @param[in]  None
 * @retval     Referee data
 */
float CHAS_Get_Power(void);
uint16_t CHAS_Max_Get_Power(void);
uint16_t CHAS_Get_PowerBuffer(void);


void follow_ctr(float Vx,float Vy,float Vw);


/**
 * @brief      ��������ٶ�����
 * @param[in]  None
 * @retval     None
 */
void Set_MaxSpeed(void);


void Constrain(int16_t *val, int16_t min, int16_t max);


/**
  * @brief      б�º���,ʹĿ�����ֵ������������ֵ
  * @param[in]  �����������,��ǰ���,�仯�ٶ�(Խ��Խ��)
  * @retval     ��ǰ���
  * @attention  
  */
float RAMP_Output(float final, float now, float ramp);
extern positionpid_t Follow_OUT_pid;
extern incrementalpid_t Follow_IN_pid;

extern int16_t temp_Vw; 
extern float DRIVE_speed[4];
#endif
