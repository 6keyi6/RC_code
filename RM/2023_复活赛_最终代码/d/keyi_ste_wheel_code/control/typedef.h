#ifndef  __TYPEDEF_H
#define  __TYPEDEF_H
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h> 
#include "can.h"
/*************CAN�������ݽӿ�************/
typedef struct
{
    CAN_RxHeaderTypeDef CAN_RxHeader;
    uint8_t CANx_Export_RxMessage[8];
} Can_Export_Data_t;

/*************CAN�������ݽӿ�************/
typedef struct
{
    CAN_HandleTypeDef *Canx;
    CAN_TxHeaderTypeDef CAN_TxHeader;
    uint8_t CANx_Send_RxMessage[8];
} Can_Send_Data_t;
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

typedef int8_t  s8;
typedef int16_t s16;
typedef int32_t s32;

typedef volatile uint8_t  vu8;
typedef volatile uint16_t vu16;
typedef volatile uint32_t vu32;

typedef volatile int8_t  vs8;
typedef volatile int16_t vs16;
typedef volatile int32_t vs32;

///* PID���� */
//typedef struct{
//	float Target; 			        //�趨Ŀ��ֵ
//	float Measured; 				    //����ֵ
//	float err; 						      //����ƫ��ֵ
//	float err_last; 				    //��һ��ƫ��
//	float err_beforeLast; 			//���ϴ�ƫ��
//	float Kp, Ki, Kd;				    //Kp, Ki, Kd����ϵ��
//	float pwm; 						      //pwm���
//	uint32_t MaxOutput;				  //����޷�
//  uint32_t IntegralLimit;			//�����޷� 
//}incrementalpid_t;

//typedef struct{
//	float Target; 					    //�趨Ŀ��ֵ
//	float Measured; 				    //����ֵ
//	float err; 						      //����ƫ��ֵ
//	float err_last; 				    //��һ��ƫ��
//	float integral_err; 			  //����ƫ��֮��
//	float Kp, Ki, Kd;				    //Kp, Ki, Kd����ϵ��
//	float pwm; 						      //pwm���
//	uint32_t MaxOutput;				  //����޷�
//  uint32_t IntegralLimit;			//�����޷� 
//	float p_out, i_out, d_out;
//}positionpid_t;

typedef struct positionpid_t
{
    float Target;     //�趨Ŀ��ֵ
    float Measured;   //����ֵ
    float err;        //����ƫ��ֵ
    float err_last;   //��һ��ƫ��
    float err_change; //���仯��
    float Kp;
    float Ki;
    float Kd; //Kp, Ki, Kd����ϵ��
    float p_out;
    float i_out;
    float d_out;               //���������ֵ
    float pwm;                 //pwm���
    float MaxOutput;           //����޷�
    float Integral_Separation; //���ַ�����ֵ
    float IntegralLimit;       //�����޷�
    float (*Position_PID)(struct positionpid_t *pid_t, float target, float measured);
} imu_positionpid_t;//ר�������ǿ����¶�PID�ṹ��


/**********ϵͳʱ��������ݽӿ�************/
typedef struct
{
    uint32_t WorldTime;      //����ʱ��
    uint32_t Last_WorldTime; //��һ������ʱ��
} WorldTime_RxTypedef;
void Get_FPS(WorldTime_RxTypedef *time, uint32_t *FPS);


extern int i,j;
#endif
