#ifndef _DEVICES_MONITOR_H_
#define _DEVICES_MONITOR_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>


/* Macro Definitions ---------------------------------------------------------*/
#define DR16_MONITOR		    (1<<0)  // DR16
#define CAN1_MONITOR			(1<<1)	// CAN1
#define CAN2_MONITOR		    (1<<2)	// CAN2
#define CHAS_DRV0_MONITOR		(1<<3)	// Chassis 3508 0
#define CHAS_DRV1_MONITOR		(1<<4)	// Chassis 3508 1
#define CHAS_DRV2_MONITOR		(1<<5)	// Chassis 3508 2
#define CHAS_DRV3_MONITOR		(1<<6)	// Chassis 3508 3
#define CHAS_RUD0_MONITOR		(1<<7)  // Steering Wheel 0
#define CHAS_RUD1_MONITOR		(1<<8)  // Steering Wheel 1
#define CHAS_RUD2_MONITOR		(1<<9)  // Steering Wheel 2
#define CHAS_RUD3_MONITOR		(1<<10) // Steering Wheel 3
#define REFEREE_MONITOR			(1<<11)	// Referee
#define SUPCAP_MONITOR			(1<<12)	// SupCap
#define POWERMETER_MONITOR		(1<<13) // Steering Wheel 3
#define COMMU_0X340_MONITOR		(1<<14) // Board Commu

#define AllDevices_MONITOR		(0x7FFF)// All Devices


#define On_line   0
#define Off_line  1

#define CRITICAL_VAL_INIT \
{\
    1,1,1,1,1,1,1,1,1,1,\
	1,1,1,1,1\
};

/* Private type --------------------------------------------------------------*/
typedef enum 
{
    Frame_DR16,
	Frame_CAN1,
	Frame_CAN2,
	Frame_CHAS_DRV0,
	Frame_CHAS_DRV1,
	Frame_CHAS_DRV2,
	Frame_CHAS_DRV3,
	Frame_CHAS_RUD0,
	Frame_CHAS_RUD1,
	Frame_CHAS_RUD2,
	Frame_CHAS_RUD3,
	Frame_REFEREE,
	Frame_SUPCAP,
	Frame_PowerMeter,
	Frame_COMMU_0X340,
	FrameCount_NUM

}FrameType_e;

typedef struct
{
	uint32_t Now;  //当前世界时间
	uint32_t Pre;  //上一次世界时间
}WorldTime_t;

extern uint16_t Critical_Val[FrameCount_NUM];        /*<! 掉帧临界值 */
extern uint32_t Devices_Frame;                         /*<! 用于检测设备是否离线 */	
extern uint16_t FrameCounter[FrameCount_NUM];         /*<! 将设备帧率存进数组 */

void Devices_Detec(void);
uint8_t Frame_Detec(uint16_t counter, uint16_t critical);
void DevicesUpdate(FrameType_e device);
uint8_t DevicesGet_State(uint32_t device);
uint16_t FPS_Calc(uint16_t delta_time);		
#endif

