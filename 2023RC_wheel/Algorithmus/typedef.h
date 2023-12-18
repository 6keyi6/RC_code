#ifndef  __TYPEDEF_H
#define  __TYPEDEF_H
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h> 
#include "can.h"
/*************CAN对外数据接口************/
//typedef struct
//{
//    CAN_RxHeaderTypeDef CAN_RxHeader;
//    uint8_t CANx_Export_RxMessage[8];
//} Can_Export_Data_t;

///*************CAN发送数据接口************/
//typedef struct
//{
//    CAN_HandleTypeDef *Canx;
//    CAN_TxHeaderTypeDef CAN_TxHeader;
//    uint8_t CANx_Send_RxMessage[8];
//} Can_Send_Data_t;
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

#endif
