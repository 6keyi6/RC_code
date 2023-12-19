#ifndef __bsp_usart_H
#define __bsp_usart_H
//#include "include.h"
#include "usart.h"
#include "Remote_Control.h"
HAL_StatusTypeDef UART_Receive_DMA_NoIT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT_IDLE(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void HAL_UART_IDLE_IRQHandler(UART_HandleTypeDef *huart);

extern void _Error_Handler(char *, int);

extern uint8_t UART_Buffer[100];
#endif 
