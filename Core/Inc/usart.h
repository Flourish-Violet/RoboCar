/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   USART1 外设配置头文件，用于 ROS 串口通信
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __USART_H__
#define __USART_H__

#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart1;

void MX_USART1_UART_Init(void);

#endif /* __USART_H__ */
