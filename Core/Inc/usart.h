/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.h
 * @brief   This file contains all the function prototypes for
 *          the usart.c file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void USART1_StartRx(void);
/* printf/puts/putchar 已通过 _write/fputc 重定向到 USART1（见 usart.c） */
void VOFA_SendFrame(const int32_t *data, uint8_t ch_num);

/* RX 行协议参数：命令行最大长度（含 NUL），取行缓冲须不小于该值 */
#define UART1_LINE_MAX 32u
/* 取走一条已完成的命令行（以 \r/\n 结尾的行）到 buf；无完成行返回 false。
 * 供 main.c 10Hz 命令轮询调用（Cmd_Poll） */
bool USART1_GetLine(char *buf, uint8_t maxlen);
/* RX 停摆自愈：接收态异常时解锁并重挂中断接收（10Hz 轮询调用） */
void USART1_EnsureRx(void);
/* 诊断：RX ISR 累计收到的字节数（中断并发读，原子 32 位） */
extern volatile uint32_t uart1_rx_bytes;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

