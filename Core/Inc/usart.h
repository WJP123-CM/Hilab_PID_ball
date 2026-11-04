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

/* RX 单缓冲参数：行最大长度（含 NUL），外部读 uart1_last_line 时需注意 */
#define UART1_LINE_MAX 32u
/* RX 停摆自愈：接收态异常时解锁并重挂中断接收 */
void USART1_EnsureRx(void);

/* ---- RX 外部可见接口（极简单缓冲） ----
 * uart1_last_line : 最近一次收到的完整行（以 '\0' 结尾）
 * uart1_last_len  : 该行实际长度（不含 '\0'）
 * uart1_last_ready: ISR 写完置 1；主循环读完置 0 即可
 * uart1_rx_bytes  : RX ISR 累计收到的总字节数（诊断用）
 * 多帧间只保留最新一行，主循环 20ms 一次完全够用 */
extern volatile char	 uart1_last_line[UART1_LINE_MAX];
extern volatile uint8_t  uart1_last_len;
extern volatile uint8_t  uart1_last_ready;
extern volatile uint32_t uart1_rx_bytes;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

