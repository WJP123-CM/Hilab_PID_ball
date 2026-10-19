/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.c
 * @brief   This file provides code for the configuration
 *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>

static uint8_t uart1_rx_byte;

void USART1_StartRx(void) {
	HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		/* 只重启接收，不回显：回显字符会混入 VOFA+ 数据流，破坏 FireWater 帧解析 */
		HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
	}
}

/* ---- 标准库输出重定向到 USART1 ----
 * GCC newlib 的 printf/puts/putchar 最终走 syscalls.c 中的弱符号 _write()
 * （默认转发给空的 __io_putchar，输出静默丢失）。这里用强符号覆盖 _write：
 * 配合 stdout 行缓冲，一行一次整段阻塞发送，传输高效。
 * fputc 供 putchar() 直连，并兼容 Keil(ARMCC) 工程移植（ARMCC 的 printf 逐字符回调 fputc）。
 * 注意：newlib-nano 不支持 %f，打印只能用 %d/%u/%x/%s。 */
int _write(int file, char *ptr, int len) {
	(void) file;
	(void) HAL_UART_Transmit(&huart1, (uint8_t *) ptr, (uint16_t) len, 100);
	return len;
}

int fputc(int ch, FILE *f) {
	(void) f;
	uint8_t b = (uint8_t) ch;
	(void) HAL_UART_Transmit(&huart1, &b, 1U, 100);
	return ch;
}

/* VOFA+ FireWater 协议：ch0,ch1,...,chN，数字逗号分隔，以 \n 结尾（文本格式）。
 * 通道用整数而非 float：newlib-nano 无 float 格式化，%.2f 会静默输出空串。
 * 测距循环里只能调用本函数，且不要再发其他文本，避免干扰 VOFA+ 的帧解析。 */
void VOFA_SendFrame(const int32_t *data, uint8_t ch_num) {
	char	msg[64];
	char   *p = msg;
	uint8_t i;
	for (i = 0; i < ch_num; i++) {
		p += (uint16_t) snprintf(p, (size_t) (msg + sizeof(msg) - p), "%s%d", i ? "," : "", (int) data[i]);
	}
	(void) HAL_UART_Transmit(&huart1, (uint8_t *) msg, (uint16_t) (p - msg), 100);
	(void) HAL_UART_Transmit(&huart1, (uint8_t *) "\n", 1U, 100);
}
/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

