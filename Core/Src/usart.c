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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static uint8_t uart1_rx_byte;   /* HAL 单字节接收槽（ISR 内部用） */

/**
 * @brief 启动 USART1 单字节中断接收（RX 链起点）
 * @note  初始化阶段调用一次；每次接收完成后由回调自行重挂，形成连续 RX
 */
void USART1_StartRx(void) {
	HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
}

/* ---- RX 极简单缓冲方案 ----
 * RX ISR 逐字节填入 uart1_last_line，遇 \r/\n 结束一行并置 ready=1。
 * main 主循环看到 ready 就读、读完清 0；新行来之前覆盖旧行（最新即有效）。
 * 队满/丢包不再考虑：主循环 20ms 一次，单缓冲区够用。
 * 不回显：回显字符会混入 VOFA+ 数据流，破坏 FireWater 帧解析。 */
volatile char	   uart1_last_line[UART1_LINE_MAX] = {0};
volatile uint8_t  uart1_last_len				   = 0;
volatile uint8_t  uart1_last_ready				   = 0;
volatile uint32_t uart1_rx_bytes				   = 0; /* RX ISR 收到的总字节数 */

/**
 * @brief RX 完成回调：逐字节填入单缓冲，遇行尾置 ready=1
 * @param huart 触发回调的串口句柄（仅处理 USART1，其它实例立即返回）
 * @note  ISR 上下文：不可阻塞；超长行自动截断；不回显；15ms 自愈由 ErrorCallback + EnsureRx 负责
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		char c = (char) uart1_rx_byte;
		uart1_rx_bytes++;

		if (c == '\r' || c == '\n') {
			if (uart1_last_len == 0u) {
				(void) 0; /* \r\n 双序列：\r 已处理完一行，\n 跳过避免覆盖 */
			} else {
				uart1_last_line[uart1_last_len] = '\0';
				uart1_last_ready = 1u;
				uart1_last_len   = 0u;
			}
		} else if (uart1_last_len < UART1_LINE_MAX - 1u) {
			uart1_last_line[uart1_last_len++] = c;
		} else {
			/* 超长行截断：继续累加 len，但不写 buffer（防止溢出） */
			uart1_last_len++;
			if (uart1_last_len >= UART1_LINE_MAX + 8u) {
				/* 断行丢弃：连续超长，可能收到垃圾 */
				uart1_last_len = 0u;
			}
		}
		HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
	}
}

/**
 * @brief RX 错误回调：强制解锁并重挂中断接收
 * @param huart 触发回调的串口句柄（仅处理 USART1）
 * @note  F1 HAL 已知问题：错误路径进入本回调时 huart->Lock 可能仍被持有，直接重挂
 *        会因 HAL_BUSY 静默失败导致 RX 永久停摆，故先强制 Unlock/READY
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		huart->Lock		= HAL_UNLOCKED;
		huart->RxState	= HAL_UART_STATE_READY;
		HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
	}
}

/**
 * @brief RX 通道自愈检查：接收态非 BUSY_RX 时解锁并重挂接收
 * @note  主循环可调用；覆盖 ORE/FE/噪声触发后 HAL 未自动恢复的停摆
 */
void USART1_EnsureRx(void) {
	if (huart1.RxState != HAL_UART_STATE_BUSY_RX) {
		huart1.Lock		= HAL_UNLOCKED;
		huart1.RxState	= HAL_UART_STATE_READY;
		HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
	}
}

/**
 * @brief newlib 标准输出重定向：强符号覆盖 syscalls.c 中的弱符号 _write
 * @param file 未使用；ptr 数据首地址；len 字节数
 * @retval 固定返回 len（声明本次全部输出成功）
 * @note  阻塞式 HAL_UART_Transmit（最长 100ms）；仅主循环/非 ISR 使用；
 *        与 VOFA 帧共用 USART1，勿在测距帧附近混用文本输出；newlib-nano 不支持 %f
 */
int _write(int file, char *ptr, int len) {
	(void) file;
	(void) HAL_UART_Transmit(&huart1, (uint8_t *) ptr, (uint16_t) len, 100);
	return len;
}

/**
 * @brief 逐字符输出（供 putchar 与 ARMCC printf 移植兼容）
 * @param ch 待发送字符；f 未使用
 * @retval 返回 ch
 * @note  阻塞发送单字节；newlib-nano 不支持 %f
 */
int fputc(int ch, FILE *f) {
	(void) f;
	uint8_t b = (uint8_t) ch;
	(void) HAL_UART_Transmit(&huart1, &b, 1U, 100);
	return ch;
}

/**
 * @brief 寄存器级阻塞发送（不经 HAL、不持有 huart 锁）
 * @param buf 数据首地址；len 字节数（VOFA 帧 ≤ 64，调用方保证）
 * @note  ISR（20Hz TIM1 遥测）安全：若走 HAL_UART_Transmit，每 20ms 会长时间
 *        持有 huart1->Lock，RX 重挂因锁返回 BUSY 且不使能 RXNE → RX 随机死亡；
 *        寄存器直发与 RX 路径零共享状态；发送期间忙等 CPU（整帧约数 ms）
 */
static void uart_tx_poll(const uint8_t *buf, uint16_t len) {
	USART1->CR1 |= USART_CR1_UE; /* 确保使能（初始化后恒为 1，防御性） */
	for (uint16_t i = 0; i < len; i++) {
		while (!(USART1->SR & USART_SR_TXE)) {
		}
		USART1->DR = buf[i];
	}
	while (!(USART1->SR & USART_SR_TC)) {
	}
}

/**
 * @brief 发送 VOFA+ FireWater 文本帧：ch0,ch1,...,chN\n
 * @param data 通道值数组（整数，按 VOFA_CH_* 顺序）；ch_num 通道数（本工程 3）
 * @retval 无
 * @note  通道用整数（newlib-nano 无 %f，%.2f 会输出空串）；仅 20Hz 主循环调用；
 *        同一时刻不得再发其它文本，避免破坏 VOFA 帧解析
 */
void VOFA_SendFrame(const int32_t *data, uint8_t ch_num) {
	char	msg[64];
	char   *p = msg;
	uint8_t i;
	for (i = 0; i < ch_num; i++) {
		p += (uint16_t) snprintf(p, (size_t) (msg + sizeof(msg) - p), "%s%d", i ? "," : "", (int) data[i]);
	}
	*p++ = '\n';
	uart_tx_poll((const uint8_t *) msg, (uint16_t) (p - msg));
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
