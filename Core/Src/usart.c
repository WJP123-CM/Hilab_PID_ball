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

static uint8_t uart1_rx_byte;

/**
 * @brief 启动 USART1 单字节中断接收（RX 链起点）
 * @note  初始化阶段调用一次；每次接收完成后由回调自行重挂，形成连续 RX
 */
void USART1_StartRx(void) {
	HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
}

/* ---- RX 行组装：上位机命令行（PID 串口整定协议，解析在 main.c Cmd_ParseLine） ----
 * RX 中断逐字节接收，\r/\n 结束成行；完成行入 4 深度环形队列待主侧 10Hz 取走。
 * 噪声鲁棒：超长行截断（解析器因缺数字拒绝）；队列满时最新完成行丢弃。
 * （命令频率远低于 10Hz 轮询，正常不会发生；EMI 垃圾行被丢弃正是期望行为）。
 * 不回显：回显字符会混入 VOFA+ 数据流，破坏 FireWater 帧解析 */
static char		   uart1_line[UART1_LINE_MAX];	/* ISR 组装区 */
#define UART1_LINE_QDEPTH 4u /* 完成行队列深度（2 的幂，索引用掩码） */
static char		   uart1_line_q[UART1_LINE_QDEPTH][UART1_LINE_MAX]; /* 完成行环形队列 */
static uint8_t	   uart1_line_len	= 0;
static volatile uint8_t uart1_q_head = 0; /* 写入位置（生产者：RX ISR） */
static volatile uint8_t uart1_q_tail = 0; /* 取走位置（消费者：10Hz 轮询） */
volatile uint32_t		uart1_rx_bytes	= 0; /* 诊断：RX ISR 收到的总字节数（VOFA ch6） */

/**
 * @brief RX 完成回调：逐字节组装命令行，遇 \r/\n 成行入 4 深环形队列
 * @param huart 触发回调的串口句柄（仅处理 USART1，其它实例立即返回）
 * @note  ISR 上下文：不可阻塞；队列满时丢弃最新完成行（命令频率远低于
 *        10Hz 轮询，正常不发生）；超长行截断，断行会被解析器拒绝；
 *        不回显，避免字符混入 VOFA FireWater 数据流
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		char c = (char) uart1_rx_byte;
		uart1_rx_bytes++;
		if (c == '\r' || c == '\n') {
			if (uart1_line_len > 0u && uart1_line_len < UART1_LINE_MAX && (uint8_t) (uart1_q_head - uart1_q_tail) < UART1_LINE_QDEPTH) {
				char *dst = uart1_line_q[uart1_q_head & (UART1_LINE_QDEPTH - 1u)];
				for (uint8_t i = 0; i < uart1_line_len; i++) {
					dst[i] = uart1_line[i];
				}
				dst[uart1_line_len] = '\0';
				uart1_q_head++;
				/* 行入队完成 */
			}
			uart1_line_len = 0u;
		} else if (uart1_line_len < UART1_LINE_MAX - 1u) {
			uart1_line[uart1_line_len++] = c;
		} /* 超长行：丢字符直至行结束（截断行会被解析器拒绝） */
		/* 只重启接收，不回显 */
		HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
	}
}

/**
 * @brief 取走一条已完成的命令行（非阻塞）
 * @param buf 输出缓冲，容量须 ≥ UART1_LINE_MAX；maxlen 缓冲长度（字节）
 * @retval true = 取到一行（已 '\0' 结尾）；false = 当前无完成行
 * @note  仅主循环调用（单消费者），10Hz 命令轮询取行后交 Cmd_ParseLine
 */
bool USART1_GetLine(char *buf, uint8_t maxlen) {
	if (uart1_q_head == uart1_q_tail) {
		return false;
	}
	const char *src = uart1_line_q[uart1_q_tail & (UART1_LINE_QDEPTH - 1u)];
	for (uint8_t i = 0; i < maxlen; i++) {
		buf[i] = src[i];
		if (src[i] == '\0') {
			break;
		}
	}
	buf[maxlen - 1u] = '\0';
	uart1_q_tail++;
	return true;
}

/**
 * @brief RX 错误回调：强制解锁并重挂中断接收
 * @param huart 触发回调的串口句柄（仅处理 USART1）
 * @note  F1 HAL 错误路径进入本回调时 huart->Lock 可能仍被持有，直接重挂
 *        会因 HAL_BUSY 静默失败导致 RX 永久停摆，故先强制 Unlock/READY
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		/* F1 HAL 已知问题：错误路径调用本回调时 huart->Lock 可能仍被持有，
		 * 直接重挂会因 HAL_BUSY 静默失败 → RX 永久停摆（实测第一条命令后
		 * 全部丢失即此症）。此处强制解锁后重挂，恢复接收 */
		huart->Lock = HAL_UNLOCKED;
		huart->RxState = HAL_UART_STATE_READY;
		HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
	}
}

/**
 * @brief RX 通道自愈检查：接收态非 BUSY_RX 时解锁并重挂接收
 * @note  10Hz 命令轮询调用；覆盖 ORE/FE/噪声触发后 HAL 未自动恢复的停摆
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
 * @param data 通道值数组（整数，按 VOFA_CH_* 顺序）；ch_num 通道数（本工程 6）
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

