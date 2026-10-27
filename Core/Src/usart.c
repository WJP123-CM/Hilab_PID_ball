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

void USART1_StartRx(void) {
	HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
}

/* ---- RX 行组装：上位机命令行（PID 串口整定协议，解析在 main.c PidTuner） ----
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

/* 取走一条完成行（无行返回 false）；buf 容量须 ≥ UART1_LINE_MAX */
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

/* RX 通道自愈检查（10Hz 由 PidTuner_Poll 调用）：任何异常路径导致接收停摆
 * （ORE/FE/噪声触发后 HAL 未自动恢复）都在这里重新挂起接收 */
void USART1_EnsureRx(void) {
	if (huart1.RxState != HAL_UART_STATE_BUSY_RX) {
		huart1.Lock		= HAL_UNLOCKED;
		huart1.RxState	= HAL_UART_STATE_READY;
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

/* 寄存器级阻塞发送：不经 HAL，不持 huart 锁。
 * 关键原因：VOFA 帧在 TIM1 中断里每 20ms 发送一次（~5ms 忙），若走
 * HAL_UART_Transmit 会长时间持有 huart1->Lock，RX 回调内的重挂
 * HAL_UART_Receive_IT 会因锁返回 BUSY 且不使能 RXNE —— RX 随机死亡
 * （实测每行命令后必丢）。寄存器直发与 RX 路径零共享状态，彻底解耦 */
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

