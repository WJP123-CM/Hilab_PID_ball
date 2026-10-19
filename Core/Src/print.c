/**
 ******************************************************************************
 * @file    print.c
 * @brief   双输出通道打印模块实现
 *          逻辑层统一格式化 + 设备发送表分发；驱动层在 usart.c / oled.c。
 *          新增输出设备：print_dev_t 加枚举 + 本文件发送表加钩子。
 ******************************************************************************
 */
#include "print.h"

#include "oled.h"  /* OLED_PrintLine */
#include "usart.h" /* huart1 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 设备发送钩子 */
static void serial_send(const char *s) {
	(void) HAL_UART_Transmit(&huart1, (uint8_t *) s, (uint16_t) strlen(s), 100);
}

static void oled_send(const char *s) {
	OLED_PrintLine(s);
}

typedef void (*print_send_t)(const char *s);

/* 设备发送表：下标与 print_dev_t 枚举一一对应 */
static const print_send_t s_send[PRINT_DEV_COUNT] = {
	[PRINT_DEV_SERIAL] = serial_send,
	[PRINT_DEV_OLED]   = oled_send,
};

/* 格式化核心：全模块唯一 vsnprintf 点，按设备分发 */
static void print_vto(print_dev_t dev, const char *fmt, va_list ap) {
	char	buf[64];
	va_list ap2;

	if ((unsigned) dev >= PRINT_DEV_COUNT) {
		return;
	}
	va_copy(ap2, ap);
	(void) vsnprintf(buf, sizeof(buf), fmt, ap2);
	va_end(ap2);
	s_send[dev](buf);
}

void print_to(print_dev_t dev, const char *fmt, ...) {
	va_list ap;

	va_start(ap, fmt);
	print_vto(dev, fmt, ap);
	va_end(ap);
}

void serial_print(const char *fmt, ...) {
	va_list ap;

	va_start(ap, fmt);
	print_vto(PRINT_DEV_SERIAL, fmt, ap);
	va_end(ap);
}

void oled_print(const char *fmt, ...) {
	va_list ap;

	va_start(ap, fmt);
	print_vto(PRINT_DEV_OLED, fmt, ap);
	va_end(ap);
}
