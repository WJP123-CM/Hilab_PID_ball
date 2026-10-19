/**
 ******************************************************************************
 * @file    print.h
 * @brief   双输出通道打印模块：统一格式化接口，按设备分发输出
 *          serial_print -> USART1 串口（立即整段发送，不经行缓冲）
 *          oled_print   -> SSD1306 OLED（光标行自动换行显示）
 *          注意：newlib-nano 不支持 %f；测距循环内严禁调用（污染 VOFA+ 数据流）。
 ******************************************************************************
 */
#ifndef __PRINT_H__
#define __PRINT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 输出设备类型：新增设备时在此追加，并保持与 print.c 发送表顺序一致 */
typedef enum {
	PRINT_DEV_SERIAL = 0, /* USART1 串口 */
	PRINT_DEV_OLED,		  /* SSD1306 OLED */
	PRINT_DEV_COUNT
} print_dev_t;

/* 核心分发：格式化后输出到指定设备 */
void print_to(print_dev_t dev, const char *fmt, ...);
/* 串口打印（printf 风格） */
void serial_print(const char *fmt, ...);
/* OLED 行打印（printf 风格，单行建议 ≤ 21 字符） */
void oled_print(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* __PRINT_H__ */
