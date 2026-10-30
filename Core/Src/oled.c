/**
 ******************************************************************************
 * @file    oled.c
 * @brief   OLED(SSD1306) 驱动封装层：初始化、文本打印、区域清除/回写。
 *
 * 职责边界：本文件只提供"屏幕原语"，不含任何页面文案/业务数据绑定——
 * 页面内容由应用层（main.c）组合调用本层接口完成。
 * 底层绘制交给 ssd1306 库（I2C2，PB10=SCL, PB11=SDA）。
 ******************************************************************************
 */
#include "oled.h"

#include "ssd1306.h"
#include "ssd1306_fonts.h"

#include <stdarg.h>
#include <stdio.h>

/**
 * @brief OLED 底层初始化：ssd1306_Init + 清屏（显存与屏幕均清空）
 * @note  初始化阶段调用一次；不涉及页面文案/进度条逻辑
 */
void OLED_Init(void) {
	ssd1306_Init();
	ssd1306_Fill(Black);
}

/* ===================== 公共接口 =========================================== */

/**
 * @brief printf 风格文本打印：在 (x,y) 显示一行 6x8 字符
 * @param x 列坐标（像素，0~127）；y 行坐标（像素，0~63）
 * @param fmt printf 格式串（newlib-nano 不支持 %f）
 * @note  只写显存不发 I2C；调用方随后用 OLED_Flush 回写对应区域；
 *        单行缓冲 32 字节，超长内容被截断
 */
void OLED_Print(uint8_t x, uint8_t y, const char *fmt, ...) {
	char	buf[32];
	va_list ap;

	va_start(ap, fmt);
	(void) vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	ssd1306_SetCursor(x, y);
	ssd1306_WriteString(buf, Font_6x8, White);
}

/**
 * @brief 清除显存中的矩形区域（填黑）
 * @param x,y 左上角像素坐标；w,h 宽高（像素，均 ≥1）
 * @note  只清显存；需再对同区域 OLED_Flush 才反映到屏幕
 */
void OLED_ClearArea(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
	ssd1306_FillRectangle(x, y, (uint8_t) (x + w - 1U), (uint8_t) (y + h - 1U), Black);
}

/**
 * @brief 把显存中的矩形区域回写到屏幕（I2C2 阻塞传输）
 * @param x,y 左上角像素坐标；w,h 宽高（像素）
 * @note  传输耗时与区域面积成正比；5Hz 页面刷新只回写改动区
 */
void OLED_Flush(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
	ssd1306_UpdateRect(x, y, w, h);
}
