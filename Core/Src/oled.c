/**
 ******************************************************************************
 * @file    oled.c
 * @brief   OLED(SSD1306) 基础驱动封装：初始化 + 文本打印。
 *          首页绘制与进度条刷新逻辑已内联到 main（主循环集中管理），本文件只保留底层原语。
 ******************************************************************************
 */
#include "oled.h"

#include "ssd1306.h"
#include "ssd1306_fonts.h"

#include <stdarg.h>
#include <stdio.h>

/* ===================== OLED 初始化 ======================================== */
/* 职责：底层驱动初始化 + 清屏。不涉及任何页面/进度条逻辑。 */
void OLED_Init(void) {
	ssd1306_Init();
	ssd1306_Fill(Black);
}

/* ===================== 公共接口 =========================================== */

void OLED_Print(uint8_t x, uint8_t y, const char *fmt, ...) {
	char	buf[32];
	va_list ap;

	va_start(ap, fmt);
	(void) vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	ssd1306_SetCursor(x, y);
	ssd1306_WriteString(buf, Font_6x8, White);
}

/* OLED_PrintLine 光标行（0~7，6x8 字体共 8 行，循环覆盖） */
static uint8_t s_line_row = 0;

void OLED_PrintLine(const char *s) {
	uint8_t y = (uint8_t) (s_line_row * 8U);

	/* 先清整行再写入，避免上次较长文本残留；ssd1306_WriteString 超宽自动截断 */
	ssd1306_FillRectangle(0, y, SSD1306_WIDTH - 1, y + 7, Black);
	OLED_Print(0, y, "%s", s);
	ssd1306_UpdateScreen();
	s_line_row = (uint8_t) ((s_line_row + 1U) % 8U);
}
