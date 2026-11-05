/**
 ******************************************************************************
 * @file    oled.c
 * @brief   OLED(SSD1306) 驱动封装层：初始化、文本打印、区域清除/回写。
 *
 * 职责边界：本文件只提供"屏幕原语"，不含任何页面文案/业务数据绑定——
 * 页面内容由应用层（main.c）组合调用本层接口完成。
 * 底层绘制交给 ssd1306 库（I2C2，PB10=SCL, PB11=SDA）。
 *
 * 刷新策略：文本/清除只写显存并置脏页标记（不阻塞 I2C）；真正的 I2C 回写
 * 由 OLED_Tick 每拍只送一页完成，把一次整屏刷新（8 页）拆成 8 拍，
 * 8 拍 × 25ms = 200ms，与页面内容更新周期对齐，降低单次阻塞时间。
 * OLED_Flush 保留为整块立即回写（仅初始化阶段使用）。
 ******************************************************************************
 */
#include "oled.h"

#include "ssd1306.h"
#include "ssd1306_fonts.h"

#include <stdarg.h>
#include <stdio.h>

/* 1 页 = 8 像素高，128x64 共 8 页 */
#define OLED_PAGE_COUNT ((uint8_t) (OLED_HEIGHT / 8u))
#define OLED_PAGE_HEIGHT 8u

/* 脏页位图：bit n 置位表示第 n 页显存已修改、待回写 */
static uint8_t s_dirty_pages;
/* 轮转起点：每拍从上次刷完的下一页开始找，避免总是先刷第 0 页 */
static uint8_t s_next_page;

/**
 * @brief 把纵向区间 [y, y+h) 覆盖的显存页置为脏页
 * @param y 起始行（像素）
 * @param h 高度（像素，≥1）
 * @note  越界部分自动钳到最后一页，不会写出页范围
 */
static void oled_mark_dirty(uint8_t y, uint8_t h) {
	uint8_t page0 = (uint8_t) (y >> 3);
	uint8_t page1 = (uint8_t) ((y + h - 1u) >> 3);
	uint8_t p;

	if (page1 >= OLED_PAGE_COUNT) {
		page1 = OLED_PAGE_COUNT - 1u;
	}

	for (p = page0; p <= page1; p++) {
		s_dirty_pages |= (uint8_t) (1u << p);
	}
}

/**
 * @brief OLED 底层初始化：ssd1306_Init + 清屏（显存与屏幕均清空）
 * @note  初始化阶段调用一次；清空后脏页标记全部复位
 */
void OLED_Init(void) {
	ssd1306_Init();
	ssd1306_Fill(Black);
	s_dirty_pages = 0;
	s_next_page	 = 0;
}

/* ===================== 公共接口 =========================================== */

/**
 * @brief printf 风格文本打印：在 (x,y) 显示一行 6x8 字符
 * @param x 列坐标（像素，0~127）；y 行坐标（像素，0~63）
 * @param fmt printf 格式串（newlib-nano 不支持 %f）
 * @note  只写显存并置脏页，不发 I2C；由 OLED_Tick 逐页回写；
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

	/* Font_6x8 行高固定 8 像素，正好一页 */
	oled_mark_dirty(y, OLED_PAGE_HEIGHT);
}

/**
 * @brief 清除显存中的矩形区域（填黑）
 * @param x,y 左上角像素坐标；w,h 宽高（像素，均 ≥1）
 * @note  只清显存并置脏页；由 OLED_Tick 逐页回写
 */
void OLED_ClearArea(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
	ssd1306_FillRectangle(x, y, (uint8_t) (x + w - 1U), (uint8_t) (y + h - 1U), Black);
	oled_mark_dirty(y, h);
}

/**
 * @brief 立即整块回写显存区域（I2C2 阻塞传输）
 * @param x,y 左上角像素坐标；w,h 宽高（像素）
 * @note  阻塞耗时与页数成正比，仅在初始化阶段（OLED_Tick 尚未运行）使用；
 *        回写完成后对应页的脏标记一并清除，避免重复发送
 */
void OLED_Flush(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
	uint8_t page0 = (uint8_t) (y >> 3);
	uint8_t page1 = (uint8_t) ((y + h - 1u) >> 3);
	uint8_t p;

	if (page1 >= OLED_PAGE_COUNT) {
		page1 = OLED_PAGE_COUNT - 1u;
	}

	ssd1306_UpdateRect(x, y, w, h);

	for (p = page0; p <= page1; p++) {
		s_dirty_pages &= (uint8_t) ~(1u << p);
	}
}

/**
 * @brief 逐页刷新节拍：每调用一次最多回写一页（128x8 像素）
 * @note  非阻塞页级步进，由主循环按 25ms 节拍调用，
 *        整屏 8 页 200ms 完成；无脏页时立即返回，不产生任何 I2C 开销
 */
void OLED_Tick(void) {
	uint8_t i;

	for (i = 0; i < OLED_PAGE_COUNT; i++) {
		uint8_t p = (uint8_t) ((s_next_page + i) % OLED_PAGE_COUNT);

		if ((s_dirty_pages & (uint8_t) (1u << p)) != 0u) {
			s_dirty_pages &= (uint8_t) ~(1u << p);
			s_next_page = (uint8_t) ((p + 1u) % OLED_PAGE_COUNT);
			ssd1306_UpdateRect(0, (uint8_t) (p * OLED_PAGE_HEIGHT), OLED_WIDTH, OLED_PAGE_HEIGHT);
			return;
		}
	}
}
