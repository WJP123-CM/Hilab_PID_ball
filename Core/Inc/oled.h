/**
 ******************************************************************************
 * @file    oled.h
 * @brief   OLED(SSD1306) 驱动封装层：初始化、文本打印、区域清除/回写。
 *
 * 职责边界：只提供屏幕原语，不含页面文案与业务数据；页面组合由应用层
 * （main.c）调用本层接口完成，避免底层文件里出现固定业务字符。
 * 底层绘制交给 ssd1306 库（I2C2，PB10=SCL, PB11=SDA）。
 ******************************************************************************
 */
#ifndef __OLED_H__
#define __OLED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 屏幕尺寸（像素） */
#define OLED_WIDTH	128u
#define OLED_HEIGHT 64u

/**
 * @brief  OLED 底层初始化：ssd1306_Init + 清屏。不涉及页面/进度条逻辑。
 *         由 main 初始化阶段显式调用一次。
 */
void OLED_Init(void);

/**
 * @brief  printf 风格文本打印：在 (x,y)（像素坐标）处显示一行，字符 6x8。
 * @note   只写入显存并置脏页标记，不发 I2C；由 OLED_Tick 逐页回写
 * @param  x  列（0~127，像素）
 * @param  y  行（0~63，像素）
 * @param  fmt  printf 格式串
 */
void OLED_Print(uint8_t x, uint8_t y, const char *fmt, ...);

/** @brief 清除矩形区域（x,y 左上角，w/h 尺寸），同时置对应脏页 */
void OLED_ClearArea(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

/**
 * @brief  立即整块回写显存区域（阻塞 I2C）。
 * @note   仅在初始化阶段使用；日常刷新请改用 OLED_Tick
 */
void OLED_Flush(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

/**
 * @brief  逐页刷新节拍：每次最多回写一页（128x8 像素），无脏页时直接返回。
 * @note   由主循环按 25ms 节拍调用，整屏 8 页 200ms 刷完
 */
void OLED_Tick(void);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H__ */
