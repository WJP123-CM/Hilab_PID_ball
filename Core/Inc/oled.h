/**
 ******************************************************************************
 * @file    oled.h
 * @brief   OLED(SSD1306) 基础驱动封装：初始化 + 文本打印。
 *          首页绘制与进度条刷新逻辑已在 main 内联，本头文件只暴露底层原语。
 *          底层绘制交给 ssd1306 库（I2C2，PB10=SCL, PB11=SDA）
 ******************************************************************************
 */
#ifndef __OLED_H__
#define __OLED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief  OLED 底层初始化：ssd1306_Init + 清屏。不涉及页面/进度条逻辑。
 *         由 main 初始化阶段显式调用一次。
 */
void OLED_Init(void);

/**
 * @brief  printf 风格文本打印：在 (x,y)（像素坐标）处显示一行，字符 6x8。
 * @param  x  列（0~127，像素）
 * @param  y  行（0~63，像素）
 * @param  fmt  printf 格式串
 */
void OLED_Print(uint8_t x, uint8_t y, const char *fmt, ...);

/**
 * @brief  行打印：将字符串显示在自动递增的光标行（0~7 行，6x8 字体循环），
 *         先清本行再写入并整屏刷新。供 print 模块的 oled_print 使用。
 * @param  s  待显示字符串（超出宽度自动截断）
 */
void OLED_PrintLine(const char *s);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H__ */
