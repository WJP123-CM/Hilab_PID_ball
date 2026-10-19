/**
 * SSD1306 配置（Hilab_PID 工程）
 * - STM32F103C8T6 (HAL 库)
 * - I2C2 (PB10=SCL, PB11=SDA)，0.96 寸 128x64
 * - 仅启用 6x8 与 16x26 两套字体以节省 Flash
 */

#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

// 芯片系列
#define STM32F1

// 总线：I2C
#define SSD1306_USE_I2C

// I2C 配置：本工程 OLED 挂在 I2C2 上
#define SSD1306_I2C_PORT        hi2c2
#define SSD1306_I2C_ADDR        (0x3C << 1)

// 屏幕尺寸：128x64（默认值，0.96 寸），无需修改
// #define SSD1306_WIDTH           128
// #define SSD1306_HEIGHT          64

// 仅启用需要的字体
#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_16x26

#endif /* __SSD1306_CONF_H__ */
