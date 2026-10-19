/*******************************************************************************
 * VL53L1X STM32F1xx Platform Layer - Header
 * 对应替换官方的 vl53l1_platform.c / vl53l1_platform_init.c 声明
 *
 * 依赖（CubeMX 已配好）：
 *   - I2C1 : PB6=SCL, PB7=SDA, 句柄 hi2c1  (见 i2c.h)
 *   - 一块 IO 脚做 XSHUT 复位脚
 *
 * 使用：在 main.c 里
 *   #include "vl53l1_platform_stm32.h"
 *   static VL53L1_Dev_t dev;
 *   VL53L1_platform_init(&dev, VL53L1X_I2C_ADDRESS, VL53L1_I2C, 100);
 *   VL53L1_WaitDeviceBooted(&dev);
 *   VL53L1_DataInit(&dev);
 *   ... 之后照常调用 vl53l1_api.h 里的函数
 ******************************************************************************/
#ifndef _VL53L1_PLATFORM_STM32_H_
#define _VL53L1_PLATFORM_STM32_H_

#include "vl53l1_platform.h"
#include "vl53l1_platform_init.h"
#include "vl53l1_ll_device.h"   /* VL53L1_I2C / VL53L1_SPI */

#ifdef __cplusplus
extern "C" {
#endif

/* VL53L1X 的 I2C 设备地址：7 位地址 0x29 左移 1 位 = 0x52（HAL 的 DevAddress 格式）。
 * 平台层直接把它传给 HAL_I2C_Mem_Read/Write，不要再做移位 */
#define VL53L1X_I2C_ADDRESS     ((uint8_t)0x52)

/* I2C 总线软件恢复：出错/数据过多超时(静默卡死)时调用，避免 HAL 轮询 I2C 永久挂死。
 * 在平台内部 ReadMulti/WriteMulti 出错时自动调用；上层(VL53L1X_Read)
 * 也可在等待数据就绪超时时显式调用以覆盖"总线没报错但数据迟迟不就绪"的情况。 */
void VL53L1X_I2C_Recover(void);

#ifdef __cplusplus
}
#endif

#endif /* _VL53L1_PLATFORM_STM32_H_ */