/**
 ******************************************************************************
 * @file    vl53l1x.h
 * @brief   VL53L1X ToF 测距传感器的简洁封装接口
 *          把官方 Full API 的冗长初始化序列和连续测距读帧逻辑封装成
 *          Init / Read 两个调用，main 里只管"初始化 + 周期读"。
 ******************************************************************************
 */
#ifndef __VL53L1X_H
#define __VL53L1X_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vl53l1_error_codes.h" /* VL53L1_Error / VL53L1_ERROR_NONE 等 */

/**
 * @brief  完整初始化 VL53L1X 并启动连续测量。
 *         内部依次完成：平台复位 → 等待固件启动 → DataInit/StaticInit
 *         → RefSPAD 管理 → 关串扰 → SHORT 距离模式 → 20ms 测量预算
 *         → 20ms 帧周期 → StartMeasurement。
 * @retval VL53L1_ERROR_NONE 成功；其他值为失败的具体错误码，
 *         失败步骤名可通过 VL53L1X_LastErrorStep() 查询。
 */
VL53L1_Error VL53L1X_Init(void);

/**
 * @brief  阻塞读取一帧测距结果，并自动清除中断、启动下一帧测量。
 *         启动后的第一帧会被内部丢弃（官方建议）。
 * @param  distance_mm  : 输出，经中值滤波(窗口5)后的距离（mm）。传感器上报无效时
 *                        保持上一次滤波输出，需结合 range_status 判断是否可信。
 * @param  range_status : 输出，状态码。
 *         - 0    : 数据有效（VL53L1_RANGESTATUS_RANGE_VALID）
 *         - -1   : 等待数据就绪超时
 *         - -2   : 读取测量数据失败
 *         - 其他 : 传感器上报的测距状态（信号弱/超量程等），距离值仅供参考
 * @retval VL53L1_ERROR_NONE 正常取到一帧（即使 range_status 非 0 也算成功取到）；
 *         其他值为底层通信错误。
 */
VL53L1_Error VL53L1X_Read(int16_t *distance_mm, int8_t *range_status);

/**
 * @brief  返回最近一次 VL53L1X_Init() 失败时的步骤名称字符串。
 *         初始化成功时返回 "OK"。用于串口打印定位问题。
 */
const char *VL53L1X_LastErrorStep(void);

#ifdef __cplusplus
}
#endif

#endif /* __VL53L1X_H */
