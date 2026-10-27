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
 * @param  distance_mm  : 输出，传感器本帧原始距离（mm），**未经任何滤波**；
 *                        无效帧的数值不可信，须结合 range_status 判断，
 *                        滤波由上层（PID 环节）自行处理。
 * @param  range_status : 输出，状态码。
 *         - 0    : 有效（RANGE_VALID）
 *         - 1    : Sigma 失败（SIGMA_FAIL，测量离散度过大）
 *         - 2    : 信号失败（SIGNAL_FAIL，回波过弱）
 *         - 3    : 低于最小量程被裁剪（RANGE_VALID_MIN_RANGE_CLIPPED）
 *         - 4    : 相位越界（OUTOFBOUNDS_FAIL）
 *         - 5    : 硬件失败（HARDWARE_FAIL，片内自检/看门狗报错，偶发可忽略）
 *         - 6    : 有效但未做绕回检查（RANGE_VALID_NO_WRAP_CHECK_FAIL）
 *         - 7    : 绕回/相位一致性失败（WRAP_TARGET_FAIL，回波弱/超量程/有串扰）
 *         - 8    : 内部算法下溢/溢出（PROCESSING_FAIL）
 *         - 9    : 串扰信号失败（XTALK_SIGNAL_FAIL）
 *         - 10   : 启动首帧（SYNCRONISATION_INT，忽略数据）
 *         - 11   : 脉冲合并（RANGE_VALID_MERGED_PULSE）
 *         - 12   : 有目标但缺信号（TARGET_PRESENT_LACK_OF_SIGNAL）
 *         - 13   : ROI 无效（MIN_RANGE_FAIL）
 *         - 14   : 负距离无效（RANGE_INVALID）
 *         - 255  : 无更新（NONE）
 *         - -1   : 等待数据就绪超时
 *         - -2   : 读取测量数据失败
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
