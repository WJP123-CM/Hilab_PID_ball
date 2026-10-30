/**
 ******************************************************************************
 * @file    tof.h
 * @brief   ToF 测距硬件模块（VL53L1X 驱动 + 逐帧滤波 + 有效帧统计）
 *
 * 这是"一个硬件一个文件"的收口模块：传感器初始化/连续测距读帧（原
 * vl53l1x.c）与测距数据处理链（原 tof_proc.c）同属 VL53L1X，合并于此，
 * 去掉原先两层之间的单语句转发。滤波算法本体仍独立在 tof_filter.c
 * （纯整数、无 HAL 依赖，可主机侧验证）。
 *
 * 职责边界：只管"测距数据本身"，不参与控制决策，也不输出串口帧。
 * 观测数据以只读快照 g_tof 暴露（写只发生在本文件），避免为每个字段
 * 提供一个薄 getter；20Hz 主循环调用 Tof_Tick() 推进，外环/OLED/VOFA 读快照。
 *
 * 硬件：I2C2（PB10=SCL，PB11=SDA），XSHUT 由平台层复位。
 ******************************************************************************
 */
#ifndef __TOF_H__
#define __TOF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tof_filter.h" /* 滤波算法与实例类型（实例由应用层持有） */

#include <stdbool.h>
#include <stdint.h>

/* 测距状态快照：本模块唯一对外数据出口（只读；写仅发生在 tof.c 的 Tof_Tick
 * 与 Tof_InitSensor）。全部在 20Hz 主循环上下文读写，无 ISR 访问，故不加 volatile。 */
typedef struct {
	int16_t		 raw_mm;	   /* 本帧原始距离（mm），无效帧为 0 */
	int16_t		 filt_mm;	   /* 滤波后距离（mm） */
	int16_t		 pos_cm_x10;   /* 相对零点的位置（0.1cm） */
	int8_t		 range_status; /* 0=有效；-1/-2=通信；-3=滤波量程拒绝 */
	uint8_t		 valid_pct;	   /* 最近 5s 窗口有效帧占比 0~100 */
	uint16_t	 alpha_milli;  /* 当前 EMA 系数（千分比） */
	const char	*last_step;	   /* 初始化失败步骤名；成功为 "OK" */
} TofState_t;

extern TofState_t g_tof;

/**
 * @brief 传感器初始化并启动连续测量（内部阻塞 ~数百 ms，分步喂狗）
 * @retval 0 = 成功；非 0 = VL53L1X 错误码（失败步骤名见 g_tof.last_step）
 * @note  本层不做任何提示输出/停机：显示与停机由应用层（main.c）决定
 */
int Tof_InitSensor(void);

/**
 * @brief 20Hz 主循环节拍：读一帧 → 滤波 → 更新快照 → 统计有效帧占比
 * @param f 滤波器实例（由应用层持有，须先 TofFilter_Init）
 * @note  无效帧不会清空滤波状态，输出保持上一有效值；本函数内含阻塞式
 *        等待（数据就绪最长 2s 超时），因此只能在主循环调用，禁止进 ISR
 */
void Tof_Tick(TofFilter_t *f);

/** @brief R 命令：清有效帧占比统计（滤波器自身状态由 TofFilter_Reset 清） */
void Tof_ResetStats(void);

#ifdef __cplusplus
}
#endif

#endif /* __TOF_H__ */
