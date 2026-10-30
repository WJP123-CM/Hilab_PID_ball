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

/* ===================== 可配置参数 =========================================
 * 传感器测距参数（TofSensorCfg_t）：必须在 Tof_InitSensor 时传入，运行时改
 *   需停止测量重新配置（本模块不提供运行时改传感器参数的接口，避免打断测距）。
 * 运行时参数（TofRuntimeCfg_t）：可随时通过 Tof_SetRuntimeCfg 在线调整，
 *   包括滤波 EMA、零点、有效距离门限、有效帧统计窗口。
 * ======================================================================== */

/**
 * @brief VL53L1X 传感器测距参数（初始化期配置，改后需重新 Tof_InitSensor 生效）
 */
typedef struct {
	uint8_t  roi_tl_x;       /* ROI 左上角 X（0~15） */
	uint8_t  roi_tl_y;       /* ROI 左上角 Y（≥ roi_br_y，y 轴反向） */
	uint8_t  roi_br_x;       /* ROI 右下角 X（≥ roi_tl_x） */
	uint8_t  roi_br_y;       /* ROI 右下角 Y（≤ roi_tl_y） */
	uint8_t  distance_mode;  /* 0=SHORT(250mm)，1=LONG(400mm+) */
	uint16_t timing_budget_us; /* 单帧测量预算（μs，须 ≤ 帧周期−4ms） */
	uint16_t inter_period_ms;  /* 连续测距帧周期（ms，须 ≥ 预算+4ms） */
	uint8_t  xtalk_enable;   /* 串扰补偿使能：0=关，1=开 */
} TofSensorCfg_t;

/**
 * @brief 运行时可调整参数（Tof_SetRuntimeCfg 在线生效，不需重启测距）
 */
typedef struct {
	uint16_t alpha_milli;    /* EMA 系数千分比（20~1000） */
	int16_t  zero_mm;        /* 零点距离（mm） */
	int16_t  min_valid_mm;   /* 最小有效距离门限（mm） */
	int16_t  max_valid_mm;   /* 最大有效距离门限（mm） */
	uint32_t valid_window_ms; /* 有效帧占比统计窗口（ms） */
} TofRuntimeCfg_t;

/** @brief 填入默认传感器配置（8×8 ROI / SHORT / 16ms 预算 / 20ms 周期 / 串扰关） */
void Tof_GetDefaultSensorCfg(TofSensorCfg_t *cfg);

/** @brief 填入当前运行时参数（从 g_tof 与滤波器实例快照） */
void Tof_GetRuntimeCfg(TofRuntimeCfg_t *cfg, const TofFilter_t *f);

/* 测距状态快照：本模块唯一对外数据出口（只读；写仅发生在 tof.c 的 Tof_Tick
 * 与 Tof_InitSensor）。全部在 20Hz 主循环上下文读写，无 ISR 访问，故不加 volatile。 */
typedef struct {
	int16_t		 raw_mm;	   /* 本帧原始距离（mm），无效帧为 0 */
	int16_t		 filt_mm;	   /* 滤波后距离（mm） */
	int16_t		 pos_cm_x10;   /* 相对零点的位置（0.1cm） */
	int8_t		 range_status; /* 0=有效；-1/-2=通信；-3=滤波量程拒绝 */
	uint8_t		 valid_pct;	   /* 最近窗口有效帧占比 0~100 */
	uint16_t	 alpha_milli;  /* 当前 EMA 系数（千分比） */
	const char	*last_step;	   /* 初始化失败步骤名；成功为 "OK" */
} TofState_t;

extern TofState_t g_tof;

/**
 * @brief 传感器初始化并启动连续测量（内部阻塞 ~数百 ms，分步喂狗）
 * @param cfg 传感器测距参数（传 NULL 则用默认配置）
 * @retval 0 = 成功；非 0 = VL53L1X 错误码（失败步骤名见 g_tof.last_step）
 * @note  本层不做任何提示输出/停机：显示与停机由应用层（main.c）决定
 */
int Tof_InitSensor(const TofSensorCfg_t *cfg);

/**
 * @brief 20Hz 主循环节拍：读一帧 → 滤波 → 更新快照 → 统计有效帧占比
 * @param f 滤波器实例（由应用层持有，须先 TofFilter_Init）
 * @note  无效帧不会清空滤波状态，输出保持上一有效值；本函数内含阻塞式
 *        等待（数据就绪最长 2s 超时），因此只能在主循环调用，禁止进 ISR
 */
void Tof_Tick(TofFilter_t *f);

/**
 * @brief 在线更新运行时参数（EMA/零点/有效距离范围/统计窗口）
 * @param cfg 新参数（字段为 0 的项保持不变，仅覆盖显式设置的字段）
 * @param f   滤波器实例（由应用层持有）
 * @note  测量类参数（ROI/模式/预算/帧周期）不在此接口，需重新 Tof_InitSensor
 */
void Tof_SetRuntimeCfg(const TofRuntimeCfg_t *cfg, TofFilter_t *f);

/** @brief R 命令：清有效帧占比统计（滤波器自身状态由 TofFilter_Reset 清） */
void Tof_ResetStats(void);

#ifdef __cplusplus
}
#endif

#endif /* __TOF_H__ */
