/**
 ******************************************************************************
 * @file    tof_filter.h
 * @brief   VL53L1X 原始测距专用数字滤波器（中值-5 + 一阶 EMA）
 *
 *          设计依据（不采用卡尔曼，卡尔曼留待后续独立模块对比）：
 *            - VL53L1X 原始帧存在偶发孤立跳变/弱回波，先用 5 点中值剔除；
 *            - 静止/低速段主要误差为量化与随机噪声，用一阶 EMA 平滑；
 *            - 全部为整数运算，不依赖 libm/HAL，可在 20Hz 主循环直接调用；
 *            - alpha 以千分比表示（300 = 0.300），可由上位机 A 命令在线修改。
 *
 *          坐标约定：
 *            pos_cm_x10 = filtered_mm - zero_mm
 *            即“滤波后距离相对零点的偏移”，单位 0.1cm；远离传感器为正；
 *            zero_mm 由 Z 命令标定。此处的符号不再作为方向开关，
 *            外环方向统一由 main.c 的 BALL_X_ENC_SIGN 决定。
 ******************************************************************************
 */
#ifndef __TOF_FILTER_H__
#define __TOF_FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "kalman.h" /* KF 主链；EMA 保留为回退保险 */

/* 中值窗口长度：3 点可剔除单帧孤立尖峰，群延迟 1 帧（50Hz 下 20ms） */
#define TOF_FILTER_MEDIAN_N 3u

/* EMA 系数默认值：800 = 0.80；越大响应越快、噪声抑制越弱，τ≈40ms（50Hz） */
#define TOF_FILTER_ALPHA_DEFAULT_MILLI 800u
#define TOF_FILTER_ALPHA_MIN_MILLI     20u
#define TOF_FILTER_ALPHA_MAX_MILLI     1000u

/* 有效原始距离范围（mm）：Short 模式标称 0~250mm；工程有效测试点最近 3~5cm。
 * 10mm 仅作"明显异常值"门限，不代表 0~2cm 可用。运行时可通过
 * TofFilter_SetValidRange 覆盖，默认值取自下方宏。 */
#define TOF_FILTER_MIN_VALID_MM 8
#define TOF_FILTER_MAX_VALID_MM 500

/* 默认零点：仅用于上电显示；正式测试必须用 Z 命令标定 */
#define TOF_FILTER_ZERO_DEFAULT_MM 131  /* 2026-10-29 实测静止零点：60s 930帧 mean=131.44mm, σ=1.87mm */

/* ===================== KF 主链参数（EMA 并行作对照） =====================
 * 接入方式：EMA 链照旧运行，仅作对比与遥测；输出始终取 KF，不切换。
 * 依据（2026-11-06 实测）：自动回退只会在传感器盲区（raw 10~14mm）触发，
 * 那里 EMA 与 KF 都不准，切换既不能救回精度，还会把 KF 状态拖到陈旧值。
 * 故去掉切换，只保留 kf_state 上报偏差/失观测计数，供排查。
 * KF 自身已有三层保护：输入有效性门限、kf_guard 发散复位、速度限幅。
 * 主机侧实测（34ms 采样，1.87mm 观测噪声）：
 *   sig=1e5 等效滞后 0.7ms（EMA 为 37.1ms）、静止 σ=1.50mm（EMA 1.21mm）；
 *   3σ 新息门控在正常运动下会锁死（每拍位移 8mm > 门限 7.6mm），故用不带门控的
 *   KF_UpdatePos，异常帧交给上面的状态/量程门限。 */
#define TOF_KF_SIG_A2	   1.0e5f  /* 过程噪声-加速度方差 (mm²/s⁴) */
#define TOF_KF_R		   3.5f	   /* 观测噪声方差 (mm²)：静止实测 σ=1.87mm */
#define TOF_KF_DT_DEFAULT  0.034f  /* 首帧 dt 兜底（10 通道实测帧间隔中位 34.3ms） */
#define TOF_KF_VEL_MAX	   800.0f  /* 速度限幅 mm/s（实测球速 max 768、p99 533） */
#define TOF_KF_GUARD_MM	   25.0f   /* KF 与 EMA 偏差超过此值计入 bad_streak（只上报） */
#define TOF_KF_GATE_N	   5.0f	   /* 新息门控倍数：|新息| > N·√S 判离群并丢弃本帧观测
									* 零延迟削峰；2026-11-06 实测单帧尖峰 332mm（真实 124mm）
									* 造成输出偏 120mm、速度打满 -800，持续 6 帧 */
#define TOF_KF_PRED_MAX_MM 20.0f   /* 位置外推量限幅（mm）：防无观测期速度漂移被放大 */
#define TOF_KF_PRED_DT_MAX 0.030f  /* 外推 Δt 上限（s）：1.5 × 实际帧间隔 20ms；
									* 不能用 TOF_KF_DT_DEFAULT（34ms）推，那会放到 51ms≈2.6 帧 */
#define TOF_KF_PRED_VMIN   80.0f   /* 外推速度下限（mm/s）：|v| 低于此值不外推，抑制静止抖动 */
#define TOF_KF_PRED_VMAX   200.0f  /* 外推速度上限（mm/s）：|v| 达到此值全额外推，中间线性过渡 */

typedef struct {
	/* 中值窗口 */
	int16_t  med_buf[TOF_FILTER_MEDIAN_N];
	uint8_t  med_idx;
	uint8_t  med_filled;

	/* EMA 状态（单位：0.001mm，避免整数除法累积误差） */
	int32_t  ema_milli_mm;
	uint16_t alpha_milli;

	/* 有效距离门限（运行时可改，默认 TOF_FILTER_MIN/MAX_VALID_MM） */
	int16_t  min_valid_mm;
	int16_t  max_valid_mm;

	/* 标定与输出 */
	int16_t  zero_mm;
	int16_t  last_raw_mm;
	int16_t  last_filt_mm; /* 最终输出（mm）：始终取 KF 位置估计 */
	bool     has_output;

	/* KF 主链运行态；EMA 并行计算仅作对照与遥测 */
	KF_t     kf;
	uint32_t kf_last_ms;	/* 上一拍时刻（真实 dt 依据，也供位置外推） */
	uint8_t  kf_bad_streak; /* 连续与 EMA 偏差超限拍数（只上报，不切换输出） */
	uint8_t  kf_miss;		/* 连续无有效观测拍数（只上报） */
	bool     kf_ready;		/* KF 已用首帧有效值初始化 */
	bool     out_use_ema;	/* true = 输出取 EMA 链（仅对照实验用，默认 false） */
} TofFilter_t;

/** @brief 初始化滤波器（配置 alpha/零点/有效范围并清空运行态） */
void TofFilter_Init(TofFilter_t *f, uint16_t alpha_milli, int16_t zero_mm);

/** @brief 清空中值窗口与 EMA 状态；保留 alpha/zero/有效范围配置 */
void TofFilter_Reset(TofFilter_t *f);

/** @brief 在线设置 EMA 系数（千分比，自动夹到 20~1000） */
void TofFilter_SetAlpha(TofFilter_t *f, uint16_t alpha_milli);

/** @brief 设置零点距离（mm，自动夹到 0~max_valid） */
void TofFilter_SetZeroMm(TofFilter_t *f, int16_t zero_mm);

/**
 * @brief 设置有效距离门限（mm），越界帧在 TofFilter_Update 中被拒绝
 * @param min 最小有效距离（<0 时取 0）
 * @param max 最大有效距离（≤min 时取默认 MAX）
 */
void TofFilter_SetValidRange(TofFilter_t *f, int16_t min, int16_t max);

/** @brief 以当前滤波输出为零点；尚无有效输出时返回 false */
bool TofFilter_ZeroHere(TofFilter_t *f);

/**
 * @brief 喂入一帧原始数据并推进滤波
 * @param  f            : 滤波器实例
 * @param  raw_mm       : 原始距离（mm）
 * @param  range_status : VL53L1X 状态（0 才有效；其他值直接丢弃）
 * @param  now_ms       : 本拍时刻（HAL_GetTick），用于 KF 的真实 dt
 * @retval true 本帧被接收并更新了滤波输出；false 本帧无效/越界被拒绝
 * @note   无效帧不推进 EMA，但会推进 KF 预测（kf_miss 累计）；
 *         输出始终取 KF，EMA 只作对照，不切换
 */
bool TofFilter_Update(TofFilter_t *f, int16_t raw_mm, int8_t range_status, uint32_t now_ms);

/** @brief 最近一次滤波位置（0.1cm）；无有效输出返回 0（纯字段读取直接用实例成员） */
int16_t TofFilter_GetCmX10(const TofFilter_t *f);

/**
 * @brief 位置外推到 now_ms 时刻（0.1cm），用于消除采样滞后
 * @param f      滤波器实例；now_ms 当前时刻（HAL_GetTick）
 * @retval 相对零点的位置（0.1cm）；无有效输出时为 0，饱和到 int16 范围
 * @note   KF 就绪时用 pos + vel·Δt 外推，Δt 压到 1.5 帧以内，外推量限幅
 *         ±TOF_KF_PRED_MAX_MM；KF 未就绪时退回普通滤波值。
 *         无阻塞、不访问外设，可在 1ms ISR 内调用。
 */
int16_t TofFilter_GetCmX10At(const TofFilter_t *f, uint32_t now_ms);

/**
 * @brief 选择输出源（仅用于 KF / EMA 对照实验与故障排查）
 * @param use_ema true = 输出取 EMA 链（无速度外推）；false = KF（默认）
 * @note  切换只影响 GetCmX10At 的取值，不改动两条链的内部状态，
 *        因此来回切换不会污染滤波器
 */
void TofFilter_SetOutputEma(TofFilter_t *f, bool use_ema);

#ifdef __cplusplus
}
#endif

#endif /* __TOF_FILTER_H__ */
