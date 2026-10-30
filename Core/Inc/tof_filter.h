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
 *            pos_cm_x10 = TOF_POS_SIGN * (filtered_mm - zero_mm)
 *            即“滤波后距离相对零点的偏移”，单位 0.1cm；zero_mm 由 Z 命令标定。
 ******************************************************************************
 */
#ifndef __TOF_FILTER_H__
#define __TOF_FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* 中值窗口长度：5 点可剔除孤立尖峰，同时只引入 2 帧群延迟（20Hz 下约 100ms） */
#define TOF_FILTER_MEDIAN_N 5u

/* EMA 系数默认值：300 = 0.30；越小越平滑、延迟越大 */
#define TOF_FILTER_ALPHA_DEFAULT_MILLI 300u
#define TOF_FILTER_ALPHA_MIN_MILLI     20u
#define TOF_FILTER_ALPHA_MAX_MILLI     1000u

/* 有效原始距离范围（mm）：Short 模式标称 0~250mm；工程有效测试点最近 3~5cm。
 * 10mm 仅作"明显异常值"门限，不代表 0~2cm 可用。运行时可通过
 * TofFilter_SetValidRange 覆盖，默认值取自下方宏。 */
#define TOF_FILTER_MIN_VALID_MM 10
#define TOF_FILTER_MAX_VALID_MM 250

/* 位置符号：+1 = 小球远离传感器方向为正；传感器安装方向相反时改为 -1 */
#define TOF_POS_SIGN (+1)

/* 默认零点：仅用于上电显示；正式测试必须用 Z 命令标定 */
#define TOF_FILTER_ZERO_DEFAULT_MM 131  /* 2026-10-29 实测静止零点：60s 930帧 mean=131.44mm, σ=1.87mm */

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
	int16_t  last_filt_mm;
	bool     has_output;
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
 * @brief 喂入一帧原始数据并推进滤波器
 * @param  raw_mm       : 原始距离（mm）
 * @param  range_status : VL53L1X 状态（0 才有效；其他值直接丢弃）
 * @retval true 本帧被接收并更新了滤波输出；false 本帧无效/越界被拒绝
 * @note   无效帧不会清空滤波状态，输出保持上一有效值
 */
bool TofFilter_Update(TofFilter_t *f, int16_t raw_mm, int8_t range_status);

/** @brief 最近一次滤波位置（0.1cm）；无有效输出返回 0（纯字段读取直接用实例成员） */
int16_t TofFilter_GetCmX10(const TofFilter_t *f);

#ifdef __cplusplus
}
#endif

#endif /* __TOF_FILTER_H__ */
