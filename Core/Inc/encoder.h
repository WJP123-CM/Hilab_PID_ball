/**
 ******************************************************************************
 * @file    encoder.h
 * @brief   编码器（MS42CG 增量式）计数接口
 *
 * 硬件（CubeMX 已配置）：
 *   A/B 相 PA6/PA7 = TIM3_CH1/CH2（TI12 四倍频 + 输入滤波 15）
 *   Z 相不再启（行程不跨 Z）；Flash 掉电保存也不再启（断电机构会动）
 *
 * 本模块只做一件事：把 16 位 CNT 的差分累加成 32 位计数，处理回绕和方向。
 ******************************************************************************
 */
#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 编码器线数（按实物铭牌改）；TIM3 TI12 四倍频 → 计数/圈 = 4×线数 */
#define ENC_PPR	1000u
#define ENC_CPR (ENC_PPR * 4u)

/* 计数方向修正：+1 = 计数增量与 Motor_SetTargetFreq 正号同向；
 *                         -1 = 计数增量与 freq 反向（负相关）。
 * 2026-11-02 实测：Motor_Init 对齐 DIR 初态后，freq>0（DIR=高）使 TIM3_CNT
 * 递增，故取 +1，速度反馈与频率同号，速度环为负反馈。 */
#define ENC_DIR_SIGN (+1)

/**
 * @brief 启动 TIM3 编码器接口，并以当前 CNT 为累计零点
 * @note  须在 MX_TIM3_Init 之后调用一次
 */
void Encoder_Init(void);

/**
 * @brief 相对计数清零：当前时刻为新的 0 点
 */
void Encoder_Zero(void);

/**
 * @brief 20ms 节拍采样：读 CNT、算差分、累加
 * @note  只在 TIM1 的 inner_tick 分支调用（严格 20ms），保证差分窗口固定；
 *        主循环读接口不打乱窗口，速度反馈不会跳动
 */
void Encoder_Tick(void);

/**
 * @brief 读编码器相对累计计数（int16 差分处理 16 位回绕 + 方向修正）
 * @retval 32 位累计计数（Encoder_Zero 后从 0 起算）
 * @note  纯读取，不采样；可在主循环/OLED 随意调用
 */
int32_t Encoder_Read(void);

/**
 * @brief 读编码器速度：最近 4 个 20ms 窗口增量的滑动平均
 * @retval float  counts / 20ms（已含方向修正，分辨率 0.25）
 * @note  纯读取，不采样；换算：vel_counts_per_s = 返回值 × 50。
 *        4 点均值用于抑制低速时编码器边沿抖动，带来约 40ms 相位滞后
 */
float Encoder_GetDelta(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */
