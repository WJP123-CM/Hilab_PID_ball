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
 * 递增，故取 +1，位置反馈与频率同号，位置环为负反馈。 */
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
 * @brief 编码器观测快照（本模块唯一数据出口，只读）
 *        写只发生在 encoder.c 的 Encoder_Tick / Encoder_Init / Encoder_Zero；
 *        控制层直接读字段，不再为每个量提供 getter。
 */
typedef struct {
	int32_t total; /* 累计计数（相对零点），角度环的反馈量 */
	int32_t delta; /* 最近一次采样的增量（已含方向修正） */
} EncoderState_t;

extern EncoderState_t g_enc;

/**
 * @brief 采样一次：读 CNT、算差分、累加、更新快照
 * @note  仿照 Tof_Tick：主循环每调一次就采一次，不做时间门控。
 */
void Encoder_Tick(void);

/**
 * @brief 读编码器相对累计计数（兼容接口，等价于 g_enc.total）
 * @retval 32 位累计计数（Encoder_Zero 后从 0 起算）
 */
int32_t Encoder_Read(void);

/**
 * @brief 读最近窗口增量（兼容接口，等价于 g_enc.delta）
 * @retval float 本窗口计数增量（已含方向修正）
 */
float Encoder_GetDelta(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */
