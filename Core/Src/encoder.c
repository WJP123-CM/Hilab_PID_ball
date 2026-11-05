/**
 ******************************************************************************
 * @file    encoder.c
 * @brief   编码器计数：16 位 CNT 回绕差分 + 方向修正 + 32 位累计
 *
 *          硬件：TIM3 编码器接口 PA6/PA7（CubeMX TI12 四倍频，已配）。
 *          去掉了 Z 相捕获和 Flash 掉电保存（行程不跨 Z、断电机构会动）。
 *
 *          采样模式对齐 tof 模块：主循环每调一次 Encoder_Tick() 就采一次，
 *          不做时间门控；观测结果写入只读快照 g_enc，控制层要数据时直接读字段。
 *          本模块只提供位置（total）与增量（delta），速度由上层按需自行派生。
 ******************************************************************************
 */
#include "encoder.h"
#include "main.h"   /* CMSIS 临界区原语 */
#include "tim.h"    /* htim3 */

/* 观测快照：唯一对外数据出口（写只发生在本文件的三个接口） */
EncoderState_t g_enc = {0, 0};

static uint16_t s_enc_last = 0u; /* 上一次采样的 CNT 原始值 */

/**
 * @brief 启动 TIM3 编码器接口，以当前 CNT 为累计零点
 * @note  初始化阶段调用一次；不使能电机、不输出脉冲
 */
void Encoder_Init(void) {
	if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
		Error_Handler();
	}
	s_enc_last = (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
	g_enc.total = 0;
	g_enc.delta = 0;
}

/**
 * @brief 相对计数清零：当前时刻为新的 0 点
 * @note  与 Encoder_Tick 互斥，避免差分基准被打断
 */
void Encoder_Zero(void) {
	uint32_t pm = __get_PRIMASK();

	__disable_irq();
	g_enc.total = 0;
	g_enc.delta = 0;
	s_enc_last	 = (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
	__set_PRIMASK(pm);
}

/**
 * @brief 采样一次：读 CNT、算差分、累加、更新快照
 * @note  仿照 Tof_Tick：主循环每调一次就采一次，不做时间门控。
 *        差分用 int16 处理 16 位 CNT 回绕，单次调用间隔内轴转过不足半圈即可
 */
void Encoder_Tick(void) {
	uint16_t now = (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
	int16_t	 d	 = (int16_t) (now - s_enc_last);
	d			 = (int16_t) (ENC_DIR_SIGN * d); /* A/B 相接反 → 方向修正 */

	s_enc_last	 = now;
	g_enc.total += d;
	g_enc.delta	 = d;
}

/**
 * @brief 读编码器相对累计计数（兼容接口，等价于 g_enc.total）
 * @retval 32 位累计计数（Encoder_Zero 后从 0 起算）
 */
int32_t Encoder_Read(void) {
	return g_enc.total;
}

/**
 * @brief 读最近窗口增量（兼容接口，等价于 g_enc.delta）
 * @retval float 本窗口计数增量（已含方向修正）
 */
float Encoder_GetDelta(void) {
	return (float) g_enc.delta;
}
