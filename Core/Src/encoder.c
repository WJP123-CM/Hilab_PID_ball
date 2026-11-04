/**
 ******************************************************************************
 * @file    encoder.c
 * @brief   编码器计数：16 位 CNT 回绕差分 + 方向修正 + 32 位累计
 *
 *          硬件：TIM3 编码器接口 PA6/PA7（CubeMX TI12 四倍频，已配）。
 *          去掉了 Z 相捕获和 Flash 掉电保存（行程不跨 Z、断电机构会动）。
 *
 *          采样时机：只在 TIM1 的 20ms 节拍里调 Encoder_Tick() 一次，
 *          窗口严格固定；Encoder_Read/GetDelta 张为纯读取，
 *          不改变 s_enc_last。这样主循环阻塞（OLED/I2C）或其它模块
 *          读计数都不会把差分窗口截断，速度反馈不会莫名跳变。
 ******************************************************************************
 */
#include "encoder.h"
#include "main.h"   /* CMSIS 临界区原语 */
#include "tim.h"    /* htim3 */

static int32_t  s_enc_total  = 0; /* 32 位累计计数（Encoder_Zero 清零） */
static uint16_t s_enc_last   = 0u; /* 上一次采样的 CNT 原始值 */
static int16_t  s_enc_last_d = 0; /* 最近一个 20ms 窗口的增量（原始，供累计） */
static int16_t  s_win[4]	 = {0, 0, 0, 0}; /* 最近 4 个窗口增量，速度平滑用 */
static uint8_t  s_win_idx	 = 0u;

/**
 * @brief 启动 TIM3 编码器接口，以当前 CNT 为累计零点
 * @note  初始化阶段调用一次；不使能电机、不输出脉冲
 */
void Encoder_Init(void) {
	if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
		Error_Handler();
	}
	s_enc_last   = (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
	s_enc_total  = 0;
	s_enc_last_d = 0;
	for (uint8_t i = 0u; i < 4u; i++) {
		s_win[i] = 0;
	}
	s_win_idx = 0u;
}

/**
 * @brief 相对计数清零：当前时刻为新的 0 点
 */
void Encoder_Zero(void) {
	uint32_t pm = __get_PRIMASK();

	__disable_irq(); /* 与 ISR 中的 Encoder_Tick 互斥 */
	s_enc_total  = 0;
	s_enc_last   = (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
	s_enc_last_d = 0;
	for (uint8_t i = 0u; i < 4u; i++) {
		s_win[i] = 0;
	}
	s_win_idx = 0u;
	__set_PRIMASK(pm);
}

/**
 * @brief 20ms 节拍采样：读 CNT、算差分、累加
 * @note  仅在 TIM1 1ms 中断的 inner_tick 分支调用（50Hz），窗口严格固定；
 *        TIM3 编码器接口无中断，s_enc_last 只有此处与 Encoder_Zero 会写
 */
void Encoder_Tick(void) {
	uint16_t now = (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
	int16_t	 d	 = (int16_t) (now - s_enc_last);

	d = (int16_t) (ENC_DIR_SIGN * d); /* A/B 相接反 → 方向修正 */
	s_enc_last	 = now;
	s_enc_last_d = d;
	s_enc_total += d;
	/* 4 点滑动平均：低速时编码器边沿抖动会让单窗增量在 0/±几 之间跳，
	 * 直接喂 PID 会让输出跟着噪声横跳；累计值仍用原始 d 保证位置准确 */
	s_win[s_win_idx] = d;
	s_win_idx		 = (uint8_t) ((s_win_idx + 1u) & 3u);
}

/**
 * @brief 读编码器相对累计计数
 * @retval 32 位累计计数（Encoder_Zero 后从 0 起算）
 * @note  纯读取，不采样，不会打乱速度差分窗口
 */
int32_t Encoder_Read(void) {
	return s_enc_total;
}

/**
 * @brief 读编码器速度：最近 4 个 20ms 窗口增量的滑动平均
 * @retval float  counts / 20ms（已含方向修正，分辨率 0.25）
 * @note  纯读取，不采样；窗口由 Encoder_Tick 在 ISR 中固定为 20ms。
 *        返回 4 点均值而非单窗原始值：低速时单窗只有 0/±1，噪声会让
 *        PID 输出横跳；均值把抖动平掉，代价是约 40ms 相位滞后
 */
float Encoder_GetDelta(void) {
	return (float) (s_win[0] + s_win[1] + s_win[2] + s_win[3]) / 4.0f;
}
