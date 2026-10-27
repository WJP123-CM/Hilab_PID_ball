/**
 ******************************************************************************
 * @file    stepper.c
 * @brief   步进电机基础驱动实现（D36A：EN/DIR 电平 + TIM2_CH4 STEP 脉冲）
 ******************************************************************************
 */
#include "stepper.h"

#include "tim.h" /* htim2 */

/* STEP 脉冲通道：TIM2_CH4 = PA3 */
#define STEP_CHANNEL TIM_CHANNEL_4

void Stepper_Init(void) {
	/* 校验 CubeMX 配置的 TIM2 分频与 1MHz 计数假设一致（重新生成时须保持）：
	 * 计数 1tick=1µs → 占空比换算与头文件中的频率换算才成立，不符即 Error_Handler */
	if (htim2.Instance->PSC != STEPPER_TIM_PSC) {
		Error_Handler();
	}

	/* STEP 占空比 ≈50%：周期 1000µs → 高电平 500µs，远大于驱动芯片
	 * 对 STEP 脉宽的最低要求（≥2µs）。用 (ARR+1)/2 动态计算，兼容 CubeMX
	 * 中调整 Period 改变脉冲频率（调速走 CubeMX，不在代码中覆盖 ARR） */
	__HAL_TIM_SET_COMPARE(&htim2, STEP_CHANNEL, (__HAL_TIM_GET_AUTORELOAD(&htim2) + 1u) / 2u);

	/* 上电安全态：不使能电机、不输出脉冲（PA1/PA2 已由 gpio.c 初始化为低） */
}

void Stepper_SetEnabled(bool enable) {
	HAL_GPIO_WritePin(Motor_enabled_GPIO_Port, Motor_enabled_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Stepper_SetDirection(bool cw) {
	HAL_GPIO_WritePin(Motor_direction_GPIO_Port, Motor_direction_Pin, cw ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Stepper_Start(void) {
	if (HAL_TIM_PWM_Start(&htim2, STEP_CHANNEL) != HAL_OK) {
		Error_Handler();
	}
}

void Stepper_Stop(void) {
	if (HAL_TIM_PWM_Stop(&htim2, STEP_CHANNEL) != HAL_OK) {
		Error_Handler();
	}
}

uint32_t Stepper_GetPulseFreqHz(void) {
	/* 计数时钟 1MHz（PSC=71，Stepper_Init 已校验） */
	return 1000000u / (__HAL_TIM_GET_AUTORELOAD(&htim2) + 1u);
}
