/**
 ******************************************************************************
 * @file    servo.c
 * @brief   舵机驱动模块实现（50Hz PWM 输出 + 角度校验）
 ******************************************************************************
 */
#include "servo.h"

#include "tim.h" /* htim2 */

/* 最后写入的角度（无编码器反馈，仅作状态查询） */
static float s_current = SERVO_MIN_ANGLE;

/* 角度 → 占空比脉宽（µs）并写入比较寄存器 */
static void Servo_WritePulse(float angle) {
	uint32_t pulse_us;

	if (angle <= SERVO_MIN_ANGLE) {
		pulse_us = SERVO_MIN_PULSE_US;
	} else if (angle >= SERVO_MAX_ANGLE) {
		pulse_us = SERVO_MAX_PULSE_US;
	} else {
		float ratio = (angle - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
		pulse_us	= SERVO_MIN_PULSE_US + (uint32_t) (ratio * (float) (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US));
	}
	__HAL_TIM_SET_COMPARE(&htim2, SERVO_PWM_CHANNEL, pulse_us);
}

void Servo_Init(void) {
	/* 校验 CubeMX 配置的 TIM2 周期/分频与 50Hz 脉宽标定一致（重新生成时须保持） */
	if (__HAL_TIM_GET_AUTORELOAD(&htim2) != SERVO_PWM_PERIOD) {
		Error_Handler();
	}
	if (htim2.Instance->PSC != SERVO_PWM_PRESCALER) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Start(&htim2, SERVO_PWM_CHANNEL) != HAL_OK) {
		Error_Handler();
	}
	(void) Servo_SetAngle(90.0f);
}

int Servo_SetAngle(float angle) {
	if ((angle < SERVO_MIN_ANGLE) || (angle > SERVO_MAX_ANGLE)) {
		return -1;
	}
	s_current = angle;
	Servo_WritePulse(angle);
	return 0;
}

float Servo_GetCurrentAngle(void) {
	return s_current;
}
