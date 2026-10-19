/**
 ******************************************************************************
 * @file    servo.h
 * @brief   舵机驱动模块（50Hz PWM，0°~180°）
 *
 * 硬件：TIM2_CH4 = PA3，PSC=71（72MHz/72 = 1MHz → 1 计数 = 1µs）
 *       Period=20000 → 20ms → 50Hz（该周期由 CubeMX 直接配置，须保持一致），
 *       脉冲宽度 500µs(0°) ~ 2500µs(180°)
 ******************************************************************************
 */
#ifndef __SERVO_H__
#define __SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h" /* HAL 定时器类型、TIM_CHANNEL_x */

/* USER CODE BEGIN Private defines */

/* 舵机 PWM 参数（50Hz 标准舵机） */
#define SERVO_PWM_PERIOD	   20000U		 /* 20ms → 50Hz（计数单位 1µs）；须与 CubeMX 中 TIM2.Period 一致 */
#define SERVO_PWM_PRESCALER	   71U			 /* 计数 1MHz（72MHz/72）；须与 CubeMX 中 TIM2.Prescaler 一致 */
#define SERVO_PWM_CHANNEL	   TIM_CHANNEL_4 /* 输出通道：TIM2_CH4 = PA3 */
#define SERVO_MIN_PULSE_US	   500U			 /* 0°  对应脉宽 0.5ms */
#define SERVO_MAX_PULSE_US	   2500U		 /* 180° 对应脉宽 2.5ms */
#define SERVO_MIN_ANGLE		   0.0f			 /* 最小角度 */
#define SERVO_MAX_ANGLE		   180.0f		 /* 最大角度 */

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/* 启动 50Hz PWM 并转到 90°；校验 TIM2 周期/分频与标定一致，不符即 Error_Handler */
void Servo_Init(void);
/* 立即转到指定角度（0~180，越界返回 -1） */
int Servo_SetAngle(float angle);
/* 返回最后写入的角度（无编码器反馈） */
float Servo_GetCurrentAngle(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H__ */
