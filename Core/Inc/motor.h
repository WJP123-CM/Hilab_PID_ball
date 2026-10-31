/**
 ******************************************************************************
 * @file    motor.h
 * @brief   步进电机驱动（D36A 驱动板 + MS42CG 电机，带符号频率控制）
 *
 * 硬件（CubeMX 已配置）：
 *   STEP 脉冲 PA3 = TIM2_CH4（PSC=71 → 计数时钟 1MHz）
 *   DIR  方向  PA2（Motor_direction_Pin，推挽输出）
 *   EN   使能  PA1（Motor_enabled_Pin，推挽输出，D36A 高电平使能）
 *
 * 本模块只负责电机：PWM 与 GPIO 下发、换向安全。
 * 频率幅值不限幅：闭环由 Motor_SetAngle 内的 PID 输出限幅决定，
 * 开环按调用方给定值直接下发。
 * 位置限幅不在本模块：由调用方（main.c ISR）按相对计数基准在下发前钳 0，
 * 编码器计数、Z 校准、掉电保存都在 encoder.c。
 *
 * D36A 控制时序：EN 高电平使能、低电平休眠（上电默认休眠，安全）；
 * 换向后须 ≥1ms 再发 STEP 脉冲（本模块换向拍只切 DIR 不发脉冲）；
 * 出厂 16 细分 → 200 步/圈 × 16 = 3200 STEP 脉冲/圈（1kHz ≈ 18.75rpm）。
 ******************************************************************************
 */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ============================ 参数宏 ====================================== */

/* D36A 出厂 16 细分：200 步/圈 × 16 = 3200 STEP 脉冲/圈；改 DIP 细分须同步改 */
#define MOTOR_PULSES_PER_REV 3200u

/* 方向电平定义：正转/反转 到 PA2 的实际电平。
 * 历史记录（2026-10-27）：多次对调后仍出现"固件未动、方向再次反转"的间歇现象，
 * 已怀疑 PA2 浮空 / 编码器 A/B 松动 / 电机相线接触不良，接线根因尚未闭环。
 * 当前工作区设置为 CW=1；H1 使能平台保持前必须先做小频率方向验证：
 * 若单向跑飞，先 H0，再查接线；不要只凭注释反复对调。 */
#define MOTOR_DIR_CW	 1u /* 正转电平 */
#define MOTOR_DIR_CCW 0u /* 反转电平 */

/* TIM2 计数时钟分频校验值：PSC=71 → 1MHz（须与 CubeMX 中 TIM2.Prescaler 一致） */
#define MOTOR_TIM_PSC 71u /* 72 分频 */

/* ==================== 内环角度控制周期（与测距同拍 50Hz） =====================
 * MOTOR_MEAS_PERIOD_MS 是测距/滤波节拍；内环角度控制每个测距周期执行一次，
 * 保证每个新滤波值当拍就参与控制。外环位置控制为 10Hz，由 main.c 5 分频。
 * main.c 的测距节拍须与本定义一致。 */
#define MOTOR_MEAS_PERIOD_MS 20u                    /* 测距周期（ms，固定基准） */
#define MOTOR_CTRL_PERIOD_MS MOTOR_MEAS_PERIOD_MS   /* 角度控制周期 = 20ms（50Hz） */

/* ============================ 驱动接口 ==================================== */

/**
 * @brief 初始化电机侧：校验 TIM2 分频配置、设定 STEP 占空比（约 50%）
 * @note  须在 MX_TIM2_Init
 * @note  编码器不在本函数初始化（见 Encoder_Init）
 */
void Motor_Init(void);

/**
 * @brief 使能/休眠驱动板（高电平使能）；休眠时绕组断电，电机可手转
 * @param enable true = 使能力矩；false = 休眠
 */
void Motor_SetEnabled(bool enable);

/**
 * @brief 设定带符号目标频率（Hz），直接驱动 PWM/DIR
 * @param freq_hz >0 正转；<0 反转；0 停止脉冲（频率幅值不做限幅）
 * @note  直接写 TIM2 PWM 频率与 DIR 电平；换向拍只切 DIR、停脉冲
 *        （D36A 换向后 ≥1ms 再发脉冲，调用周期需 ≥1ms）。
 *        位置限幅由调用方在下发前完成（本函数不做位置判断）。
 */
void Motor_SetTargetFreq(int32_t freq_hz);

/**
 * @brief 设定目标角度（编码器计数）：角度环位置式 PID → 直接驱动 PWM/DIR
 * @param target_cnt 目标编码器计数值（相对零点）
 * @param ctrl_p     角度环比例增益（Hz / 编码器计数偏差）：Out = Kp × 角度误差，
 *                   等价于一个控制周期的脉冲数 = Out × MOTOR_CTRL_PERIOD_MS / 1000
 * @note  内环由调用方每个测距周期（MOTOR_CTRL_PERIOD_MS = 20ms，50Hz）调用一次。
 *        位置式 PID：Target=目标角度, Actual=Encoder_Read(), Out=频率(Hz)。
 *        PID 输出直接写 TIM2 PWM 频率与 DIR 电平，保证定角度响应速度；
 *        换向拍只切 DIR、停脉冲（D36A 换向后 ≥1ms 再发脉冲）。
 *        方向由 motor.c 的 MOTOR_ENC_HZ_SIGN 决定（1 = freq>0 使 enc 增大，0 = 相反）。
 */
void Motor_SetAngle(int32_t target_cnt, float ctrl_p);

/**
 * @brief 读取角度环 PID 算出的目标频率（Hz，带符号）
 */
int32_t Motor_GetAngleTargetFreq(void);

/**
 * @brief 当前实际输出的带符号脉冲频率（Hz，0 = 停止）
 * @retval 最近一次下发的频率（带符号）；换向/停止拍为 0
 */
int32_t Motor_GetPulseFreqHz(void);

/* ===================== 低层 PWM 原语（直接操控 TIM2） ====================== */

/** @brief 设定 STEP 脉冲频率（Hz），占空比 ≈50% */
void PwmSetFreq(uint32_t freq_hz);

/** @brief 启动 STEP 脉冲输出（幂等） */
void PwmStart(void);

/** @brief 停止 STEP 脉冲输出（幂等，绕组保持锁定） */
void PwmStop(void);

/**
 * @brief 设定 DIR 方向电平
 * @param cw true = 正转（MOTOR_DIR_CW），false = 反转（MOTOR_DIR_CCW）
 */
void Motor_SetDir(bool cw);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
