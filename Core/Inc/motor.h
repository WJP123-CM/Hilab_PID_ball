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
 * 本模块只负责电机：频率限幅 / 死区 / 斜率限制 / 换向安全 + PWM 与 GPIO 下发。
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

/* 频率限幅（Hz）：|freq| < MIN 视为停止（脉冲过疏只会引起抖动）；
 * MAX 按电机矩频特性约束（1kHz≈18.75rpm，2kHz≈37.5rpm） */
#define MOTOR_FREQ_MIN_HZ 0	   /* 0Hz 最小频率 */
#define MOTOR_FREQ_MAX_HZ 2000 /* 2kHz 最大频率 */

/* 斜率限制：每拍频率变化量上限（Hz/拍）。50Hz 调用下 100Hz/拍 = 5kHz/s²，
 * 频率平滑渐变是防失步的核心——步进电机对频率阶跃敏感 */
#define MOTOR_SLEW_HZ_PER_STEP 10 /* 100Hz/拍 最大频率变化量 */

/* 方向电平定义：正转/反转 到 PA2 的实际电平。
 * 历史记录（2026-10-27）：多次对调后仍出现"固件未动、方向再次反转"的间歇现象，
 * 已怀疑 PA2 浮空 / 编码器 A/B 松动 / 电机相线接触不良，接线根因尚未闭环。
 * 当前工作区设置为 CW=1；H1 使能平台保持前必须先做小频率方向验证：
 * 若单向跑飞，先 H0，再查接线；不要只凭注释反复对调。 */
#define MOTOR_DIR_CW	 1u /* 正转电平 */
#define MOTOR_DIR_CCW 0u /* 反转电平 */

/* TIM2 计数时钟分频校验值：PSC=71 → 1MHz（须与 CubeMX 中 TIM2.Prescaler 一致） */
#define MOTOR_TIM_PSC 71u /* 72 分频 */

/* ==================== 核心决策状态机（纯软件，无硬件依赖） ================ */

typedef struct {
	int32_t cur_freq; /* 当前实际输出的带符号频率（Hz），0 = 脉冲停止 */
	bool	dir_cw;	  /* 当前 DIR 电平（true = 正转方向） */
} MotorSm_t;

typedef struct {
	int32_t out_freq;	 /* 本拍应输出的带符号频率（Hz） */
	bool	dir_cw;		 /* 本拍 DIR 电平 */
	bool	pulse_en;	 /* 本拍是否输出脉冲 */
	bool	dir_changed; /* 本拍发生换向（只建立 DIR 电平不发脉冲，
						  * 满足 D36A "换向后 ≥1ms 再发脉冲"时序） */
} MotorCmd_t;

/**
 * @brief 状态机初始化（上电安全态：输出 0，方向默认正转）
 * @param sm 状态机实例（由调用方持有）
 */
void MotorSm_Init(MotorSm_t *sm);

/**
 * @brief 核心决策：带符号目标频率 → 本拍输出决策
 * @param sm        状态机实例
 * @param target_hz 目标频率（Hz，带符号）
 * @retval 本拍输出决策（见 MotorCmd_t）
 * @note  处理顺序：限幅(±MAX) → 死区(|f|<MIN 视为 0) → 斜率限制(每拍 ≤SLEW)
 *        → 换向安全（方向不同时本拍输出 0、只切 DIR、dir_changed=true）
 * @note  以固定周期调用（本工程 50Hz），斜率以"每拍最大变化量"表达；
 *        纯软件逻辑，无硬件依赖，可主机侧单测
 */
MotorCmd_t MotorSm_Update(MotorSm_t *sm, int32_t target_hz);

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
 * @brief 设定带符号目标频率（Hz），以固定周期调用（本工程 50Hz）
 * @param freq_hz >0 正转；<0 反转；0 停止脉冲
 * @note  脉冲启停、方向切换、ARR 更新均由本函数按 MotorSm 决策自动完成；
 *        位置限幅由调用方在下发前完成（本函数不做位置判断）
 */
void Motor_SetTargetFreq(int32_t freq_hz);

/**
 * @brief 当前实际输出的带符号脉冲频率（Hz，0 = 停止）
 * @retval 经斜率限制后的真实输出；加减速/换向过程中与目标值不同
 */
int32_t Motor_GetPulseFreqHz(void);

/* ===================== 低层 PWM 原语（绕过状态机，直接操控 TIM2） ==========
 * 供需要最简控制逻辑的场景使用：测距→频率 直接映射，不经斜率/换向状态机。
 * 注意：调用方需自行处理方向、启停顺序与频率限幅（≥ MOTOR_FREQ_MIN_HZ）。 */

/** @brief 设定 STEP 脉冲频率（Hz，≥ MOTOR_FREQ_MIN_HZ），占空比 ≈50% */
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
