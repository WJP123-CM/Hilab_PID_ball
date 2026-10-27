/**
 ******************************************************************************
 * @file    stepper.h
 * @brief   步进电机基础驱动（D36A 驱动板 + MS42CG 电机，开环脉冲控制）
 *
 * 硬件接线（CubeMX 已配置）：
 *   - STEP 脉冲：PA3 = TIM2_CH4（PSC=71 → 计数时钟 1MHz，Period 由 CubeMX 决定）
 *   - DIR  方向：PA2（Motor_direction_Pin，推挽输出）
 *   - EN   使能：PA1（Motor_enabled_Pin，推挽输出，D36A 高电平使能）
 *
 * D36A 控制时序（参考 WHEELTEC 官方 C8T6+D36A+MS42 例程）：
 *   - EN：高电平使能，低电平休眠（上电默认休眠，安全）
 *   - DIR：电平保持定向；切换方向后须 ≥1ms 再发 STEP 脉冲
 *   - STEP：每个上升沿走一个细分步；D36A 出厂 16 细分，
 *           200 步/圈 × 16 = 3200 脉冲/圈（细分改变时须同步修改宏）
 *
 * 脉冲频率由 CubeMX 中 TIM2 的 Prescaler/Period 决定（本工程约定不在代码中
 * 覆盖 CubeMX 参数），当前配置 PSC=71 / Period=999 → 1.00kHz，
 * 对应转速 = 1000×60/3200 = 18.75 rpm；调速须在 CubeMX 改 Period 后重新生成。
 ******************************************************************************
 */
#ifndef __STEPPER_H__
#define __STEPPER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h" /* HAL 类型、Motor_enabled/Motor_direction 引脚标签 */

#include <stdbool.h>
#include <stdint.h>

/* ============================ 参数宏 ====================================== */

/* D36A 出厂 16 细分：200 步/圈 × 16 = 3200 STEP 脉冲/圈；改 DIP 细分须同步改 */
#define STEPPER_PULSES_PER_REV 3200u

/* 方向电平定义（与官方例程一致：DIR=1 电机正转/顺时针；若整机方向相反，
 * 只需对调下面两个宏，不必改驱动逻辑） */
#define STEPPER_DIR_CW	1u /* 正转电平 */
#define STEPPER_DIR_CCW 0u /* 反转电平 */

/* TIM2 计数时钟分频校验值：PSC=71 → 1MHz（须与 CubeMX 中 TIM2.Prescaler 一致） */
#define STEPPER_TIM_PSC 71u

/* ============================ 接口 ======================================== */

/* 初始化：校验 TIM2 分频配置、设定 STEP 占空比（约 50%）；
 * 不启动脉冲、不使能电机（上电安全态，须显式调用 SetEnabled+Start 才会转动） */
void Stepper_Init(void);

/* 使能/休眠驱动板（高电平使能）；休眠时绕组断电，电机可手转 */
void Stepper_SetEnabled(bool enable);

/* 设置方向电平（cw=true 正转）；
 * 注意：若电机正在转（Start 状态），改向后须等待 ≥1ms 时序，由调用方保证 */
void Stepper_SetDirection(bool cw);

/* 启动 STEP 脉冲输出（开始连续转动）；未使能时仅启动脉冲、电机不动 */
void Stepper_Start(void);

/* 停止 STEP 脉冲输出（停止转动，绕组保持锁定） */
void Stepper_Stop(void);

/* 当前脉冲频率（Hz）= 1MHz / (Period+1)，由 CubeMX 配置决定，仅供显示/换算 */
uint32_t Stepper_GetPulseFreqHz(void);

#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_H__ */
