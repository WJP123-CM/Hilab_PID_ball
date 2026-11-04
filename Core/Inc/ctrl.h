/**
 ******************************************************************************
 * @file    ctrl.h
 * @brief   应用层控制：级联闭环（内环编码器位置 + 外环小球位置）+ 模式管理
 *          + Z 相掉电保存 + VOFA 遥测 + 按键急停。
 *
 *          职责：只做控制决策与状态管理，不直接操作外设寄存器（通过 motor/
 *          encoder/pid/tof 模块接口）；不做串口解析（cmd.c）。
 *          内环 Ctrl_InnerTick 在 TIM1 ISR 50Hz 调用，须 O(1) 不可阻塞。
 ******************************************************************************
 */
#ifndef __CTRL_H__
#define __CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "pid.h" /* PIDGain_t */

/* ---- 调度周期（与 TIM1 1ms 基础节拍配合的软件分频） ---- */
#define CTRL_PERIOD_MS  20u  /* 平台保持控制：50Hz */
#define Out_PERIOD_MS	20u	 /* ToF 测距/滤波/外环/VOFA：20Hz */
#define CMD_PERIOD_MS   100u /* 串口命令解析：10Hz */
#define OLED_PERIOD_MS  200u /* OLED 刷新：5Hz */

/* ---- 上电行为与电机总开关 ---- */
#define PLATFORM_HOLD_ON_BOOT 1  /* 上电自动进入平台保持模式 */
#define MOTOR_CONTROL_ENABLE  1  /* 0=不驱动电机（脱机调参），1=正常输出 */

/* ===================== 控制接口 ===================== */

/** @brief 初始化执行器/编码器/内外环，按 PLATFORM_HOLD_ON_BOOT 进入上电模式 */
void Ctrl_Init(void);

/** @brief 切换控制模式：0=释放 1=平台保持 2=小球闭环 */
void Ctrl_SetMode(uint8_t mode);

/** @brief 外环 20Hz：ToF 位置 → 内环目标偏移（仅模式 2 推进） */
void Ctrl_OuterTick(void);

/** @brief 内环 50Hz（TIM1 ISR）：编码器位置环跟踪目标偏移 */
void Ctrl_InnerTick(void);

/** @brief Z 相事件与掉电保存服务（5Hz 主循环） */
void Ctrl_EncService(void);

/** @brief 20Hz VOFA 遥测帧（滤波距离/脉冲频率/编码器计数） */
void Ctrl_SendTelemetry(void);

/** @brief 消费按键事件（急停锁存/解除） */
void Ctrl_PollKeyEvents(void);

/** @brief 复位内外环积分与目标偏移（模式切换 / R 命令） */
void Ctrl_ResetLoops(void);

/* ---- 状态查询（供 main.c OLED 显示） ---- */
uint8_t Ctrl_GetMode(void);          /* 0=释放 1=保持 2=闭环 */
uint8_t Ctrl_IsEstop(void);          /* 1=已急停锁存 */

/* ---- 供 cmd.c 在线整定外环增益 ---- */
void Ctrl_SetBallGain(PIDGain_t gain, float value);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_H */
