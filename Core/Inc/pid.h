/**
 ******************************************************************************
 * @file    pid.h
 * @brief   标准单环位置式 PID 控制器（可复用模块，无外部依赖）
 *          命名约定对齐倒立摆参考工程：
 *            Target / Actual / Out（输入输出）
 *            Kp / Ki / Kd（增益）
 *            Error0 / Error1 / ErrorInt（误差与积分）
 *            OutMax / OutMin（输出限幅）
 *          位置式核心：Out = Kp*Error0 + Ki*ErrorInt + Kd*(Error0 - Error1)
 *          PID_Update 仅接收结构体指针：调用方先写 Target/Actual，再读 Out。
 *          本模块只做算术运算，不依赖任何 HAL/外设，可在 ISR 或主循环中直接调用。
 *          注意：F103 无 FPU，float 为软件浮点；按 50Hz 以下控制频率使用开销可忽略，
 *          且严禁与 %f 打印混用（工程禁用 %f 家族）。
 ******************************************************************************
 */
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief  PID 控制器实例（字段命名对齐倒立摆参考工程）
 *         Target / Actual 由调用方在每步调用前写入；
 *         Out 由 PID_Update 写入；Error0 / Error1 / ErrorInt 由内部维护。
 */
typedef struct {
	/* ---- 输入 / 输出（调用方读写） ---- */
	float Target;	/* 目标设定值 */
	float Actual;	/* 当前反馈值（测量值） */
	float Out;		/* PID 输出（已限幅） */

	/* ---- 控制参数（运行期可调，属应用层动态量） ---- */
	float Kp;		/* 比例增益 */
	float Ki;		/* 积分增益（按采样步长离散化） */
	float Kd;		/* 微分增益 */
	float OutMax;	/* 输出上限 */
	float OutMin;	/* 输出下限 */

	/* ---- 运行态（内部维护） ---- */
	float Error0;	/* 本次误差 = Target - Actual */
	float Error1;	/* 上次误差 */
	float ErrorInt;	/* 误差积分（累加值，抗饱和限幅） */

	/* ---- 扩展：积分限幅抗饱和（>0 启用，0 = 不限幅） ---- */
	float i_limit;
} PID_t;

/**
 * @brief  控制器配置（一次性初始化用），字段含义与 PID_t 同名成员一致
 */
typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float OutMin;
	float OutMax;
	float i_limit; /* 积分项限幅阈值（>0 启用，抗饱和） */
} PIDCfg_t;

/** 在线整定时的单个增益选择 */
typedef enum {
	PID_GAIN_KP = 0,
	PID_GAIN_KI,
	PID_GAIN_KD,
	PID_GAIN_CNT
} PIDGain_t;

/**
 * @brief  按配置结构初始化 PID 并复位运行态
 * @param  pid : 控制器实例指针； cfg : 参数配置（只读）
 */
void PID_InitFromCfg(PID_t *pid, const PIDCfg_t *cfg);

/**
 * @brief  在线修改单个增益（供上位机整定，只改参数不清运行态）
 * @param  pid   : 控制器实例指针
 * @param  gain  : PID_GAIN_KP / KI / KD，越界值忽略
 * @param  value : 新增益（已按每采样步离散化）
 */
void PID_SetOneGain(PID_t *pid, PIDGain_t gain, float value);

/**
 * @brief  复位运行态：误差历史与积分清零
 * @param  pid : 控制器实例指针
 */
void PID_Reset(PID_t *pid);

/**
 * @brief  单步位置式 PID 计算
 * @param  pid : 控制器实例指针（调用前须写好 pid->Target 与 pid->Actual）
 * @note   计算结果写入 pid->Out（已限幅）。
 *         离散化约定：增益按"每采样步"为单位整定，要求调用方以固定周期调用；
 *         若采样周期改变，需按比例重新整定 Ki / Kd。
 */
void PID_Update(PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
