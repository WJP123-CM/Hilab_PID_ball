/**
 ******************************************************************************
 * @file    pid.c
 * @brief   标准单环位置式 PID 控制器实现（命名对齐倒立摆参考工程）
 *          计算流程（每步）：
 *            1) Error0 = Target - Actual
 *            2) ErrorInt += Error0（Ki==0 时清零，避免调试期积分堆积）
 *            3) ErrorInt 限幅到 ±i_limit（抗积分饱和）
 *            4) Out = Kp*Error0 + Ki*ErrorInt + Kd*(Error0 - Error1)
 *            5) Out 限幅到 [OutMin, OutMax]
 *          无外部依赖：不使用 HAL/libm，便于复用与宿主机仿真验证。
 ******************************************************************************
 */
#include "pid.h"

/* ===================== 初始化 / 配置接口 ================================== */

/**
 * @brief  按参数表初始化并复位运行态
 */
void PID_InitFromCfg(PID_t *pid, const PIDCfg_t *cfg) {
	pid->Kp      = cfg->Kp;
	pid->Ki      = cfg->Ki;
	pid->Kd      = cfg->Kd;
	pid->OutMin  = cfg->OutMin;
	pid->OutMax  = cfg->OutMax;
	pid->i_limit = cfg->i_limit;
	PID_Reset(pid);
}

/**
 * @brief  在线改单个增益（供上位机整定，不清运行态）
 */
void PID_SetOneGain(PID_t *pid, PIDGain_t gain, float value) {
	switch (gain) {
	case PID_GAIN_KP:
		pid->Kp = value;
		break;
	case PID_GAIN_KI:
		pid->Ki = value;
		break;
	case PID_GAIN_KD:
		pid->Kd = value;
		break;
	default:
		break; /* 非法的增益选择：忽略，保持原参数 */
	}
}

/**
 * @brief  清积分与误差历史（切换工况或重新使能时调用）
 */
void PID_Reset(PID_t *pid) {
	pid->Error0   = 0.0f;
	pid->Error1   = 0.0f;
	pid->ErrorInt = 0.0f;
	pid->Out      = 0.0f;
}

/* ===================== 单步计算 =========================================== */

/**
 * @brief  单步位置式 PID 计算
 * @param  pid : 控制器实例（调用前须写好 Target 与 Actual，不可并发调用）
 * @note   结果写入 pid->Out（已限幅到 [OutMin, OutMax]）；
 *         调用周期须与增益离散化时的采样周期一致（本工程内环 50Hz）。
 */
void PID_Update(PID_t *pid) {
	/* 1) 误差刷新：Error1=上次误差，Error0=本次误差 */
	pid->Error1 = pid->Error0;
	pid->Error0 = pid->Target - pid->Actual;

	/* 2) 积分累加：Ki 不为 0 才累加，否则清零（便于调试） */
	if (pid->Ki != 0.0f) {
		pid->ErrorInt += pid->Error0;
	} else {
		pid->ErrorInt = 0.0f;
	}

	/* 3) 积分限幅抗饱和（i_limit > 0 启用） */
	if (pid->i_limit > 0.0f) {
		if (pid->ErrorInt > pid->i_limit) {
			pid->ErrorInt = pid->i_limit;
		} else if (pid->ErrorInt < -pid->i_limit) {
			pid->ErrorInt = -pid->i_limit;
		}
	}

	/* 4) 位置式 PID 输出 */
	pid->Out = pid->Kp * pid->Error0
	         + pid->Ki * pid->ErrorInt
	         + pid->Kd * (pid->Error0 - pid->Error1);

	/* 5) 输出限幅 */
	if (pid->Out > pid->OutMax) {
		pid->Out = pid->OutMax;
	}
	if (pid->Out < pid->OutMin) {
		pid->Out = pid->OutMin;
	}
}
