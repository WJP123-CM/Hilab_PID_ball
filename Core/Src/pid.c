/**
 ******************************************************************************
 * @file    pid.c
 * @brief   标准单环位置式 PID 控制器实现（命名对齐倒立摆参考工程）
 *          计算流程（每步）：
 *            1) Error0 = Target - Actual
 *            2) ErrorInt += Error0（Ki==0 时清零，避免调试期积分堆积）
 *            3) ErrorInt 限幅到 ±i_limit（抗积分饱和）
 *            4) Out = Kp*Error0 + Ki*ErrorInt + Kd*(Error0 - Error1)
 *            4.5) 制动包络（brake_gain>0 启用）：按 √(2·K·|e|) 动态夹 Out，
 *                 仅误差减小时生效（正在靠近目标时制动，远离时放行）
 *            5) Out 限幅到 [OutMin, OutMax]（固定上限，与包络双重保险）
 *          无 HAL 依赖；使用 math.h 的 sqrtf（F103 软件浮点，20ms 节拍开销可忽略）。
 ******************************************************************************
 */
#include "pid.h"

#include <math.h>


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
	/* 制动包络默认关闭，调用方按需手动赋值（brake_gain > 0 启用） */
	pid->brake_gain = 0.0f;
	pid->brake_min	= 0.0f;
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
	/* 0) 死区判断：误差进入 ±deadzone 时保持上一拍 Out（不重算、不积分），
	 *    速度环靠上一拍输出维持当前转速，避免在目标附近来回抖。
	 *    例外：Target=0（停机）必须走正常计算，否则速度衰减进死区后
	 *    会一直保持上一拍非零输出，电机停不下来 */
	if (pid->deadzone > 0.0f && pid->Target != 0.0f) {
		float e = pid->Target - pid->Actual;
		if (e < pid->deadzone && e > -pid->deadzone) {
			return;
		}
	}

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
	pid->Out = pid->Kp * pid->Error0 + pid->Ki * pid->ErrorInt + pid->Kd * (pid->Error0 - pid->Error1);

	/* 4.5) 制动包络（brake_gain > 0 启用）：Out 权限随剩余误差 √(2·K·|e|) 收缩，
	 *      仅误差减小时生效；绝对上限复用 OutMax，下限 brake_min 防卡死 */
	if (pid->brake_gain > 0.0f) {
		float e	  = pid->Error0;
		float env = sqrtf(2.0f * pid->brake_gain * (e < 0.0f ? -e : e));
		if (pid->brake_min > 0.0f && env < pid->brake_min) env = pid->brake_min;
		if (env > pid->OutMax) env = pid->OutMax;
		if ((pid->Error0 - pid->Error1) < 0.0f) { /* 误差在减小 → 正在靠近目标 */
			if (pid->Out > env) pid->Out = env;
			if (pid->Out < -env) pid->Out = -env;
		}
	}

	/* 5) 输出限幅（固定上限，与包络双重保险） */
	if (pid->Out > pid->OutMax) {
		pid->Out = pid->OutMax;
	}
	if (pid->Out < pid->OutMin) {
		pid->Out = pid->OutMin;
	}
}
