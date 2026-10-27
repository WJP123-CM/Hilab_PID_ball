/**
 ******************************************************************************
 * @file    pid.c
 * @brief   标准单环位置式 PID 控制器实现
 *          计算流程（每步）：
 *            1) err = setpoint - feedback（可选死区处理）
 *            2) I 项：integral += Ki*err，随后限幅到 ±i_limit（抗积分饱和）
 *            3) D 项：Δe 差分（首拍置 0 防踢变），可选一阶低通滤波
 *            4) out = Kp*err + integral + Kd*d_filt，最后输出限幅
 *          无外部依赖：不使用 HAL/libm，便于复用与宿主机仿真验证。
 ******************************************************************************
 */
#include "pid.h"

/* ===================== 初始化 / 配置接口 ================================== */

void PID_Init(PID_t *pid, float kp, float ki, float kd,
			  float out_min, float out_max, float integral_limit) {
	pid->kp			= kp;
	pid->ki			= ki;
	pid->kd			= kd;
	pid->out_min	= out_min;
	pid->out_max	= out_max;
	pid->i_limit	= integral_limit;

	/* 预留扩展项默认关闭：死区 0、微分直通 */
	pid->deadband	= 0.0f;
	pid->d_alpha	= 1.0f;

	PID_Reset(pid);
}

void PID_SetGains(PID_t *pid, float kp, float ki, float kd) {
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

void PID_SetOutputLimit(PID_t *pid, float out_min, float out_max) {
	pid->out_min = out_min;
	pid->out_max = out_max;
}

void PID_SetIntegralLimit(PID_t *pid, float integral_limit) {
	pid->i_limit = integral_limit;
}

void PID_SetDeadband(PID_t *pid, float deadband) {
	pid->deadband = (deadband > 0.0f) ? deadband : 0.0f;
}

void PID_SetDerivativeFilter(PID_t *pid, float alpha) {
	/* 有效范围 (0,1]：越界值一律按"直通"处理，避免滤波器发散 */
	if ((alpha > 0.0f) && (alpha <= 1.0f)) {
		pid->d_alpha = alpha;
	} else {
		pid->d_alpha = 1.0f;
	}
}

void PID_Reset(PID_t *pid) {
	pid->integral	= 0.0f;
	pid->prev_err	= 0.0f;
	pid->d_filt		= 0.0f;
	pid->first_run	= 1U;
}

/* ===================== 单步计算 =========================================== */

float PID_Update(PID_t *pid, float setpoint, float feedback) {
	float err = setpoint - feedback;
	float d_raw;
	float out;

	/* 死区（预留扩展）：误差在死区内按 0 处理，控制器不响应微小波动 */
	if ((pid->deadband > 0.0f) && (err < pid->deadband) && (err > -pid->deadband)) {
		err = 0.0f;
	}

	/* ---- I 项：先累加后限幅（抗积分饱和：设定值阶跃/输出长时间受限时，
	 *      积分不再无限累积，避免退饱和后的超调与震荡） ---- */
	pid->integral += pid->ki * err;
	if (pid->integral > pid->i_limit) {
		pid->integral = pid->i_limit;
	} else if (pid->integral < -pid->i_limit) {
		pid->integral = -pid->i_limit;
	}

	/* ---- D 项：误差差分；首拍跳过（prev_err 尚无意义），可选一阶低通 ---- */
	d_raw = (pid->first_run != 0U) ? 0.0f : (err - pid->prev_err);
	pid->first_run = 0U;
	pid->prev_err	= err;
	if (pid->d_alpha < 1.0f) {
		pid->d_filt += pid->d_alpha * (d_raw - pid->d_filt);
	} else {
		pid->d_filt = d_raw;
	}

	/* ---- 合成输出并限幅 ---- */
	out = pid->kp * err + pid->integral + pid->kd * pid->d_filt;
	if (out > pid->out_max) {
		out = pid->out_max;
	} else if (out < pid->out_min) {
		out = pid->out_min;
	}
	return out;
}
