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

/**
 * @brief  按参数表初始化并复位运行态（参数表只读，越界项就地回退到安全值）
 * @param  pid : 控制器实例； cfg : 增益/限幅/死区/微分滤波系数
 */
void PID_InitFromCfg(PID_t *pid, const PIDCfg_t *cfg) {
	pid->kp		 = cfg->kp;
	pid->ki		 = cfg->ki;
	pid->kd		 = cfg->kd;
	pid->out_min = cfg->out_min;
	pid->out_max = cfg->out_max;
	pid->i_limit = cfg->i_limit;

	/* 可选扩展项就地校验（越界 → 关闭死区 / 微分直通），不再转调独立 setter */
	pid->deadband = (cfg->deadband > 0.0f) ? cfg->deadband : 0.0f;
	pid->d_alpha  = ((cfg->d_alpha > 0.0f) && (cfg->d_alpha <= 1.0f)) ? cfg->d_alpha : 1.0f;

	PID_Reset(pid);
}

/**
 * @brief  在线改单个增益（供上位机整定，不清运行态）
 * @param  pid : 控制器实例； gain : KP/KI/KD； value : 新增益（每采样步离散化）
 */
void PID_SetOneGain(PID_t *pid, PIDGain_t gain, float value) {
	switch (gain) {
	case PID_GAIN_KP:
		pid->kp = value;
		break;
	case PID_GAIN_KI:
		pid->ki = value;
		break;
	case PID_GAIN_KD:
		pid->kd = value;
		break;
	default:
		break; /* 非法的增益选择：忽略，保持原参数 */
	}
}

/**
 * @brief  清积分/微分历史并置首拍标志（切换工况或重新使能时调用）
 */
void PID_Reset(PID_t *pid) {
	pid->integral	= 0.0f;
	pid->prev_err	= 0.0f;
	pid->d_filt		= 0.0f;
	pid->first_run	= 1U;
}

/* ===================== 单步计算 =========================================== */

/**
 * @brief  单步位置式 PID 计算
 * @param  pid      : 控制器实例（内含运行态，不可并发调用）
 * @param  setpoint : 设定值； feedback : 反馈量（与 setpoint 同量纲）
 * @retval 限幅后的控制输出（out_min~out_max）
 * @note   调用周期须与增益离散化时的采样周期一致（本工程内环 50Hz、外环 20Hz）；
 *         首拍 D 项按 0 处理，无外部依赖、可主机侧单测
 */
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
