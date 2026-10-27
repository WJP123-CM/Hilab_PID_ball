/**
 ******************************************************************************
 * @file    kalman.c
 * @brief   基础卡尔曼滤波观测器实现（2 维恒速 CV 模型）
 *
 *          预测：x' = F x,  P' = F P F^T + Q,  F=[[1,dt],[0,1]], Q=diag(q_pos,q_vel)
 *          更新：标量观测 z = H x + v，按 H=[1,0]（位置）/ [0,1]（速度）分别实现
 *
 *          无外部依赖：不使用 HAL/libm（含 isfinite/sqrtf 均手写规避），
 *          可在 ISR 或主循环直接调用；20ms 周期下每拍约数十次浮点乘加。
 ******************************************************************************
 */
#include "kalman.h"

/** 观测噪声方差下限：防止 R=0 导致增益除零/数值病态 */
#define KF_R_MIN 1.0e-6f

/** 默认预测步长：KF_Init 传入非法 dt 时的兜底值（对应本工程 20ms 节拍） */
#define KF_DEFAULT_DT 0.02f

/**
 * @brief 有限性检查（手写，规避 libm 的 isfinite 与 -ffast-math 兼容性问题）
 *        IEEE754：NaN 自比较为假；Inf 自减得 NaN。仅限非 fast-math 编译（本工程未启用）。
 */
static int kf_is_finite(float v) {
	return (v == v) && ((v - v) == 0.0f);
}

/**
 * @brief 发散保护：协方差非法或超限时自动复位（保位置、清速度、协方差置大）
 * @retval 1 健康；0 触发了保护复位
 */
static int kf_guard(KF_t *kf) {
	if (!kf_is_finite(kf->p00) || !kf_is_finite(kf->p11) ||
		kf->p00 > kf->max_cov || kf->p11 > kf->max_cov ||
		!kf_is_finite(kf->pos) || !kf_is_finite(kf->vel)) {
		float keep_pos = kf_is_finite(kf->pos) ? kf->pos : 0.0f;
		kf->vel = 0.0f;
		kf->pos = keep_pos;
		kf->p00 = kf->max_cov;	/* 协方差置大：等价于"重新不确定"，靠后续观测重新收敛 */
		kf->p11 = kf->max_cov;
		kf->p01 = 0.0f;
		kf->p10 = 0.0f;
		kf->diverge_cnt++;
		return 0;
	}
	return 1;
}

/* ===================== 初始化 / 配置接口 ================================== */

void KF_Init(KF_t *kf, float init_pos, float init_vel, float dt,
			 float q_pos, float q_vel, float init_cov) {
	kf->dt		= (kf_is_finite(dt) && dt > 0.0f) ? dt : KF_DEFAULT_DT;
	kf->q_pos	= (kf_is_finite(q_pos) && q_pos > 0.0f) ? q_pos : 1.0f;
	kf->q_vel	= (kf_is_finite(q_vel) && q_vel > 0.0f) ? q_vel : 100.0f;
	kf->max_cov	= KF_DEFAULT_MAX_COV;
	kf->diverge_cnt = 0U;
	KF_Reset(kf, init_pos, init_vel, init_cov);
}

void KF_SetDt(KF_t *kf, float dt) {
	if (kf_is_finite(dt) && dt > 0.0f) {
		kf->dt = dt;
	}
}

void KF_SetProcessNoise(KF_t *kf, float q_pos, float q_vel) {
	if (kf_is_finite(q_pos) && q_pos > 0.0f) {
		kf->q_pos = q_pos;
	}
	if (kf_is_finite(q_vel) && q_vel > 0.0f) {
		kf->q_vel = q_vel;
	}
}

void KF_SetMaxCov(KF_t *kf, float max_cov) {
	if (kf_is_finite(max_cov) && max_cov > 0.0f) {
		kf->max_cov = max_cov;
	}
}

void KF_Reset(KF_t *kf, float pos, float vel, float cov) {
	kf->pos = kf_is_finite(pos) ? pos : 0.0f;
	kf->vel = kf_is_finite(vel) ? vel : 0.0f;
	if (!kf_is_finite(cov) || cov <= 0.0f) {
		cov = 1000.0f;
	}
	kf->p00 = cov;
	kf->p11 = cov;
	kf->p01 = 0.0f;
	kf->p10 = 0.0f;
}

/* ===================== 预测 / 更新 ======================================== */

int KF_Predict(KF_t *kf) {
	float dt = kf->dt;
	float p00, p01, p10, p11;

	if (!kf_is_finite(dt) || dt <= 0.0f) {
		return 0;
	}

	/* 状态外推：pos' = pos + vel*dt */
	kf->pos += kf->vel * dt;

	/* 协方差传播：P' = F P F^T + Q（展开形式，避免矩阵库） */
	p00 = kf->p00;
	p01 = kf->p01;
	p10 = kf->p10;
	p11 = kf->p11;
	kf->p00 = p00 + dt * (p10 + p01) + dt * dt * p11 + kf->q_pos;
	kf->p01 = p01 + dt * p11;
	kf->p10 = p10 + dt * p11;
	kf->p11 = p11 + kf->q_vel;

	return kf_guard(kf);
}

int KF_UpdatePos(KF_t *kf, float z, float r) {
	float s, k0, k1, innov;
	float p00, p01, p10, p11;

	/* 输入合法性：观测值须有限，R 须为有限正数 */
	if (!kf_is_finite(z) || !kf_is_finite(r) || r <= 0.0f) {
		return 0;
	}
	if (r < KF_R_MIN) {
		r = KF_R_MIN;
	}

	p00 = kf->p00;
	p01 = kf->p01;
	p10 = kf->p10;
	p11 = kf->p11;

	/* H=[1,0]：S = p00 + R，K = [p00, p10]^T / S */
	s		= p00 + r;
	k0		= p00 / s;
	k1		= p10 / s;
	innov	= z - kf->pos;
	kf->pos += k0 * innov;
	kf->vel += k1 * innov;

	/* P = (I - K H) P（Joseph 简化形，CV 模型下保持对称正定的常用近似） */
	kf->p00 = (1.0f - k0) * p00;
	kf->p01 = (1.0f - k0) * p01;
	kf->p10 = p10 - k1 * p00;
	kf->p11 = p11 - k1 * p01;

	return kf_guard(kf);
}

int KF_UpdateVel(KF_t *kf, float z, float r) {
	float s, k0, k1, innov;
	float p00, p01, p10, p11;

	if (!kf_is_finite(z) || !kf_is_finite(r) || r <= 0.0f) {
		return 0;
	}
	if (r < KF_R_MIN) {
		r = KF_R_MIN;
	}

	p00 = kf->p00;
	p01 = kf->p01;
	p10 = kf->p10;
	p11 = kf->p11;

	/* H=[0,1]：S = p11 + R，K = [p01, p11]^T / S */
	s		= p11 + r;
	k0		= p01 / s;
	k1		= p11 / s;
	innov	= z - kf->vel;
	kf->pos += k0 * innov;
	kf->vel += k1 * innov;

	kf->p00 = p00 - k0 * p10;
	kf->p01 = p01 - k0 * p11;
	kf->p10 = (1.0f - k1) * p10;
	kf->p11 = (1.0f - k1) * p11;

	return kf_guard(kf);
}

/* ===================== 输出 ============================================== */

float KF_GetPos(const KF_t *kf) {
	return kf->pos;
}

float KF_GetVel(const KF_t *kf) {
	return kf->vel;
}

float KF_GetPosVariance(const KF_t *kf) {
	return kf->p00;
}
