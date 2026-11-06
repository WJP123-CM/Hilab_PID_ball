/**
 ******************************************************************************
 * @file    kalman.c
 * @brief   基础卡尔曼滤波观测器实现（2 维恒速 CV 模型）
 *
 *          预测：x' = F x,  P' = F P F^T + Q,  F=[[1,dt],[0,1]]
 *          Q 由单参数 sig_a2 与 dt 生成（连续白噪声加速度模型）：
 *            q00 = sig_a2·dt³/3,  q01 = q10 = sig_a2·dt²/2,  q11 = sig_a2·dt
 *          更新：标量观测 z = H x + v，按 H=[1,0]（位置）/ [0,1]（速度）分别实现
 *
 *          无外部依赖：不使用 HAL/libm（含 isfinite/sqrtf 均手写规避），
 *          可在 ISR 或主循环直接调用；20ms 周期下每拍约数十次浮点乘加。
 *
 *          协方差只存上三角 p00/p01/p11（p10 恒等于 p01），故更新后只需
 *          对角非负钳位，不再需要对称平均修补（2026-11-05 简化）。
 *
 *          数值硬化（2026-10-29）：负方差纳入发散判定、速度限幅 vel_max、
 *          新增新息门控 KF_UpdatePosGated。
 ******************************************************************************
 */
#include "kalman.h"

/** 观测噪声方差下限：防止 R=0 导致增益除零/数值病态 */
#define KF_R_MIN 1.0e-6f

/** 默认预测步长：KF_Init 传入非法 dt 时的兜底值（对应本工程 20ms 节拍） */
#define KF_DEFAULT_DT 0.02f

/** 门控放行拍数：连续越限达到该值按真实台阶放行（防锁死） */
#define KF_GATE_RELEASE_N 3u

/**
 * @brief 有限性检查（手写，规避 libm 的 isfinite 与 -ffast-math 兼容性问题）
 *        IEEE754：NaN 自比较为假；Inf 自减得 NaN。仅限非 fast-math 编译（本工程未启用）。
 */
static int kf_is_finite(float v) {
	return (v == v) && ((v - v) == 0.0f);
}

/**
 * @brief 速度限幅：抑制尖峰/发散对速度态的污染（工程抗饱和，非理论环节）
 */
static void kf_clamp_vel(KF_t *kf) {
	if (kf->vel > kf->vel_max) {
		kf->vel = kf->vel_max;
	} else if (kf->vel < -kf->vel_max) {
		kf->vel = -kf->vel_max;
	}
}

/**
 * @brief 协方差钳位：非负对角 + 正定性（|p01| <= sqrt(p00*p11)）
 *        简单形式 (I-KH)P 在单精度舍入下对角可能变负、交叉项可能越界，
 *        对角钳零后 p01 仍是独立状态量，故需按正定性再收一次
 *        （对称性由只存上三角天然保证）
 */
static void kf_clamp_cov(KF_t *kf) {
	float lim2;

	if (kf->p00 < 0.0f) {
		kf->p00 = 0.0f;
	}
	if (kf->p11 < 0.0f) {
		kf->p11 = 0.0f;
	}

	lim2 = kf->p00 * kf->p11; /* 用平方比较，避免 sqrtf */
	if (kf->p01 * kf->p01 > lim2) {
		kf->p01 = (lim2 > 0.0f) ? (lim2 / kf->p01) : 0.0f; /* 缩到边界内，保号 */
	}
}

/**
 * @brief 发散保护：协方差非法/为负/超限时自动复位（保位置、清速度、协方差置大）
 * @retval 1 健康；0 触发了保护复位
 */
static int kf_guard(KF_t *kf) {
	if (!kf_is_finite(kf->p00) || !kf_is_finite(kf->p01) || !kf_is_finite(kf->p11) ||
		kf->p00 < 0.0f || kf->p11 < 0.0f ||
		kf->p00 > kf->max_cov || kf->p11 > kf->max_cov ||
		!kf_is_finite(kf->pos) || !kf_is_finite(kf->vel)) {
		float keep_pos = kf_is_finite(kf->pos) ? kf->pos : 0.0f;
		kf->vel = 0.0f;
		kf->pos = keep_pos;
		kf->p00 = kf->max_cov;	/* 协方差置大：等价于"重新不确定"，靠后续观测重新收敛 */
		kf->p11 = kf->max_cov;
		kf->p01 = 0.0f;
		kf->diverge_cnt++;
		return 0;
	}
	return 1;
}

/* ===================== 初始化 / 配置接口 ================================== */

void KF_Init(KF_t *kf, float init_pos, float init_vel, float dt,
			 float sig_a2, float init_cov) {
	kf->dt		= (kf_is_finite(dt) && dt > 0.0f) ? dt : KF_DEFAULT_DT;
	kf->sig_a2	= (kf_is_finite(sig_a2) && sig_a2 > 0.0f) ? sig_a2 : KF_DEFAULT_SIG_A2;
	kf->max_cov	= KF_DEFAULT_MAX_COV;
	kf->vel_max	= KF_DEFAULT_VEL_MAX;
	kf->diverge_cnt = 0U;
	kf->outlier_cnt = 0U;
	kf->gate_streak = 0U;
	KF_Reset(kf, init_pos, init_vel, init_cov);
}

void KF_SetDt(KF_t *kf, float dt) {
	if (kf_is_finite(dt) && dt > 0.0f) {
		kf->dt = dt;
	}
}

void KF_SetAccNoise(KF_t *kf, float sig_a2) {
	if (kf_is_finite(sig_a2) && sig_a2 > 0.0f) {
		kf->sig_a2 = sig_a2;
	}
}

void KF_SetMaxCov(KF_t *kf, float max_cov) {
	if (kf_is_finite(max_cov) && max_cov > 0.0f) {
		kf->max_cov = max_cov;
	}
}

void KF_SetVelLimit(KF_t *kf, float vel_max) {
	if (kf_is_finite(vel_max) && vel_max > 0.0f) {
		kf->vel_max = vel_max;
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
	kf->gate_streak = 0U;
}

/* ===================== 预测 / 更新 ======================================== */

int KF_PredictAccel(KF_t *kf, float a) {
	float	dt = kf->dt;
	float	p00, p01, p11;
	float	dt2, q00, q01, q11;

	if (!kf_is_finite(dt) || dt <= 0.0f) {
		return 0;
	}
	if (!kf_is_finite(a)) {
		a = 0.0f;
	}

	/* 状态外推：a 为已知加速度，属确定性输入，不引入不确定度（Q 不变） */
	kf->pos += kf->vel * dt + 0.5f * a * dt * dt;
	kf->vel += a * dt;

	/* 连续白噪声加速度模型的离散 Q：只用 sig_a2 一个参数，随 dt 自动缩放 */
	dt2 = dt * dt;
	q00 = kf->sig_a2 * dt2 * dt / 3.0f;
	q01 = kf->sig_a2 * dt2 * 0.5f;
	q11 = kf->sig_a2 * dt;

	p00 = kf->p00;
	p01 = kf->p01;
	p11 = kf->p11;

	/* P' = F P F^T + Q（展开形式，利用对称性 p10 == p01） */
	kf->p00 = p00 + 2.0f * dt * p01 + dt2 * p11 + q00;
	kf->p01 = p01 + dt * p11 + q01;
	kf->p11 = p11 + q11;

	return kf_guard(kf);
}

/* 无控制输入的兼容入口，等价于 a = 0 */
int KF_Predict(KF_t *kf) {
	return KF_PredictAccel(kf, 0.0f);
}

/* 位置观测更新公共核（H=[1,0] 标量卡尔曼更新；输入合法性由调用方检查） */
static int kf_update_pos_core(KF_t *kf, float z, float r) {
	float s, k0, k1, innov;
	float p00, p01, p11;

	if (r < KF_R_MIN) {
		r = KF_R_MIN;
	}

	p00 = kf->p00;
	p01 = kf->p01;
	p11 = kf->p11;

	/* H=[1,0]：S = p00 + R，K = [p00, p01]^T / S */
	s		= p00 + r;
	k0		= p00 / s;
	k1		= p01 / s;
	innov	= z - kf->pos;
	kf->pos += k0 * innov;
	kf->vel += k1 * innov;

	/* P = (I - K H) P（简单形式；非 Joseph，舍入下对角可能变负，
	 *  故更新后钳位非负，见 kf_clamp_cov） */
	kf->p00 = (1.0f - k0) * p00;
	kf->p01 = (1.0f - k0) * p01;
	kf->p11 = p11 - k1 * p01;

	kf_clamp_cov(kf);
	kf_clamp_vel(kf);
	return kf_guard(kf);
}

int KF_UpdatePos(KF_t *kf, float z, float r) {
	/* 输入合法性：观测值须有限，R 须为有限正数 */
	if (!kf_is_finite(z) || !kf_is_finite(r) || r <= 0.0f) {
		return 0;
	}
	return kf_update_pos_core(kf, z, r);
}

int KF_UpdatePosGated(KF_t *kf, float z, float r, float gate_n) {
	float s, innov;

	if (!kf_is_finite(z) || !kf_is_finite(r) || r <= 0.0f ||
		!kf_is_finite(gate_n) || gate_n <= 0.0f) {
		return 0;
	}

	/* 新息门控：innov² > gate_n²·S 判离群（S = p00 + R 为新息方差） */
	if (r < KF_R_MIN) {
		r = KF_R_MIN;
	}
	s	  = kf->p00 + r;
	innov = z - kf->pos;
	if (innov * innov > gate_n * gate_n * s) {
		kf->outlier_cnt++;
		if (++kf->gate_streak < KF_GATE_RELEASE_N) {
			return 2;
		}
		/* 连续越限达到放行拍数：按真实台阶处理，重新收敛 */
	}
	kf->gate_streak = 0U;
	return kf_update_pos_core(kf, z, r);
}

int KF_UpdateVel(KF_t *kf, float z, float r) {
	float s, k0, k1, innov;
	float p00, p01, p11;

	if (!kf_is_finite(z) || !kf_is_finite(r) || r <= 0.0f) {
		return 0;
	}
	if (r < KF_R_MIN) {
		r = KF_R_MIN;
	}

	p00 = kf->p00;
	p01 = kf->p01;
	p11 = kf->p11;

	/* H=[0,1]：S = p11 + R，K = [p01, p11]^T / S */
	s		= p11 + r;
	k0		= p01 / s;
	k1		= p11 / s;
	innov	= z - kf->vel;
	kf->pos += k0 * innov;
	kf->vel += k1 * innov;

	kf->p00 = p00 - k0 * p01;
	kf->p01 = p01 - k0 * p11;
	kf->p11 = (1.0f - k1) * p11;

	kf_clamp_cov(kf);
	kf_clamp_vel(kf);
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
