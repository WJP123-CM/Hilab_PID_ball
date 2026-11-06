/**
 ******************************************************************************
 * @file    kalman.h
 * @brief   基础卡尔曼滤波观测器（2 维恒速 CV 模型，可复用模块，无外部依赖）
 *
 *          状态向量 x = [position, velocity]^T，预测模型：
 *            pos' = pos + vel*dt,  vel' = vel
 *          观测通道（各自独立的观测噪声 R，可混合使用）：
 *            - 位置观测 H=[1,0]：编码器计数换算位置 / VL53L1X 测距值
 *            - 速度观测 H=[0,1]：编码器差分速度（预留扩展）
 *
 *          融合方式：同一滤波器实例上，编码器与测距各自调用 KF_UpdatePos()
 *          并传入各自的 R —— 噪声小（R 小）的源权重自动更高，无需额外逻辑。
 *
 *          过程噪声单参数化：只给一个加速度噪声方差 sig_a2（位置单位^2/s^4），
 *          Q 由 dt 在 KF_Predict 内生成（连续白噪声加速度模型）：
 *            q00 = sig_a2·dt³/3,  q01 = q10 = sig_a2·dt²/2,  q11 = sig_a2·dt
 *          好处：噪声参数只有 1 个，换预测周期时不必重新整定；
 *          且 q01 非零，位置/速度协方差不再是各自独立增长——纯对角 Q 会让
 *          速度估计滞后于真实运动（CV 模型的交叉项本应存在）。
 *          取值判据：令每拍位置方差增量 q00 ≈ 观测噪声方差 R，此时稳态增益
 *          K ≈ 0.5~0.6，等效 EMA 系数 α≈0.55、时间常数 ≈2 拍。
 *
 *          用法（3 步闭环）：
 *            1) 上电一次：KF_Init(&kf, ...);
 *            2) 每个控制周期：KF_Predict(&kf);
 *            3) 有观测时：KF_UpdatePos(&kf, z, R_SENSOR);  读结果 KF_GetPos/Vel()
 *          无观测（测距 status!=0 等）时跳过 update 只 predict，协方差自然增长，
 *          观测恢复后增益自动回升重新收敛。
 *
 *          协方差只存对称阵的 3 个元素 p00/p01/p11（p10 恒等于 p01），
 *          更新后只需非负钳位，不再需要"对称平均"修补（2026-11-05）。
 *
 *          数值硬化：速度限幅 vel_max 抑制尖峰对速度态的污染；
 *          KF_UpdatePosGated 提供新息门控（孤立尖峰剔除，连续越限自动放行）。
 *
 *          ===================== 实际接入（本工程 tof_filter.c）=============
 *          // 首帧有效观测时初始化：测距域 mm，R = 静止实测 σ=1.87mm → 3.5mm²
 *          // sig_a2 取 1e5：主机实测 1e5 是速度滞后/噪声的拐点（26ms 采样，
 *          // 位置 RMSE 2.7mm，速度滞后 1 拍，静止速度抖动 32mm/s）
 *          KF_Init(&f->kf, (float) raw_mm, 0.0f, 0.026f, 1.0e5f, 1000.0f);
 *          KF_SetVelLimit(&f->kf, 500.0f);   // 球在杆上物理速度上限约 480mm/s
 *
 *          // 每拍（Tof_Tick）：dt 取真实帧间隔并限幅 5~60ms
 *          KF_SetDt(&f->kf, dt);
 *          KF_Predict(&f->kf);               // 无效帧也预测，协方差自然增长
 *          if (valid) {
 *              KF_UpdatePos(&f->kf, (float) raw_mm, 3.5f); // 不带门控（实测会锁死）
 *          }
 *          float pos = KF_GetPos(&f->kf);   // -> pos_cm_x10 = pos - zero_mm
 *          float vel = KF_GetVel(&f->kf);   // 外环 D 项 / 遥测（预留）
 *          =====================================================================
 *
 *          注意：F103 无 FPU，float 为软件浮点；20ms 周期下本模块每拍约
 *          数十次浮点乘加，开销可忽略。严禁对输出使用 %f 打印（工程禁用）。
 ******************************************************************************
 */
#ifndef __KALMAN_H
#define __KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 默认发散保护阈值：协方差对角线超过该值判定发散，自动复位保护 */
#define KF_DEFAULT_MAX_COV 1.0e9f

/** 默认速度限幅（位置单位/s）：按被测对象物理速度上限设，抑制尖峰污染速度态 */
#define KF_DEFAULT_VEL_MAX 1000.0f

/** 默认加速度噪声方差（位置单位^2/s^4）：KF_Init 传入非法值时的兜底。
 *  本工程测距域推导约 6.0e5 (mm^2/s^4)，对应 σ_a ≈ 770mm/s²（杆倾 4.5° 的 g·sinθ） */
#define KF_DEFAULT_SIG_A2 1.0e6f

/**
 * @brief  卡尔曼滤波器实例（2 维 CV 模型）
 *         参数字段可经 KF_Set* 接口在线修改；运行态字段由模块内部维护，
 *         外部只读（如经 KF_GetPos/GetVel），勿直接改写。
 */
typedef struct {
	/* ---- 配置参数 ---- */
	float	dt;			/* 预测步长(s)，须与调用 KF_Predict 的实际周期一致 */
	float	sig_a2;		/* 加速度噪声方差(位置单位^2/s^4)：越大跟踪越快、越不平滑 */
	float	max_cov;	/* 发散保护阈值（协方差上限），默认 KF_DEFAULT_MAX_COV */
	float	vel_max;	/* 速度限幅（位置单位/s），默认 KF_DEFAULT_VEL_MAX */

	/* ---- 状态估计（输出） ---- */
	float	pos;		/* 位置估计 */
	float	vel;		/* 速度估计 */

	/* ---- 协方差矩阵 P 的上三角（对称，p10 恒等于 p01 故不单独存） ---- */
	float	p00, p01, p11;

	/* ---- 诊断 ---- */
	uint16_t diverge_cnt;	/* 发散保护复位次数（观测用，正常应恒为 0） */
	uint16_t outlier_cnt;	/* 门控拒绝次数（仅 KF_UpdatePosGated 累计） */
	uint8_t	 gate_streak;	/* 门控连续越限计数（内部用，达放行拍数后清零） */
} KF_t;

/**
 * @brief  初始化滤波器
 * @param  kf        : 滤波器实例指针
 * @param  init_pos  : 初始位置（不确定可传 0）
 * @param  init_vel  : 初始速度（不确定可传 0）
 * @param  dt        : 固定预测周期(s)，如 20ms 传 0.02f；非法值内部夹为 0.02f
 * @param  sig_a2    : 加速度噪声方差(位置单位^2/s^4)，典型 1e5~1e7
 * @param  init_cov  : 初始协方差（初始不确定度），典型 1e3~1e6，越大收敛越快
 */
void KF_Init(KF_t *kf, float init_pos, float init_vel, float dt,
			 float sig_a2, float init_cov);

/** @brief 在线修改预测步长（>0，非法值忽略） */
void KF_SetDt(KF_t *kf, float dt);

/** @brief 在线修改加速度噪声方差（>0，非法值忽略） */
void KF_SetAccNoise(KF_t *kf, float sig_a2);

/** @brief 在线修改发散保护阈值（>0，非法值忽略） */
void KF_SetMaxCov(KF_t *kf, float max_cov);

/** @brief 在线修改速度限幅（>0，非法值忽略） */
void KF_SetVelLimit(KF_t *kf, float vel_max);

/** @brief 复位状态估计与协方差（切换工况时建议调用；配置参数保留） */
void KF_Reset(KF_t *kf, float pos, float vel, float cov);

/**
 * @brief  状态预测（推进一拍）：pos += vel*dt，Q 由 sig_a2 与 dt 生成后
 *         按 P' = F P F^T + Q 传播
 * @retval 1 正常；0 参数非法或触发发散保护（已自动复位，协方差置大）
 * @note   必须按固定周期调用（与 dt 一致），即使当拍没有任何观测也要调用
 */
int KF_Predict(KF_t *kf);

/**
 * @brief  带控制输入的预测（球杆系统预留接口，暂不接入，KF_Predict 等价 a=0）
 * @param  a : 已知加速度，单位与 pos 一致（本工程 mm/s²）；NaN/Inf 按 0 处理
 * @retval 同 KF_Predict
 * @note   a 是确定性输入（杆倾角决定的加速度），不引入不确定度，Q 不变。
 *
 * ===================== 接入方案（标定后再启用）=====================
 * 数据源：上一拍编码器快照（Tof_Tick 在 Encoder_Tick 之前执行，天然滞后一拍）
 *   a_mm_s2 = K_ACC_PER_COUNT * (g_enc.total - THETA_ZERO)
 *
 * THETA_ZERO：球静止、闭环稳住时读一次 g_enc.total（不是装配值 660）
 * K_ACC_PER_COUNT：由"θ 覆盖量程 + 球自由滚动"数据线性回归得到
 *   对每段 10~15 帧位置拟合 p0+v0*t+a*t²/2 求 a，再回归 a vs counts
 *   预期量级 1~2 mm/s²/count（200counts≈3° 时 a≈367mm/s²）
 *
 * 接入后 sig_a2 应从 6e5 降至 1e5 或更低（原"杆角决定的那部分加速度"由 a 承担）
 * 注意：不要用 encoder_pid.Target（指令角超前一拍）；实际 θ 用 g_enc.total
 * ==================================================================
 */
int KF_PredictAccel(KF_t *kf, float a);

/**
 * @brief  位置观测更新（标量卡尔曼更新，H=[1,0]）
 * @param  z : 位置观测值（编码器位置 / 测距距离，单位与 pos 一致）
 * @param  r : 该传感器的观测噪声方差（单位^2）：R 越小表示越信任该源
 * @retval 1 已融合；0 输入非法（z 为 NaN/Inf 或 r<=0/NaN/Inf），已跳过本拍
 * @note   测距无效帧（status!=0）不要调用本函数，直接跳过即可
 */
int KF_UpdatePos(KF_t *kf, float z, float r);

/**
 * @brief  位置观测更新 + 新息门控（孤立尖峰抑制）
 * @param  z      : 位置观测值
 * @param  r      : 观测噪声方差
 * @param  gate_n : 门限倍数，innov² > gate_n²·S 判定为离群（典型 3.0）
 * @retval 1 已融合；0 输入非法；2 被门控拒绝（已计 outlier_cnt）
 * @note   连续 KF_GATE_RELEASE_N 拍越限自动放行（真实台阶不被锁死）；
 *         已经中值等前置剔除尖峰的链路直接用 KF_UpdatePos 即可
 */
int KF_UpdatePosGated(KF_t *kf, float z, float r, float gate_n);

/**
 * @brief  速度观测更新（H=[0,1]，预留扩展：编码器差分速度等）
 * @param  z : 速度观测值（单位/ s）
 * @param  r : 观测噪声方差
 * @retval 1 已融合；0 输入非法，已跳过
 */
int KF_UpdateVel(KF_t *kf, float z, float r);

/** @brief 读位置估计 */
float KF_GetPos(const KF_t *kf);

/** @brief 读速度估计 */
float KF_GetVel(const KF_t *kf);

/** @brief 读位置估计方差（p00，评估当前估计置信度，调试用；不开方以避 libm） */
float KF_GetPosVariance(const KF_t *kf);

#ifdef __cplusplus
}
#endif

#endif /* __KALMAN_H */
