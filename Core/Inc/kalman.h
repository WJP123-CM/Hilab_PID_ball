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
 *          用法（3 步闭环）：
 *            1) 上电一次：KF_Init(&kf, ...);
 *            2) 每个固定控制周期（本工程 20ms）：KF_Predict(&kf);
 *            3) 有观测时：KF_UpdatePos(&kf, z, R_SENSOR);  读结果 KF_GetPos/Vel()
 *          无观测（测距 status!=0 等）时跳过 update 只 predict，协方差自然增长，
 *          观测恢复后增益自动回升重新收敛。
 *
 *          ===================== 接入示例（本工程 main.c）=====================
 *          static KF_t s_kf;
 *          // 初始化（上电一次）：dt=20ms 控制周期
 *          KF_Init(&s_kf, 0.0f, 0.0f, 0.02f, 5.0f, 200.0f, 1000.0f);
 *
 *          // TIM1 回调 20ms 分支内（沿用现有测距+VOFA 代码位置）：
 *          KF_Predict(&s_kf);
 *          // 通道 1：编码器位置（TIM3 计数 -> 物理位置，系数按机械结构换算）
 *          float enc_pos = (float)(int16_t)__HAL_TIM_GET_COUNTER(&htim3)
 *                          * ENC_MM_PER_COUNT;
 *          KF_UpdatePos(&s_kf, enc_pos, R_ENCODER);   // 如 4.0f (mm^2)
 *          // 通道 2：测距（仅有效帧喂入，无效帧跳过即自动纯预测）
 *          if (range_status == 0) {
 *              KF_UpdatePos(&s_kf, (float)distance_mm, R_RANGER); // 如 900.0f
 *          }
 *          float pos = KF_GetPos(&s_kf);   // 融合后位置 -> 送 PID 反馈
 *          float vel = KF_GetVel(&s_kf);   // 融合后速度
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

/**
 * @brief  卡尔曼滤波器实例（2 维 CV 模型）
 *         参数字段可经 KF_Set* 接口在线修改；运行态字段由模块内部维护，
 *         外部只读（如经 KF_GetPos/GetVel），勿直接改写。
 */
typedef struct {
	/* ---- 配置参数 ---- */
	float	dt;			/* 预测步长(s)，须与调用 KF_Predict 的实际周期一致 */
	float	q_pos;		/* 过程噪声-位置项(mm^2)：越大跟踪越快、越不平滑 */
	float	q_vel;		/* 过程噪声-速度项(mm^2/s^2)：速度突变跟踪能力 */
	float	max_cov;	/* 发散保护阈值（协方差上限），默认 KF_DEFAULT_MAX_COV */

	/* ---- 状态估计（输出） ---- */
	float	pos;		/* 位置估计 */
	float	vel;		/* 速度估计 */

	/* ---- 协方差矩阵 P（2x2，内部维护） ---- */
	float	p00, p01;
	float	p10, p11;

	/* ---- 诊断 ---- */
	uint16_t diverge_cnt;	/* 发散保护复位次数（观测用，正常应恒为 0） */
} KF_t;

/**
 * @brief  初始化滤波器
 * @param  kf        : 滤波器实例指针
 * @param  init_pos  : 初始位置（不确定可传 0）
 * @param  init_vel  : 初始速度（不确定可传 0）
 * @param  dt        : 固定预测周期(s)，如 20ms 传 0.02f；非法值内部夹为 0.02f
 * @param  q_pos     : 过程噪声-位置项(mm^2)，典型 1~100，越大响应越快
 * @param  q_vel     : 过程噪声-速度项(mm^2/s^2)，典型 10~1000
 * @param  init_cov  : 初始协方差（初始不确定度），典型 1e3~1e6，越大收敛越快
 */
void KF_Init(KF_t *kf, float init_pos, float init_vel, float dt,
			 float q_pos, float q_vel, float init_cov);

/** @brief 在线修改预测步长（>0，非法值忽略） */
void KF_SetDt(KF_t *kf, float dt);

/** @brief 在线修改过程噪声（预留：后续自适应调参入口） */
void KF_SetProcessNoise(KF_t *kf, float q_pos, float q_vel);

/** @brief 在线修改发散保护阈值（>0，非法值忽略） */
void KF_SetMaxCov(KF_t *kf, float max_cov);

/** @brief 复位状态估计与协方差（切换工况时建议调用；配置参数保留） */
void KF_Reset(KF_t *kf, float pos, float vel, float cov);

/**
 * @brief  状态预测（推进一拍）：pos += vel*dt，协方差按 F P F^T + Q 传播
 * @retval 1 正常；0 参数非法或触发发散保护（已自动复位，协方差置大）
 * @note   必须按固定周期调用（与 dt 一致），即使当拍没有任何观测也要调用
 */
int KF_Predict(KF_t *kf);

/**
 * @brief  位置观测更新（标量卡尔曼更新，H=[1,0]）
 * @param  z : 位置观测值（编码器位置 / 测距距离，单位与 pos 一致）
 * @param  r : 该传感器的观测噪声方差（单位^2）：R 越小表示越信任该源
 * @retval 1 已融合；0 输入非法（z 为 NaN/Inf 或 r<=0/NaN/Inf），已跳过本拍
 * @note   测距无效帧（status!=0）不要调用本函数，直接跳过即可
 */
int KF_UpdatePos(KF_t *kf, float z, float r);

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
