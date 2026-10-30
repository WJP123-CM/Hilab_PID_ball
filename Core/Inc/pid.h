/**
 ******************************************************************************
 * @file    pid.h
 * @brief   标准单环位置式 PID 控制器（可复用模块，无外部依赖）
 *          位置式核心：out = Kp*e + ∫Ki*e + Kd*Δe，含输出限幅、积分限幅抗饱和；
 *          预留死区、微分滤波、在线整定等扩展接口。
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
 * @brief  PID 控制器实例
 *         参数字段可经 PID_Set* 接口在线修改；运行态字段由模块内部维护，
 *         外部仅可通过 PID_Reset() 清零，勿直接改写。
 */
typedef struct {
	/* ---- 控制参数（运行期可调，属应用层动态量） ---- */
	float	kp;			/* 比例增益 */
	float	ki;			/* 积分增益（按采样步长离散化，见 PID_Update 说明） */
	float	kd;			/* 微分增益 */
	float	out_min;	/* 输出下限 */
	float	out_max;	/* 输出上限 */
	float	i_limit;	/* 积分项限幅阈值：|integral| <= i_limit（抗饱和） */

	/* ---- 预留扩展配置（默认关闭/直通） ---- */
	float	deadband;	/* 误差死区：|err| < deadband 时按 0 处理；0 = 关闭 */
	float	d_alpha;	/* 微分一阶低通系数 0~1：1 = 不滤波（预留） */

	/* ---- 运行态（内部维护） ---- */
	float	integral;	/* 积分项累积值（已含 ki，已限幅） */
	float	prev_err;	/* 上一拍误差（供微分差分） */
	float	d_filt;		/* 微分滤波器状态（预留） */
	uint8_t	first_run;	/* 首拍标志：首拍不输出微分，避免上电瞬间 D 项踢变 */
} PID_t;

/**
 * @brief  控制器配置（一次性初始化用）：把 7 个参数打包，避免长参数列表
 *         在调用处重复书写与顺序错配；字段含义与 PID_t 同名成员一致
 */
typedef struct {
	float kp;	   /* 比例增益 */
	float ki;	   /* 积分增益（按每采样步离散化） */
	float kd;	   /* 微分增益 */
	float out_min; /* 输出下限 */
	float out_max; /* 输出上限 */
	float i_limit; /* 积分项限幅阈值（>0，抗饱和） */
	float deadband;/* 误差死区：|err| < deadband 按 0 处理；0 = 关闭 */
	float d_alpha; /* 微分一阶低通系数 (0,1]：1 = 直通 */
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
 * @param  pid : 控制器实例指针
 * @param  cfg : 参数配置（指向常量表即可，本函数只读取）
 * @note   deadband/d_alpha 就地校验：越界值按"关闭死区/微分直通"处理
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
 * @brief  复位运行态：积分清零、微分历史清零、首拍标志置位
 * @param  pid : 控制器实例指针
 * @note   切换工况/重新上电设定值时调用，避免旧积分拖累
 */
void PID_Reset(PID_t *pid);

/**
 * @brief  单步计算：位置式 PID，每次调用推进一步并即时返回输出
 * @param  pid      : 控制器实例指针
 * @param  setpoint : 目标设定值
 * @param  feedback : 当前反馈值（测量值）
 * @retval PID 输出（已含输出限幅）
 * @note   离散化约定：本函数不含 dt 参数，增益按"每采样步"为单位整定，
 *         要求调用方以固定周期调用（本工程为 TIM1 回调 20ms 分支，dt=0.02s）。
 *         若采样周期改变，需按比例重新整定 ki/kd。
 */
float PID_Update(PID_t *pid, float setpoint, float feedback);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
