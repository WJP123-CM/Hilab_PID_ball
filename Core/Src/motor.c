/**
 ******************************************************************************
 * @file    motor.c
 * @brief   步进电机驱动实现（D36A：EN/DIR 电平 + TIM2_CH4 STEP 脉冲）
 *          角度环位置式 PID 直接驱动 PWM/DIR；开环频率接口同样直驱。
 ******************************************************************************
 */
#include "motor.h"

#include "encoder.h" /* Encoder_Read：角度环反馈 */
#include "main.h"	 /* Motor_enabled / Motor_direction 引脚标签 */
#include "pid.h"	 /* 角度环位置式 PID */
#include "tim.h"	 /* htim2 */

#include <stdint.h>

/* STEP 脉冲通道：TIM2_CH4 = PA3 */
#define STEP_CHANNEL TIM_CHANNEL_4

/* 编码器 e ↔ 脉冲频率 Hz 极性（内环唯一方向开关）：
 *   1 = 正相关：freq>0 使编码器计数增大
 *   0 = 负相关：freq>0 使编码器计数减小（当前接线） */
#define MOTOR_ENC_HZ_SIGN 0

/* ==================== 硬件下发（TIM2 PWM / GPIO） ======================== */

static bool		s_running = false; /* STEP 脉冲输出状态（内部启停幂等） */
static int32_t	s_cur_freq	  = 0;	   /* 最近一次下发的带符号频率（Hz，供遥测） */
static bool		s_last_dir_cw = true;  /* 最近一次 DIR 方向（换向安全用） */

/* ---- 角度环位置式 PID（编码器计数 → 脉冲频率） ----
 * Target=目标角度(计数), Actual=Encoder_Read(), Out=频率(Hz)。
 * 方向约定：编码器 e ↔ freq 的极性由 MOTOR_ENC_HZ_SIGN 决定。
 * s_angle_target_freq 供遥测；s_last_dir_cw 用于换向安全（换向后本拍不发脉冲）。 */
static PID_t   s_angle_pid;
static int32_t s_angle_target_freq = 0;

/**
 * @brief 校验 TIM2 分频假设、设定 STEP 占空比
 * @note  初始化阶段调用一次；不使能电机、不输出脉冲（PA1/PA2 已由 gpio.c 置低）；
 *        PSC 与 1MHz 计数假设不符即 Error_Handler
 */
void Motor_Init(void) {
	/* 校验 CubeMX 配置的 TIM2 分频与 1MHz 计数假设一致（重新生成时须保持）：
	 * 计数 1tick=1µs → 占空比换算与频率换算才成立，不符即 Error_Handler */
	if (htim2.Instance->PSC != MOTOR_TIM_PSC) {
		Error_Handler();
	}

	/* STEP 占空比 ≈50%：用 (ARR+1)/2 动态计算，远大于驱动芯片对 STEP
	 * 脉宽的最低要求（≥2µs） */
	__HAL_TIM_SET_COMPARE(&htim2, STEP_CHANNEL, (__HAL_TIM_GET_AUTORELOAD(&htim2) + 1u) / 2u);

	/* 角度环 PID 初始化：编码器计数 → 脉冲频率，位置式 PID 直接定角度。
	 * Kp 由调用方（ctrl_p）每拍写入；Ki/Kd 起步为 0，可在线整定。
	 * 输出限幅 ±200Hz（远低于电机矩频上限），i_limit=0 不限积分。 */
	s_angle_pid.Kp		= 0.0f;
	s_angle_pid.Ki		= 0.0f;
	s_angle_pid.Kd		= 0.0f;
	s_angle_pid.OutMin	= -200.0f;
	s_angle_pid.OutMax	= 200.0f;
	s_angle_pid.i_limit = 0.0f;
	PID_Reset(&s_angle_pid);
	s_angle_target_freq = 0;
	s_last_dir_cw		= true;
	/* 上电安全态：不使能电机、不输出脉冲（PA1/PA2 已由 gpio.c 初始化为低） */
}

/**
 * @brief 电机驱动使能（EN 引脚电平，高电平使能）
 * @param enable true = 使能；false = 释放
 * @note  使能前须确保目标频率已置 0，避免突然起转
 */
void Motor_SetEnabled(bool enable) {
	HAL_GPIO_WritePin(Motor_enabled_GPIO_Port, Motor_enabled_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 启动 STEP 脉冲输出（幂等）
 * @note  已运行时直接返回（HAL 对已运行通道再次 Start 会返回 BUSY）；
 *        HAL 失败进 Error_Handler
 */
void PwmStart(void) {
	if (s_running) {
		return;
	}
	if (HAL_TIM_PWM_Start(&htim2, STEP_CHANNEL) != HAL_OK) {
		Error_Handler();
	}
	s_running = true;
}

/**
 * @brief 停止 STEP 脉冲输出（幂等，绕组保持锁定）
 * @note  已停止时直接返回；HAL 失败进 Error_Handler
 */
void PwmStop(void) {
	if (!s_running) {
		return;
	}
	if (HAL_TIM_PWM_Stop(&htim2, STEP_CHANNEL) != HAL_OK) {
		Error_Handler();
	}
	s_running = false;
}

/**
 * @brief 写 TIM2 ARR 设定 STEP 脉冲频率
 * @param freq_hz 目标频率（Hz，调用方已取绝对值）
 * @note  频率未变时直接返回、不动 CNT：本函数每 20ms 被调一次，若无条件清 CNT，
 *        周期 >20ms（<50Hz）的 PWM 永远走不完翻转点、输出恒定电平，电机收不到
 *        任何脉冲沿（2026-10-31 定位"20Hz 不转、120Hz 转"根因）。
 *        频率变化时先清 CNT 再写 ARR（ARPE=0 立即生效），防新 ARR < 当前 CNT 时
 *        计数器绕满 0xFFFF 才回绕产生超长脉冲间隔；占空比维持 ≈50%。
 *        TIM2 为 16 位定时器，1MHz 时基下频率下限 16Hz（ARR≤65535）。
 */
void PwmSetFreq(uint32_t freq_hz) {
	uint32_t arr = (1000000u / freq_hz) - 1u;
	if (arr > 65535u) {
		arr = 65535u; /* 16 位 ARR 保护：<16Hz 钳到 16Hz，防 ARR/CCR 截断 */
	}
	if (arr == (uint32_t) __HAL_TIM_GET_AUTORELOAD(&htim2)) {
		return; /* 频率未变：不打断进行中的 PWM 周期 */
	}
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_AUTORELOAD(&htim2, arr);
	__HAL_TIM_SET_COMPARE(&htim2, STEP_CHANNEL, (arr + 1u) / 2u); /* 维持 ≈50% 占空比 */
}

/**
 * @brief 下发目标频率：直接驱动 DIR/启停/PWM
 * @param freq_hz 目标脉冲频率（Hz，带符号，正 = CW）
 * @note  直接写 TIM2 PWM 频率与 DIR 电平；换向拍只切 DIR、停脉冲
 *        （D36A 要求换向后 ≥1ms 再发脉冲，调用周期需 ≥1ms）。
 *        位置限幅由调用方在下发前完成，本函数不做位置判断。
 */
void Motor_SetTargetFreq(int32_t freq_hz) {
	s_cur_freq = freq_hz;

	if (freq_hz == 0) {
		PwmStop(); /* 目标 0：停脉冲，保持 DIR 与绕组锁定 */
		return;
	}

	bool cw = (freq_hz > 0);
	/* 换向安全：方向变化时本拍只切 DIR、不发脉冲（下一拍从新方向启动） */
	if (cw != s_last_dir_cw) {
		Motor_SetDir(cw);
		PwmStop();
		s_cur_freq	  = 0;
		s_last_dir_cw = cw;
		return;
	}

	PwmSetFreq((uint32_t) (freq_hz > 0 ? freq_hz : -freq_hz));
	PwmStart();
}

/**
 * @brief 读取最近一次下发的带符号频率（Hz，0 = 停脉冲）
 * @note  供 VOFA 遥测；换向/停止拍返回 0
 */
int32_t Motor_GetPulseFreqHz(void) {
	return s_cur_freq;
}

/**
 * @brief 设定目标角度（编码器计数）：角度环 PID → 直接驱动 PWM/DIR
 * @param target_cnt 目标编码器计数值（相对零点）
 * @param ctrl_p     角度环比例增益（Hz / 编码器计数偏差）：Out = Kp × 角度误差，
 *                   等价于一个控制周期的脉冲数 = Out × MOTOR_CTRL_PERIOD_MS / 1000
 * @note  内环由调用方每个测距周期（MOTOR_CTRL_PERIOD_MS = 20ms，50Hz）调用一次。
 *        位置式 PID：Target=目标角度, Actual=Encoder_Read(), Out=频率(Hz)。
 *        输出直接写 TIM2 PWM 频率与 DIR 电平，以保证定角度响应速度；
 *        换向拍只切 DIR、停脉冲（D36A 要求换向后 ≥1ms 再发脉冲，
 *        20ms 控制周期满足 ≥1ms 要求）。
 *        方向由 MOTOR_ENC_HZ_SIGN 决定：1 = freq>0 使 enc 增大，0 = 相反。
 */
void Motor_SetAngle(int32_t target_cnt, float ctrl_p) {
	s_angle_pid.Kp	   = 2.0f; /* 比例增益由调用方每拍给定，便于在线整定 */
	s_angle_pid.Target = (float) target_cnt;
	s_angle_pid.Actual = (float) Encoder_Read();
	PID_Update(&s_angle_pid);

	/* Out>0 = 需要编码器计数增大；按 MOTOR_ENC_HZ_SIGN 换算成带符号频率 */
	// int32_t freq		= (MOTOR_ENC_HZ_SIGN != 0) ? (int32_t) s_angle_pid.Out : -(int32_t) s_angle_pid.Out;
	int32_t freq		= (int32_t) s_angle_pid.Out;
	s_angle_target_freq = freq;

	/* 误差为零附近：停脉冲，绕组保持锁定（静态力矩定角度） */
	if (freq == 0) {
		PwmStop();
		s_cur_freq = 0; /* 同步遥测：实际输出 0 */
		return;
	}

	bool cw = (freq > 0);
	/* 换向安全：方向变化时本拍只切 DIR、不发脉冲（下一拍从新方向启动） */
	if (cw != s_last_dir_cw) {
		Motor_SetDir(cw);
		PwmStop();
		s_cur_freq	  = 0;
		s_last_dir_cw = cw;
		return;
	}

	/* 同方向：直接设定 PWM 频率并启动 */
	PwmSetFreq((uint32_t) (freq > 0 ? freq : -freq));
	PwmStart();
	s_cur_freq = freq; /* 同步遥测：实际输出频率 */
}

/**
 * @brief 读取角度环 PID 算出的目标频率（Hz，带符号）
 */
int32_t Motor_GetAngleTargetFreq(void) {
	return s_angle_target_freq;
}

/* ===================== 低层 PWM 原语（直接操控 TIM2） ====================== */

void Motor_SetDir(bool cw) {
	HAL_GPIO_WritePin(Motor_direction_GPIO_Port, Motor_direction_Pin,
					  (cw ? MOTOR_DIR_CW : MOTOR_DIR_CCW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
