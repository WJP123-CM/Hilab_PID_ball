/**
 ******************************************************************************
 * @file    motor.c
 * @brief   步进电机驱动实现（D36A：EN/DIR 电平 + TIM2_CH4 STEP 脉冲）
 *          含核心决策状态机（MotorSm，纯软件无硬件依赖，可主机侧单测）
 ******************************************************************************
 */
#include "motor.h"

#include "encoder.h" /* Encoder_ReadAbs：软限幅判向 */
#include "main.h"	 /* Motor_enabled / Motor_direction 引脚标签 */
#include "tim.h"	 /* htim2 */

/* STEP 脉冲通道：TIM2_CH4 = PA3 */
#define STEP_CHANNEL TIM_CHANNEL_4

/* ==================== 核心决策状态机（纯软件，无硬件依赖） ================ */

/**
 * @brief 复位决策状态机（输出 0、方向 CW）
 * @param sm 状态机实例
 * @note  纯软件无硬件依赖，可主机侧单测
 */
void MotorSm_Init(MotorSm_t *sm) {
	sm->cur_freq = 0;
	sm->dir_cw	 = true;
}

/**
 * @brief 单拍决策：限幅 → 死区 → 斜率限制 → 换向安全
 * @param sm 状态机实例；target_hz 目标脉冲频率（Hz，带符号，正 = CW）
 * @retval 本拍下发指令（输出频率/方向/是否换向/是否发脉冲）
 * @note  每 20ms 调用一次；cur_freq/dir_cw 为跨拍状态，换向拍输出 0 且只切 DIR
 */
MotorCmd_t MotorSm_Update(MotorSm_t *sm, int32_t target_hz) {
	MotorCmd_t cmd = {0, sm->dir_cw, false, false};

	/* 1. 限幅：钳到 ±MOTOR_FREQ_MAX_HZ */
	if (target_hz > MOTOR_FREQ_MAX_HZ) {
		target_hz = MOTOR_FREQ_MAX_HZ;
	} else if (target_hz < -MOTOR_FREQ_MAX_HZ) {
		target_hz = -MOTOR_FREQ_MAX_HZ;
	}

	/* 2. 死区：|target| < MIN 视为停止（频率过低脉冲过疏，只会引起抖动） */
	if ((target_hz > -MOTOR_FREQ_MIN_HZ) && (target_hz < MOTOR_FREQ_MIN_HZ)) {
		target_hz = 0;
	}

	/* 3. 斜率限制：本拍输出向目标步进，单拍变化 ≤ MOTOR_SLEW_HZ_PER_STEP */
	int32_t next = sm->cur_freq;
	if (target_hz > next + MOTOR_SLEW_HZ_PER_STEP) {
		next += MOTOR_SLEW_HZ_PER_STEP;
	} else if (target_hz < next - MOTOR_SLEW_HZ_PER_STEP) {
		next -= MOTOR_SLEW_HZ_PER_STEP;
	} else {
		next = target_hz;
	}

	/* 4. 换向安全：本拍输出非零且方向与当前 DIR 不同 → 截停本拍输出，
	 * 只切换 DIR 电平不发脉冲；状态记为 0，下一拍从新方向 0 斜升。
	 * 斜率限制保证换向前频率已平滑过零区，不存在高频硬换向 */
	if (next != 0) {
		bool cw = (next > 0);
		if (cw != sm->dir_cw) {
			cmd.dir_cw		= cw;
			cmd.dir_changed = true;
			cmd.pulse_en	= false;
			cmd.out_freq	= 0;
			sm->cur_freq	= 0;
			sm->dir_cw		= cw;
			return cmd;
		}
	}

	/* 5. 正常输出（含 0 = 停脉冲、保持 DIR） */
	cmd.out_freq = next;
	cmd.pulse_en = (next != 0);
	sm->cur_freq = next;
	return cmd;
}

/* ==================== 硬件下发（TIM2 PWM / GPIO） ======================== */

static bool		s_running = false; /* STEP 脉冲输出状态（内部启停幂等） */
static MotorSm_t s_sm;			   /* 速度控制状态机（决策见 MotorSm_Update） */

/**
 * @brief 校验 TIM2 分频假设、设定 STEP 占空比、复位决策状态机
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

	MotorSm_Init(&s_sm);
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
static void PwmStart(void) {
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
static void PwmStop(void) {
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
 * @param freq_hz 目标频率（Hz，≥ MOTOR_FREQ_MIN_HZ，调用方已取绝对值并限幅）
 * @note  先清 CNT 再写 ARR（ARPE=0 立即生效），防新 ARR < 当前 CNT 时计数器
 *        绕满 0xFFFF 才回绕产生超长脉冲间隔；占空比维持 ≈50%
 */
static void PwmSetFreq(uint32_t freq_hz) {
	uint32_t arr = (1000000u / freq_hz) - 1u;
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_AUTORELOAD(&htim2, arr);
	__HAL_TIM_SET_COMPARE(&htim2, STEP_CHANNEL, (arr + 1u) / 2u); /* 维持 ≈50% 占空比 */
}

/**
 * @brief 下发目标频率：软限幅 → 状态机决策 → DIR/启停/PWM
 * @param freq_hz 目标脉冲频率（Hz，带符号，正 = CW）
 * @note  每 20ms 调用（TIM1 内环 ISR）；朝越界方向一律按 0 处理（停脉冲、
 *        绕组锁定），反向永远放行；换向拍只切 DIR 不发脉冲
 */
void Motor_SetTargetFreq(int32_t freq_hz) {
	MotorCmd_t cmd;
	int32_t	   pos_dir;
	int32_t	   abs_pos;

	/* 绝对位置软限位：仅在电机朝"越界方向"运动时切断。
	 * 位置反馈由 Encoder_Read 每 20ms 更新，故限位响应延迟 ≤20ms。
	 * 反向（退回区间内）永远放行，避免顶死在限位后无法退出 */
	abs_pos	 = Encoder_ReadAbs();
	pos_dir	 = (freq_hz > 0) ? ENC_POS_DIR_SIGN : ((freq_hz < 0) ? -ENC_POS_DIR_SIGN : 0);
	if (((pos_dir > 0) && (abs_pos >= (int32_t) ENC_POS_LIM_MAX)) ||
		((pos_dir < 0) && (abs_pos <= (int32_t) ENC_POS_LIM_MIN))) {
		freq_hz = 0; /* 越界方向一律按"目标 0"处理：停脉冲、绕组保持锁定 */
	}

	cmd = MotorSm_Update(&s_sm, freq_hz);

	if (cmd.dir_changed) {
		/* 换向拍：只建立 DIR 电平不发脉冲（D36A 要求换向后 ≥1ms 再发脉冲，
		 * 50Hz 调用间隔 20ms 天然满足；下一拍从 0 斜升）。
		 * 方向电平由 MOTOR_DIR_CW/CCW 宏决定，改宏会真正生效。 */
		PwmStop();
		HAL_GPIO_WritePin(Motor_direction_GPIO_Port, Motor_direction_Pin,
						  (cmd.dir_cw ? MOTOR_DIR_CW : MOTOR_DIR_CCW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		return;
	}

	if (!cmd.pulse_en) {
		PwmStop(); /* 目标 0（或斜率降至 0）：停脉冲，保持 DIR 与绕组锁定 */
		return;
	}

	PwmSetFreq((uint32_t) (cmd.out_freq > 0 ? cmd.out_freq : -cmd.out_freq));
	PwmStart();
}

/**
 * @brief 读斜率限制后的真实输出频率
 * @retval 当前下发频率（Hz，带符号；0 = 停脉冲）
 * @note  供 20Hz VOFA 遥测
 */
int32_t Motor_GetPulseFreqHz(void) {
	return s_sm.cur_freq; /* 斜率限制后的真实输出（带符号） */
}
