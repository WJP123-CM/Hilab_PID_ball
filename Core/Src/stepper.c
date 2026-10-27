/**
 ******************************************************************************
 * @file    stepper.c
 * @brief   步进电机驱动实现（D36A：EN/DIR 电平 + TIM2_CH4 STEP 脉冲）
 *          含核心决策状态机（StepperSm，纯软件无硬件依赖，main.c 上电自检覆盖）
 ******************************************************************************
 */
#include "stepper.h"

#include "tim.h" /* htim2/htim3 */

/* STEP 脉冲通道：TIM2_CH4 = PA3 */
#define STEP_CHANNEL TIM_CHANNEL_4

/* ==================== 核心决策状态机（纯软件，无硬件依赖） ================ */

void StepperSm_Init(StepperSm_t *sm) {
	sm->cur_freq = 0;
	sm->dir_cw	 = true;
}

StepperCmd_t StepperSm_Update(StepperSm_t *sm, int32_t target_hz) {
	StepperCmd_t cmd = {0, sm->dir_cw, false, false};

	/* 1. 限幅：钳到 ±STEPPER_FREQ_MAX_HZ */
	if (target_hz > STEPPER_FREQ_MAX_HZ) {
		target_hz = STEPPER_FREQ_MAX_HZ;
	} else if (target_hz < -STEPPER_FREQ_MAX_HZ) {
		target_hz = -STEPPER_FREQ_MAX_HZ;
	}

	/* 2. 死区：|target| < MIN 视为停止（频率过低脉冲过疏，只会引起抖动） */
	if ((target_hz > -STEPPER_FREQ_MIN_HZ) && (target_hz < STEPPER_FREQ_MIN_HZ)) {
		target_hz = 0;
	}

	/* 3. 斜率限制：本拍输出向目标步进，单拍变化 ≤ STEPPER_SLEW_HZ_PER_STEP */
	int32_t next = sm->cur_freq;
	if (target_hz > next + STEPPER_SLEW_HZ_PER_STEP) {
		next += STEPPER_SLEW_HZ_PER_STEP;
	} else if (target_hz < next - STEPPER_SLEW_HZ_PER_STEP) {
		next -= STEPPER_SLEW_HZ_PER_STEP;
	} else {
		next = target_hz;
	}

	/* 4. 换向安全：本拍输出非零且方向与当前 DIR 不同 → 截停本拍输出，
	 * 只切换 DIR 电平不发脉冲；状态记为 0，下一拍从新方向 0 斜升。
	 * 斜率限制保证换向前频率已平滑过零区，不存在高频硬换向 */
	if (next != 0) {
		bool cw = (next > 0);
		if (cw != sm->dir_cw) {
			cmd.dir_cw	  = cw;
			cmd.dir_changed = true;
			cmd.pulse_en  = false;
			cmd.out_freq  = 0;
			sm->cur_freq  = 0;
			sm->dir_cw	  = cw;
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

/* 编码器累计位置状态（TIM3 CNT 为 16 位，回绕由 ReadEncoder 差分累计） */
static int32_t		s_enc_total = 0;	 /* 32 位累计计数 */
static uint16_t		s_enc_last	= 0u;	 /* 上一次读取的 CNT 原始值 */
static bool			s_running	= false; /* STEP 脉冲输出状态（内部启停幂等） */
static StepperSm_t	s_sm;				 /* 速度控制状态机（决策见 stepper_logic.c） */

void Stepper_Init(void) {
	/* 校验 CubeMX 配置的 TIM2 分频与 1MHz 计数假设一致（重新生成时须保持）：
	 * 计数 1tick=1µs → 占空比换算与频率换算才成立，不符即 Error_Handler */
	if (htim2.Instance->PSC != STEPPER_TIM_PSC) {
		Error_Handler();
	}

	/* STEP 占空比 ≈50%：用 (ARR+1)/2 动态计算，远大于驱动芯片对 STEP
	 * 脉宽的最低要求（≥2µs） */
	__HAL_TIM_SET_COMPARE(&htim2, STEP_CHANNEL, (__HAL_TIM_GET_AUTORELOAD(&htim2) + 1u) / 2u);

	/* 编码器接口启动（TIM3 CH1/CH2 = PA6/PA7，CubeMX 已配 TI12 四倍频 +
	 * 输入滤波 15，硬件级毛刺抑制）；随后以当前 CNT 为累计零点 */
	if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
		Error_Handler();
	}
	s_enc_last	= (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
	s_enc_total = 0;

	StepperSm_Init(&s_sm);
	/* 上电安全态：不使能电机、不输出脉冲（PA1/PA2 已由 gpio.c 初始化为低） */
}

/* 编码器累计位置清零（对位基准/测试归零用）：当前时刻为新的 0 点 */
void Stepper_EncoderZero(void) {
	s_enc_total = 0;
	s_enc_last	= (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
}

void Stepper_SetEnabled(bool enable) {
	HAL_GPIO_WritePin(Motor_enabled_GPIO_Port, Motor_enabled_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 内部：启动 STEP 脉冲输出（幂等——HAL_TIM_PWM_Start 对已运行通道再次
 * 调用返回 HAL_BUSY，不加状态判断送入 Error_Handler 会关中断死循环） */
static void PwmStart(void) {
	if (s_running) {
		return;
	}
	if (HAL_TIM_PWM_Start(&htim2, STEP_CHANNEL) != HAL_OK) {
		Error_Handler();
	}
	s_running = true;
}

/* 内部：停止 STEP 脉冲输出（幂等，绕组保持锁定） */
static void PwmStop(void) {
	if (!s_running) {
		return;
	}
	if (HAL_TIM_PWM_Stop(&htim2, STEP_CHANNEL) != HAL_OK) {
		Error_Handler();
	}
	s_running = false;
}

/* 内部：写 TIM2 ARR 设定脉冲频率（freq_hz 已限幅，调用方保证 ≥ MIN）。
 * 先清零 CNT 再写 ARR（ARPE=0 立即生效）：防新 ARR < 当前 CNT 时
 * 计数器绕满 0xFFFF 才回绕，产生一个超长脉冲间隔 */
static void PwmSetFreq(uint32_t freq_hz) {
	uint32_t arr = (1000000u / freq_hz) - 1u;
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_AUTORELOAD(&htim2, arr);
	__HAL_TIM_SET_COMPARE(&htim2, STEP_CHANNEL, (arr + 1u) / 2u); /* 维持 ≈50% 占空比 */
}

void Stepper_SetTargetFreq(int32_t freq_hz) {
	StepperCmd_t cmd = StepperSm_Update(&s_sm, freq_hz);

	if (cmd.dir_changed) {
		/* 换向拍：只建立 DIR 电平不发脉冲（D36A 要求换向后 ≥1ms 再发脉冲，
		 * 50Hz 调用间隔 20ms 天然满足；下一拍从 0 斜升） */
		PwmStop();
		HAL_GPIO_WritePin(Motor_direction_GPIO_Port, Motor_direction_Pin,
						  cmd.dir_cw ? GPIO_PIN_SET : GPIO_PIN_RESET);
		return;
	}

	if (!cmd.pulse_en) {
		PwmStop(); /* 目标 0（或斜率降至 0）：停脉冲，保持 DIR 与绕组锁定 */
		return;
	}

	PwmSetFreq((uint32_t) (cmd.out_freq > 0 ? cmd.out_freq : -cmd.out_freq));
	PwmStart();
}

int32_t Stepper_GetPulseFreqHz(void) {
	return s_sm.cur_freq; /* 斜率限制后的真实输出（带符号） */
}

int32_t Stepper_ReadEncoder(void) {
	uint16_t now = (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
	/* int16_t 差分天然处理 16 位回绕（正反向越界均正确）；
	 * 安全条件：两次调用间隔内计数变化 < 32768。50Hz 调用下对应
	 * 转速上限 ≈ 24600rpm，远超本电机工况，恒安全 */
	s_enc_total += (int16_t) (now - s_enc_last);
	s_enc_last	= now;
	return s_enc_total;
}
