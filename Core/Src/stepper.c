/**
 ******************************************************************************
 * @file    stepper.c
 * @brief   步进电机驱动实现（D36A：EN/DIR 电平 + TIM2_CH4 STEP 脉冲）
 *          含核心决策状态机（StepperSm，纯软件无硬件依赖，main.c 上电自检覆盖）
 ******************************************************************************
 */
#include "stepper.h"

#include "stm32f1xx_hal_flash_ex.h" /* Flash 页擦除（掉电位置保存） */
#include "tim.h"					/* htim2/htim3 */

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
static int32_t		s_enc_total = 0;	 /* 32 位累计计数（相对：EncoderZero 清零） */
static uint16_t		s_enc_last	= 0u;	 /* 上一次读取的 CNT 原始值 */
static int32_t		s_enc_abs	= 0;	 /* 绝对累计计数（不随 EncoderZero 清零，
										  * 上电由 Flash 恢复，Z 相校准） */
static volatile uint32_t s_z_cnt	  = 0u;	  /* Z 相脉冲总数（诊断） */
static volatile bool	 s_z_pending  = false; /* 有未处理的 Z 事件 */
static volatile bool	 s_store_dirty = true; /* 需要落盘（上电未确认前为脏） */
static bool			s_running	= false; /* STEP 脉冲输出状态（内部启停幂等） */
static StepperSm_t	s_sm;				 /* 速度控制状态机（决策见 StepperSm_Update） */

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
	/* Z 相（TIM3_CH3 = PB0）输入捕获中断：每圈一个脉冲，ISR 只登记事件 */
	if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	s_enc_last	= (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
	s_enc_total = 0;
	s_enc_abs	= 0; /* 真实值由上电 Stepper_StoreRestore 恢复 */

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
	StepperCmd_t cmd;
	int32_t		 pos_dir;
	int32_t		 abs_pos;

	/* 绝对位置软限位：仅在电机朝"越界方向"运动时切断。
	 * 位置反馈由 ReadEncoder 每 20ms 更新，故限位响应延迟 ≤20ms。
	 * 反向（退回区间内）永远放行，避免顶死在限位后无法退出 */
	abs_pos	 = s_enc_abs;
	pos_dir	 = (freq_hz > 0) ? ENC_POS_DIR_SIGN : ((freq_hz < 0) ? -ENC_POS_DIR_SIGN : 0);
	if (((pos_dir > 0) && (abs_pos >= (int32_t) ENC_POS_LIM_MAX)) ||
		((pos_dir < 0) && (abs_pos <= (int32_t) ENC_POS_LIM_MIN))) {
		freq_hz = 0; /* 越界方向一律按"目标 0"处理：停脉冲、绕组保持锁定 */
	}

	cmd = StepperSm_Update(&s_sm, freq_hz);

	if (cmd.dir_changed) {
		/* 换向拍：只建立 DIR 电平不发脉冲（D36A 要求换向后 ≥1ms 再发脉冲，
		 * 50Hz 调用间隔 20ms 天然满足；下一拍从 0 斜升）。
		 * 方向电平由 STEPPER_DIR_CW/CCW 宏决定，改宏会真正生效。 */
		PwmStop();
		HAL_GPIO_WritePin(Motor_direction_GPIO_Port, Motor_direction_Pin,
						  (cmd.dir_cw ? STEPPER_DIR_CW : STEPPER_DIR_CCW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
	uint16_t now;
	int16_t	 d;
	uint32_t pm;

	/* 临界区：本函数既被 50Hz TIM1 ISR 调用（模式1/2），
	 * 也被 5Hz 主循环 Enc_Service 调用（模式0）。若主循环在执行中途被
	 * ISR 抢占，两次调用会出现差分重算，累计值被多计。临界区仅几条指令 */
	pm = __get_PRIMASK();
	__disable_irq();
	/* int16_t 差分天然处理 16 位回绕（正反向越界均正确）；
	 * 安全条件：两次调用间隔内计数变化 < 32768。50Hz 调用下对应
	 * 转速上限 ≈ 24600rpm，远超本电机工况，恒安全 */
	now			= (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
	d			= (int16_t) (now - s_enc_last);
	/* 统一方向：STEPPER_ENC_DIR_SIGN 修正 A/B 相接反，使计数增量与 freq 同向
	 * （否则内环成正反馈、软限幅判断方向也反） */
	d			= (int16_t) (STEPPER_ENC_DIR_SIGN * d);
	s_enc_total += d;
	s_enc_abs   += d;
	s_enc_last	= now;
	__set_PRIMASK(pm);

	if (d != 0) {
		s_store_dirty = true; /* 位置变化，待主循环落盘 */
	}
	return s_enc_total;
}

/* ==================== 绝对位置 / Z 相 / 掉电保存 ======================== */

/* Flash 数据页布局（12 字节，WORD 编程） */
typedef struct {
	uint32_t magic;
	uint32_t enc_abs;
	uint32_t sum;
} EncStore_t;

static uint32_t enc_store_sum(uint32_t v) {
	return STEPPER_ENC_STORE_MAGIC ^ v;
}

int32_t Stepper_ReadEncoderAbs(void) {
	return s_enc_abs;
}

uint32_t Stepper_GetZCount(void) {
	return s_z_cnt;
}

/* ISR 内只登记，不做计算/不写 Flash */
void Stepper_ZIndexEvent(void) {
	s_z_cnt++;
	s_z_pending	  = true;
	s_store_dirty = true;
}

bool Stepper_ProcessZ(void) {
	int32_t q;
	int32_t r;

	if (!s_z_pending) {
		return false;
	}
	s_z_pending = false;

	/* Z 为整圈零点：对齐到最近整圈（C 除法向零截断，余数过半圈才进位） */
	q = s_enc_abs / (int32_t) STEPPER_ENC_CPR;
	r = s_enc_abs % (int32_t) STEPPER_ENC_CPR;
	if (r > (int32_t) STEPPER_ENC_CPR / 2) {
		q++;
	} else if (r < -((int32_t) STEPPER_ENC_CPR / 2)) {
		q--;
	}
	s_enc_abs = q * (int32_t) STEPPER_ENC_CPR;
	return true;
}

void Stepper_StoreMarkDirty(void) {
	s_store_dirty = true;
}

bool Stepper_StorePending(void) {
	return s_store_dirty;
}

bool Stepper_StoreRestore(void) {
	const EncStore_t *d = (const EncStore_t *) STEPPER_ENC_STORE_ADDR;

	if ((d->magic != STEPPER_ENC_STORE_MAGIC) || (d->sum != enc_store_sum(d->enc_abs))) {
		return false; /* 未写过/被擦除（全 0xFF）或校验失败 */
	}
	s_enc_abs	  = (int32_t) d->enc_abs;
	s_store_dirty = false;
	return true;
}

bool Stepper_StoreSave(void) {
	EncStore_t			  d;
	FLASH_EraseInitTypeDef ei = {0};
	uint32_t			  page_err = 0u;
	const uint32_t		 *p;
	uint32_t			  addr;
	uint8_t				  i;
	HAL_StatusTypeDef	  ok;

	d.magic	  = STEPPER_ENC_STORE_MAGIC;
	d.enc_abs = (uint32_t) s_enc_abs;
	d.sum	  = enc_store_sum(d.enc_abs);

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
	ei.TypeErase   = FLASH_TYPEERASE_PAGES;
	ei.PageAddress = STEPPER_ENC_STORE_ADDR;
	ei.NbPages	   = 1u;
	ok			   = HAL_FLASHEx_Erase(&ei, &page_err);
	if (ok == HAL_OK) {
		/* 目标地址必须是 Flash（STEPPER_ENC_STORE_ADDR），源数据在栈上；
		 * 早期版本把栈地址当编程地址传入，导致擦除后写入静默丢失 */
		p	 = (const uint32_t *) &d;
		for (i = 0u; i < (sizeof(EncStore_t) / 4u); i++) {
			addr = STEPPER_ENC_STORE_ADDR + (4u * i);
			ok	 = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, p[i]);
			if (ok != HAL_OK) {
				break;
			}
		}
	}
	HAL_FLASH_Lock();
	s_store_dirty = false; /* 失败不重试：避免主循环被反复擦写阻塞 */
	return ok == HAL_OK;
}

/* ==================== 编码器速度滤波（实现见 stepper.h 噪声模型注释） ===== */

void StepperSpdFilt_Init(StepperSpdFilt_t *f, uint8_t n, float alpha) {
	if (n < 1u) {
		n = 1u;
	}
	if (n > STEPPER_SPDF_NMAX) {
		n = STEPPER_SPDF_NMAX;
	}
	if ((alpha <= 0.0f) || (alpha > 1.0f)) {
		alpha = 1.0f; /* 越界一律直通，防滤波器发散 */
	}
	f->n	 = n;
	f->alpha = alpha;
	StepperSpdFilt_Reset(f);
}

void StepperSpdFilt_Reset(StepperSpdFilt_t *f) {
	for (uint8_t i = 0; i < STEPPER_SPDF_NMAX; i++) {
		f->pos[i] = 0;
	}
	f->idx		= 0;
	f->filled	= 0;
	f->lpf		= 0.0f;
}

int32_t StepperSpdFilt_Update(StepperSpdFilt_t *f, int32_t enc_pos) {
	int32_t oldest = f->pos[f->idx]; /* 先取最老样本再覆盖（N=1 时即上一拍位置） */
	f->pos[f->idx] = enc_pos;
	f->idx = (uint8_t) ((f->idx + 1u) % f->n);
	if (f->filled < f->n) {
		f->filled++;
	}
	if (f->filled < f->n) {
		return 0; /* 未满窗：与上电静止段语义一致 */
	}
	return (enc_pos - oldest) * STEPPER_SPDF_SCALE / (int32_t) f->n;
}

float StepperSpdFilt_UpdateLpf(StepperSpdFilt_t *f, float raw_hz) {
	f->lpf += f->alpha * (raw_hz - f->lpf);
	return f->lpf;
}
