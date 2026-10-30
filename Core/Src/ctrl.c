/**
 ******************************************************************************
 * @file    ctrl.c
 * @brief   级联闭环控制实现：内环（编码器位置，50Hz ISR）+ 外环（小球位置，
 *          20Hz 主循环），模式状态机，Z 相掉电保存，VOFA 遥测，按键急停。
 ******************************************************************************
 */
#include "ctrl.h"

#include "encoder.h"
#include "key.h"
#include "motor.h"
#include "pid.h"
#include "tof.h"
#include "tof_filter.h"
#include "usart.h" /* VOFA_SendFrame */

/* ===================== 内环：平台保持（编码器位置环） ===================== */
#define HOLD_PID_KP       2.0f   /* Hz / 编码器计数 */
#define HOLD_PID_KI       0.05f
#define HOLD_PID_KD       0.0f
#define HOLD_PID_ILIM     50.0f  /* 积分限幅（Hz） */
#define HOLD_OUT_LIMIT    400.0f /* 输出限幅（Hz） */
#define HOLD_DEADBAND_CNT 3.0f   /* 误差死区（计数） */

/* ===================== 外环：小球位置闭环 ===================== */
#define BALL_PID_KP        2.0f   /* 计数 / 0.1cm */
#define BALL_PID_KI        0.05f
#define BALL_PID_KD        0.0f
#define BALL_PID_ILIM      100.0f /* 积分限幅（计数） */
#define BALL_PID_OUT_LIM   400.0f /* 外环输出限幅（计数） */
#define BALL_PID_DEADBAND  3.0f   /* 死区 ±0.3cm */
#define BALL_LOSS_MAX_FRAMES 10u  /* ToF 连续无效 0.5s → 外环回中 */

/* 符号链（2026-10-30，ENC_DIR_SIGN=+1 时推导）：
 *   freq>0 → enc↓；target>fb 需 freq<0 使 enc↑ → HOLD_PID_SIGN=-1
 *   pos>0 需 s_ball_ofs_cnt>0 → PID_Update(0,pos>0) 为负 → BALL_PID_SIGN=+1
 * 改符号必须与 encoder.h 的 ENC_DIR_SIGN 同步（ENC_POS_DIR_SIGN 已于
 * 2026-10-31 随 motor.c abs 软限位一并删除，限幅统一在 main.c ISR）。 */
#define HOLD_PID_SIGN (-1)
#define BALL_PID_SIGN (1)

/* ===================== VOFA 通道定义 ===================== */
enum {
	VOFA_CH_FILT_MM = 0,
	VOFA_CH_PULSE_FREQ,
	VOFA_CH_ENC_CNT,
	VOFA_CH_NUM
};

/* ===================== 控制状态 ===================== */
static PID_t          s_hold_pid;
static PID_t          s_ball_pid;
static volatile uint8_t  s_ctrl_mode     = 0; /* 0=释放 1=保持 2=闭环 */
static volatile int16_t  s_ball_ofs_cnt  = 0; /* 外环输出：内环目标偏移 */
static volatile uint8_t  s_tof_lost_n    = 0; /* ToF 连续无效帧计数 */
static volatile uint8_t  s_key_estop     = 0; /* 急停锁存 */
static int32_t        s_store_abs_ref   = 0; /* 掉电保存基准 */

static const PIDCfg_t s_hold_cfg = {
	.kp = HOLD_PID_KP, .ki = HOLD_PID_KI, .kd = HOLD_PID_KD,
	.out_min = -HOLD_OUT_LIMIT, .out_max = HOLD_OUT_LIMIT,
	.i_limit = HOLD_PID_ILIM, .deadband = HOLD_DEADBAND_CNT, .d_alpha = 1.0f,
};
static const PIDCfg_t s_ball_cfg = {
	.kp = BALL_PID_KP, .ki = BALL_PID_KI, .kd = BALL_PID_KD,
	.out_min = -BALL_PID_OUT_LIM, .out_max = BALL_PID_OUT_LIM,
	.i_limit = BALL_PID_ILIM, .deadband = BALL_PID_DEADBAND, .d_alpha = 1.0f,
};

/* ===================== 实现 ===================== */

void Ctrl_Init(void) {
	Motor_Init();
	Encoder_Init();
	(void)Encoder_StoreRestore();
	PID_InitFromCfg(&s_hold_pid, &s_hold_cfg);
	PID_InitFromCfg(&s_ball_pid, &s_ball_cfg);
	s_ctrl_mode = 0;
	s_ball_ofs_cnt = 0;
	s_tof_lost_n = 0;
	Motor_SetTargetFreq(0);
	Motor_SetEnabled(false);
#if PLATFORM_HOLD_ON_BOOT
	Ctrl_SetMode(1);
#endif
}

void Ctrl_SetMode(uint8_t mode) {
	s_ctrl_mode = 0;
	Motor_SetTargetFreq(0);
	if (mode == 0u) {
		Encoder_StoreMarkDirty();
		Motor_SetEnabled(false);
		return;
	}
	s_key_estop = 0;
	Encoder_Zero();
	Ctrl_ResetLoops();
#if MOTOR_CONTROL_ENABLE
	Motor_SetEnabled(true);
#endif
	s_ctrl_mode = mode;
}

void Ctrl_OuterTick(void) {
	float out;
	if (s_ctrl_mode != 2u) return;
	if (g_tof.range_status != 0) {
		if (s_tof_lost_n < 255u) s_tof_lost_n++;
		if (s_tof_lost_n >= BALL_LOSS_MAX_FRAMES) {
			PID_Reset(&s_ball_pid);
			s_ball_ofs_cnt = 0;
		}
		return;
	}
	s_tof_lost_n = 0;
	out = BALL_PID_SIGN * PID_Update(&s_ball_pid, 0.0f, (float)g_tof.pos_cm_x10);
	s_ball_ofs_cnt = (int16_t)out;
}

void Ctrl_InnerTick(void) {
	int32_t fb;
	float target, f_out;
	if (s_ctrl_mode == 0u) return;
	target = (s_ctrl_mode == 2u) ? (float)s_ball_ofs_cnt : 0.0f;
	fb = Encoder_Read();
	f_out = HOLD_PID_SIGN * PID_Update(&s_hold_pid, target, (float)fb);
#if MOTOR_CONTROL_ENABLE
	Motor_SetTargetFreq((int32_t)f_out);
#else
	(void)f_out;
#endif
}

void Ctrl_EncService(void) {
	int32_t abs;
	(void)Encoder_Read();
	(void)Encoder_ProcessZ();
	if (s_ctrl_mode != 0u) {
		s_store_abs_ref = Encoder_ReadAbs();
		return;
	}
	abs = Encoder_ReadAbs();
	if (abs != s_store_abs_ref) {
		s_store_abs_ref = abs;
		return;
	}
	if (Encoder_StorePending()) {
		(void)Encoder_StoreSave();
	}
}

void Ctrl_SendTelemetry(void) {
	int32_t vofa[VOFA_CH_NUM] = {
		[VOFA_CH_FILT_MM]    = (int32_t)g_tof.filt_mm,
		[VOFA_CH_PULSE_FREQ] = Motor_GetPulseFreqHz(),
		[VOFA_CH_ENC_CNT]    = Encoder_Read(),
	};
	VOFA_SendFrame(vofa, (uint8_t)VOFA_CH_NUM);
}

void Ctrl_PollKeyEvents(void) {
	KeyEvent_t ke;
	while ((ke = Key_PollEvent()) != KEY_EVT_NONE) {
		if (ke == KEY_EVT_PRESS) {
			if (s_key_estop == 0u) {
				s_key_estop = 1u;
				Ctrl_SetMode(0);
			} else {
				s_key_estop = 0u;
#if MOTOR_CONTROL_ENABLE
				Motor_SetEnabled(true);
#endif
			}
		}
	}
}

void Ctrl_ResetLoops(void) {
	PID_Reset(&s_hold_pid);
	PID_Reset(&s_ball_pid);
	s_ball_ofs_cnt = 0;
	s_tof_lost_n = 0;
}

/* ===================== 状态查询 ===================== */
uint8_t Ctrl_GetMode(void)   { return s_ctrl_mode; }
uint8_t Ctrl_IsEstop(void)   { return s_key_estop; }

/* ===================== 供 cmd.c 在线整定 ===================== */
void Ctrl_SetBallGain(PIDGain_t gain, float value) {
	PID_SetOneGain(&s_ball_pid, gain, value);
}
