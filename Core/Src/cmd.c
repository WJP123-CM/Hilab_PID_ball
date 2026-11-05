/**
 ******************************************************************************
 * @file    cmd.c
 * @brief   上位机行协议解析与执行。
 *
 *          命令集：
 *            A<milli>  设置 EMA 系数（20~1000，1000=直通）
 *            Z         把当前滤波值设为零点；Z<mm> 显式设置零点距离
 *            P/I/D<v>  外环增益千分比在线整定（0~10000，已禁用）
 *            R         清除滤波器窗口/EMA、有效帧统计
 *            C         编码器当前点归为零点：相对+绝对计数清零并写入 Flash
 *            F<hz>     开环频率覆盖（Hz，带符号，内部限幅）；F0/F 退出回闭环
 *
 *          未知命令/EMI 噪声静默丢弃。控制模式不开放串口切换。
 ******************************************************************************
 */
#include "cmd.h"

#include <stdbool.h>
#include <stdlib.h> /* strtol */

#include "encoder.h"    /* Encoder_ZeroAbsAndSave */
#include "main.h"       /* App_SetOpenloopFreq */
#include "tof.h"        /* g_tof_filter, Tof_ResetStats */
#include "tof_filter.h" /* TofFilter_* 接口与参数范围 */
#include "usart.h"      /* USART1_* */

/** @brief 解析并执行一条命令（未知命令静默丢弃） */
static void Cmd_ParseLine(const char *line) {
	char  key;
	char *end;
	long  val     = 0;
	bool  has_val = false;

	while (*line == ' ' || *line == '\t') line++;
	if (*line == '\0') return;

	/* 大小写归一：'a'~'z' 转大写 */
	key = ((*line >= 'a') && (*line <= 'z')) ? (char)(*line - ('a' - 'A')) : *line;

	if (line[1] != '\0') {
		val     = strtol(line + 1, &end, 10);
		has_val = (end != (line + 1));
	}

	switch (key) {
	case 'A':
		if (has_val && val >= (long)TOF_FILTER_ALPHA_MIN_MILLI && val <= (long)TOF_FILTER_ALPHA_MAX_MILLI) {
			TofFilter_SetAlpha(&g_tof_filter, (uint16_t)val);
		}
		break;
	case 'Z':
		if (has_val) {
			if (val >= 0 && val <= (long)TOF_FILTER_MAX_VALID_MM) {
				TofFilter_SetZeroMm(&g_tof_filter, (int16_t)val);
			}
		} else {
			(void)TofFilter_ZeroHere(&g_tof_filter);
		}
		break;
	case 'P':
	case 'I':
	case 'D':
		/* PID 增益命令暂不支持，保留命令解析兼容性 */
		break;
	case 'R':
		TofFilter_Reset(&g_tof_filter);
		Tof_ResetStats();
		break;
	case 'C':
		/* 当前点归为编码器零点（断电后不保持） */
		Encoder_Zero();
		break;
	case 'F':
		/* 开环频率覆盖：F60/F-60 直驱电机钉方向链（enc 应减/增），
		 * F0 或裸 F 退出回闭环；限幅与限位钳位由 main.c ISR 执行 */
		App_SetOpenloopFreq(has_val ? (int32_t) val : 0);
		break;
	default:
		break;
	}
}

void Cmd_Poll(void) {
	char line[UART1_LINE_MAX];

	USART1_EnsureRx();
	if (uart1_last_ready) {
		for (uint8_t i = 0; i < UART1_LINE_MAX; i++) {
			line[i] = uart1_last_line[i];
			if (line[i] == '\0') {
				break;
			}
		}
		uart1_last_ready = 0;
		Cmd_ParseLine(line);
	}
}
