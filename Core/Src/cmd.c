/**
 ******************************************************************************
 * @file    cmd.c
 * @brief   上位机行协议解析与执行。
 *
 *          命令集：
 *            A<milli>  设置 EMA 系数（20~1000，1000=直通）
 *            Z         把当前滤波值设为零点；Z<mm> 显式设置零点距离
 *            P/I/D<v>  外环增益千分比在线整定（0~10000）
 *            R         清除滤波器窗口/EMA、有效帧统计、内外环积分
 *
 *          未知命令/EMI 噪声静默丢弃。控制模式不开放串口切换。
 ******************************************************************************
 */
#include "cmd.h"

#include <stdbool.h>
#include <stdlib.h> /* strtol */

#include "ctrl.h"       /* Ctrl_ResetLoops, Ctrl_SetBallGain */
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
		if (has_val && val >= 0 && val <= 10000) {
			PIDGain_t gain = (key == 'P') ? PID_GAIN_KP : ((key == 'I') ? PID_GAIN_KI : PID_GAIN_KD);
			Ctrl_SetBallGain(gain, (float)val / 1000.0f);
		}
		break;
	case 'R':
		TofFilter_Reset(&g_tof_filter);
		Tof_ResetStats();
		Ctrl_ResetLoops();
		break;
	default:
		break;
	}
}

void Cmd_Poll(void) {
	char line[UART1_LINE_MAX];

	USART1_EnsureRx();
	while (USART1_GetLine(line, (uint8_t)sizeof line)) {
		Cmd_ParseLine(line);
	}
}
