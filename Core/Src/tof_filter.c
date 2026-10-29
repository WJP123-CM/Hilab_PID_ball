/**
 ******************************************************************************
 * @file    tof_filter.c
 * @brief   VL53L1X 原始测距专用数字滤波器实现（中值-5 + 一阶 EMA）
 *
 *          处理顺序：
 *            1) 状态/量程门限：status != 0 或 raw 越界 → 丢弃，输出保持；
 *            2) 5 点中值：剔除孤立跳变；
 *            3) 一阶 EMA：平滑随机噪声，alpha 千分比在线可调；
 *            4) 零点换算：输出相对 zero_mm 的位置（0.1cm）。
 ******************************************************************************
 */
#include "tof_filter.h"

/* 对前 n 个样本做中值（n<=5）；小数组插入排序，无动态内存 */
static int16_t median_of(const int16_t *buf, uint8_t n) {
	int16_t v[TOF_FILTER_MEDIAN_N];
	uint8_t i;
	uint8_t j;

	if (n == 0u) {
		return 0;
	}
	for (i = 0; i < n; i++) {
		v[i] = buf[i];
	}
	for (i = 1; i < n; i++) {
		int16_t key = v[i];
		j			= i;
		while (j > 0u && v[j - 1u] > key) {
			v[j] = v[j - 1u];
			j--;
		}
		v[j] = key;
	}
	return v[n / 2u];
}

void TofFilter_Init(TofFilter_t *f, uint16_t alpha_milli, int16_t zero_mm) {
	if (f == 0) {
		return;
	}
	TofFilter_SetAlpha(f, alpha_milli);
	TofFilter_SetZeroMm(f, zero_mm);
	TofFilter_Reset(f);
}

void TofFilter_Reset(TofFilter_t *f) {
	uint8_t i;

	if (f == 0) {
		return;
	}
	for (i = 0; i < TOF_FILTER_MEDIAN_N; i++) {
		f->med_buf[i] = 0;
	}
	f->med_idx		= 0u;
	f->med_filled	= 0u;
	f->ema_milli_mm	= 0;
	f->last_raw_mm	= 0;
	f->last_med_mm	= 0;
	f->last_filt_mm	= 0;
	f->has_output	= false;
	f->valid_cnt	= 0u;
	f->reject_cnt	= 0u;
}

void TofFilter_SetAlpha(TofFilter_t *f, uint16_t alpha_milli) {
	if (f == 0) {
		return;
	}
	if (alpha_milli < TOF_FILTER_ALPHA_MIN_MILLI) {
		alpha_milli = TOF_FILTER_ALPHA_MIN_MILLI;
	} else if (alpha_milli > TOF_FILTER_ALPHA_MAX_MILLI) {
		alpha_milli = TOF_FILTER_ALPHA_MAX_MILLI;
	}
	f->alpha_milli = alpha_milli;
}

void TofFilter_SetZeroMm(TofFilter_t *f, int16_t zero_mm) {
	if (f == 0) {
		return;
	}
	if (zero_mm < 0) {
		zero_mm = 0;
	} else if (zero_mm > TOF_FILTER_MAX_VALID_MM) {
		zero_mm = TOF_FILTER_MAX_VALID_MM;
	}
	f->zero_mm = zero_mm;
}

bool TofFilter_ZeroHere(TofFilter_t *f) {
	if (f == 0 || !f->has_output) {
		return false;
	}
	f->zero_mm = f->last_filt_mm;
	return true;
}

bool TofFilter_Update(TofFilter_t *f, int16_t raw_mm, int8_t range_status) {
	int16_t med;
	int32_t target_milli;

	if (f == 0) {
		return false;
	}

	/* 无效帧/越界值：不进入窗口，输出保持上一有效值 */
	if (range_status != 0) {
		f->reject_cnt++;
		return false;
	}
	if (raw_mm < TOF_FILTER_MIN_VALID_MM || raw_mm > TOF_FILTER_MAX_VALID_MM) {
		f->reject_cnt++;
		return false;
	}

	f->last_raw_mm				= raw_mm;
	f->med_buf[f->med_idx]		= raw_mm;
	f->med_idx					= (uint8_t) ((f->med_idx + 1u) % TOF_FILTER_MEDIAN_N);
	if (f->med_filled < TOF_FILTER_MEDIAN_N) {
		f->med_filled++;
	}
	med = median_of(f->med_buf, f->med_filled);
	f->last_med_mm = med;

	/* EMA：首次有效输出直接对齐中值，避免上电从 0 缓慢爬升 */
	target_milli = (int32_t) med * 1000;
	if (!f->has_output) {
		f->ema_milli_mm = target_milli;
		f->has_output	= true;
	} else {
		f->ema_milli_mm += ((int32_t) f->alpha_milli * (target_milli - f->ema_milli_mm)) / 1000;
	}

	f->last_filt_mm = (int16_t) ((f->ema_milli_mm + 500) / 1000);
	f->valid_cnt++;
	return true;
}

int16_t TofFilter_GetMm(const TofFilter_t *f) {
	if (f == 0 || !f->has_output) {
		return 0;
	}
	return f->last_filt_mm;
}

int16_t TofFilter_GetMedianMm(const TofFilter_t *f) {
	if (f == 0 || !f->has_output) {
		return 0;
	}
	return f->last_med_mm;
}

int16_t TofFilter_GetCmX10(const TofFilter_t *f) {
	int32_t v;

	if (f == 0 || !f->has_output) {
		return 0;
	}
	v = ((int32_t) f->last_filt_mm - (int32_t) f->zero_mm) * TOF_POS_SIGN;
	if (v > 32767) {
		v = 32767;
	} else if (v < -32768) {
		v = -32768;
	}
	return (int16_t) v;
}

bool TofFilter_HasOutput(const TofFilter_t *f) {
	return (f != 0) && f->has_output;
}

uint32_t TofFilter_GetValidCount(const TofFilter_t *f) {
	return (f == 0) ? 0u : f->valid_cnt;
}

uint16_t TofFilter_GetAlphaMilli(const TofFilter_t *f) {
	return (f == 0) ? 0u : f->alpha_milli;
}

int16_t TofFilter_GetZeroMm(const TofFilter_t *f) {
	return (f == 0) ? 0 : f->zero_mm;
}
