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

/**
 * @brief 求前 n 个样本的中值（拷贝后插入排序，无动态内存）
 * @param buf 样本数组；n 样本数（0~TOF_FILTER_MEDIAN_N）
 * @retval 中值；n == 0 时返回 0
 * @note  仅 TofFilter_Update 调用；n ≤ 5，排序开销为常数级
 */
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

/**
 * @brief 初始化滤波器：设定 alpha 与零点后清空窗口和统计
 * @param f 滤波器实例；alpha_milli EMA 千分比（越界自动夹取）；zero_mm 零点（mm）
 */
void TofFilter_Init(TofFilter_t *f, uint16_t alpha_milli, int16_t zero_mm) {
	if (f == 0) {
		return;
	}
	TofFilter_SetAlpha(f, alpha_milli);
	TofFilter_SetValidRange(f, TOF_FILTER_MIN_VALID_MM, TOF_FILTER_MAX_VALID_MM);
	TofFilter_SetZeroMm(f, zero_mm);
	TofFilter_Reset(f);
}

/**
 * @brief 清中值窗口/EMA/统计（保留 alpha 与零点），用于 R 命令与重新起测
 */
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
	f->last_filt_mm	= 0;
	f->has_output	= false;
}

/**
 * @brief 设置 EMA 系数（千分比），越界自动夹到 [MIN, MAX]
 */
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

/**
 * @brief 设置零点距离（mm），越界自动夹到 [0, max_valid_mm]
 */
void TofFilter_SetZeroMm(TofFilter_t *f, int16_t zero_mm) {
	if (f == 0) {
		return;
	}
	if (zero_mm < 0) {
		zero_mm = 0;
	} else if (zero_mm > f->max_valid_mm) {
		zero_mm = f->max_valid_mm;
	}
	f->zero_mm = zero_mm;
}

/**
 * @brief 设置有效距离门限（mm），越界帧在 TofFilter_Update 中被拒绝
 * @param min 最小有效距离（<0 时取 0）；max 最大有效距离（≤min 时取默认 MAX）
 */
void TofFilter_SetValidRange(TofFilter_t *f, int16_t min, int16_t max) {
	if (f == 0) {
		return;
	}
	if (min < 0) {
		min = 0;
	}
	if (max <= min) {
		max = TOF_FILTER_MAX_VALID_MM;
	}
	f->min_valid_mm = min;
	f->max_valid_mm = max;
}

/**
 * @brief 以当前滤波值为新零点
 * @retval true = 已标定；false = 尚无有效输出，保持原零点
 */
bool TofFilter_ZeroHere(TofFilter_t *f) {
	if (f == 0 || !f->has_output) {
		return false;
	}
	f->zero_mm = f->last_filt_mm;
	return true;
}

/**
 * @brief 推入一帧原始测距值并推进滤波链（中值-5 → EMA）
 * @param  raw_mm       原始距离（mm）；range_status 传感器状态（0 = 有效）
 * @retval true = 本帧有效并已更新输出；false = 被拒（状态非 0 或越界），输出保持
 * @note   调用周期须与传感器帧率一致（本工程 20Hz）
 */
bool TofFilter_Update(TofFilter_t *f, int16_t raw_mm, int8_t range_status) {
	int16_t med;
	int32_t target_milli;

	if (f == 0) {
		return false;
	}

	/* 无效帧/越界值：不进入窗口，输出保持上一有效值 */
	if (range_status != 0) {
		return false;
	}
	if (raw_mm < f->min_valid_mm || raw_mm > f->max_valid_mm) {
		return false;
	}

	f->last_raw_mm				= raw_mm;
	f->med_buf[f->med_idx]		= raw_mm;
	f->med_idx					= (uint8_t) ((f->med_idx + 1u) % TOF_FILTER_MEDIAN_N);
	if (f->med_filled < TOF_FILTER_MEDIAN_N) {
		f->med_filled++;
	}
	med = median_of(f->med_buf, f->med_filled);

	/* EMA：首次有效输出直接对齐中值，避免上电从 0 缓慢爬升 */
	target_milli = (int32_t) med * 1000;
	if (!f->has_output) {
		f->ema_milli_mm = target_milli;
		f->has_output	= true;
	} else {
		f->ema_milli_mm += ((int32_t) f->alpha_milli * (target_milli - f->ema_milli_mm)) / 1000;
	}

	f->last_filt_mm = (int16_t) ((f->ema_milli_mm + 500) / 1000);
	return true;
}

/**
 * @brief 相对零点的位置（0.1cm）
 * @retval (滤波值 - 零点) × TOF_POS_SIGN；无有效输出时为 0，饱和到 int16 范围
 */
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
