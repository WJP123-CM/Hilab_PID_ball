/**
 ******************************************************************************
 * @file    tof_filter.c
 * @brief   VL53L1X 原始测距专用数字滤波器实现（中值-5 + 一阶 EMA）
 *
 * ====================== 整体结构（四级流水线，逐帧调用） ======================
 *
 *   raw_mm, status
 *       │
 *       ▼
 *  ┌─────────────┐   无效帧：输出保持上一有效值，不污染窗口
 *  │ 1. 有效性门限 │   条件：range_status != 0        →  丢弃（传感器报信号弱等）
 *  │  (状态+量程)  │        raw ∉ [8, 500]mm          →  丢弃（近距盲区/超量程垃圾）
 *  └──────┬──────┘
 *         │ 有效帧
 *         ▼
 *  ┌─────────────┐   窗口：循环缓冲 med_buf[N]，新值覆盖最旧值
 *  │ 2. N 点中值  │   作用：剔除孤立尖峰（st=5/7 弱回波产生的跳变）
 *  └──────┬──────┘   特性：非线性、对单帧脉冲免疫；群延迟 = (N-1)/2 帧，群延迟是指从窗口填满到输出输入的延迟时间。
 *         │ med       启动期：med_filled < N 时用已填充的部分算中值，首帧即输出
 *         ▼
 *  ┌─────────────┐   公式：ema = ema + alpha·(med - ema)
 *  │ 3. 一阶 EMA  │         alpha ∈ [0.02, 1.00]，千分比存储（300 = 0.300）
 *  │ (指数移动平均)│   作用：抑制随机噪声（量化噪声 + 散粒噪声）
 *  └──────┬──────┘   特性：线性时不变、指数滞后；时间常数 τ = −T/ln(1−α)
 *         │ filt      首帧：直接赋值 ema = med，避免上电从 0 缓慢爬升
 *         ▼
 *  ┌─────────────┐   pos = filt_mm − zero_mm（远离传感器为正）
 *  │ 4. 零点换算  │   输出单位：0.1 cm（pos_cm_x10）
 *  └─────────────┘   zero_mm：Z 命令标定的静止零点距离
 *
 * ====================== 可配置参数（宏 + 运行时接口） ======================
 *
 * 【中值窗口】
 *   TOF_FILTER_MEDIAN_N  (固定编译宏，=3)
 *     窗口长度。越大抗尖峰越强，但群延迟越大（每增 2 点延迟 +1 帧）。
 *     N=3 时延迟 1 帧 = 20 ms（50 Hz），可去除单帧孤立尖峰。
 *
 * 【EMA 系数】
 *   alpha_milli  (运行时可调，A 命令，范围 [20, 1000]‰)
 *     EMA 平滑系数，千分比。1000 = 直通（不滤波），越小越平滑但延迟越大。
 *     默认 800 (α=0.80) → τ ≈ 40 ms，阶跃到 95% 约 120 ms。
 *     调参方向：噪声大→减小 alpha；响应慢→增大 alpha。
 *   TOF_FILTER_ALPHA_DEFAULT/MIN/MAX_MILLI  (编译宏，限定上下限与默认值)
 *
 * 【有效距离门限】
 *   min_valid_mm / max_valid_mm  (运行时可调，2026-10-31 定为 8 / 500 mm)
 *     原始距离越此范围的帧被视为无效并丢弃（拒绝时 Tof_Tick 上报 -3）；
 *     max_valid_mm 同时用于 TofFilter_SetZeroMm 的零点夹取。
 *
 * 【零点与位置】
 *   zero_mm  (运行时可调，Z 命令标定，默认 131 mm)
 *     静止时小球到传感器的期望距离。pos_cm_x10 = filt − zero。
 *     本模块不再提供方向符号；外环方向统一由 main.c 的 BALL_X_ENC_SIGN 决定。
 *
 * ====================== 数值实现细节 ======================
 *
 *   - EMA 内部状态 ema_milli_mm 以 0.001 mm 为单位存储，避免整数除法
 *     累积截断误差；输出时四舍五入到 mm。
 *   - 中值排序在局部栈数组 v[N] 上做插入排序，不占用实例 RAM；
 *     N≤3 时排序开销为常数级。
 *   - 所有运算为纯整数，不依赖 libm/HAL，可在 50 Hz 主循环直接调用。
 *
 * ====================== 延迟构成（50 Hz，T=20 ms） ======================
 *
 *   中值固定延迟：(N−1)/2 = 1 帧 = 20 ms（不可压缩）
 *   EMA 指数滞后：τ ≈ 40 ms（α=0.80），阶跃 63% 用时
 *   首帧可用性：启动期用部分窗口中值，第 1 帧有效数据即有输出
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
 * @brief 推入一帧原始测距值并推进滤波链（中值-3 → EMA）
 * @param  raw_mm       原始距离（mm）；range_status 传感器状态（0 = 有效）
 * @retval true = 本帧有效并已更新输出；false = 被拒（状态非 0），输出保持
 * @note   调用周期须与传感器帧率一致（本工程 50Hz）
 */
bool TofFilter_Update(TofFilter_t *f, int16_t raw_mm, int8_t range_status) {
	int16_t med;
	int32_t target_milli;

	if (f == 0) {
		return false;
	}

	/* 无效帧/越界值：不进入窗口，输出保持上一有效值。
	 * 2026-10-31 量程门放宽为 [8, 500]mm：只挡近距盲区读数与超量程垃圾值 */
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
 * @param f 滤波器实例
 * @retval 滤波值 − 零点值（0.1cm，远离传感器为正）；无有效输出时为 0，饱和到 int16 范围
 */
int16_t TofFilter_GetCmX10(const TofFilter_t *f) {
	int32_t v;

	if (f == 0 || !f->has_output) {
		return 0;
	}
	v = (int32_t) f->last_filt_mm - (int32_t) f->zero_mm;
	if (v > 32767) {
		v = 32767;
	} else if (v < -32768) {
		v = -32768;
	}
	return (int16_t) v;
}
