/**
 ******************************************************************************
 * @file    tof_filter.c
 * @brief   VL53L1X 原始测距专用数字滤波器实现（KF 主链 + 中值-3/EMA 对照链）
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
 * ====================== KF 主链（与 EMA 并行，EMA 只作对照） ======================
 *   每拍：KF_Predict(dt = 真实帧间隔，限幅 5~60ms)
 *   有效帧：KF_UpdatePos(raw, R=3.5mm²)  ← 不加新息门控（实测会锁死）
 *   输出：始终取 KF 位置；EMA 链照常运行，仅用于遥测对比与偏差告警
 *         （实测自动回退只在传感器盲区触发，那里 EMA 同样不准，故不切换）
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
 *   - EMA 链为纯整数；KF 主链为浮点（仍不依赖 libm/HAL），时间基准由调用方传入。
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

	/* KF 主链：下次有效帧重新初始化；诊断计数一并清零 */
	f->kf_last_ms	 = 0u;
	f->kf_bad_streak = 0u;
	f->kf_miss		 = 0u;
	f->kf_ready		 = false;
	/* 上电后到第一帧有效数据之前，外环不应基于无效数据动作 */
	f->last_valid_ms = 0u;
	f->stale		 = true;
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
 * @brief 推入一帧原始测距值并推进滤波链（KF 主链 + 中值-3/EMA 对照链）
 * @param  raw_mm       原始距离（mm）；range_status 传感器状态（0 = 有效）
 * @param  now_ms       本拍时刻（HAL_GetTick），供 KF 计算真实 dt
 * @retval true = 本帧有效并已更新输出；false = 被拒（状态非 0 / 越界），输出保持
 * @note   调用周期须与传感器帧率一致（本工程 50Hz）；无效帧仍推进 KF 预测
 */
bool TofFilter_Update(TofFilter_t *f, int16_t raw_mm, int8_t range_status, uint32_t now_ms) {
	int16_t med;
	int32_t target_milli;
	int16_t ema_mm;
	float	dt;
	float	kf_pos;
	float	diff;
	bool	valid;
	bool	accepted = false;

	if (f == 0) {
		return false;
	}

	/* 有效性门限：状态为 0 且原始距离在量程内（与接入 KF 前一致） */
	valid = (range_status == 0) && (raw_mm >= f->min_valid_mm) && (raw_mm <= f->max_valid_mm);

	/* ---- 数据过期判定（必须先于预测）----
	 * 超过 TOF_STALE_MS 没有有效帧：停止 KF 预测并置 stale，外环据此冻结输出。
	 * 否则 KF 会按最后速度一直外推（实测 -24mm/s × 90s ≈ 2m），把外环推到饱和、
	 * 杆顶到机械限位。恢复时用本帧真实测距重新播种，丢弃漂移后的旧状态。 */
	if (valid) {
		if (f->stale && f->kf_ready) {
			KF_Reset(&f->kf, (float) raw_mm, 0.0f, 1000.0f);
			f->kf_last_ms = now_ms;
		}
		f->last_valid_ms = now_ms;
		f->stale		 = false;
	} else if ((uint32_t) (now_ms - f->last_valid_ms) > TOF_STALE_MS) {
		f->stale = true;
	}

	/* ---- KF：首帧有效观测时初始化，之后每次调用推进一拍（dt 取真实间隔） ---- */
	if (!f->kf_ready) {
		if (valid) {
			KF_Init(&f->kf, (float) raw_mm, 0.0f, TOF_KF_DT_DEFAULT, TOF_KF_SIG_A2, 1000.0f);
			KF_SetVelLimit(&f->kf, TOF_KF_VEL_MAX);
			f->kf_last_ms = now_ms;
			f->kf_ready	  = true;
		}
	} else if (!f->stale) {
		dt			  = (float) (now_ms - f->kf_last_ms) / 1000.0f;
		f->kf_last_ms = now_ms;
		if (dt < 0.005f) {
			dt = 0.005f; /* 同拍重复调用保护 */
		}
		if (dt > 0.060f) {
			dt = 0.060f; /* 主循环卡顿：压到 60ms，避免 Q 与协方差突跳 */
		}
		KF_SetDt(&f->kf, dt);
		KF_Predict(&f->kf); /* 无效帧也要预测，协方差自然增长；过期后不再预测 */
	}

	if (!valid) {
		if (f->kf_miss < 255u) {
			f->kf_miss++;
		}
	} else {
		f->kf_miss	   = 0u;
		f->last_raw_mm = raw_mm;

		/* ---- EMA 链：保持原实现，仅作对照与遥测 ---- */
		f->med_buf[f->med_idx] = raw_mm;
		f->med_idx			   = (uint8_t) ((f->med_idx + 1u) % TOF_FILTER_MEDIAN_N);
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

		/* ---- KF 观测更新：新息门控（零延迟削峰） ----
		 * KF 有速度预测后正常运动的新息≈纯噪声（±2mm），5σ 门限不会误判；
		 * 单帧尖峰（实测 332mm vs 真实 124mm）新息达 208mm，当帧即被拒绝。
		 * 连续 3 拍越限会自动放行（KF_GATE_RELEASE_N），真实阶跃不受影响。 */
		accepted = true;
		if (f->kf_ready) {
			if (KF_UpdatePosGated(&f->kf, (float) raw_mm, TOF_KF_R, TOF_KF_GATE_N) == 2) {
				accepted = false; /* 本帧观测判为离群：KF 只保留预测 */
				if (f->kf_miss < 255u) {
					f->kf_miss++;
				}
			}
		}
	}

	if (!f->has_output) {
		return accepted; /* 尚无有效帧：last_filt_mm 保持初值 0 */
	}

	ema_mm = (int16_t) ((f->ema_milli_mm + 500) / 1000);

	if (!f->kf_ready) {
		f->last_filt_mm = ema_mm; /* 理论上与 has_output 同步，兜底 */
		return accepted;
	}

	/* ---- 输出：始终取 KF；EMA 只算不选，供遥测对比与排查 ---- */
	kf_pos = KF_GetPos(&f->kf);
	diff   = kf_pos - (float) ema_mm;
	if (diff < 0.0f) {
		diff = -diff;
	}

	/* 偏差超限只累计计数并上报（kf_state），不切换输出：
	 * 实测触发位置全在传感器盲区，那里 EMA 同样不准，切换反而把 KF 拖旧 */
	if (diff > TOF_KF_GUARD_MM) {
		if (f->kf_bad_streak < 255u) {
			f->kf_bad_streak++;
		}
	} else {
		f->kf_bad_streak = 0u;
	}

	f->last_filt_mm = (int16_t) (kf_pos + 0.5f);
	return accepted;
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

/**
 * @brief 位置外推到 now_ms 时刻（0.1cm），消除"上次采样到现在"的滞后
 * @param f 滤波器实例；now_ms 当前时刻（HAL_GetTick）
 * @retval 相对零点的位置（0.1cm）；无有效输出为 0
 * @note  外部按绝对时间差计算，每次调用独立重算，不累积、不改写 KF 状态。
 *        Δt 压到 1.5 帧以内、外推量限幅 ±TOF_KF_PRED_MAX_MM，
 *        防止主循环卡顿或长时间无观测时速度漂移被放大。
 */
int16_t TofFilter_GetCmX10At(const TofFilter_t *f, uint32_t now_ms) {
	float	dtp, dp, rel, vel, av, w;
	int32_t v;

	if (f == 0 || !f->has_output) {
		return 0;
	}

	/* 对照实验：输出切到 EMA 链（无速度估计，故不外推） */
	if (f->out_use_ema) {
		v = (int32_t) ((f->ema_milli_mm + 500) / 1000) - (int32_t) f->zero_mm;
		if (v > 32767) {
			v = 32767;
		} else if (v < -32768) {
			v = -32768;
		}
		return (int16_t) v;
	}

	if (!f->kf_ready || f->stale) {
		/* 未初始化或数据过期：返回最后一次有效滤波值（冻结），不做速度外推 */
		v = (int32_t) f->last_filt_mm - (int32_t) f->zero_mm;
	} else {
		dtp = (float) (uint32_t) (now_ms - f->kf_last_ms) * 0.001f;
		if (dtp > TOF_KF_PRED_DT_MAX) {
			dtp = TOF_KF_PRED_DT_MAX;
		}

		/* 速度死区 + 线性过渡：静止时速度估计有 ±30mm/s 噪声，
		 * 直接乘 dt 会让静止输出 σ 增大 25%；|v| 低于 VMIN 时不外推 */
		vel = KF_GetVel(&f->kf);
		av	= (vel >= 0.0f) ? vel : -vel;
		if (av <= TOF_KF_PRED_VMIN) {
			dp = 0.0f;
		} else {
			w = (av >= TOF_KF_PRED_VMAX)
					? 1.0f
					: (av - TOF_KF_PRED_VMIN) / (TOF_KF_PRED_VMAX - TOF_KF_PRED_VMIN);
			dp = vel * dtp * w;
			if (dp > TOF_KF_PRED_MAX_MM) {
				dp = TOF_KF_PRED_MAX_MM;
			} else if (dp < -TOF_KF_PRED_MAX_MM) {
				dp = -TOF_KF_PRED_MAX_MM;
			}
		}

		rel = KF_GetPos(&f->kf) + dp - (float) f->zero_mm;
		v	= (int32_t) (rel + (rel >= 0.0f ? 0.5f : -0.5f));
	}

	if (v > 32767) {
		v = 32767;
	} else if (v < -32768) {
		v = -32768;
	}
	return (int16_t) v;
}

/**
 * @brief 数据是否过期（超过 TOF_STALE_MS 没有有效帧）
 * @retval true = KF 已停止预测、输出为最后一次有效值（冻结）
 * @note  过期期间外环应停止积分；P/D 因位置冻结而自动保持
 */
bool TofFilter_IsStale(const TofFilter_t *f) {
	return (f != 0) && f->stale;
}

/**
 * @brief 选择输出源（KF / EMA 对照实验用）
 * @param use_ema true = 取 EMA 链；false = 取 KF（默认）
 */
void TofFilter_SetOutputEma(TofFilter_t *f, bool use_ema) {
	if (f != 0) {
		f->out_use_ema = use_ema;
	}
}
