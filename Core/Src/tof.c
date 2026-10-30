/**
 ******************************************************************************
 * @file    tof.c
 * @brief   ToF 测距硬件模块实现（VL53L1X 驱动 + 滤波调用 + 有效帧统计）
 *
 *          处理顺序：读一帧原始距离/状态 → 状态与量程门限 → 5 点中值
 *          → 一阶 EMA → 零点换算（0.1cm）→ 有效帧占比统计。
 ******************************************************************************
 */
#include "tof.h"

#include "i2c.h"	  /* hi2c1（平台层配置为 I2C2） */
#include "iwdg.h"	  /* hiwdg：初始化各步骤之间喂狗，避免累计阻塞触发 IWDG */
#include "main.h"	  /* HAL_Delay / HAL_GetTick / GPIO_InitTypeDef */
#include "tof_filter.h"
#include "vl53l1_api.h"
#include "vl53l1_platform_stm32.h"

/* 有效帧占比统计周期：5s（非刷新周期） */
#define VALID_WIN_MS 5000u

/* ==================== 传感器驱动（本模块私有） ========================== */

static VL53L1_Dev_t s_dev; /* 设备句柄 */

/* 执行一步初始化；失败时记录步骤名并跳出。定义成 do-while(0) 以便在 if 块里安全使用 */
#define VL53_INIT_STEP(expr, step_name)   \
	do {                                  \
		VL53L1_Error _st = (expr);        \
		HAL_IWDG_Refresh(&hiwdg);         \
		if (_st != VL53L1_ERROR_NONE) {   \
			g_tof.last_step = (step_name); \
			return _st;                   \
		}                                 \
	} while (0)

/**
 * @brief 传感器初始化并启动连续测量（内部阻塞 ~数百 ms，各步之间喂狗）
 * @retval 0 = 成功；非 0 = VL53L1X 错误码（失败步骤名见 g_tof.last_step）
 * @note  仅上电初始化阶段调用；本层不做提示输出/停机，显示与停机由 main.c 决定
 */
int Tof_InitSensor(void) {
	g_tof.last_step = "OK";

	/* 0. 总线准备：PB6/PB7 开漏复用模式下使能 STM32 内部上拉，并等传感器上电稳定。
	 *    若模块自带了外部上拉电阻，这段代码不会产生副作用，可放心保留。 */
	{
		GPIO_InitTypeDef i2c_gpio = {0};
		i2c_gpio.Pin			  = GPIO_PIN_6 | GPIO_PIN_7;
		i2c_gpio.Mode			  = GPIO_MODE_AF_OD;
		i2c_gpio.Pull			  = GPIO_PULLUP;
		i2c_gpio.Speed			  = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &i2c_gpio);
	}
	HAL_Delay(100);

	/* 1. 平台初始化：I2C 8 位地址 0x52、I2C 通信、400kHz、通过 XSHUT 复位 */
	VL53_INIT_STEP(VL53L1_platform_init(&s_dev, VL53L1X_I2C_ADDRESS, VL53L1_I2C, 400), "platform_init");
	/* 2. 等待芯片内部固件启动完成 */
	VL53_INIT_STEP(VL53L1_WaitDeviceBooted(&s_dev), "WaitDeviceBooted");
	/* 3. 数据初始化 + 静态初始化：从芯片读出默认参数填入句柄 */
	VL53_INIT_STEP(VL53L1_DataInit(&s_dev), "DataInit");
	VL53_INIT_STEP(VL53L1_StaticInit(&s_dev), "StaticInit");
	/* 4. 参考 SPAD 管理（裸模块上电执行一次即可） */
	VL53_INIT_STEP(VL53L1_PerformRefSpadManagement(&s_dev), "PerformRefSpadManagement");
	/* 5. ROI：缩到中心 8×8 SPAD，min range 从默认 ~70mm 降到 ~35mm
	 *    （4×4 时信噪比太低，硬件自检频繁炸；8×8 是近距离 min range 与
	 *    信噪比的合适折中）。TopLeftY≥BotRightY、width/height=间隔数。
	 *    ROI 越小 → min range 越低 → 信噪比越差 → 远距离越容易 5/7。
	 *    恢复全阵列：{0, 15, 15, 0}。 */
	{
		VL53L1_UserRoi_t roi = {4, 11, 11, 4}; /* 8×8 SPAD：TL(4,11)→BR(11,4)，间隔 7→8 SPAD */
		VL53_INIT_STEP(VL53L1_SetUserROI(&s_dev, &roi), "SetUserROI");
	}
	/* 6. 测距参数：关串扰补偿、SHORT 模式（250mm 近距离最优、抗环境光）、
	 *    16ms 测量预算、20ms 帧周期 → 50Hz（与 TIM1 的 20ms 采样回调对齐）。
	 *    注意：库强制要求 帧周期 ≥ 测量预算 + 4ms 保护(见 VL53L1_StartMeasurement)，
	 *    故 20ms 帧周期下预算取 16ms（20 = 16 + 4）恰好通过校验。 */
	VL53_INIT_STEP(VL53L1_SetXTalkCompensationEnable(&s_dev, 0), "SetXTalkCompensationEnable");
	VL53_INIT_STEP(VL53L1_SetDistanceMode(&s_dev, VL53L1_DISTANCEMODE_SHORT), "SetDistanceMode");
	VL53_INIT_STEP(VL53L1_SetMeasurementTimingBudgetMicroSeconds(&s_dev, 16000), "SetTimingBudget");
	VL53_INIT_STEP(VL53L1_SetInterMeasurementPeriodMilliSeconds(&s_dev, 20), "SetInterMeasurementPeriod");
	/* 6. 启动连续测量模式 */
	VL53_INIT_STEP(VL53L1_StartMeasurement(&s_dev), "StartMeasurement");

	return 0;
}

/**
 * @brief 阻塞读取一帧原始测距（未滤波）
 * @param distance_mm 输出原始距离（mm），失败时置 0
 * @param range_status 输出传感器状态（0 = 有效）；通信失败置 -1/-2
 * @retval VL53L1X 库错误码；0 = 通信正常（不代表测量有效，须看 range_status）
 * @note  含最长 2s 数据就绪等待，只能主循环调用；启动后第一帧丢弃不返回
 */
static VL53L1_Error TofDrive_Read(int16_t *distance_mm, int8_t *range_status) {
	static uint8_t					first_frame = 1; /* 官方建议丢弃启动后的第一帧数据 */
	VL53L1_RangingMeasurementData_t range;
	VL53L1_Error					st;

	/* 阻塞等待本帧数据就绪（内部 2s 超时） */
	st = VL53L1_WaitMeasurementDataReady(&s_dev);
	if (st != VL53L1_ERROR_NONE) {
		*range_status = -1; /* 等待数据就绪超时 */
		*distance_mm  = 0;
		/* 覆盖"总线没报 HAL 错误但数据迟迟不就绪"的静默卡死：先软件复位 I2C 总线，
		 * 再清除中断并启动下一帧，避免死区反复出现。 */
		VL53L1X_I2C_Recover();
		(void) VL53L1_ClearInterruptAndStartMeasurement(&s_dev);
		return st;
	}

	st = VL53L1_GetRangingMeasurementData(&s_dev, &range);
	if (st != VL53L1_ERROR_NONE) {
		*range_status = -2; /* 读取测量数据失败 */
		*distance_mm  = 0;
		(void) VL53L1_ClearInterruptAndStartMeasurement(&s_dev);
		return st;
	}

	/* 清除数据就绪中断并自动开始下一帧测量（连续模式必须调用） */
	(void) VL53L1_ClearInterruptAndStartMeasurement(&s_dev);

	if (first_frame) {
		/* 第一帧丢弃：不返回给上层，但已启动下一帧 */
		first_frame	  = 0;
		*range_status = -1;
		*distance_mm  = 0;
		return VL53L1_ERROR_NONE;
	}

	/* 直接输出原始测量值，不做任何滤波；数据可信度由 range_status 告知，
	 * 滤波（中值/低通）统一在本模块 Tof_Tick 里推进 */
	*distance_mm  = (int16_t) range.RangeMilliMeter;
	*range_status = (int8_t) range.RangeStatus; /* 0 = 有效，其他 = 信号弱/超量程等 */
	return VL53L1_ERROR_NONE;
}

/* ==================== 处理链状态（主循环独占写，其它文件只读） ========== */

/* 对外只读快照：写只发生在本文件（Tof_Tick / Tof_InitSensor） */
TofState_t g_tof = {
	.raw_mm		  = 0,
	.filt_mm	  = 0,
	.pos_cm_x10	  = 0,
	.range_status = -1,
	.valid_pct	  = 0,
	.alpha_milli  = TOF_FILTER_ALPHA_DEFAULT_MILLI,
	.last_step	  = "OK",
};

/* ---- 有效帧占比：每帧实时结算（随 OLED 5Hz 刷新），计数器 5s 清零 ---- */
static volatile uint32_t s_win_total  = 0; /* 窗口内已处理帧数 */
static volatile uint32_t s_win_valid  = 0; /* 窗口内有效帧数（out_st==0） */
static volatile uint32_t s_win_t0	  = 0; /* 窗口起始时刻 ms */

/* ==================== 公共接口 =========================================== */

/**
 * @brief 20Hz 节拍：读一帧 → 滤波 → 更新 g_tof 快照 → 累计有效帧占比
 * @param f 滤波器实例（应用层持有，须先 TofFilter_Init）
 * @note  无效帧不清空滤波状态，输出保持上一有效值；含阻塞等待，禁止进 ISR；
 *        有效帧占比每 5s 清零重计，供 OLED 显示
 */
void Tof_Tick(TofFilter_t *f) {
	int16_t dist_mm = 0;
	int8_t	rstat	= -1;
	int8_t	out_st;
	uint32_t now_ms;

	(void) TofDrive_Read(&dist_mm, &rstat);
	out_st = rstat; /* 通信错误码 -1/-2 原样上报 */

	if (rstat == 0) {
		g_tof.raw_mm = dist_mm;
		if (!TofFilter_Update(f, dist_mm, rstat)) {
			out_st = -3; /* 状态有效但原始值越界，滤波器拒绝 */
		} else {
			out_st = 0;
		}
		g_tof.filt_mm	 = f->last_filt_mm; /* 有效帧才写，无有效输出时保持 0 */
		g_tof.pos_cm_x10 = TofFilter_GetCmX10(f);
	} else {
		g_tof.raw_mm = 0; /* 通信失败帧不输出伪距离 */
	}
	g_tof.range_status	 = out_st;
	g_tof.alpha_milli	 = f->alpha_milli; /* A 命令改值后同步回显 */

	/* 有效帧占比：每帧实时结算（OLED 5Hz 直接显示），计数器每 5s 清零重计 */
	s_win_total++;
	if (out_st == 0) {
		s_win_valid++;
	}
	g_tof.valid_pct = (uint8_t) ((s_win_valid * 100u + s_win_total / 2u) / s_win_total);

	now_ms = HAL_GetTick();
	if ((uint32_t) (now_ms - s_win_t0) >= VALID_WIN_MS) {
		s_win_t0	= now_ms;
		s_win_total = 0;
		s_win_valid = 0;
	}
}

/**
 * @brief 清有效帧占比统计并重置 5s 窗口
 * @note  R 命令调用；滤波器自身状态由调用方另行 TofFilter_Reset
 */
void Tof_ResetStats(void) {
	s_win_total	 = 0;
	s_win_valid	 = 0;
	s_win_t0	 = HAL_GetTick();
	g_tof.valid_pct	 = 0;
}
