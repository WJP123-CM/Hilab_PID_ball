/**
 ******************************************************************************
 * @file    vl53l1x.c
 * @brief   VL53L1X ToF 测距传感器封装实现
 ******************************************************************************
 */
#include "vl53l1x.h"

#include "i2c.h"  /* hi2c1 */
#include "main.h" /* HAL_Delay、GPIO_InitTypeDef 等 */
#include "vl53l1_api.h"
#include "vl53l1_platform_stm32.h"

#include <string.h>

/* ===================== 模块私有变量 ======================================= */
static VL53L1_Dev_t s_dev;				/* 设备句柄 */
static const char  *s_last_step = "OK"; /* 最近一次失败的步骤名 */

/* ===================== 私有辅助宏 ========================================= */
/* 执行一步初始化；失败时记录步骤名并跳出。定义成 do-while(0) 以便在 if 块里安全使用 */
#define VL53_INIT_STEP(expr, step_name) \
	do {                                \
		VL53L1_Error _st = (expr);      \
		if (_st != VL53L1_ERROR_NONE) { \
			s_last_step = (step_name);  \
			return _st;                 \
		}                               \
	} while (0)

/* ===================== 公共接口 =========================================== */

VL53L1_Error VL53L1X_Init(void) {
	s_last_step = "OK";

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

	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1X_Read(int16_t *distance_mm, int8_t *range_status) {
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
	 * 滤波（中值/低通等）统一放到上层 PID 环节处理 */
	*distance_mm  = (int16_t) range.RangeMilliMeter;
	*range_status = (int8_t) range.RangeStatus; /* 0 = 有效，其他 = 信号弱/超量程等 */
	return VL53L1_ERROR_NONE;
}

const char *VL53L1X_LastErrorStep(void) {
	return s_last_step;
}
