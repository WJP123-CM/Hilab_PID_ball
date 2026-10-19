/*******************************************************************************
 * VL53L1X STM32F1xx HAL 移植 - Platform 层
 * ----------------------------------------------------------------------------
 * 这份文件用于【替换】官方库里的这两个文件（它们在 E:\stm32使用库及教程\激光测距模块
 * 里是给 Windows 评估板用的，带 <windows.h>，不能在 STM32 上直接编译）：
 *     API/platform/src/vl53l1_platform.c
 *     API/platform/src/vl53l1_platform_init.c
 *
 * 用法：把本文件 + 头文件放进工程，并在 CMake/Keil 里加入编译。
 *       I2C 用 CubeMX 配好的 I2C1（PB6=SCL, PB7=SDA），句柄 hi2c1。
 *
 * 接线提醒：
 *   VL53L1X  SCL <-> PB6,  SDA <-> PB7
 *   VL53L1X  XSHUT <-> 一个 IO（默认 PC13，改下方宏）
 *   VL53L1X  GPIO1 不用可悬空
 ******************************************************************************/
#include "vl53l1_platform_stm32.h" /* 拉进 prototype、VL53L1_Dev_t、VL53L1_I2C 等 */

#include "i2c.h"  /* hi2c1 */
#include "main.h" /* HAL: HAL_GetTick, HAL_Delay, GPIO, DWT */

/* ============================= 可修改区 ==================================== */
#define VL53L1_I2C_HANDLE	  hi2c1 /* I2C 句柄 */
#define VL53L1_I2C_TIMEOUT_MS 100	/* 单次 I2C 超时(ms) */
#define VL53L1_I2C_CHUNK	  255	/* 每次 I2C 最多发的字节数 */

/* XSHUT 复位脚，按你的接线改 */
#define VL53L1X_XSHUT_PORT GPIOC
#define VL53L1X_XSHUT_PIN  GPIO_PIN_13
/* ========================================================================== */

/* 根据 VL53L1X_XSHUT_PORT 宏使能对应 GPIO 时钟（与端口绑定，不再硬编码）
 * 注意：GPIOx 在 HAL 头文件里展开为指针，不能用 #if 预处理比较，
 *       故改用运行时 if；端口宏是常量，编译时会自动优化掉无关分支。
 *       STM32F103xB 只有 GPIOA~GPIOE。 */
static void VL53L1X_XSHUT_CLK_Enable(void) {
	if (VL53L1X_XSHUT_PORT == GPIOA) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
	} else if (VL53L1X_XSHUT_PORT == GPIOB) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
	} else if (VL53L1X_XSHUT_PORT == GPIOC) {
		__HAL_RCC_GPIOC_CLK_ENABLE();
	} else if (VL53L1X_XSHUT_PORT == GPIOD) {
		__HAL_RCC_GPIOD_CLK_ENABLE();
	} else if (VL53L1X_XSHUT_PORT == GPIOE) {
		__HAL_RCC_GPIOE_CLK_ENABLE();
	} else {
		/* 端口未匹配：检查上方 VL53L1X_XSHUT_PORT 宏是否只在 GPIOA~E 中选择 */
	}
}

/* 官方库只对 I2C 路径做了长数据分块；这里也做同样处理，避免超长一次传输 */

/* ---------------- 内部小工具：微秒延时（DWT 周期计数器） ------------------ */
static void VL53L1_DelayUsHelper(uint32_t us) {
	/* 使能 DWT 周期计数器（只需一次） */
	if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0U;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}

	uint32_t start = DWT->CYCCNT;
	uint32_t ticks = (uint32_t) ((uint64_t) SystemCoreClock * (uint64_t) us / 1000000UL);
	while ((uint32_t) (DWT->CYCCNT - start) < ticks) {}
}

/* ===================== Comms 初始 / 关闭 ================================== */

VL53L1_Error VL53L1_CommsInitialise(VL53L1_Dev_t *pdev, uint8_t comms_type, uint16_t comms_speed_khz) {
	(void) pdev;
	(void) comms_speed_khz;
	if (comms_type != VL53L1_I2C) {
		return VL53L1_ERROR_CONTROL_INTERFACE;
	}
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_CommsClose(VL53L1_Dev_t *pdev) {
	(void) pdev;
	return VL53L1_ERROR_NONE;
}

/* ===================== 核心：多字节读写 ==================================== */
/* 注意：pdev->i2c_slave_address 存的是 8 位形式 0x52（=7位地址0x29 左移1位）。
 *       STM32F1 HAL 的 DevAddress 参数要求的正是这种"左移1位"的形式
 *       （HAL 内部 I2C_7BIT_ADD_WRITE 会自行右移），所以直接传，不能再 >>1，
 *       否则总线上发出的地址会变成 0x0A，从机 NACK。 */

/* ---------------- I2C 总线恢复（软件释放总线） ------------------------------
 * STM32F1 HAL 轮询 I2C 出错后不会自动清 BUSY 位，从机拉低 SDA 或时钟拉伸会导致
 * 永久挂死。恢复步骤：
 *   1) DeInit 关闭 I2C 外设，把 SCL/SDA 暂时改配为普通输出（推挽）；
 *   2) 对 SCL 打 9 个时钟脉冲，把卡在中间传输的从机"顶"出来；
 *   3) 再拉出 STOP 条件（SCL 高时 SDA 由低转高）；
 *   4) GPIO 恢复为开漏复用，重新 Init I2C。
 * 注意：若 SDA 被从机持续拉死后 9 脉冲仍无效，则仍会走 IWDG 复位兜底。 */
void VL53L1X_I2C_Recover(void) {
	GPIO_InitTypeDef gpio = {0};
	int32_t			 i;

	/* 1. 关闭外设 + 还原因子：清掉挂起的错误状态 */
	HAL_I2C_DeInit(&VL53L1_I2C_HANDLE);

	/* 2. SCL/SDA 改为普通推挽输出，模拟开漏时序 */
	gpio.Pin   = VL53_SCL_Pin | VL53_SDA_Pin;
	gpio.Mode  = GPIO_MODE_OUTPUT_PP;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(VL53_SCL_GPIO_Port, &gpio);

	/* 先释放：都拉高 */
	HAL_GPIO_WritePin(VL53_SCL_GPIO_Port, VL53_SCL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(VL53_SDA_GPIO_Port, VL53_SDA_Pin, GPIO_PIN_SET);
	VL53L1_DelayUsHelper(5);

	/* 3. SCL 打 9 个脉冲（从机在数据传输中途可能处于中间状态，靠翻转 SCL 复位它） */
	for (i = 0; i < 9; i++) {
		HAL_GPIO_WritePin(VL53_SCL_GPIO_Port, VL53_SCL_Pin, GPIO_PIN_RESET);
		VL53L1_DelayUsHelper(5);
		HAL_GPIO_WritePin(VL53_SCL_GPIO_Port, VL53_SCL_Pin, GPIO_PIN_SET);
		VL53L1_DelayUsHelper(5);
	}

	/* 4. 发出 START 后立刻 STOP，让所有从机复位其内部总线状态机 */
	HAL_GPIO_WritePin(VL53_SDA_GPIO_Port, VL53_SDA_Pin, GPIO_PIN_RESET);
	VL53L1_DelayUsHelper(5);
	HAL_GPIO_WritePin(VL53_SCL_GPIO_Port, VL53_SCL_Pin, GPIO_PIN_RESET);
	VL53L1_DelayUsHelper(5);
	HAL_GPIO_WritePin(VL53_SCL_GPIO_Port, VL53_SCL_Pin, GPIO_PIN_SET); /* 先放 SCL，再放 SDA → STOP */
	VL53L1_DelayUsHelper(5);
	HAL_GPIO_WritePin(VL53_SDA_GPIO_Port, VL53_SDA_Pin, GPIO_PIN_SET);
	VL53L1_DelayUsHelper(5);

	/* 5. 恢复为开漏复用，重新初始化 I2C 外设 */
	gpio.Mode  = GPIO_MODE_AF_OD;
	gpio.Pull  = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(VL53_SCL_GPIO_Port, &gpio);
	HAL_GPIO_Init(VL53_SDA_GPIO_Port, &gpio);
	(void) HAL_I2C_Init(&VL53L1_I2C_HANDLE);
}

VL53L1_Error VL53L1_WriteMulti(VL53L1_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count) {
	uint16_t addr7 = (uint16_t) pdev->i2c_slave_address;
	uint32_t pos   = 0;
	uint32_t size;

	while (pos < count) {
		size = (count - pos > VL53L1_I2C_CHUNK) ? VL53L1_I2C_CHUNK : (count - pos);
		if (HAL_I2C_Mem_Write(&VL53L1_I2C_HANDLE, addr7, index + (uint16_t) pos, I2C_MEMADD_SIZE_16BIT,
							  (uint8_t *) (pdata + pos), (uint16_t) size, VL53L1_I2C_TIMEOUT_MS) != HAL_OK) {
			VL53L1X_I2C_Recover(); /* 出错即软件复位总线，避免永久挂死 */
			return VL53L1_ERROR_CONTROL_INTERFACE;
		}
		pos += size;
	}
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_ReadMulti(VL53L1_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count) {
	uint16_t addr7 = (uint16_t) pdev->i2c_slave_address;
	uint32_t pos   = 0;
	uint32_t size;

	while (pos < count) {
		size = (count - pos > VL53L1_I2C_CHUNK) ? VL53L1_I2C_CHUNK : (count - pos);
		if (HAL_I2C_Mem_Read(&VL53L1_I2C_HANDLE, addr7, index + (uint16_t) pos, I2C_MEMADD_SIZE_16BIT,
							 (uint8_t *) (pdata + pos), (uint16_t) size, VL53L1_I2C_TIMEOUT_MS) != HAL_OK) {
			VL53L1X_I2C_Recover(); /* 出错即软件复位总线，避免永久挂死 */
			return VL53L1_ERROR_CONTROL_INTERFACE;
		}
		pos += size;
	}
	return VL53L1_ERROR_NONE;
}

/* ===================== 单字节 / 双字节 / 四字节 读写 ====================== */
/* 与官方实现一致：大端序，全部走 WriteMulti / ReadMulti */
VL53L1_Error VL53L1_WrByte(VL53L1_Dev_t *pdev, uint16_t index, uint8_t data) {
	return VL53L1_WriteMulti(pdev, index, &data, 1);
}

VL53L1_Error VL53L1_WrWord(VL53L1_Dev_t *pdev, uint16_t index, uint16_t data) {
	uint8_t buf[2];
	buf[0] = (uint8_t) (data >> 8);
	buf[1] = (uint8_t) (data & 0x00FF);
	return VL53L1_WriteMulti(pdev, index, buf, 2);
}

VL53L1_Error VL53L1_WrDWord(VL53L1_Dev_t *pdev, uint16_t index, uint32_t data) {
	uint8_t buf[4];
	buf[0] = (uint8_t) (data >> 24);
	buf[1] = (uint8_t) ((data & 0x00FF0000) >> 16);
	buf[2] = (uint8_t) ((data & 0x0000FF00) >> 8);
	buf[3] = (uint8_t) (data & 0x000000FF);
	return VL53L1_WriteMulti(pdev, index, buf, 4);
}

VL53L1_Error VL53L1_RdByte(VL53L1_Dev_t *pdev, uint16_t index, uint8_t *pdata) {
	return VL53L1_ReadMulti(pdev, index, pdata, 1);
}

VL53L1_Error VL53L1_RdWord(VL53L1_Dev_t *pdev, uint16_t index, uint16_t *pdata) {
	VL53L1_Error status;
	uint8_t		 buf[2];
	status = VL53L1_ReadMulti(pdev, index, buf, 2);
	*pdata = (uint16_t) (((uint16_t) buf[0] << 8) | (uint16_t) buf[1]);
	return status;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_Dev_t *pdev, uint16_t index, uint32_t *pdata) {
	VL53L1_Error status;
	uint8_t		 buf[4];
	status = VL53L1_ReadMulti(pdev, index, buf, 4);
	*pdata = ((uint32_t) buf[0] << 24) | ((uint32_t) buf[1] << 16) | ((uint32_t) buf[2] << 8) | (uint32_t) buf[3];
	return status;
}

/* ===================== 延时 =============================================== */

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us) {
	(void) pdev;
	if (wait_us > 0) {
		VL53L1_DelayUsHelper((uint32_t) wait_us);
	}
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms) {
	(void) pdev;
	if (wait_ms > 0) {
		HAL_Delay((uint32_t) wait_ms);
	}
	return VL53L1_ERROR_NONE;
}

/* ===================== 定时器相关（本驱动未使用，给 0 即可） ============== */

VL53L1_Error VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz) {
	*ptimer_freq_hz = 0;
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTimerValue(int32_t *ptimer_count) {
	*ptimer_count = 0;
	return VL53L1_ERROR_NONE;
}

/* ===================== GPIO =============================================== */

VL53L1_Error VL53L1_GpioSetMode(uint8_t pin, uint8_t mode) {
	(void) pin;
	(void) mode;
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioSetValue(uint8_t pin, uint8_t value) {
	(void) pin;
	(void) value;
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioGetValue(uint8_t pin, uint8_t *pvalue) {
	(void) pin;
	*pvalue = 0;
	return VL53L1_ERROR_NONE;
}

/* XSHUT：value=0 复位，value=1 运行 */
VL53L1_Error VL53L1_GpioXshutdown(uint8_t value) {
	GPIO_InitTypeDef g = {0};

	VL53L1X_XSHUT_CLK_Enable(); /* 时钟按端口宏使能（幂等），并每次都确保引脚已配成输出 */

	g.Pin	= VL53L1X_XSHUT_PIN;
	g.Mode	= GPIO_MODE_OUTPUT_PP;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(VL53L1X_XSHUT_PORT, &g);

	HAL_GPIO_WritePin(VL53L1X_XSHUT_PORT, VL53L1X_XSHUT_PIN, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioCommsSelect(uint8_t value) {
	(void) value;
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioPowerEnable(uint8_t value) {
	(void) value;
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioInterruptEnable(void (*function)(void), uint8_t edge_type) {
	(void) function;
	(void) edge_type;
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GpioInterruptDisable(void) {
	return VL53L1_ERROR_NONE;
}

/* ===================== 系统时基 =========================================== */

VL53L1_Error VL53L1_GetTickCount(VL53L1_Dev_t *pdev, uint32_t *ptick_count_ms) {
	(void) pdev;
	*ptick_count_ms = (uint32_t) HAL_GetTick();
	return VL53L1_ERROR_NONE;
}

/* 轮询指定寄存器直到满足 value & mask，超时返回 TIME_OUT */
VL53L1_Error VL53L1_WaitValueMaskEx(VL53L1_Dev_t *pdev, uint32_t timeout_ms, uint16_t index, uint8_t value,
									uint8_t mask, uint32_t poll_delay_ms) {
	VL53L1_Error status		= VL53L1_ERROR_NONE;
	uint32_t	 start_ms	= (uint32_t) HAL_GetTick();
	uint8_t		 byte_value = 0;

	(void) poll_delay_ms;

	do {
		status = VL53L1_RdByte(pdev, index, &byte_value);
		if (status != VL53L1_ERROR_NONE) {
			/* I2C 读失败：直接返回真实错误，不要伪装成超时，便于调试 */
			return status;
		}
		if ((byte_value & mask) == value) {
			pdev->new_data_ready_poll_duration_ms = 0;
			return VL53L1_ERROR_NONE;
		}
	} while (((uint32_t) HAL_GetTick() - start_ms) < timeout_ms);

	pdev->new_data_ready_poll_duration_ms = timeout_ms;
	return VL53L1_ERROR_TIME_OUT;
}

/* ===================== 平台 init / terminate ============================= */

VL53L1_Error VL53L1_platform_init(VL53L1_Dev_t *pdev, uint8_t i2c_slave_address, uint8_t comms_type,
								  uint16_t comms_speed_khz) {
	VL53L1_Error status;

	pdev->i2c_slave_address = i2c_slave_address;
	pdev->comms_type		= comms_type;
	pdev->comms_speed_khz	= comms_speed_khz;

	/* 复位器件：先拉低 XSHUT，再拉高完成上电启动 */
	status = VL53L1_GpioXshutdown(0U);
	if (status == VL53L1_ERROR_NONE) {
		VL53L1_WaitMs(pdev, 50);
		status = VL53L1_GpioXshutdown(1U);
	}

	if (status == VL53L1_ERROR_NONE) {
		status = VL53L1_CommsInitialise(pdev, comms_type, comms_speed_khz);
	}

	return status;
}

VL53L1_Error VL53L1_platform_terminate(VL53L1_Dev_t *pdev) {
	(void) pdev;
	return VL53L1_GpioXshutdown(0U);
}
