/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "gpio.h"
#include "i2c.h"
#include "iwdg.h"
#include "motor.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmd.h"		/* 上位机行协议：A/Z/P/I/D/R */
/* ctrl.c 已禁用，用户自行在 main.c 编写控制逻辑 */
#include "encoder.h"	/* 编码器计数/Z 校准/掉电保存 */
#include "key.h"		/* PA0 按键消抖与事件队列 */
#include "oled.h"		/* OLED 屏幕原语 */
#include "pid.h"		/* 位置式 PID 控制器（标准命名：Target/Actual/Out/Kp/Ki/Kd） */
#include "tof.h"		/* 测距 → 滤波 → 有效帧占比（含 g_tof_filter） */

#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 调度周期：测距/滤波/位置闭环同拍 20ms（50Hz，单环） */
#define MEAS_PERIOD_MS MOTOR_MEAS_PERIOD_MS /* 测距/滤波/位置闭环：50Hz */
#define CMD_PERIOD_MS  100u					/* 串口命令：10Hz（主循环执行） */
#define OLED_PERIOD_MS 200u					/* OLED 刷新：5Hz（主循环执行） */

/* 单环位置 PID：小球位置(pos_cm_x10) → 脉冲频率(Hz)，每个测距拍推进一次。
 * 方向开关（唯一）：1 = 正相关（pos>0 时 freq>0），0 = 负相关。
 * Target=0 时 Out = Kp×(0−pos)，故正相关对应 Kp 取负号。 */
#define BALL_POS_HZ_SIGN 1
#define BALL_POS_HZ_DIR	 (BALL_POS_HZ_SIGN ? -1.0f : 1.0f)
#define BALL_KP_HZ_PER_CM_X10 4.0f /* Hz / 0.1cm 偏差（pos=30 即 3cm 时 120Hz） */

/* 电机使能：0=不驱动电机（脱机调参），1=正常输出 */
#define MOTOR_CONTROL_ENABLE 1

/* 位置闭环参数不再用宏限制，统一存放在 PID_t 实例 s_ball_pid 的字段里
 * （Kp/Ki/Kd/OutMin/OutMax），上电初始化时赋值，运行期可在线修改。 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ---- 调度标志（TIM1 ISR 置位，主循环消费） ---- */
static volatile uint8_t s_meas_tick	 = 0; /* 测距/滤波/VOFA 20ms */
static volatile uint8_t s_cmd_tick   = 0; /* 串口命令解析 100ms */
static volatile uint8_t s_oled_tick  = 0; /* OLED 刷新 200ms */
/* 主循环心跳：1 = 最近 1s 内主循环至少跑过一圈；ISR 据此决定是否喂狗 */
static volatile uint8_t s_loop_alive = 0;

/* ---- 开环频率覆盖（cmd.c F 命令写，主循环闭环读；方向链钉死实验用） ---- */
static volatile uint8_t s_openloop_en	= 0; /* 1 = 开环覆盖生效，0 = 位置闭环 */
static volatile int32_t s_openloop_freq = 0; /* 开环目标频率（Hz，带符号，已限幅） */
/* ---- 位置闭环 PID 实例（参数存字段，非宏；50Hz 主循环推进） ---- */
static PID_t s_ball_pid;
/* 控制状态（s_ctrl_mode/s_key_estop）与滤波器实例（g_tof_filter）已迁至
 * ctrl.c / tof.c，本文件通过 Ctrl_GetMode()/Ctrl_IsEstop()/g_tof_filter 访问。 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* 控制与命令接口由 ctrl.h / cmd.h 提供（非 static）。 */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 页面文案与取数都在 main 层就地实现（各页仅一处使用，不另设封装）；
 * OLED 模块只提供屏幕原语，每次改动只回写改动区。 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_IWDG_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	/* IWDG 自 MX_IWDG_Init 起已运行，初始化分步喂狗防误复位 */
	HAL_IWDG_Refresh(&hiwdg);

	OLED_Init();
	HAL_IWDG_Refresh(&hiwdg);

	/* 复位诊断：第二行显示上次复位源，P=上电/掉电 I=IWDG N=NRST S=软件 */
	{
		char rst_ch = 'S';

		if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
			rst_ch = 'P';
		} else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
			rst_ch = 'I';
		} else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
			rst_ch = 'N';
		}
		__HAL_RCC_CLEAR_RESET_FLAGS();
		OLED_Print(0, 8, "RST:%c", rst_ch);
		OLED_Flush(0, 8, OLED_WIDTH, 8);
	}
	HAL_IWDG_Refresh(&hiwdg);

	/* VL53L1X 初始化 + 启动连续测量（Short 模式 250mm）。
	 * 内部阻塞 ~数百 ms；失败在应用层显示错误页并持续喂狗停机 */
	{
		int err = Tof_InitSensor(NULL);

		if (err != 0) {
			OLED_Print(0, 16, "VL53 FAIL");
			OLED_Print(0, 24, "step:%s err:%d", g_tof.last_step, err);
			OLED_Flush(0, 16, OLED_WIDTH, 16);
			while (1) {
				HAL_IWDG_Refresh(&hiwdg);
			}
		}
	}
	HAL_IWDG_Refresh(&hiwdg);

	/* 滤波器与零点初始化 */
	TofFilter_Init(&g_tof_filter, TOF_FILTER_ALPHA_DEFAULT_MILLI, TOF_FILTER_ZERO_DEFAULT_MM);
	/* 硬件标定零点：2026-10-29 实测 60s/930帧 mean=131.4mm, σ=1.87mm——
	 * 小球静止时 VL53L1X 原始距离的期望值；Z 命令可在线覆盖 */
	TofFilter_SetZeroMm(&g_tof_filter, 131);

	/* 执行器与编码器初始化：Motor_Init 只校验 TIM2 配置、Encoder_Init 启动编码器，
	 * 两者都不使能电机、不输出脉冲；Encoder_StoreRestore 恢复掉电保存的绝对位置 */
	Motor_Init();
	Encoder_Init();
	(void) Encoder_StoreRestore();

	/* 单环位置 PID 初始化：小球位置(pos_cm_x10) → 脉冲频率(Hz)；
	 * 增益符号由 BALL_POS_HZ_SIGN 统一决定，输出限幅 ±120Hz（实测 400Hz 会失步）。 */
	s_ball_pid.Kp	   = BALL_POS_HZ_DIR * BALL_KP_HZ_PER_CM_X10;
	s_ball_pid.Ki	   = 0.0f;
	s_ball_pid.Kd	   = 0.0f;
	s_ball_pid.OutMin  = -120.0f;
	s_ball_pid.OutMax  = 120.0f;
	s_ball_pid.i_limit = 0.0f; /* 0 = 不限幅积分（Ki=0 时无影响） */
	PID_Reset(&s_ball_pid);

#if MOTOR_CONTROL_ENABLE
	Motor_SetEnabled(true); /* EN 引脚使能：绕组通电，闭环可直接发脉冲 */
#endif
	HAL_IWDG_Refresh(&hiwdg);

	/* 接收上位机命令（A/Z/P/I/D/R）；VOFA 数据流仍只含数值，不混入文本 */
	USART1_StartRx();

	/* SysTick 优先级=1（须高于 TIM1=2）：ISR 内 HAL_GetTick() 走时，
	 * HAL_I2C 的 100ms 超时才能生效，防止 VL53L1X_I2C_Recover 死等 */
	HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);

	/* TIM1 启动（CubeMX 配置 Counter Period=999 → 1ms 基础节拍） */
	if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) {
		Error_Handler();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* 主循环心跳：任何一步卡死超过 1 个 IWDG 周期都会停止喂狗并复位 */
		s_loop_alive = 1;

		/* 测距/滤波/单环位置闭环：同一个 50Hz 拍推进 */
		if (s_meas_tick) {
			s_meas_tick = 0;
			Tof_Tick(&g_tof_filter); /* 测距/滤波每 MEAS_PERIOD_MS */

			if (s_openloop_en) {
				/* F 命令开环：直接下发带符号频率（绕过位置 PID），用于钉死方向链 */
				Motor_SetTargetFreq(s_openloop_freq);
			} else {
				/* 单环位置 PID：Target=0, Actual=小球位置(0.1cm)，Out=脉冲频率(Hz) */
				s_ball_pid.Target = 0.0f;
				s_ball_pid.Actual = (float) g_tof.pos_cm_x10;
				PID_Update(&s_ball_pid);
				Motor_SetTargetFreq((int32_t) s_ball_pid.Out);
			}
			/* VOFA 遥测：ch0=filt_mm / ch1=pulse_freq_hz / ch2=enc_cnt */
			{
				int32_t vofa[3] = {
					(int32_t) g_tof.filt_mm,
					Motor_GetPulseFreqHz(),
					Encoder_Read(),
				};
				VOFA_SendFrame(vofa, 3);
			}
		}

		/* 10Hz 串口命令解析：一次可排空队列中的多条命令 */
		if (s_cmd_tick) {
			s_cmd_tick = 0;
		//	Cmd_Poll(); /* F/F0 开环方向标定、C 编码器归零 */
		}

		/* 5Hz：刷运行时页面 */
		if (s_oled_tick) {
			s_oled_tick = 0;

			/* 运行时页面：原始距离 / 零点校准后位置 / 有效帧占比 / 编码器绝对位置 */
			int16_t pos	  = g_tof.pos_cm_x10;
			char	sign  = (pos < 0) ? '-' : ' ';
			int32_t abs_pos = (pos < 0) ? -(int32_t) pos : (int32_t) pos;
			uint8_t mot_en	= 0u;
#if MOTOR_CONTROL_ENABLE
			mot_en = 1u;
#endif

			OLED_ClearArea(0, 0, OLED_WIDTH, 64);
			OLED_Print(0, 0,  "RAW:%4d mm", (int) g_tof.raw_mm);
			OLED_Print(0, 8,  "POS:%c%ld.%ld cm", sign, (long) (abs_pos / 10), (long) (abs_pos % 10));
			OLED_Print(0, 16, "VAL:%3lu%%", (unsigned long) g_tof.valid_pct);
			OLED_Print(0, 24, "ENC:%6ld", (long) Encoder_ReadAbs());
			OLED_Print(0, 32, "M:%u", (unsigned) mot_en);
			OLED_Flush(0, 0, OLED_WIDTH, 40);
		}

		/* 按键事件消费（非阻塞）：当前无 ctrl 模式管理，可在此处自行处理按键 */
		/* Ctrl_PollKeyEvents(); 已禁用 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* 级联闭环（Ctrl_*）与串口命令（Cmd_*）已迁至 Core/Src/ctrl.c 与 cmd.c。
 * 本文件只保留 HAL 回调（TIM1 分频调度 + TIM3 Z 相捕获）。 */

/**
 * @brief 开环频率覆盖入口（串口 F 命令）：进入/退出开环直驱模式
 * @param freq_hz 带符号目标频率（Hz，直接下发，不做限幅）；0 = 退出回闭环
 * @note  主循环上下文调用（cmd.c），meas 节拍读取；开环下直接把频率写进 s_openloop_freq
 */
void App_SetOpenloopFreq(int32_t freq_hz) {
	s_openloop_freq = freq_hz;
	s_openloop_en	= (freq_hz != 0) ? 1u : 0u;
}

/**
 * @brief TIM1 1ms 节拍：分频驱动内环/测距/命令/OLED 节拍，采样按键并按秒喂狗
 * @param htim 触发回调的定时器（仅处理 TIM1，其它实例立即返回）
 * @note  ISR 上下文：不可阻塞、不可调用持有外设锁的 HAL 接口；控制放 ISR 保证
 *        50Hz，测距/滤波/VOFA/命令/OLED 只置标志由主循环执行；
 *        喂狗条件：tick_ms 回到 0 且主循环心跳（s_loop_alive）正常
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	static uint16_t tick_ms = 0;

	if (htim->Instance != TIM1) {
		return;
	}

	if (++tick_ms >= 1000u) {
		tick_ms = 0;
	}
	/* 控制（PID）已迁至主循环 meas 节拍，ISR 只做测距/命令/OLED 分频与喂狗 */
	if ((tick_ms % MEAS_PERIOD_MS) == 0) {
		s_meas_tick = 1;
	}
	if ((tick_ms % CMD_PERIOD_MS) == 0) {
		s_cmd_tick = 1;
	}
	if ((tick_ms % OLED_PERIOD_MS) == 0) {
		s_oled_tick = 1;
	}

	/* 按键 1ms 采样：只读引脚与计数，耗时常数级，不影响 50Hz 控制 */
	Key_Tick1ms();

	/* 主循环心跳正常才喂狗：主循环卡死超过 1 个周期即触发 IWDG 复位 */
	if (tick_ms == 0) {
		if (s_loop_alive) {
			s_loop_alive = 0;
			HAL_IWDG_Refresh(&hiwdg);
		}
	}
}

/**
 * @brief Z 相（TIM3_CH3 = PB0）捕获回调：ISR 内只登记事件
 * @param htim 触发回调的定时器（仅处理 TIM3）
 * @note  ISR 上下文：O(1) 不可阻塞；对齐与落盘留给 5Hz Ctrl_EncService
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		Encoder_ZIndexEvent();
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	__disable_irq();
	while (1) {}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
