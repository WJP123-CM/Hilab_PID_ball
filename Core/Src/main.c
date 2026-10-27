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
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"		   /* OLED 基础封装（OLED_Init/OLED_Print/OLED_PrintLine） */
#include "print.h"		   /* 双输出通道打印模块（serial_print / oled_print） */
#include "ssd1306.h"	   /* SSD1306 底层绘制（首页/进度条内联于 main） */
#include "ssd1306_fonts.h" /* Font_6x8 / Font_7x10 等字体定义 */
#include "stepper.h"	   /* 步进电机基础驱动（D36A + MS42CG） */
#include "vl53l1x.h"	   /* VL53L1X 测距传感器封装 */

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* TIM1 基础节拍与软件分频：TIM1 计数时钟 = 72MHz/(PSC=71+1) = 1MHz。
 * 基础节拍 1ms 由 CubeMX 的 TIM1 Counter Period=999 决定（须在 .ioc 中设置，
 * 代码中不得覆盖）；各业务逻辑在回调内按下列周期软件分频 */
#define MEAS_PERIOD_MS 20u	/* 测距+VOFA：每 20ms 一次 → 50Hz（OLED 由主循环消费） */
#define CMD_PERIOD_MS  100u /* 上位机命令解析：每 100ms 一次 → 10Hz（预留） */
#define OLED_PERIOD_MS 200u /* OLED 刷新：每 200ms 一次 → 5Hz（历史条每格 200ms） */
#define STATUS_HIST_LEN 10u /* 状态历史格数：10 格 × 200ms = 覆盖最近 2 秒 */
#define IWDG_PERIOD_MS  1000u /* IWDG 喂狗：每 1000ms 一次 → 1Hz（预留） */
#define RANGE_FAIL_RESET_CNT 10u /* 测距连续通信失败阈值：10 次×20ms=200ms 判定断联 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ISR→主循环 共享数据（最新值语义）：OLED 局部回写约 12ms 较慢，放主循环；
 * VOFA 帧短(~1ms@115200)且要求 dt 精确 20ms，故留在 ISR 内直接发送 */
static volatile uint8_t s_oled_need_update	= 0; /* 有新数据待 OLED 刷新 */
static volatile int8_t	s_oled_range_status	= 0; /* 最新一帧状态码（OLED 定性观测用） */
static int8_t			s_status_hist[STATUS_HIST_LEN]; /* 状态历史条（右端最新，左移出） */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_IWDG_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	USART1_StartRx();
	HAL_IWDG_Refresh(&hiwdg); /* IWDG 自 MX_IWDG_Init 起已运行，初始化分步喂狗防误复位 */

	OLED_Init();
  oled_print("HILAP TEST");
	HAL_IWDG_Refresh(&hiwdg);

	/* VL53L1X 初始化：失败时在 OLED 显示错误并喂狗冻结（不再向 USART1 printf，避免污染 VOFA 测距流） */
	{
		VL53L1_Error err = VL53L1X_Init();
		if (err != VL53L1_ERROR_NONE) {
			oled_print("VL53L1X init FAIL");
			oled_print("step=%s err=%d", VL53L1X_LastErrorStep(), (int) err);
			while (1) {
				HAL_IWDG_Refresh(&hiwdg); /* 维持喂狗使错误持续显示在 OLED，不再 IWDG 反复复位重连 */
			}
		}
	}
	HAL_IWDG_Refresh(&hiwdg);

	/* 步进电机基础驱动初始化：校验 TIM2 配置并设定 STEP 占空比；
	 * 上电安全态（不使能、不输出脉冲），转动须显式调用
	 * Stepper_SetEnabled(true) + Stepper_Start()（闭环控制后续开发） */
	Stepper_Init();
	HAL_IWDG_Refresh(&hiwdg);

	/* SysTick 优先级 = 1（须高于 TIM1=2）：使回调内 HAL_GetTick() 仍走时，HAL_I2C 的
	 * 100ms 超时才能生效——否则 ISR 内 SysTick 冻结，一次瞬时 I2C 错误会令
	 * HAL_I2C_Mem_Read 永不返回，平台层总线恢复(VL53L1X_I2C_Recover)得不到执行，
	 * 只能靠 IWDG 反复复位恢复。
	 * 注：该项由 stm32f1xx_hal_conf.h 的 TICK_INT_PRIORITY 决定，CubeMX 界面通常不
	 * 暴露，故暂在代码中设置（本工程唯一保留的优先级代码设置，待确认后处理）。 */
	HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);

	/* TIM1 的全部配置由 CubeMX（.ioc / tim.c）管理，代码中不得覆盖：
	 *   - Counter Period = 999 → 1ms 基础节拍（业务逻辑在回调内软件分频）
	 *   - update 中断优先级 = 2，低于 USART1(0)，使其可抢占回调内长阻塞段避免 RX 溢出
	 * 中断已在 tim.c 的 HAL_TIM_Base_MspInit 中使能，这里只需启动定时器计数。 */
	if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) {
		Error_Handler();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* 测距读帧、VOFA 发送与喂狗都在 TIM1 回调内完成（喂狗唯一入口在 ISR）；
		 * 主循环只负责 OLED 刷新：消费 ISR 置出的标志(最新值语义)，固定行显示 +
		 * 局部回写(1 页 128B)，单次阻塞约 12ms@100kHz，不影响 ISR 内的 50Hz VOFA */
		if (s_oled_need_update) {
			static uint32_t last_refresh_ms = 0;
			uint32_t now_ms = HAL_GetTick();
			if ((now_ms - last_refresh_ms) >= OLED_PERIOD_MS) {
				last_refresh_ms = now_ms;
				s_oled_need_update = 0;
				/* 状态历史条：每个刷新周期采样一次当前状态码，新值右移入、
				 * 旧值左移出，右端为最新；每格固定 2 字符宽 */
				for (uint8_t i = 1U; i < STATUS_HIST_LEN; i++) {
					s_status_hist[i - 1U] = s_status_hist[i];
				}
				s_status_hist[STATUS_HIST_LEN - 1U] = s_oled_range_status;

				/* 拼 2 字符/格的定宽文本：负数 "-1"、个位 " 0"、两位数 "10" */
				char line[STATUS_HIST_LEN * 2U + 1U];
				for (uint8_t i = 0; i < STATUS_HIST_LEN; i++) {
					int8_t st = s_status_hist[i];
					if (st < 0) { /* -1/-2 等驱动错误码 */
						line[2U * i]		= '-';
						line[2U * i + 1U]	= (char) ('0' - st);
					} else if (st < 10) {
						line[2U * i]		= ' ';
						line[2U * i + 1U]	= (char) ('0' + st);
					} else { /* 10~14 等两位数 */
						line[2U * i]		= (char) ('0' + st / 10);
						line[2U * i + 1U]	= (char) ('0' + st % 10);
					}
				}
				line[STATUS_HIST_LEN * 2U] = '\0';

				/* 固定行 1 整条重绘（行 0 的 HILAP TEST 保留）：先清行再写入，
				 * 只回写该行 1 页显存，不整屏刷 1KB */
				ssd1306_FillRectangle(0, 8, SSD1306_WIDTH - 1, 15, Black);
				OLED_Print(0, 8, "%s", line);
				ssd1306_UpdateRect(0, 8, SSD1306_WIDTH, 8);
			}
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* TIM1 基础节拍中断回调（1ms）：内部按逻辑软件分频。
 *  - 每 20ms（50Hz）：阻塞读一帧测距并经 VOFA 输出（帧短 ~1ms@115200，dt 精确；
 *    后续 PID 计算同放此分支）；OLED 刷新较慢，置标志交主循环处理
 *  - 喂狗：仅当测距通信健康时刷新 IWDG——连续 RANGE_FAIL_RESET_CNT 次(200ms)
 *    读帧失败判定测距模块断联 → 停喂 → IWDG(≈1.64s) 复位重新初始化；
 *    因 tick_ms 在 100 处回绕，实际每 100ms 判一次
 *  - 每 100ms（tick_ms==0，10Hz）：上位机命令解析（预留，直接在此分支内添加即可） */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance != TIM1) {
		return;
	}

	static uint16_t tick_ms = 0;
	if (++tick_ms >= CMD_PERIOD_MS) {
		tick_ms = 0; /* 100ms 大周期（20 与 100 的最小公倍数） */
	}

	static uint8_t range_fail_cnt = 0; /* 测距连续通信失败计数 */

	/* 50Hz：测距读帧 + VOFA 输出（帧短 ~1ms，dt 精确 20ms；OLED 较慢，交主循环） */
	if ((tick_ms % MEAS_PERIOD_MS) == 0) {
		int16_t distance_mm;
		int8_t	range_status;
		int32_t vofa[2]; /* ch0=距离(mm)，ch1=状态(0=有效/-1=超时/-2=读失败/其他=测距状态) */
		VL53L1_Error st = VL53L1X_Read(&distance_mm, &range_status);

		/* 通信健康监控：-1/-2 为通信层失败（状态码 5/7 等光学失败不算断联）；
		 * 连续 RANGE_FAIL_RESET_CNT 次失败 → 停止喂狗 → IWDG 复位重新初始化 */
		if (st == VL53L1_ERROR_NONE) {
			range_fail_cnt = 0;
		} else if (range_fail_cnt < RANGE_FAIL_RESET_CNT) {
			range_fail_cnt++;
		}

		vofa[0] = distance_mm;
		vofa[1] = range_status;
		VOFA_SendFrame(vofa, 2);
		s_oled_range_status	= range_status;
		s_oled_need_update	= 1u;
	}
  /* 喂狗（仅测距通信健康时）：tick_ms 在 100 处回绕，实际每 100ms 判一次。
   * 断联（连续失败超阈值）后停喂 → IWDG(≈1.64s) 复位，上电重新初始化测距模块 */
 	if ((tick_ms % IWDG_PERIOD_MS == 0) && (range_fail_cnt < RANGE_FAIL_RESET_CNT)) {
    HAL_IWDG_Refresh(&hiwdg);
  }
		/* 10Hz：上位机命令解析（预留，想加时把下面注释换成函数调用即可） */
	if (tick_ms == 0) {
		/* Command_Poll(); */
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
