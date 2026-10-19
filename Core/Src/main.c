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
#include "servo.h"		   /* 舵机驱动模块 */
#include "ssd1306.h"	   /* SSD1306 底层绘制（首页/进度条内联于 main） */
#include "ssd1306_fonts.h" /* Font_6x8 / Font_7x10 等字体定义 */
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
#define MEAS_PERIOD_MS 20u	/* 测距+VOFA：每 20ms 一次 → 50Hz */
#define CMD_PERIOD_MS  100u /* 上位机命令解析：每 100ms 一次 → 10Hz（预留） */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
  /* USER CODE BEGIN 2 */
	USART1_StartRx();
	HAL_IWDG_Refresh(&hiwdg); /* IWDG 自 MX_IWDG_Init 起已运行，初始化分步喂狗防误复位 */

	OLED_Init();
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("Hilab PID", Font_6x8, White);
	ssd1306_UpdateScreen();
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

	/* 舵机初始化：启动 50Hz PWM 并转到 90°（TIM2 周期一致性由 Servo_Init 校验） */
	Servo_Init();
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
		/* 测距与 VOFA 输出已在 TIM1 回调内完成，主循环仅维持看门狗 */
		HAL_IWDG_Refresh(&hiwdg);
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
 *  - 每 20ms（50Hz）：阻塞读一帧测距并通过 VOFA 输出（后续 PID 计算同放此分支，dt=20ms）
 *  - 每 100ms（10Hz）：上位机命令解析（预留，直接在此分支内添加即可）
 * 阻塞段内 SysTick 冻结时，传感器挂死时的超时兜底由 IWDG(1.64s) 复位恢复。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance != TIM1) {
		return;
	}

	static uint16_t tick_ms = 0;
	if (++tick_ms >= CMD_PERIOD_MS) {
		tick_ms = 0; /* 100ms 大周期（20 与 100 的最小公倍数） */
	}

	/* 50Hz：测距 + VOFA 输出 */
	if ((tick_ms % MEAS_PERIOD_MS) == 0) {
		int16_t distance_mm;
		int8_t	range_status;
		int32_t vofa[2]; /* ch0=距离(mm)，ch1=状态(0=有效/-1=超时/-2=读失败/其他=测距状态) */

		VL53L1X_Read(&distance_mm, &range_status);
		vofa[0] = distance_mm;
		vofa[1] = range_status;
		VOFA_SendFrame(vofa, 2);
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
