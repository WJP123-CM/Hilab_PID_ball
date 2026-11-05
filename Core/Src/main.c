/* USER CODE BEGIN Header */
/**C:/msys64/msys2_shell.cmd -defterm -no-start -use-full-path -here -ucrt64 -c "cmake --build build/Debug"
C:/msys64/msys2_shell.cmd -defterm -no-start -use-full-path -here -ucrt64 -c "openocd -f openocd_stlink.cfg -c 'program
build/Debug/Hilab_PID.elf verify reset exit'"
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
#include "cmd.h"	 /* 上位机行协议：A/Z/P/I/D/R */
#include "encoder.h" /* 编码器计数/Z 校准/掉电保存 */
#include "key.h"	 /* PA0 按键消抖与事件队列 */
#include "oled.h"	 /* OLED 屏幕原语 */
#include "pid.h"	 /* 位置式 PID 控制器（标准命名：Target/Actual/Out/Kp/Ki/Kd） */
#include "tof.h"	 /* 测距 → 滤波 → 有效帧占比（含 g_tof_filter） */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>	/* snprintf */
#include <stdlib.h> /* strtol */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 调度周期：内环=外环=20ms（50Hz，同拍） */
#define Inner_PERIOD_MS	   20u					 /* 内环：编码器定速 PID，50Hz */
#define Outter_PERIOD_MS   (Inner_PERIOD_MS * 5) /* 外环：位置→速度目标，100ms */
#define CMD_PERIOD_MS  100u					 /* 串口命令：10Hz（主循环执行） */
#define OLED_PERIOD_MS 200u					 /* OLED 刷新：5Hz（主循环执行） */
#define VOFA_PERIOD_MS	 Inner_PERIOD_MS	   /* VOFA 20ms 1次 */
#define ENC_TICK_PERIOD_MS 10					 /* 编码器 20ms 1次 */
#define TOF_TICK_PERIOD_MS Inner_PERIOD_MS		 /* TOF 20ms 1次 */

/* 单环位置 PID：小球位置(pos_cm_x10) → 脉冲频率(Hz)，每个测距拍推进一次。
 * 方向开关（唯一）：1 = 正相关（pos>0 时 freq>0），0 = 负相关。
 * Target=0 时 Out = Kp×(0−pos)，故正相关对应 Kp 取负号。 */
#define BALL_POS_HZ_SIGN	  1
#define BALL_POS_HZ_DIR		  (BALL_POS_HZ_SIGN ? -1.0f : 1.0f) // 正相关（pos>0 时 freq>0）
#define BALL_KP_HZ_PER_CM_X10 4.0f								/* Hz / 0.1cm 偏差（pos=30 即 3cm 时 120Hz） */
#define MOTOR_CONTROL_ENABLE  1
#define ZERO_THETA			  660
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ---- 串口 RX 解析结果（文件级，主循环读写） ---- */
#define RX_VAL_MAX 5u						/* 一行最多解析 5 个逗号分隔数值：Kp,Ki,Kd,brake_gain,brake_min */
static float			rx_val[RX_VAL_MAX]; /* 解析出的数值（支持小数） */
static uint8_t			rx_cnt	 = 0;		/* 本行实际解析出的个数（0~RX_VAL_MAX） */
static volatile uint8_t rx_valid = 0;		/* 1 = 收到新参数行，等主循环接管写入一次 */

/* ---- 调度标志（TIM1 ISR 置位，主循环消费） ---- */
static volatile uint8_t inner_tick	= 0; /* 内环：编码器 PID + TOF 20ms */
static volatile uint8_t outter_tick = 0; /* 外环：位置 PID → 速度目标 100ms */
static volatile uint8_t cmd_tick	= 0; /* 串口命令解析 100ms */
static volatile uint8_t oled_tick	= 0; /* OLED 刷新 200ms */
static volatile uint8_t vofa_tick	= 0; /* VOFA 20ms 1次 */
static volatile uint32_t tof_tick	 = 0;
static volatile uint32_t enc_tick	 = 0;
/* 主循环心跳：1 = 最近 1s 内主循环至少跑过一圈；ISR 据此决定是否喂狗 */
static volatile uint8_t loop_alive = 0;
bool					runstate   = 1; /* 默认上电即运行，按键单击翻转为停止 */
/* ---- 位置外环 PID（ToF 反馈 → 速度目标 counts/s） ---- */
PID_t ball_pid = {
	.Kp			= 5.0f,
	.Ki			= 0.0f,
	.Kd			= 40.0f,
	.OutMin		= -200.0f,
	.OutMax		= 200.0f,
	.i_limit	= 0.0f,
	.Target		= 0.0f,
	.brake_gain = 4.0f,
	.brake_min	= 0.5f,
	.Out		= 200.0f,
};
/* ---- 速度内环 PID（编码器反馈 → 步进频率 Hz） ---- */
PID_t encoder_pid = {
	.Kp		 = 11.0f,
	.Ki		 = 0.0f,
	.Kd		 = 2.0f,
	.OutMin	 = -10000.0f, /* 频率限幅 ±300Hz（D36A 启动频率内，防丢步） */
	.OutMax	 = 10000.0f,
	.i_limit = 10000.0f,
	.Target	 = 0.0f,
};
/* 滤波器实例 g_tof_filter 定义于 tof.c，本文件通过 tof.h 访问。 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* 命令接口由 cmd.h 提供（非 static）。 */
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
	  __HAL_RCC_CLEAR_RESET_FLAGS(); // 清除复位标志位，避免重复显示
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
		  HAL_IWDG_Refresh(&hiwdg);
		  HAL_Delay(3000);
	  }
  }
  HAL_IWDG_Refresh(&hiwdg);

  /* 滤波器与零点初始化 */
  {
	  TofFilter_Init(&g_tof_filter, TOF_FILTER_ALPHA_DEFAULT_MILLI, TOF_FILTER_ZERO_DEFAULT_MM);
	  /* 硬件标定零点：2026-10-29 实测 60s/930帧 mean=131.4mm, σ=1.87mm——
	   * 小球静止时 VL53L1X 原始距离的期望值；Z 命令可在线覆盖 */
	  TofFilter_SetZeroMm(&g_tof_filter, 131);
  }
  /* 执行器与编码器启动：两者都必须在 MX_TIM2_Init/MX_TIM3_Init 之后调用 */
  Motor_Init();	  /* 校验 TIM2 分频、设定 STEP 占空比、对齐 DIR 初态 */
  Encoder_Init(); /* 启动 TIM3 编码器接口，否则 CEN 不置位、CNT 恒 0 */

  /* 串级双 PID 初始化：


  /* 接收上位机命令（A/Z/P/I/D/R）；VOFA 数据流仍只含数值，不混入文本 */
  USART1_StartRx();

  /* SysTick 优先级=1（须高于 TIM1=2）：ISR 内 HAL_GetTick() 走时，
   * HAL_I2C 的 100ms 超时才能生效，防止 VL53L1X_I2C_Recover 死等 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);

  /* TIM1 启动（CubeMX 配置 Counter Period=999 → 1ms 基础节拍） */
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) {
	  Error_Handler();
  }

#if MOTOR_CONTROL_ENABLE
  Motor_SetEnabled(true); /* EN 引脚使能：绕组通电，闭环可直接发脉冲 */
#endif
  HAL_IWDG_Refresh(&hiwdg);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	  /* USER CODE END WHILE */

	  /* 串口参数接管：仅在收到新参数行时写入一次，禁止每圈无条件覆盖 */
	  if (rx_valid) {
		  rx_valid = 0;
		  if (rx_cnt > 0) ball_pid.Kp = rx_val[0];
		  if (rx_cnt > 1) ball_pid.Ki = rx_val[1];
		  if (rx_cnt > 2) ball_pid.Kd = rx_val[2];
		  if (rx_cnt > 3) ball_pid.brake_gain = rx_val[3];
		  if (rx_cnt > 4) ball_pid.brake_min = rx_val[4];
	  }
	  /* USER CODE BEGIN 3 */
	  {
		  KeyEvent_t evt;
		  while ((evt = Key_PollEvent()) != KEY_EVT_NONE) {
			  if (evt == KEY_EVT_CLICK) {
				  runstate = !runstate;
			  }
		  }
	  }
	  /* 主循环心跳：IWDG 据此判断是否卡死，任何一步超过 ≈1s 没转圈都会触发复位 */
	  loop_alive = 1;

	  /* 外环（100ms）：位置 PID → 速度目标 */

	  /* 编码器主循环节拍：内部按 20ms 时间门控，更新 g_enc 快照 */
	  if (enc_tick) {
		  enc_tick = 0;
		  Encoder_Tick();
	  }
	  if (tof_tick) {
		  tof_tick = 0;
		  Tof_Tick(&g_tof_filter);
	  }

	  /* 内环（20ms）：编码器速度反馈 + 速度 PID → 电机频率 */

	  /* VOFA 遥测 9 通道（调试用）：
	   * ch6=Kd×100  ch7=Error0×100  ch8=Error1×100 */
	  if (vofa_tick) {
		  vofa_tick		  = 0;
		  int32_t vofa[9] = {(int32_t) encoder_pid.Target,		  (int32_t) encoder_pid.Actual,
							 (int32_t) encoder_pid.Out,			  (int32_t) ball_pid.Target,
							 (int32_t) ball_pid.Actual,			  (int32_t) ball_pid.Out,
							 (int32_t) (ball_pid.Kd * 100.0f),	  (int32_t) (ball_pid.Error0 * 100.0f),
							 (int32_t) (ball_pid.Error1 * 100.0f)};
		  VOFA_SendFrame(vofa, 9);
	  }

	  /* 100ms：读最新 RX 行、按逗号分隔解析整数 */
	  if (cmd_tick) {
		  cmd_tick = 0;
		  if (uart1_last_ready) {
			  const char *p = (const char *) uart1_last_line;
			  char		 *end;

			  rx_cnt = 0;
			  for (uint8_t i = 0; i < RX_VAL_MAX; i++) {
				  while (*p == ' ' || *p == '\t')
					  p++;
				  if (*p == '\0') {
					  break;
				  }
				  float v = strtof(p, &end);
				  if (end == p) {
					  break;
				  }
				  rx_val[rx_cnt++] = v;
				  p				   = end;
				  if (*p == ',') {
					  p++;
					  continue;
				  }
				  break;
			  }
			  uart1_last_ready = 0;
			  if (rx_cnt > 0) {
				  rx_valid = 1;
			  } /* 交给主循环写入一次 */
		  }
	  }

	  /* 5Hz：刷运行时页面 */
	  if (oled_tick) {
		  oled_tick = 0;

		  int16_t pos	  = g_tof.pos_cm_x10;
		  char	  sign	  = (pos < 0) ? '-' : ' ';
		  int32_t abs_pos = (pos < 0) ? -(int32_t) pos : (int32_t) pos;
		  uint8_t mot_en  = 0u;
		  char	  rx_str[40];
		  char	 *p = rx_str;
#if MOTOR_CONTROL_ENABLE
			mot_en = 1u;
#endif

			for (uint8_t i = 0; i < rx_cnt && i < RX_VAL_MAX; i++) {
				if (i) *p++ = ',';
				p += snprintf(p, sizeof(rx_str) - (size_t) (p - rx_str), "%d", (int) rx_val[i]);
			}
			*p = '\0';

			OLED_ClearArea(0, 0, OLED_WIDTH, 64);
			OLED_Print(0, 0,  "RAW:%4d mm", (int) g_tof.raw_mm);
			OLED_Print(0, 8,  "POS:%c%ld.%ld cm", sign, (long) (abs_pos / 10), (long) (abs_pos % 10));
			OLED_Print(0, 16, "VAL:%3lu%%", (unsigned long) g_tof.valid_pct);
			OLED_Print(0, 24, "ENC:%6ld", (long) Encoder_Read());
			OLED_Print(0, 32, "M:%u R:%u", (unsigned) mot_en, (unsigned) runstate);
			OLED_Print(0, 48, "RX:%d %s", (int) rx_cnt, rx_str);
			OLED_Flush(0, 0, OLED_WIDTH, 64);
	  }

	  /* 按键事件消费（非阻塞） */
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
/**
 * @brief TIM1 1ms 节拍：分频驱动内环/外环/VOFA/命令/OLED 节拍，采样按键并按秒喂狗
 * @param htim 触发回调的定时器（仅处理 TIM1，其它实例立即返回）
 * @note  ISR 上下文：不可阻塞、不可调用持有外设锁的 HAL 接口；PID/测距在主循环执行
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	static uint16_t tick_ms = 0;
	static uint8_t	ctrl_tick = 0;

	if (htim->Instance != TIM1) {
		return;
	}

	if (++tick_ms >= 1000u) {
		tick_ms = 0;
		if (++ctrl_tick >= 1) {
			ctrl_tick = 0;
			// ball_pid.Out = -ball_pid.Out;
		}
	}
	if (tick_ms % Outter_PERIOD_MS == 0) {
		ball_pid.Target = 0.0f;
		ball_pid.Actual = -(float) g_tof.pos_cm_x10;
		PID_Update(&ball_pid);
		encoder_pid.Target = ZERO_THETA + ball_pid.Out; /* counts/s，与编码器速度反馈同量纲 */
	}
	if ((tick_ms % Inner_PERIOD_MS) == 0) {
		inner_tick		   = 1;
		encoder_pid.Actual = g_enc.total; /* counts/s */
		PID_Update(&encoder_pid);
		if (runstate) {
			Motor_SetTargetFreq((int32_t) encoder_pid.Out);
		} else {
			Motor_SetTargetFreq(0);
		}
	}
	if ((tick_ms % ENC_TICK_PERIOD_MS) == 0) {
		enc_tick = 1;
	}
	if ((tick_ms % TOF_TICK_PERIOD_MS) == 0) {
		tof_tick = 1;
	}
	if ((tick_ms % CMD_PERIOD_MS) == 0) {
		cmd_tick = 1;
	}
	if ((tick_ms % OLED_PERIOD_MS) == 0) {
		oled_tick = 1;
	}
	if ((tick_ms % VOFA_PERIOD_MS) == 0) {
		vofa_tick = 1;
	}

	/* 按键 1ms 采样：只读引脚与计数，耗时常数级，不影响 50Hz 控制 */
	Key_Tick1ms();

	/* 主循环心跳正常才喂狗：主循环卡死超过 1 个周期即触发 IWDG 复位 */
	if (tick_ms == 0) {
		if (loop_alive) {
			loop_alive = 0;
			HAL_IWDG_Refresh(&hiwdg);
		}
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
