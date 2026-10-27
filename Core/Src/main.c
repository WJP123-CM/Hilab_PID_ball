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
#include "pid.h"		   /* 位置式 PID（位置环控制器） */
#include "print.h"		   /* 双输出通道打印模块（serial_print / oled_print） */
#include "ssd1306.h"	   /* SSD1306 底层绘制（首页/进度条内联于 main） */
#include "ssd1306_fonts.h" /* Font_6x8 / Font_7x10 等字体定义 */
#include "stepper.h"	   /* 步进电机基础驱动（D36A + MS42CG） */
#include "vl53l1x.h"	   /* VL53L1X 测距传感器封装 */

#include <stdio.h>
#include <stdlib.h> /* strtol（PID 整定命令解析） */

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
#define OLED_PERIOD_MS 200u /* OLED 刷新：每 200ms 一次 → 5Hz */
#define IWDG_PERIOD_MS  1000u /* IWDG 喂狗：每 1000ms 一次 → 1Hz（预留） */
#define RANGE_FAIL_RESET_CNT 10u /* 测距连续通信失败阈值：10 次×20ms=200ms 判定断联 */

/* ---- 位置环整定参数：目标在 0 与半圈间阶跃，VOFA 发 目标/反馈/频率 ---- */
#define CL_STEP_PERIOD_MS 4000u /* 阶跃周期 4s */
#define CL_STEP_CNT		 2000	/* 阶跃幅度：半圈 = 编码器 4000 计数/2 */
/* ---- 位置环 PID 起步值（Hz/计数量纲），经串口数据迭代整定 ---- */
#define CL_PID_KP	0.90f /* 2026-10-27 阶跃整定：双向超调3.3%、上升2.2s */
#define CL_PID_KI	0.16f /* 消稳态残差；过大→超调缓慢增大 */
#define CL_PID_KD	0.35f /* 压振铃；再增收益递减（机械间隙主导） */
#define CL_PID_DFILT 0.15f	 /* 微分低通系数，压阶跃瞬间 D 项冲击 */
#define CL_PID_ILIM	 200.0f	 /* 积分限幅（Hz） */

/* ---- 速度环整定参数：目标速度方波 ±SPD_TEST_HZ，VOFA 通道布局与位置环一致 ----
 * 反馈量纲：编码器 20ms 差分 × SPD_FB_SCALE = 等效脉冲频率(Hz)
 * （50Hz采样 × 3200脉冲/圈 ÷ 4000计数/圈 = ×40），与指令同量纲 */
#define LOOP_UNDER_TEST	 1 /* 0=位置环(已整定冻结) 1=速度环(当前) */
#define SPD_TEST_PERIOD_MS 4000u /* 方波半周期 2s */
#define SPD_TEST_HZ	 1000 /* 目标速度幅值 ≈18.75rpm */
#define SPD_FB_SCALE	 40   /* 计数/20ms → 等效脉冲Hz 换算 */
/* 起步值（Hz/Hz 量纲），经串口数据迭代整定 */
#define SPD_PID_KP	0.80f
#define SPD_PID_KI	0.02f
#define SPD_PID_KD	0.05f
#define SPD_PID_DFILT 0.10f /* 反馈量化 ±40Hz，D 项必须重滤波 */
#define SPD_PID_ILIM	400.0f /* 积分限幅（Hz） */

/* ---- PID 串口在线整定（行协议，10Hz 解析，实现在 USER CODE 4 PidTuner_*） ----
 * 命令以 \n 或 \r 结尾，参数一律毫单位（实际值×1000）整数，避免浮点解析：
 *   P<milli> / I<milli> / D<milli> : 设当前通道 Kp/Ki/Kd（"P 350"与"P350"均可）
 *   C<id>  : 切换整定通道          R : 复位当前通道 PID 运行态（积分/微分清零）
 *   Z      : 编码器累计位置清零（对位基准）+ 当前通道 PID 运行态复位
 * 通道表：0=电机位置环 1=小球控制(预留) 2=测距控制(预留) 3=电机速度环
 * 非法行静默丢弃（EMI 噪声鲁棒）；参数回显经 VOFA ch3~ch5 数值通道承载，
 * USART1 数据流上不出现任何文本，保证 FireWater 帧解析纯净 */
#define PID_TUNE_CH_NUM	  4u
/* 当前环路的整定通道下标（通道表见上；LOOP_UNDER_TEST 是环路选择，两者不同轴） */
#if LOOP_UNDER_TEST == 0
#define ACTIVE_TUNE_CH 0 /* 位置环 */
#else
#define ACTIVE_TUNE_CH 3 /* 速度环 */
#endif
#define PID_TUNE_MILLI_MAX	  100000L	  /* |增益| ≤ 100.000 */
#define PID_TUNE_MILLI_SCALE  1000.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 位置环状态（ISR 独占写，OLED 主循环只读，最新值语义，须 volatile） */
static PID_t			s_pos_pid;					/* 位置环控制器（误差计数 → 带符号频率 Hz） */
static volatile int32_t s_target_cnt	= 0;		/* 当前目标位置（编码器计数） */
static volatile int32_t s_feedback_cnt	= 0;		/* 最新反馈位置（编码器计数） */
static volatile int32_t s_freq_cmd		= 0;		/* 带符号频率指令(Hz)，符号=方向 */

/* ---- PID 多通道整定框架：电机位置/速度环现役，小球/测距为预留扩展通道 ----
 * 预留通道上电即初始化（限幅占位同电机环，实际输出量纲待执行器接入时定义），
 * 可立刻经串口 C1/C2 预整定参数；切换通道不影响运行中的环 */
static PID_t		s_ball_pid;									  /* 小球控制环（预留） */
static PID_t		s_dist_pid;									  /* 测距控制环（预留） */
static PID_t		s_spd_pid;									   /* 电机速度环 */
static PID_t *const s_tune_ch[PID_TUNE_CH_NUM] = {&s_pos_pid, &s_ball_pid, &s_dist_pid, &s_spd_pid};
static volatile uint8_t s_tune_active = ACTIVE_TUNE_CH; /* 当前整定通道（串口 C 命令切换） */

/* 速度环状态（ISR 独占写，OLED/VOFA 只读）：方波目标 + 编码器差分实测 */
static volatile int32_t s_spd_target_hz = 0;  /* 目标速度（等效脉冲频率 Hz，带符号） */
static volatile int32_t s_spd_fb_hz		= 0;  /* 实测速度（编码器 20ms 差分换算） */
static int32_t		s_spd_last_enc = 0;		  /* 上一拍编码器累计值（差分基准） */
/* 串口链路诊断计数（VOFA ch7~ch10）：定位命令丢失发生在哪一级 */
static volatile uint32_t s_getl_cnt  = 0; /* GetLine 成功取行次数 */
static volatile uint32_t s_parse_cnt = 0; /* ParseLine 入口次数 */
static volatile char	s_last_cmd  = 0;  /* 最近一次解析的命令字母 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint32_t Stepper_SelfTest(void); /* 步进电机核心逻辑上电自检（USER CODE 4 实现） */
static void PidTuner_Poll(void);		/* 上位机命令解析（10Hz，USER CODE 4 实现） */
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
	/* USART1 RX 中断接收（PID 串口整定需要收上位机命令）：恢复启用。
	 * 电机 EMI 噪声可能注入坏字节——行协议解析对坏行静默丢弃（见 PidTuner）；
	 * 若出现 RX 中断风暴致复位循环（RST:I），应断开 PA10 再排查 RX 线抗干扰 */
	USART1_StartRx();
	HAL_IWDG_Refresh(&hiwdg); /* IWDG 自 MX_IWDG_Init 起已运行，初始化分步喂狗防误复位 */

	OLED_Init();
  oled_print("HILAP TEST");
	HAL_IWDG_Refresh(&hiwdg);

	/* 复位诊断（先读后清）：OLED 行 3 显示上次复位源，定位周期性复位——
	 * P=上电/掉电复位（电机电流冲击致电源跌落的特征） I=IWDG 看门狗超时
	 * N=NRST 引脚 S=软件复位。烧录后首次上电显示 P 属正常；
	 * 运行中周期性重启后若仍为 P → 电源跌落；变为 I → 看门狗（停喂狗/卡死） */
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
		OLED_Print(0, 24, "RST:%c", rst_ch);
		ssd1306_UpdateRect(0, 24, SSD1306_WIDTH, 8);
	}
	HAL_IWDG_Refresh(&hiwdg);

	/* 步进电机核心逻辑上电自检（StepperSm 纯决策，无硬件动作）：
	 * 失败 → OLED 显示失败数并冻结（持续喂狗）；通过则继续启动流程 */
	{
		uint32_t fails = Stepper_SelfTest();
		if (fails != 0u) {
			oled_print("SM TEST FAIL %lu", (unsigned long) fails);
			while (1) {
				HAL_IWDG_Refresh(&hiwdg);
			}
		}
	}
	HAL_IWDG_Refresh(&hiwdg);

	/* VL53L1X 初始化：速度测试期间禁用——电机 EMI 曾致 I2C1 总线锁死
	 * （VL53 侧 SDA 拉死），本初始化内部阻塞会超过 IWDG 周期(≈1.64s)
	 * → RST:I 复位循环，永远进不了主循环。恢复测距业务时取消注释，
	 * 并须先解决 I2C1 布线抗干扰 + 总线锁死恢复（SCL 打 9 脉冲）问题 */
#if 0
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
#endif
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

	/* ==================== 位置环 PID 整定 ====================
	 * 闭环在 TIM1 回调 20ms 分支内运行（50Hz）：目标每 4s 在 0 与半圈
	 * (2000 计数)间阶跃，位置误差 → PID → 带符号频率 → SetTargetFreq
	 * （内部斜率+换向安全）。VOFA 发 ch0=目标/ch1=反馈/ch2=频率指令，
	 * 串口抓数迭代整定 Kp/Ki/Kd。喂狗在回调内自动进行。
	 * 若电机持续加速跑飞 → 编码器方向与 DIR 不一致，对调 A/B 相接线 */
	PID_Init(&s_pos_pid, CL_PID_KP, CL_PID_KI, CL_PID_KD,
			 -(float) STEPPER_FREQ_MAX_HZ, (float) STEPPER_FREQ_MAX_HZ, CL_PID_ILIM);
	PID_SetDerivativeFilter(&s_pos_pid, CL_PID_DFILT);
	/* 预留整定通道：小球/测距环输出量纲未定，先占位同电机环限幅；
	 * 执行器接入后只需替换此处限幅与 50Hz 分支中的 Update 调用 */
	PID_Init(&s_ball_pid, 0.0f, 0.0f, 0.0f,
			 -(float) STEPPER_FREQ_MAX_HZ, (float) STEPPER_FREQ_MAX_HZ, CL_PID_ILIM);
	PID_Init(&s_dist_pid, 0.0f, 0.0f, 0.0f,
			 -(float) STEPPER_FREQ_MAX_HZ, (float) STEPPER_FREQ_MAX_HZ, CL_PID_ILIM);
	/* 速度环：误差(等效Hz) → 输出频率指令(Hz)，符号即方向 */
	PID_Init(&s_spd_pid, SPD_PID_KP, SPD_PID_KI, SPD_PID_KD,
			 -(float) STEPPER_FREQ_MAX_HZ, (float) STEPPER_FREQ_MAX_HZ, SPD_PID_ILIM);
	PID_SetDerivativeFilter(&s_spd_pid, SPD_PID_DFILT);
	Stepper_SetEnabled(true);	 /* D36A 高电平使能，绕组上电 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* 整定显示：行1=通道/目标/反馈，行2=当前通道 Kp/Ki/Kd（毫单位）；
		 * 200ms 刷新 + 局部回写（2 页显存，仅主循环可见，不影响 ISR） */
		{
			static uint32_t last_refresh_ms = 0;
			uint32_t		now_ms			= HAL_GetTick();
			if ((now_ms - last_refresh_ms) >= OLED_PERIOD_MS) {
				PID_t *tp = s_tune_ch[s_tune_active];
				last_refresh_ms = now_ms;
				ssd1306_FillRectangle(0, 8, SSD1306_WIDTH - 1, 23, Black);
#if LOOP_UNDER_TEST == 0
				OLED_Print(0, 8, "C%d T%d F%d",
						   (int) s_tune_active, (int) s_target_cnt, (int) s_feedback_cnt);
#else
				OLED_Print(0, 8, "C%d T%d F%d",
						   (int) s_tune_active, (int) s_spd_target_hz, (int) s_spd_fb_hz);
#endif
				OLED_Print(0, 16, "P%d I%d D%d",
						   (int) (tp->kp * PID_TUNE_MILLI_SCALE),
						   (int) (tp->ki * PID_TUNE_MILLI_SCALE),
						   (int) (tp->kd * PID_TUNE_MILLI_SCALE));
				ssd1306_UpdateRect(0, 8, SSD1306_WIDTH, 16);
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

/* ---- 步进电机核心逻辑上电自检（StepperSm 纯软件决策，无硬件动作，微秒级） ----
 * 覆盖：正频率上升/下降、负频率切换（换向拍必静默且恰好一次）、极限值
 * （超 MAX 钳位 / 死区 / 目标 0 斜停拍数）；全程校验不变式：每拍 |Δf| ≤ SLEW。
 * 返回失败数（0 = 全部通过） */
static uint32_t Stepper_SelfTest(void) {
	uint32_t	fails = 0;
	StepperSm_t sm;
	StepperCmd_t c;
	int32_t		d;
	int			dc_cnt;

	/* 正频上升 0→+1000：每拍 |Δ|≤SLEW，最终到顶、无换向 */
	StepperSm_Init(&sm);
	dc_cnt = 0;
	for (int i = 0; i < 30; i++) {
		int32_t prev = sm.cur_freq;
		c	 = StepperSm_Update(&sm, 1000);
		d	 = c.out_freq - prev;
		if (d > STEPPER_SLEW_HZ_PER_STEP || d < -STEPPER_SLEW_HZ_PER_STEP) fails++;
		if (c.dir_changed) dc_cnt++;
	}
	if (c.out_freq != 1000 || !c.pulse_en || dc_cnt != 0) fails++;

	/* 正频下降 +1000→+300：到 300、无换向 */
	dc_cnt = 0;
	for (int i = 0; i < 30; i++) {
		int32_t prev = sm.cur_freq;
		c	 = StepperSm_Update(&sm, 300);
		d	 = c.out_freq - prev;
		if (d > STEPPER_SLEW_HZ_PER_STEP || d < -STEPPER_SLEW_HZ_PER_STEP) fails++;
		if (c.dir_changed) dc_cnt++;
	}
	if (c.out_freq != 300 || !c.pulse_en || dc_cnt != 0) fails++;

	/* 负频率切换 +500→-800：恰好一次换向、换向拍静默、负输出时 DIR 已切 */
	for (int i = 0; i < 20; i++) StepperSm_Update(&sm, 500);
	dc_cnt = 0;
	for (int i = 0; i < 40; i++) {
		int32_t prev = sm.cur_freq;
		c	 = StepperSm_Update(&sm, -800);
		d	 = c.out_freq - prev;
		if (d > STEPPER_SLEW_HZ_PER_STEP || d < -STEPPER_SLEW_HZ_PER_STEP) fails++;
		if (c.dir_changed) { dc_cnt++; if (c.pulse_en || c.out_freq != 0) fails++; }
		if (c.out_freq < 0 && sm.dir_cw) fails++;
	}
	if (c.out_freq != -800 || c.dir_cw || dc_cnt != 1) fails++;

	/* 极限：+99999 钳到 MAX；|f|<MIN 死区为 0；目标 0 斜停恰 1000/SLEW 拍 */
	StepperSm_Init(&sm);
	for (int i = 0; i < 100; i++) c = StepperSm_Update(&sm, 99999);
	if (c.out_freq != STEPPER_FREQ_MAX_HZ) fails++;
	StepperSm_Init(&sm);
	c = StepperSm_Update(&sm, STEPPER_FREQ_MIN_HZ - 1);
	if (c.out_freq != 0 || c.pulse_en) fails++;
	for (int i = 0; i < 30; i++) StepperSm_Update(&sm, 1000);
	{
		int steps = 0;
		do { c = StepperSm_Update(&sm, 0); steps++; } while (c.out_freq != 0 && steps < 100);
		if (c.out_freq != 0 || steps != 1000 / STEPPER_SLEW_HZ_PER_STEP) fails++;
	}
	return fails;
}

/* ---- PID 串口在线整定（PidTuner）----
 * 协议见 USER CODE PD 注释。解析规则：
 *  - 仅接受 P/I/D/C/R 六个命令字母（大小写均可），其余字符开头的行静默丢弃
 *  - 数值用 strtol 整数解析（毫单位），缺数字/越界行整体丢弃，不部分生效
 *  - 在 TIM1 回调 10Hz 分支调用（ISR 上下文）：短小无阻塞；RX 中断(优先级 0)
 *    仅写行缓冲，与本函数无共享可写状态，无竞态 */
static void PidTuner_ParseLine(const char *line) {
	char  cmd;
	char *end;
	long  val;

	s_parse_cnt++;
	while (*line == ' ' || *line == '\t') {
		line++; /* 容忍命令字母前空白 */
	}
	cmd = *line;
	if (cmd == '\0') {
		return;
	}
	s_last_cmd = cmd;

	switch (cmd) {
	case 'C':
	case 'c': /* 切换整定通道：id 越界静默丢弃 */
		val = strtol(line + 1, &end, 10);
		if (end != line + 1 && val >= 0 && val < (long) PID_TUNE_CH_NUM) {
			s_tune_active = (uint8_t) val;
		}
		return;
	case 'R':
	case 'r': /* 复位当前通道运行态（积分/微分清零，参数保留） */
		PID_Reset(s_tune_ch[s_tune_active]);
		return;
	case 'Z':
	case 'z': /* 编码器对位清零 + 当前通道 PID 运行态复位 */
		Stepper_EncoderZero();
		s_spd_last_enc = 0; /* 速度环差分基准同步归零，防止清零后一拍坏差分 */
		PID_Reset(s_tune_ch[s_tune_active]);
		return;
	case 'P':
	case 'p':
	case 'I':
	case 'i':
	case 'D':
	case 'd': /* 设增益：整体生效或整体丢弃 */
		val = strtol(line + 1, &end, 10);
		if (end == line + 1 || val > PID_TUNE_MILLI_MAX || val < -PID_TUNE_MILLI_MAX) {
			return;
		}
		{
			PID_t *pid = s_tune_ch[s_tune_active];
			float  v	  = (float) val / PID_TUNE_MILLI_SCALE;
			float  kp	  = pid->kp;
			float  ki	  = pid->ki;
			float  kd	  = pid->kd;
			if (cmd == 'P' || cmd == 'p') {
				kp = v;
			} else if (cmd == 'I' || cmd == 'i') {
				ki = v;
			} else {
				kd = v;
			}
			PID_SetGains(pid, kp, ki, kd); /* 只改参数不清积分；需清历史发 R */
		}
		return;
	default:
		return; /* 未知命令/EMI 噪声字节 → 静默丢弃 */
	}
}

static void PidTuner_Poll(void) {
	char line[UART1_LINE_MAX];
	USART1_EnsureRx(); /* RX 停摆自愈（电机 EMI 可致 ORE 后接收死掉） */
	if (USART1_GetLine(line, (uint8_t) sizeof line)) {
		s_getl_cnt++;
		PidTuner_ParseLine(line);
	}
}

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

	/* 50Hz：环路 PID 整定 + VOFA 输出（LOOP_UNDER_TEST 编译期选择位置/速度环）。
	 * 注意：本测试期间跳过 VL53L1X 读帧——电机 EMI 曾致 I2C1 总线锁死，
	 * 恢复测距业务时须先解决抗干扰并还原"测距健康才喂狗"保护逻辑 */
	if ((tick_ms % MEAS_PERIOD_MS) == 0) {
#if LOOP_UNDER_TEST == 0
		/* 阶跃目标：每 CL_STEP_PERIOD_MS 在 0 与 +CL_STEP_CNT(半圈) 间切换 */
		static uint16_t step_tick_ms = 0;
		step_tick_ms += MEAS_PERIOD_MS;
		if (step_tick_ms >= CL_STEP_PERIOD_MS) {
			step_tick_ms = 0;
			s_target_cnt = (s_target_cnt == 0) ? CL_STEP_CNT : 0;
		}

		/* 位置环：误差(计数) → PID → 带符号频率(Hz) → 统一下发
		 * （斜率限制+安全换向由 SetTargetFreq 内部完成，50Hz 调用保证时序） */
		s_feedback_cnt = Stepper_ReadEncoder();
		float f_out	   = PID_Update(&s_pos_pid, (float) s_target_cnt, (float) s_feedback_cnt);
		s_freq_cmd	   = (int32_t) f_out;
		Stepper_SetTargetFreq(s_freq_cmd);
#else
		/* 速度环：方波目标每 SPD_TEST_PERIOD_MS 在 ±SPD_TEST_HZ 间切换 */
		static uint16_t spd_tick_ms = 0;
		spd_tick_ms += MEAS_PERIOD_MS;
		if (spd_tick_ms >= SPD_TEST_PERIOD_MS) {
			spd_tick_ms = 0;
			s_spd_target_hz = (s_spd_target_hz > 0) ? -SPD_TEST_HZ : SPD_TEST_HZ;
		}

		/* 反馈：编码器累计值 20ms 差分 → 等效脉冲频率（ReadEncoder 内部处理回绕） */
		int32_t enc_now = Stepper_ReadEncoder();
		s_spd_fb_hz		= (enc_now - s_spd_last_enc) * SPD_FB_SCALE;
		s_spd_last_enc	= enc_now;

		/* 速度环：误差(等效Hz) → PID → 带符号频率(Hz) → 统一下发
		 * （SetTargetFreq 内部斜率限制天然抑制方波阶跃冲击） */
		float f_out = PID_Update(&s_spd_pid, (float) s_spd_target_hz, (float) s_spd_fb_hz);
		s_freq_cmd	= (int32_t) f_out;
		Stepper_SetTargetFreq(s_freq_cmd);
#endif
		/* VOFA ch0~2=目标/反馈/频率指令（当前环路量纲），ch3~5=当前整定通道
		 * Kp/Ki/Kd（毫单位），ch6~9=RX字节数/取行数/解析数/最近命令字母 */
		PID_t *tp = s_tune_ch[s_tune_active];
#if LOOP_UNDER_TEST == 0
		int32_t ch0 = s_target_cnt, ch1 = s_feedback_cnt;
#else
		int32_t ch0 = s_spd_target_hz, ch1 = s_spd_fb_hz;
#endif
		int32_t vofa[10] = {
			ch0, ch1, s_freq_cmd,
			(int32_t) (tp->kp * PID_TUNE_MILLI_SCALE),
			(int32_t) (tp->ki * PID_TUNE_MILLI_SCALE),
			(int32_t) (tp->kd * PID_TUNE_MILLI_SCALE),
			(int32_t) uart1_rx_bytes, (int32_t) s_getl_cnt,
			(int32_t) s_parse_cnt, (int32_t) s_last_cmd,
		};
		VOFA_SendFrame(vofa, 10);
	}
  /* 喂狗：速度测试期间无条件喂狗（VL53 读帧已跳过，见 20ms 分支注释）。
   * tick_ms 在 100 处回绕，实际每 100ms 判一次。 */
 	if (tick_ms % IWDG_PERIOD_MS == 0) {
    HAL_IWDG_Refresh(&hiwdg);
  }
		/* 10Hz：上位机命令解析（PID 串口在线整定） */
	if (tick_ms == 0) {
		PidTuner_Poll();
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
