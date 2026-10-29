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
#include "oled.h"		   /* OLED 基础封装 */
#include "ssd1306.h"	   /* SSD1306 底层绘制（ssd1306_FillRectangle/UpdateRect 等） */
#include "vl53l1x.h"	   /* VL53L1X ToF 测距传感器封装 */
#include "tof_filter.h"	   /* 中值-5 + EMA 数字滤波器 */
#include "stepper.h"	   /* 步进电机 / 编码器（平台保持用） */
#include "pid.h"		   /* 平台位置保持 PID */

#include <stdio.h>
#include <stdlib.h> /* strtol（串口参数解析） */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* TIM1 基础节拍 = 1ms（CubeMX Counter Period=999）。业务软件分频：
 * 测距/滤波/VOFA 放主循环，避免阻塞 ISR；平台保持放 ISR 保证 50Hz 节拍 */
#define CTRL_PERIOD_MS    20u  /* 平台保持控制：50Hz */
#define MEAS_PERIOD_MS    50u  /* ToF 测距 + 滤波 + VOFA：20Hz */
#define CMD_PERIOD_MS    100u  /* 串口命令解析：10Hz */
#define OLED_PERIOD_MS   200u  /* OLED 刷新：5Hz */
#define VALID_WIN_MS	   5000u /* 有效帧占比统计周期：5s（非刷新周期） */
#define IWDG_PERIOD_MS  1000u  /* 主循环心跳检查 + 喂狗：1Hz */

/* 平台保持（摆杆初始角度锁定）：H1 以当前编码器位置为零点并保持；
 * 默认上电不使能，避免首次上电时电机突然动作。H0 关闭并释放电机。 */
#define PLATFORM_HOLD_ON_BOOT 0		 /* 0 = 不使能，1 = 使能 */
#define HOLD_PID_KP		   2.0f	  /* Hz / 编码器计数 */
#define HOLD_PID_KI		   0.05f
#define HOLD_PID_KD		   0.0f
#define HOLD_PID_ILIM	   50.0f  /* 积分限幅（Hz） */
#define HOLD_OUT_LIMIT	   400.0f /* 输出限幅（Hz） */
#define HOLD_DEADBAND_CNT  3.0f	  /* 误差死区（计数） */

/* 小球位置闭环（外环，20Hz 与 ToF 同拍）：反馈 = EMA 滤波后的 pos_cm_x10
 * （相对 Z 命令标定的零点，0 = 管中心 O 点），输出 = 内环角度目标偏移
 * （编码器计数），由内环（50Hz 编码器位置环）跟踪。卡尔曼接入前反馈固定
 * 走 TofFilter（中值-5 + EMA），后续只换反馈源，环结构不变。
 * 默认值均为保守起步值，须按阶跃响应在线整定（P/I/D 命令，千分比）。 */
#define BALL_PID_KP			 2.0f  /* 计数 / 0.1cm：1cm 偏差 → 20 计数 ≈ 1.8° */
#define BALL_PID_KI			 0.05f  /* 消静差；杆存在倾角-重力静差时必需 */
#define BALL_PID_KD			 0.0f  /* ToF 群延迟大（中值2帧+EMA），起步不用 D */
#define BALL_PID_ILIM		 100.0f /* 积分限幅（计数） */
#define BALL_PID_OUT_LIM	 400.0f /* 外环输出限幅（计数，≈36°）防目标角度翻车 */
#define BALL_PID_DEADBAND_X10 3.0f  /* 死区 ±0.3cm（≈1.6σ，σ=1.87mm 实测） */
/* 环路符号：+1 表示"PID 输出正偏移 → pos 减小"。符号错则正反馈跑飞：
 * 判定实验——B1 后把球手动偏到 +2cm 松手，若杆动作使 pos 继续增大，
 * 立即 B0 并把本宏改 -1。TOF_POS_SIGN 改动后本宏同样要跟着取反。 */
#define BALL_PID_SIGN		 (+1)
#define BALL_LOSS_MAX_FRAMES 10u /* ToF 连续无效 0.5s → 外环回中（防失联跑飞） */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ---- ToF 滤波与数据状态（主循环独占写，ISR/OLED 只读） ---- */
static TofFilter_t s_filter;

static volatile int16_t  s_raw_mm	   = 0;	 /* 本帧原始距离（mm），无效帧为 0 */
static volatile int16_t  s_filt_mm	   = 0;	 /* 滤波后距离（mm） */
static volatile int16_t  s_pos_cm_x10 = 0;	 /* 滤波后位置（0.1cm） */
static volatile int8_t	 s_range_st	   = -1; /* 0=有效；-1/-2=通信；-3=滤波量程拒绝 */
static volatile uint32_t s_valid_cnt   = 0;	 /* 滤波器累计有效帧 */
static volatile uint8_t  s_seg_id	   = 0;	 /* 测试段号（S 命令） */

/* ---- 有效帧占比：每帧实时结算（随 OLED 5Hz 刷新），计数器 5s 清零 ---- */
static volatile uint32_t s_win_total  = 0; /* 窗口内已处理帧数 */
static volatile uint32_t s_win_valid  = 0; /* 窗口内有效帧数（out_st==0） */
static volatile uint32_t s_win_t0	  = 0; /* 窗口起始时刻 ms */
static volatile uint8_t	 s_valid_pct = 0; /* 0~100，最近窗口有效帧占比 */

/* ---- 调度标志（TIM1 ISR 置位，主循环消费） ---- */
static volatile uint8_t s_meas_tick = 0;
static volatile uint8_t s_cmd_tick	= 0;
static volatile uint8_t s_oled_tick = 0;
static volatile uint8_t s_loop_alive = 0; /* 主循环心跳：ISR 每秒检查一次 */

/* ---- 控制模式：0=释放 1=平台保持（内环目标0） 2=小球闭环（外环→内环目标） ----
 * s_ctrl_mode/s_ball_ofs_cnt 由主循环写、TIM1 ISR 读，须 volatile */
static PID_t			s_hold_pid;					/* 内环：编码器位置环 */
static PID_t			s_ball_pid;					/* 外环：小球位置环（ToF pos → 计数偏移） */
static volatile uint8_t s_ctrl_mode	   = 0;
static volatile int16_t s_ball_ofs_cnt   = 0;		/* 外环输出：内环目标偏移（计数） */
static volatile uint8_t s_tof_lost_n	   = 0;		/* 外环连续无效帧计数 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Tof_Process(void);
static void TofCmd_Poll(void);
static void TofCmd_ParseLine(const char *line);
static void Loop_Init(void);
static void Loop_SetMode(uint8_t mode);
static void BallOuter_Tick(void);
static void InnerLoop_Tick(void);
static void Enc_Service(void);
static void OLED_Refresh(void);
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
	/* IWDG 自 MX_IWDG_Init 起已运行，初始化分步喂狗防误复位 */
	HAL_IWDG_Refresh(&hiwdg);

	OLED_Init();
	OLED_Print(0, 0, "TOF FILTER TEST");
	ssd1306_UpdateRect(0, 0, SSD1306_WIDTH, 8);
	HAL_IWDG_Refresh(&hiwdg);

	/* 复位诊断：OLED 第二行显示上次复位源——
	 * P=上电/掉电 I=IWDG N=NRST S=软件。首次上电 P 属正常 */
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
		ssd1306_UpdateRect(0, 8, SSD1306_WIDTH, 8);
	}
	HAL_IWDG_Refresh(&hiwdg);

	/* VL53L1X 初始化 + 启动连续测量（Short 模式 250mm）。
	 * 内部阻塞 ~数百 ms，期间持续喂狗防 IWDG 误复位 */
	{
		VL53L1_Error err = VL53L1X_Init();
		if (err != VL53L1_ERROR_NONE) {
			OLED_Print(0, 16, "VL53 FAIL");
			OLED_Print(0, 24, "step:%s err:%d", VL53L1X_LastErrorStep(), (int) err);
			ssd1306_UpdateRect(0, 16, SSD1306_WIDTH, 16);
			while (1) { HAL_IWDG_Refresh(&hiwdg); }
		}
	}
	HAL_IWDG_Refresh(&hiwdg);

	/* 滤波器与平台保持初始化：Stepper_Init 只校验 TIM2 配置并启动编码器，
	 * 不使能电机、不输出脉冲；需要 H1 命令才进入保持 */
	TofFilter_Init(&s_filter, TOF_FILTER_ALPHA_DEFAULT_MILLI, TOF_FILTER_ZERO_DEFAULT_MM);

	/* 硬件标定零点：2026-10-29 实测 60s/930帧 mean=131.4mm, σ=1.87mm。
	 * 小球静止时 VL53L1X 原始距离的期望值。Z 命令可在线覆盖此值。 */
	TofFilter_SetZeroMm(&s_filter, 131);
	Loop_Init();
#if PLATFORM_HOLD_ON_BOOT
	Loop_SetMode(1);
#endif
	HAL_IWDG_Refresh(&hiwdg);

	/* 接收上位机命令（A/Z/S/H/B/P/I/D/R）；VOFA 数据流仍只含数值，不混入文本 */
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

		/* 测距/滤波/VOFA 在主循环执行，避免阻塞 TIM1 的 50Hz 保持控制 */
		if (s_meas_tick) {
			s_meas_tick = 0;
			Tof_Process();
		}

		/* 串口命令解析：10Hz，一次可排空队列中的多条命令 */
		if (s_cmd_tick) {
			s_cmd_tick = 0;
			TofCmd_Poll();
		}

		/* OLED 5Hz 刷新：先处理 Z 事件与掉电保存，再刷屏 */
		if (s_oled_tick) {
			s_oled_tick = 0;
			Enc_Service();
			OLED_Refresh();
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

/* ===================== ToF：测距 → 滤波 → VOFA（主循环 20Hz） ============= */

static void Tof_Process(void) {
	int16_t dist_mm = 0;
	int8_t	rstat	= -1;
	int8_t	out_st;

	VL53L1X_Read(&dist_mm, &rstat);
	out_st = rstat; /* 通信错误码 -1/-2 原样上报 */

	if (rstat == 0) {
		s_raw_mm = dist_mm;
		if (!TofFilter_Update(&s_filter, dist_mm, rstat)) {
			out_st = -3; /* 状态有效但原始值越界，滤波器拒绝 */
		} else {
			out_st = 0;
		}
		s_filt_mm	 = TofFilter_GetMm(&s_filter);
		s_pos_cm_x10 = TofFilter_GetCmX10(&s_filter);
		s_valid_cnt	 = TofFilter_GetValidCount(&s_filter);
	} else {
		s_raw_mm = 0; /* 通信失败帧不输出伪距离 */
	}
	s_range_st = out_st;

	/* 有效帧占比：每帧实时结算（OLED 5Hz 直接显示），计数器每 5s 清零重计 */
	s_win_total++;
	if (out_st == 0) {
		s_win_valid++;
	}
	if (s_win_total > 0) {
		s_valid_pct = (uint8_t) ((s_win_valid * 100u + s_win_total / 2u) / s_win_total);
	}
	uint32_t now_ms = HAL_GetTick();
	if ((uint32_t) (now_ms - s_win_t0) >= VALID_WIN_MS) {
		s_win_t0	= now_ms;
		s_win_total = 0;
		s_win_valid = 0;
	}

	/* 外环与滤波同拍（20Hz）：pos 更新后立刻推进外环，避免 50Hz 重复吃旧数据 */
	BallOuter_Tick();

	/* VOFA FireWater：ch0=raw_mm, ch1=filt_mm, ch2=pos_cm_x10,
	 * ch3=status, ch4=valid_cnt, ch5=alpha_milli, ch6=seg, ch7=ctrl_mode,
	 * ch8=ball_ofs_cnt（外环输出/内环目标偏移）, ch9=pulse_freq_hz,
	 * ch10=enc_cnt（内环反馈）, ch11=zero_mm */
	int32_t vofa[12] = {
		(int32_t) s_raw_mm,
		(int32_t) s_filt_mm,
		(int32_t) s_pos_cm_x10,
		(int32_t) s_range_st,
		(int32_t) s_valid_cnt,
		(int32_t) TofFilter_GetAlphaMilli(&s_filter),
		(int32_t) s_seg_id,
		(int32_t) s_ctrl_mode,
		(int32_t) s_ball_ofs_cnt,
		Stepper_GetPulseFreqHz(),
		Stepper_ReadEncoder(),
		(int32_t) TofFilter_GetZeroMm(&s_filter),
	};
	VOFA_SendFrame(vofa, 12);
}

/* ===================== 串口命令：A/Z/S/H/B/P/I/D/R ========================= */
/* A<milli> : 设置 EMA 系数（20~1000，1000=直通）
 * Z        : 把当前滤波值设为零点；Z<mm> 显式设置零点距离
 * S<id>    : 设置测试段号（0~99，随 VOFA ch6 回传）
 * H / H1   : 平台保持——以当前编码器位置为零点锁定摆杆；H0 关闭并释放电机
 * B / B1   : 小球闭环——外环以 ToF 零点为目标控制小球到 O 点；B0 关闭并释放
 *            （B 与 H 互斥，同一电机任一时刻只挂一个环）
 * P/I/D<v> : 外环增益千分比在线整定（0~10000，如 P2500 = Kp 2.500）
 * R        : 清除滤波器窗口/EMA 与内外环积分（参数与零点保留） */
static void TofCmd_ParseLine(const char *line) {
	char  cmd;
	char *end;
	long  val	  = 0;
	bool  has_val = false;

	while (*line == ' ' || *line == '\t') {
		line++;
	}
	cmd = *line;
	if (cmd == '\0') {
		return;
	}

	if (line[1] != '\0') {
		val		= strtol(line + 1, &end, 10);
		has_val = (end != (line + 1));
	}

	switch (cmd) {
	case 'A':
	case 'a':
		if (has_val && val >= (long) TOF_FILTER_ALPHA_MIN_MILLI && val <= (long) TOF_FILTER_ALPHA_MAX_MILLI) {
			TofFilter_SetAlpha(&s_filter, (uint16_t) val);
		}
		break;
	case 'Z':
	case 'z':
		if (has_val) {
			if (val >= 0 && val <= (long) TOF_FILTER_MAX_VALID_MM) {
				TofFilter_SetZeroMm(&s_filter, (int16_t) val);
			}
		} else {
			(void) TofFilter_ZeroHere(&s_filter);
		}
		break;
	case 'S':
	case 's':
		if (has_val && val >= 0 && val <= 99) {
			s_seg_id = (uint8_t) val;
		}
		break;
	case 'H':
	case 'h':
		if (!has_val || val != 0) {
			Loop_SetMode(1);
		} else {
			Loop_SetMode(0);
		}
		break;
	case 'B':
	case 'b':
		if (!has_val || val != 0) {
			Loop_SetMode(2);
		} else {
			Loop_SetMode(0);
		}
		break;
	case 'P':
	case 'p':
		if (has_val && val >= 0 && val <= 10000) {
			PID_SetGains(&s_ball_pid, (float) val / 1000.0f, s_ball_pid.ki, s_ball_pid.kd);
		}
		break;
	case 'I':
	case 'i':
		if (has_val && val >= 0 && val <= 10000) {
			PID_SetGains(&s_ball_pid, s_ball_pid.kp, (float) val / 1000.0f, s_ball_pid.kd);
		}
		break;
	case 'D':
	case 'd':
		if (has_val && val >= 0 && val <= 10000) {
			PID_SetGains(&s_ball_pid, s_ball_pid.kp, s_ball_pid.ki, (float) val / 1000.0f);
		}
		break;
	case 'R':
	case 'r':
		TofFilter_Reset(&s_filter);
		PID_Reset(&s_hold_pid);
		PID_Reset(&s_ball_pid);
		s_ball_ofs_cnt = 0;
		s_tof_lost_n   = 0;
		s_win_total = 0;
		s_win_valid = 0;
		s_win_t0	= HAL_GetTick();
		s_valid_pct = 0;
		break;
	default:
		break; /* 未知命令/EMI 噪声静默丢弃 */
	}
}

static void TofCmd_Poll(void) {
	char line[UART1_LINE_MAX];

	USART1_EnsureRx(); /* RX 停摆自愈 */
	while (USART1_GetLine(line, (uint8_t) sizeof line)) {
		TofCmd_ParseLine(line);
	}
}

/* ===================== 闭环控制：内环（编码器） + 外环（小球位置） ========= */
/* 级联结构（项目知识笔记 §13 术语）：外环 20Hz，反馈 ToF pos（EMA 滤波后、
 * 相对零点，0 = O 点），输出摆杆角度目标偏移；内环 50Hz ISR，编码器位置环
 * 跟踪该偏移。模式 1（H）是外环断开、偏移恒 0 的退化形态，即平台保持。
 * 模式 2（B）使能外环，目标 = 外环输出。 */

static void Loop_Init(void) {
	Stepper_Init(); /* 校验 TIM2 PSC、启动 TIM3 编码器与 Z 相捕获；不使能电机 */
	(void) Stepper_StoreRestore(); /* 恢复掉电前的编码器绝对位置（校验失败则为 0） */
	PID_Init(&s_hold_pid, HOLD_PID_KP, HOLD_PID_KI, HOLD_PID_KD,
			 -HOLD_OUT_LIMIT, HOLD_OUT_LIMIT, HOLD_PID_ILIM);
	PID_SetDeadband(&s_hold_pid, HOLD_DEADBAND_CNT);
	PID_Init(&s_ball_pid, BALL_PID_KP, BALL_PID_KI, BALL_PID_KD,
			 -BALL_PID_OUT_LIM, BALL_PID_OUT_LIM, BALL_PID_ILIM);
	PID_SetDeadband(&s_ball_pid, BALL_PID_DEADBAND_X10);
	s_ctrl_mode	   = 0;
	s_ball_ofs_cnt = 0;
	s_tof_lost_n   = 0;
	Stepper_SetTargetFreq(0);
	Stepper_SetEnabled(false);
}

static void Loop_SetMode(uint8_t mode) {
	/* 先停控制，再清零/复位，最后才置新模式，避免 ISR 在复位中途用旧状态 */
	s_ctrl_mode = 0;
	Stepper_SetTargetFreq(0);
	if (mode == 0u) {
		Stepper_StoreMarkDirty(); /* 停机：主循环在静止后把绝对位置落盘 */
		Stepper_SetEnabled(false);
		return;
	}
	Stepper_EncoderZero(); /* 以当前摆杆角度为内环零点 */
	PID_Reset(&s_hold_pid);
	PID_Reset(&s_ball_pid);
	s_ball_ofs_cnt = 0;
	s_tof_lost_n   = 0;
	Stepper_SetEnabled(true);
	s_ctrl_mode = mode;
}

/* 外环（20Hz，主循环随 Tof_Process 调用）：pos_cm_x10 → 内环目标偏移。
 * 仅模式 2 且本帧 ToF 有效时推进；连续无效超阈值则复位并回中，防失联跑飞 */
static void BallOuter_Tick(void) {
	float out;

	if (s_ctrl_mode != 2u) {
		return;
	}
	if (s_range_st != 0) {
		if (s_tof_lost_n < 255u) {
			s_tof_lost_n++;
		}
		if (s_tof_lost_n >= BALL_LOSS_MAX_FRAMES) {
			PID_Reset(&s_ball_pid);
			s_ball_ofs_cnt = 0; /* 回中：内环目标回到摆杆保持零点 */
		}
		return;
	}
	s_tof_lost_n = 0;

	/* err = 0 - pos（PID 内部计算）；SIGN 吸收机械链路符号，见宏注释 */
	out			 = BALL_PID_SIGN * PID_Update(&s_ball_pid, 0.0f, (float) s_pos_cm_x10);
	s_ball_ofs_cnt = (int16_t) out; /* PID 输出已限幅 ±BALL_PID_OUT_LIM(400)，不溢出 */
}

/* 内环（50Hz，TIM1 ISR）：编码器位置环，目标 = 外环偏移（模式1恒为0） */
static void InnerLoop_Tick(void) {
	int32_t fb;
	float	target;
	float	f_out;

	if (s_ctrl_mode == 0u) {
		return;
	}
	target = (s_ctrl_mode == 2u) ? (float) s_ball_ofs_cnt : 0.0f;
	fb	   = Stepper_ReadEncoder();
	f_out  = PID_Update(&s_hold_pid, target, (float) fb);
	Stepper_SetTargetFreq((int32_t) f_out);
}

/* ===================== Z 相事件与掉电保存：5Hz 主循环服务 ================= */
/* Flash 擦除/编程期间 CPU 从 Flash 取指停顿约 20~40ms，会打断 50Hz 控制环，
 * 因此只在模式 0（控制已停、电机休眠）且摆杆静止时才落盘。运行中直接掉电
 * 时保存值滞后到最后一次停机位置——这是"断电后杆保持不动"前提下的取舍。 */
static int32_t s_store_abs_ref = 0;

static void Enc_Service(void) {
	int32_t abs;

	(void) Stepper_ReadEncoder(); /* 模式 0 时内环 ISR 不读编码器，由这里刷新计数 */
	(void) Stepper_ProcessZ();	  /* Z 相：绝对计数对齐到最近整圈 */

	if (s_ctrl_mode != 0u) {
		s_store_abs_ref = Stepper_ReadEncoderAbs(); /* 控制中只跟踪，不擦 Flash */
		return;
	}

	abs = Stepper_ReadEncoderAbs();
	if (abs != s_store_abs_ref) {
		s_store_abs_ref = abs; /* 仍在动：等静止后再存 */
		return;
	}
	if (Stepper_StorePending()) {
		(void) Stepper_StoreSave();
	}
}

/* ===================== OLED：5Hz 局部刷新 ================================ */

static void OLED_Refresh(void) {
	int16_t pos		= s_pos_cm_x10;
	char	sign	= (pos < 0) ? '-' : ' ';
	int32_t abs_pos = (pos < 0) ? -(int32_t) pos : (int32_t) pos;

	ssd1306_FillRectangle(0, 16, SSD1306_WIDTH - 1, 63, Black);
	OLED_Print(0, 16, "RAW:%4d mm", (int) s_raw_mm);
	OLED_Print(0, 24, "FLT:%4d mm", (int) s_filt_mm);
	OLED_Print(0, 32, "POS:%c%ld.%ld cm", sign, (long) (abs_pos / 10), (long) (abs_pos % 10));
	OLED_Print(0, 40, "ST:%d A:%d", (int) s_range_st, (int) TofFilter_GetAlphaMilli(&s_filter));
	OLED_Print(0, 48, "V:%3lu%% M:%d S:%d", (unsigned long) s_valid_pct, (int) s_ctrl_mode, (int) s_seg_id);
	OLED_Print(0, 56, "ENC:%6ld Z:%lu", (long) Stepper_ReadEncoderAbs(),
			   (unsigned long) Stepper_GetZCount());
	ssd1306_UpdateRect(0, 16, SSD1306_WIDTH, 48);
}

/* ===================== TIM1 1ms 节拍 ===================================== */
/* 控制放 ISR 保证 50Hz；测距/滤波/VOFA/命令/OLED 放主循环，避免阻塞控制 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	static uint16_t tick_ms = 0;

	if (htim->Instance != TIM1) {
		return;
	}

	if (++tick_ms >= 1000u) {
		tick_ms = 0;
	}

	if ((tick_ms % CTRL_PERIOD_MS) == 0) {
		InnerLoop_Tick();
	}
	if ((tick_ms % MEAS_PERIOD_MS) == 0) {
		s_meas_tick = 1;
	}
	if ((tick_ms % CMD_PERIOD_MS) == 0) {
		s_cmd_tick = 1;
	}
	if ((tick_ms % OLED_PERIOD_MS) == 0) {
		s_oled_tick = 1;
	}

	/* 主循环心跳正常才喂狗：主循环卡死超过 1 个周期即触发 IWDG 复位 */
	if (tick_ms == 0) {
		if (s_loop_alive) {
			s_loop_alive = 0;
			HAL_IWDG_Refresh(&hiwdg);
		}
	}
}

/* Z 相（TIM3_CH3 = PB0）捕获回调：ISR 内只登记事件，对齐/落盘在主循环 Enc_Service */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		Stepper_ZIndexEvent();
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
