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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"	/* 编码器计数/Z 校准/掉电保存 */
#include "key.h"		/* PA0 按键消抖与事件队列 */
#include "motor.h"		/* 步进电机脉冲/方向/使能 */
#include "oled.h"		/* OLED 屏幕原语（页面文案在本文件组合） */
#include "pid.h"		/* 内外环 PID */
#include "tof.h"		/* 测距 → 滤波 → 有效帧占比 */
#include "tof_filter.h" /* 命令参数范围（A/Z 命令夹取门限） */

#include <stdbool.h>
#include <stdlib.h> /* strtol（串口参数解析） */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* TIM1 基础节拍 = 1ms（CubeMX Counter Period=999）。业务软件分频：
 * 测距/滤波/VOFA 放主循环，避免阻塞 ISR；平台保持放 ISR 保证 50Hz 节拍 */
#define CTRL_PERIOD_MS 20u	/* 平台保持控制：0.05s */
#define MEAS_PERIOD_MS 50u	/* ToF 测距 + 滤波 + VOFA：0.05s */
#define CMD_PERIOD_MS  100u /* 串口命令解析：0.1s*/
#define OLED_PERIOD_MS 200u /* OLED 刷新：0.2s*/

/* ---- 平台保持（H，摆杆锁定）：以编码器位置为零点的内环 ---- */
#define HOLD_PID_KP		  2.0f /* Hz / 编码器计数 */
#define HOLD_PID_KI		  0.05f
#define HOLD_PID_KD		  0.0f
#define HOLD_PID_ILIM	  50.0f	 /* 积分限幅（Hz） */
#define HOLD_OUT_LIMIT	  400.0f /* 输出限幅（Hz） */
#define HOLD_DEADBAND_CNT 3.0f	 /* 误差死区（计数） */

/* ---- 小球位置闭环（B，外环 20Hz）：反馈 = 滤波后 pos_cm_x10（0 = O 点），
 *      输出 = 内环角度目标偏移（编码器计数）。默认值为保守起步值，
 *      须按阶跃响应在线整定（P/I/D 命令，千分比） ---- */
#define BALL_PID_KP			  2.0f	 /* 计数 / 0.1cm：1cm 偏差 → 20 计数 ≈ 1.8° */
#define BALL_PID_KI			  0.05f	 /* 消静差；杆存在倾角-重力静差时必需 */
#define BALL_PID_KD			  0.0f	 /* ToF 群延迟大（中值2帧+EMA），起步不用 D */
#define BALL_PID_ILIM		  100.0f /* 积分限幅（计数） */
#define BALL_PID_OUT_LIM	  400.0f /* 外环输出限幅（计数，≈36°）防目标角度翻车 */
#define BALL_PID_DEADBAND_X10 3.0f	 /* 死区 ±0.3cm（≈1.6σ，σ=1.87mm 实测） */
/* 环路符号：+1 表示"PID 输出正偏移 → pos 减小"。符号错则正反馈跑飞。
 * 符号链（用户确认：编码器示数与 ToF 测距示数正相关）：
 *   freq>0 → enc 增大（ENC_POS_DIR_SIGN=+1）
 *   enc 增大 ↔ ToF 读数增大 ↔ pos 增大
 *   外环 err=0-pos：pos>0 → 输出为负 → 编码器目标偏移为负 → enc 减小 → pos 减小
 * 故 +1 自洽，无需实测翻转。若实物上 pos 与 enc 一增一减，先查 A/B 相接线
 * 与 TOF_POS_SIGN，再同步取反本宏（不可只改一个）。 */
#define BALL_PID_SIGN		 (+1)
#define BALL_LOSS_MAX_FRAMES 10u /* ToF 连续无效 0.5s → 外环回中（防失联跑飞） */

/* 上电是否自动进入平台保持（0 = 不使能，1 = 使能） */
#define PLATFORM_HOLD_ON_BOOT 1

/* 电机总开关：0 = 不操控电机（EN 保持休眠、不下发脉冲频率），
 *                 1 = 正常输出（内环 PID → TIM2 脉冲 + EN 使能）
 * 注意：EN 只管电机功率级（H 桥输出），不是驱动板电源开关——驱动板仍上电，
 *       对最小系统板的供电不受影响。关掉后测距/滤波/编码器/串口整定照常运行，
 *       用于脱机调传感器与增益 */
#define MOTOR_CONTROL_ENABLE 0

/* ================= VOFA 串口帧通道定义（唯一权威表） =================
 * 原则：串口只送实时流变量；低频状态量（控制模式/急停/绝对位置/Z 计数/
 * 电机使能）随 OLED 页面观察，不占串口带宽。
 *   idx  名称            单位        来源
 *   0    enc_rel         enc_cnt     内环反馈 Encoder_Read()
 *   1    hold_target     enc_cnt     内环目标（模式2=外环偏移，其余 0）
 *   2    hold_integral   Hz×1000     内环积分项（判积分饱和）
 *   3    pulse_freq_hz   Hz          实际脉冲频率 Motor_GetPulseFreqHz()
 *   4    pos_cm_x10      0.1cm       小球位置（ToF 滤波后）
 *   5    ball_ofs_cnt    enc_cnt     外环输出
 * 增删通道：只改本枚举 + Ctrl_SendTelemetry 填表 + 上位机 channels.json。 */
enum {
	VOFA_CH_ENC_REL = 0,   /* 0 内环反馈（编码器计数） */
	VOFA_CH_HOLD_TARGET,   /* 1 内环目标（编码器计数） */
	VOFA_CH_HOLD_INTEGRAL, /* 2 内环积分项 ×1000（Hz） */
	VOFA_CH_PULSE_FREQ,	   /* 3 实际脉冲频率（Hz） */
	VOFA_CH_POS_CM_X10,	   /* 4 小球位置（0.1cm） */
	VOFA_CH_BALL_OFS,	   /* 5 外环输出（编码器计数） */
	VOFA_CH_NUM			   /* 通道总数（=帧长） */
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ---- 调度标志（TIM1 ISR 置位，主循环消费） ---- */
static volatile uint8_t s_meas_tick = 0; /* 测距/滤波/VOFA 0.05s */
static volatile uint8_t s_cmd_tick	= 0; /* 串口命令解析 0.1s */
static volatile uint8_t s_oled_tick = 0; /* OLED 刷新 0.2s */
/* 主循环心跳：1 = 最近 1s 内主循环至少跑过一圈；ISR 据此决定是否喂狗 */
static volatile uint8_t s_loop_alive = 0;

/* ---- 控制状态：由主循环写、TIM1 ISR 读，须 volatile ---- */
static PID_t			s_hold_pid;			/* 内环：编码器位置环 */
static PID_t			s_ball_pid;			/* 外环：小球位置环（ToF pos → 计数偏移） */
static volatile uint8_t s_ctrl_mode	   = 0; /* 0=释放 1=平台保持 2=小球闭环 */
static volatile int16_t s_ball_ofs_cnt = 0; /* 外环输出：内环目标偏移（计数） */
static volatile uint8_t s_tof_lost_n   = 0; /* 外环连续无效帧计数 */
/* 急停锁存：1 = 已按 PA0 使电机休眠，松手不自动恢复 */
static volatile uint8_t s_key_estop = 0;

/* 掉电保存基准：最后一次落盘/跟踪的绝对位置 */
static int32_t s_store_abs_ref = 0;

/* 测距滤波器实例：算法在 tof_filter.c（纯整数），状态由应用层持有——
 * 与下面的 PID 实例同一归属方式，命令直接操作实例，不再套一层转发 */
static TofFilter_t s_tof_filter;

/* 内外环参数表：全部集中在此，PID 模块只负责按表初始化（.deadband/.d_alpha
 * 由 PID_InitFromCfg 就地校验，调用处不重复写校验逻辑） */
static const PIDCfg_t s_hold_cfg = {
	.kp		  = HOLD_PID_KP,
	.ki		  = HOLD_PID_KI,
	.kd		  = HOLD_PID_KD,
	.out_min  = -HOLD_OUT_LIMIT,
	.out_max  = HOLD_OUT_LIMIT,
	.i_limit  = HOLD_PID_ILIM,
	.deadband = HOLD_DEADBAND_CNT,
	.d_alpha  = 1.0f, // 直通
};

static const PIDCfg_t s_ball_cfg = {
	.kp		  = BALL_PID_KP,
	.ki		  = BALL_PID_KI,
	.kd		  = BALL_PID_KD,
	.out_min  = -BALL_PID_OUT_LIM,
	.out_max  = BALL_PID_OUT_LIM,
	.i_limit  = BALL_PID_ILIM,
	.deadband = BALL_PID_DEADBAND_X10,
	.d_alpha  = 1.0f, // 直通微分
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* 应用层：命令解析、级联控制、Z/掉电保存服务（仅本文件使用） */
static void Cmd_ParseLine(const char *line);
static void Cmd_Poll(void);
static void Ctrl_Init(void);
static void Ctrl_SetMode(uint8_t mode);
static void Ctrl_OuterTick(void);
static void Ctrl_InnerTick(void);
static void Ctrl_EncService(void);
static void Ctrl_SendTelemetry(void);
static void Ctrl_PollKeyEvents(void);
static void Ctrl_ResetLoops(void);
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
	/* 启动页标题行 */
	OLED_Print(0, 0, "TOF FILTER TEST");
	OLED_Flush(0, 0, OLED_WIDTH, 8);
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
		int err = Tof_InitSensor();

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

	/* 滤波器与零点 + 执行器/编码器/内外环初始化（Motor_Init 只校验 TIM2 配置、
	 * Encoder_Init 只启动编码器，两者都不使能电机、不输出脉冲；上电模式由
	 * Ctrl_Init 按 PLATFORM_HOLD_ON_BOOT 决定） */
	TofFilter_Init(&s_tof_filter, TOF_FILTER_ALPHA_DEFAULT_MILLI, TOF_FILTER_ZERO_DEFAULT_MM);
	/* 硬件标定零点：2026-10-29 实测 60s/930帧 mean=131.4mm, σ=1.87mm——
	 * 小球静止时 VL53L1X 原始距离的期望值；Z 命令可在线覆盖 */
	TofFilter_SetZeroMm(&s_tof_filter, 131);
	Ctrl_Init();
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

		/* 20Hz：测距/滤波/统计 → 外环 → VOFA。放主循环避免阻塞 50Hz 内环 */
		if (s_meas_tick) {
			s_meas_tick = 0;
			Tof_Tick(&s_tof_filter);
			Ctrl_OuterTick();
			Ctrl_SendTelemetry();
		}

		/* 10Hz 串口命令解析：一次可排空队列中的多条命令 */
		if (s_cmd_tick) {
			s_cmd_tick = 0;
			Cmd_Poll();
		}

		/* 5Hz：先处理 Z 事件与掉电保存，再刷运行时页面 */
		if (s_oled_tick) {
			s_oled_tick = 0;
			Ctrl_EncService();

			/* 运行时页面：测距/滤波/位置/状态/占比/模式/编码器绝对位置 */
			int16_t pos		= g_tof.pos_cm_x10;
			char	sign	= (pos < 0) ? '-' : ' ';
			int32_t abs_pos = (pos < 0) ? -(int32_t) pos : (int32_t) pos;

			OLED_ClearArea(0, 16, OLED_WIDTH, 48);
			OLED_Print(0, 16, "RAW:%4d mm", (int) g_tof.raw_mm);
			OLED_Print(0, 24, "FLT:%4d mm", (int) g_tof.filt_mm);
			OLED_Print(0, 32, "POS:%c%ld.%ld cm", sign, (long) (abs_pos / 10), (long) (abs_pos % 10));
			OLED_Print(0, 40, "ST:%d A:%d", (int) g_tof.range_status, (int) g_tof.alpha_milli);
			OLED_Print(0, 48, "V:%3lu%% M:%c", (unsigned long) g_tof.valid_pct,
					   (s_key_estop != 0u) ? 'E' : (char) ('0' + s_ctrl_mode));
			OLED_Print(0, 56, "ENC:%6ld Z:%lu", (long) Encoder_ReadAbs(), (unsigned long) Encoder_GetZCount());
			OLED_Flush(0, 16, OLED_WIDTH, 48);
		}

		/* 按键事件消费（非阻塞）：急停锁存 / 解除锁存 */
		Ctrl_PollKeyEvents();
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

/* ===================== 应用层：级联闭环 =================================== */
/* 外环（20Hz）反馈 ToF 位置（相对零点，0 = O 点），输出摆杆角度目标偏移；
 * 内环（50Hz ISR）用编码器位置环跟踪该偏移。模式 1（H）是外环断开、偏移
 * 恒 0 的退化形态，即平台保持；模式 2（B）使能外环。
 * 时序：Ctrl_InnerTick ← TIM1 每 20ms；Ctrl_OuterTick ← 20Hz 紧跟 Tof_Tick；
 *       Ctrl_SendTelemetry ← 20Hz 紧跟 Ctrl_OuterTick；Ctrl_EncService ← 5Hz；
 *       Ctrl_PollKeyEvents ← 主循环每圈。 */

/**
 * @brief 初始化执行器/编码器/内外环，并按 PLATFORM_HOLD_ON_BOOT 进入上电模式
 * @note  Motor_Init/Encoder_Init 只校验与启动外设，不使能电机、不输出脉冲
 */
static void Ctrl_Init(void) {
	Motor_Init();				   /* 校验 TIM2 PSC；不使能电机、不输出脉冲 */
	Encoder_Init();				   /* 启动 TIM3 编码器与 Z 相捕获 */
	(void) Encoder_StoreRestore(); /* 恢复掉电前的绝对位置（校验失败则为 0） */
	PID_InitFromCfg(&s_hold_pid, &s_hold_cfg);
	PID_InitFromCfg(&s_ball_pid, &s_ball_cfg);
	s_ctrl_mode	   = 0;
	s_ball_ofs_cnt = 0;
	s_tof_lost_n   = 0;
	Motor_SetTargetFreq(0);
	Motor_SetEnabled(false);

#if PLATFORM_HOLD_ON_BOOT
	Ctrl_SetMode(1);
#endif
}

/**
 * @brief 切换控制模式：0=释放 1=平台保持 2=小球闭环
 * @note  先停控制，再清零/复位，最后才置新模式，避免 ISR 在复位中途用旧状态；
 *        当前仅 Ctrl_Init 按 PLATFORM_HOLD_ON_BOOT 进入模式 1，模式 2 无入口
 *        （串口不再开放模式切换，保留供后续板载/硬件触发）
 */
static void Ctrl_SetMode(uint8_t mode) {
	s_ctrl_mode = 0;
	Motor_SetTargetFreq(0);
	if (mode == 0u) {
		Encoder_StoreMarkDirty(); /* 停机：主循环在静止后把绝对位置落盘 */
		Motor_SetEnabled(false);
		return;
	}
	s_key_estop = 0;   /* 进入非 0 模式即解除按键急停锁存（当前仅上电初始化进入模式 1） */
	Encoder_Zero();	   /* 以当前摆杆角度为内环零点 */
	Ctrl_ResetLoops(); /* 内外环积分/目标偏移/失联计数统一复位 */
#if MOTOR_CONTROL_ENABLE
	Motor_SetEnabled(true);
#endif
	s_ctrl_mode = mode;
}

/**
 * @brief 外环 20Hz：pos_cm_x10 → 内环目标偏移
 * @note  仅模式 2 且本帧 ToF 有效时推进；连续无效超阈值则复位并回中，防失联跑飞
 */
static void Ctrl_OuterTick(void) {
	float out;

	if (s_ctrl_mode != 2u) {
		return;
	}
	if (g_tof.range_status != 0) {
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
	out			   = BALL_PID_SIGN * PID_Update(&s_ball_pid, 0.0f, (float) g_tof.pos_cm_x10);
	s_ball_ofs_cnt = (int16_t) out; /* PID 输出已限幅 ±BALL_PID_OUT_LIM(400)，不溢出 */
}

/**
 * @brief 内环 50Hz（TIM1 ISR）：编码器位置环，目标 = 外环偏移（模式1恒为0）
 * @note  ISR 上下文：只做一次 PID 计算与一次 TIM2 下发，不可阻塞
 */
static void Ctrl_InnerTick(void) {
	int32_t fb;
	float	target;
	float	f_out;

	if (s_ctrl_mode == 0u) {
		return;
	}
	target = (s_ctrl_mode == 2u) ? (float) s_ball_ofs_cnt : 0.0f;
	fb	   = Encoder_Read();
	f_out  = PID_Update(&s_hold_pid, target, (float) fb);
#if MOTOR_CONTROL_ENABLE
	Motor_SetTargetFreq((int32_t) f_out);
#else
	(void) f_out; /* 总开关关闭：PID 照算便于观察积分/目标，但不驱动电机 */
#endif
}

/**
 * @brief Z 相事件与掉电保存服务（5Hz 主循环）
 * @note  Flash 擦除/编程期间 CPU 从 Flash 取指停顿约 20~40ms，会打断 50Hz
 *        控制环，因此只在模式 0（控制已停、电机休眠）且摆杆静止时才落盘。
 *        运行中直接掉电时保存值滞后到最后一次停机位置——这是"断电后杆保持
 *        不动"前提下的取舍。
 */
static void Ctrl_EncService(void) {
	int32_t abs;

	(void) Encoder_Read();	   /* 模式 0 时内环 ISR 不读编码器，由这里刷新计数 */
	(void) Encoder_ProcessZ(); /* Z 相：绝对计数对齐到最近整圈 */

	if (s_ctrl_mode != 0u) {
		s_store_abs_ref = Encoder_ReadAbs(); /* 控制中只跟踪，不擦 Flash */
		return;
	}

	abs = Encoder_ReadAbs();
	if (abs != s_store_abs_ref) {
		s_store_abs_ref = abs; /* 仍在动：等静止后再存 */
		return;
	}
	if (Encoder_StorePending()) {
		(void) Encoder_StoreSave();
	}
}

/**
 * @brief 20Hz VOFA FireWater 帧（通道定义见本文件 PD 枚举）
 * @note  只送实时流变量；模式/急停/绝对位置/Z 计数/使能等低频状态量走 OLED
 */
static void Ctrl_SendTelemetry(void) {
	int32_t hold_tgt		  = (s_ctrl_mode == 2u) ? (int32_t) s_ball_ofs_cnt : 0;
	int32_t vofa[VOFA_CH_NUM] = {
		[VOFA_CH_ENC_REL]		= Encoder_Read(),
		[VOFA_CH_HOLD_TARGET]	= hold_tgt,
		[VOFA_CH_HOLD_INTEGRAL] = (int32_t) (s_hold_pid.integral * 1000.0f),
		[VOFA_CH_PULSE_FREQ]	= Motor_GetPulseFreqHz(),
		[VOFA_CH_POS_CM_X10]	= (int32_t) g_tof.pos_cm_x10,
		[VOFA_CH_BALL_OFS]		= (int32_t) s_ball_ofs_cnt,
	};
	VOFA_SendFrame(vofa, (uint8_t) VOFA_CH_NUM);
}

/**
 * @brief 消费按键事件（非阻塞）：PRESS 急停锁存，再按一次解除并恢复驱动使能
 * @note  锁存后仍为模式 0；再次按下只解除锁存并恢复驱动使能，不自动回到
 *        模式 1（回到模式 1 需重新上电）；松手不自动恢复
 */
static void Ctrl_PollKeyEvents(void) {
	KeyEvent_t ke;

	while ((ke = Key_PollEvent()) != KEY_EVT_NONE) {
		if (ke == KEY_EVT_PRESS) {
			if (s_key_estop == 0u) {
				s_key_estop = 1u;
				Ctrl_SetMode(0);
			} else {
				s_key_estop = 0u;
#if MOTOR_CONTROL_ENABLE
				Motor_SetEnabled(true); /* 恢复力矩，杆锁在当前角度 */
#endif
			}
		}
	}
}

/** @brief R 命令：复位内外环积分与目标偏移（2 个调用点：模式切换、R 命令） */
static void Ctrl_ResetLoops(void) {
	PID_Reset(&s_hold_pid);
	PID_Reset(&s_ball_pid);
	s_ball_ofs_cnt = 0;
	s_tof_lost_n   = 0;
}

/* ===================== 应用层：上位机行协议 ===============================
 * A<milli> : 设置 EMA 系数（20~1000，1000=直通）
 * Z        : 把当前滤波值设为零点；Z<mm> 显式设置零点距离
 * P/I/D<v> : 外环增益千分比在线整定（0~10000，如 P2500 = Kp 2.500）
 * R        : 清除滤波器窗口/EMA 与内外环积分（参数与零点保留）
 *
 * 上位机只做参数整定；控制模式不开放串口切换，由板载按键与
 * PLATFORM_HOLD_ON_BOOT（上电进入模式 1）决定。
 */

/**
 * @brief 解析并执行一条命令（未知命令/EMI 噪声静默丢弃）
 * @param line 以 '\0' 结尾的单行文本（已去掉行尾）
 */
static void Cmd_ParseLine(const char *line) {
	char  key;
	char *end;
	long  val	  = 0;
	bool  has_val = false;

	while (*line == ' ' || *line == '\t') {
		line++;
	}
	if (*line == '\0') {
		return;
	}
	/* 大小写归一：'a'~'z' 统一转大写，下面每个命令只写一遍分支 */
	key = ((*line >= 'a') && (*line <= 'z')) ? (char) (*line - ('a' - 'A')) : *line;

	if (line[1] != '\0') {
		val		= strtol(line + 1, &end, 10);
		has_val = (end != (line + 1));
	}

	switch (key) {
	case 'A':
		if (has_val && val >= (long) TOF_FILTER_ALPHA_MIN_MILLI && val <= (long) TOF_FILTER_ALPHA_MAX_MILLI) {
			TofFilter_SetAlpha(&s_tof_filter, (uint16_t) val);
		}
		break;
	case 'Z':
		if (has_val) {
			if (val >= 0 && val <= (long) TOF_FILTER_MAX_VALID_MM) {
				TofFilter_SetZeroMm(&s_tof_filter, (int16_t) val);
			}
		} else {
			(void) TofFilter_ZeroHere(&s_tof_filter);
		}
		break;
	case 'P':
	case 'I':
	case 'D':
		/* 三者格式一致：千分比 → 实数，只改对应单个增益 */
		if (has_val && val >= 0 && val <= 10000) {
			PIDGain_t gain = (key == 'P') ? PID_GAIN_KP : ((key == 'I') ? PID_GAIN_KI : PID_GAIN_KD);

			PID_SetOneGain(&s_ball_pid, gain, (float) val / 1000.0f);
		}
		break;
	case 'R':
		TofFilter_Reset(&s_tof_filter);
		Tof_ResetStats();
		Ctrl_ResetLoops();
		break;
	default:
		break; /* 未知命令/EMI 噪声静默丢弃 */
	}
}

/**
 * @brief 10Hz 排空串口行队列（先自愈 RX，再逐行解析）
 * @note  非阻塞；一次可处理队列中多条命令
 */
static void Cmd_Poll(void) {
	char line[UART1_LINE_MAX];

	USART1_EnsureRx(); /* RX 停摆自愈 */
	while (USART1_GetLine(line, (uint8_t) sizeof line)) {
		Cmd_ParseLine(line);
	}
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

	if ((tick_ms % CTRL_PERIOD_MS) == 0) {
		Ctrl_InnerTick();
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
