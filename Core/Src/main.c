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
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmd.h" /* 上位机行协议：A/Z/P/I/D/R */
/* ctrl.c 已禁用，用户自行在 main.c 编写控制逻辑 */
#include "encoder.h" /* 编码器计数/Z 校准/掉电保存 */
#include "key.h"	 /* PA0 按键消抖与事件队列 */
#include "motor.h"	 /* 电机控制（含 g_mot_en） */
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
#define Inner_PERIOD_MS		20u					  /* 内环：编码器位置 PID，50Hz */
#define Outter_PERIOD_MS	(Inner_PERIOD_MS * 5) /* 外环：位置→内环位置目标，100ms */
#define CMD_PERIOD_MS  100u					 /* 串口命令：10Hz（主循环执行） */
#define OLED_PERIOD_MS		200u				  /* OLED 页面内容更新：5Hz（主循环执行） */
#define OLED_TICK_PERIOD_MS 25u					  /* OLED 逐页回写：25ms 一页，8 页整屏 200ms */
#define VOFA_PERIOD_MS	 Inner_PERIOD_MS	   /* VOFA 20ms 1次 */
#define TOF_PERIOD_MS		20					  /* TOF 20ms 1次 */
#define ENCODER_PERIOD_MS	20					  /* 编码器 50ms 1次 */
#define ENC_TICK_PERIOD_MS	20					  /* 编码器 50ms 1次 */
#define TOF_TICK_PERIOD_MS	20					  /* TOF 20ms 1次 */
/* 单环位置 PID：小球位置(pos_cm_x10) → 脉冲频率(Hz)，每个测距拍推进一次。
 * 方向开关（唯一）：1 = 正相关（pos>0 时 freq>0），0 = 负相关。
 * Target=0 时 Out = Kp×(0−pos)，故正相关对应 Kp 取负号。 */
#define BALL_POS_HZ_SIGN	  1
#define BALL_POS_HZ_DIR		  (BALL_POS_HZ_SIGN ? -1.0f : 1.0f) // 正相关（pos>0 时 freq>0）
#define BALL_KP_HZ_PER_CM_X10 4.0f								/* Hz / 0.1cm 偏差（pos=30 即 3cm 时 120Hz） */
/* 外环积分偏置：吃零点误差与"维持角"的常值偏差，位置相关的部分由它在每个停留点自己学。
 * 必须加在制动包络外面 —— |e|=48mm 时包络已把 Out 夹到 44 counts，积分加在里面会被一起夹掉。
 * 增益取自串口第 2 个参数（千分比）：0 = 关闭（上电默认），10 = 0.01。
 * 积分时间 = Kp/Ki = 2/0.01 = 200 拍 = 20s，远离 4.5s 的极限环周期。
 * 限幅 80 counts：够补实测最大维持角偏差（85），又不至于把杆带偏。 */
#define BALL_I_BIAS_MAX		  80.0f /* 积分偏置默认上限（counts）；可经串口第 8 参数在线改 */
/* 条件积分（2026-11-07）：只在球"卡住"时累加，球一动就冻结，避免把动量积成过冲。
 * 卡住判据不能直接用 kf_vel：静止噪声 σ≈8.5mm/s，而慢速爬行只有 ~8mm/s，
 * 两者重叠。故先做一阶低通（外环 100ms，α=0.2 → τ≈0.45s，噪声降到 ~3mm/s），
 * 低通后仍低于 5mm/s 才算静止。误差死区避免在目标附近继续堆偏置。 */
#define BALL_I_STUCK_MM_S	  5.0f /* 低通后速度阈值（mm/s）：低于此值视为卡住 */
#define BALL_I_DEADBAND		  2.0f /* 误差死区（mm）：|e| 小于此值不再积分 */
/* 运动泄放：球真正滚起来后，积分偏置按 τ≈1.4s 向 0 泄放。
 * 静摩擦突破所需的偏置对动摩擦来说偏大，不泄放会带着它冲过头
 * （实测 Ki=0.03/cap=60 阶跃过冲 +35mm）。泄放只在高速度段生效，
 * 5~15mm/s 的慢爬行保持偏置，避免"泄放→停住→再积"的小极限环。 */
#define BALL_I_MOVE_MM_S	  15.0f /* 泄放起始速度（mm/s） */
#define BALL_I_LEAK			  0.93f /* 每拍（100ms）泄放系数：τ≈1.4s */
#define MOTOR_CONTROL_ENABLE  1
#define ZERO_THETA			  680
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ---- 串口 RX 解析结果（文件级，主循环读写） ---- */
#define RX_VAL_MAX                                                                                                         \
	8u										/* 一行最多解析 8 个逗号分隔数值：                                             \
											 * Kp,Ki,Kd,brake_gain,brake_min,src,target,bias_max                           \
											 * src: 0/省略 = KF 输出（默认），1 = EMA 输出（仅对照实验） \
											 * target: 外环目标位置，单位 0.1cm（正 = 球靠近传感器）     \
											 * bias_max: 积分偏置限幅（counts），省略则保持当前值 */
static float			rx_val[RX_VAL_MAX]; /* 解析出的数值（支持小数） */
static uint8_t			rx_cnt	 = 0;		/* 本行实际解析出的个数（0~RX_VAL_MAX） */
static volatile uint8_t rx_valid = 0;		/* 1 = 收到新参数行，等主循环接管写入一次 */

/* ---- 调度标志（TIM1 ISR 置位，主循环消费） ---- */
static volatile uint8_t inner_tick	= 0; /* 内环：编码器 PID + TOF 20ms */
static volatile uint8_t	 outter_tick	= 0; /* 外环：位置 PID → 内环位置目标 100ms */
static volatile uint8_t cmd_tick	= 0; /* 串口命令解析 100ms */
static volatile uint8_t	 oled_tick		= 0; /* OLED 页面内容更新 200ms */
static volatile uint8_t	 oled_page_tick = 0; /* OLED 逐页回写 25ms */
static volatile uint8_t vofa_tick	= 0; /* VOFA 20ms 1次 */
static volatile uint32_t tof_tick		= 0;
static volatile uint32_t enc_tick		= 0;
/* 主循环心跳：1 = 最近 1s 内主循环至少跑过一圈；ISR 据此决定是否喂狗 */
static volatile uint8_t loop_alive = 0;
bool					runstate   = 1; /* 默认上电即运行，按键单击翻转为停止 */
/* ---- 外环积分偏置（挂在制动包络外，见 BALL_I_BIAS_MAX 的说明） ----
 * 不用 ball_pid.Ki：PID_Update 内的 I 项会和 P+D 一起被包络夹掉，
 * 而这里要的正是"包络夹不住的常值角度"。 */
static float s_ball_ki = 0.0f; /* 积分增益（0 = 关闭；串口第 2 参数，千分比） */
static float s_i_bias  = 0.0f; /* 积分偏置（counts），直接叠加到内环目标 */
/* 卡住判据用的低通速度（mm/s）：外环 100ms 更新，α=0.2 → τ≈0.45s */
static float s_vel_slow = 0.0f;
/* 偏置限幅（counts）：限的是 Ki·Σe 这个"积分输出"本身，不是 Σe。
 * 好处是它的物理含义固定——最多能补多少 counts 的杆角，跟 Ki 取多少无关。 */
static float s_i_bias_max = BALL_I_BIAS_MAX;
/* ---- 位置外环 PID（ToF 反馈 → 内环位置目标 counts） ----
 * 上电默认值 = 2026-11-06 实测最优（见 .trae/documents/摆杆闭环调参与KF实测记录.md 第六章）：
 * 收敛 5s、过冲 14~29mm、稳态 σ 1.4~2.9mm。上位机不发参数时也必须能稳住，
 * 故这里直接写最优值，不再保留旧的 Kp=5/Kd=40/brake=4 整定起点。 */
PID_t ball_pid = {
	.Kp			= 2.0f,
	.Ki			= 0.0f,
	.Kd			= 15.0f,
	.OutMin		= -200.0f,
	.OutMax		= 200.0f,
	/* 积分限幅：0 表示不限幅（无限累积 → 积分饱和）。摩擦死区约 15 counts，
	 * 稳态输出全由积分提供时 ErrorInt ≈ 15/Ki（Ki=0.1 → 150），故留 2 倍裕量 */
	.i_limit	= 300.0f,
	.Target		= 0.0f,
	.brake_gain = 20.0f, /* 包络增益上限：25 卡死（偏差 47mm）、35/50 极限环 */
	.brake_min	= 0.5f,
	.Out		= 200.0f,
	.use_vel_d	= 1, /* D 项改用 KF 速度输入，不再对位置差分（见 PID_t 的 Vel 说明） */
};
/* ---- 位置内环 PID（编码器反馈 → 步进频率 Hz） ---- */
PID_t encoder_pid = {
	.Kp		 = 11.0f,
	.Ki		 = 0.0f,
	.Kd		 = 2.0f,
	.OutMin	 = -10000.0f, /* 频率限幅 ±300Hz（D36A 启动频率内，防丢步） */
	.OutMax	 = 10000.0f,
	.i_limit = 10000.0f,
	.Target	 = 0.0f,
};
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
  MX_DMA_Init();
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
   * 外环位置（ToF pos_cm_x10）→ 内环位置目标（counts）
   * 内环位置（编码器 counts）→ 步进频率（Hz）
   * 方向符号由 BALL_POS_HZ_DIR 统一决定 */

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

    /* USER CODE BEGIN 3 */
	  /* 串口参数接管：仅在收到新参数行时写入一次，禁止每圈无条件覆盖 */
	  if (rx_valid) {
		  rx_valid = 0;
		  if (rx_cnt > 0) ball_pid.Kp = rx_val[0];
		  /* Ki 用千分比：10 = 0.01。不写 ball_pid.Ki —— PID_Update 内的 I 项会和
		   * P+D 一起被制动包络夹掉，这里改由包络外的 s_i_bias 实现。
		   * Ki=0 时同时清掉已积累的偏置，方便做"开/关积分"的 A/B 对照 */
		  if (rx_cnt > 1) {
			  s_ball_ki = rx_val[1] * 0.001f;
			  if (s_ball_ki == 0.0f) {
				  s_i_bias = 0.0f;
			  }
		  }
		  if (rx_cnt > 2) ball_pid.Kd = rx_val[2];
		  if (rx_cnt > 3) ball_pid.brake_gain = rx_val[3];
		  /* brake_min 用千分比：500 = 0.5 counts。其余参数全整数解析，
		   * 以避开 strtof（会拉入约 18KB 浮点库，Flash 容量不够） */
		  if (rx_cnt > 4) ball_pid.brake_min = rx_val[4] * 0.001f;
		  /* 第 6 个参数：输出源切换，0/省略 = KF（默认），1 = EMA（对照实验） */
		  if (rx_cnt > 5) TofFilter_SetOutputEma(&g_tof_filter, rx_val[5] > 0.5f);
		  /* 第 7 个参数：外环目标（0.1cm）。只在收到命令时写一次，不再每周期覆盖。
		   * 符号链：pos = filt − zero，Actual = −pos，故稳态 pos = −Target，
		   * 即 filt = zero − Target。限幅直接按"球到传感器的距离"设定，
		   * 不再按 Target 的正负写上下限（那样会把方向搞反） */
		  if (rx_cnt > 6) {
			  float filt = (float) g_tof_filter.zero_mm - rx_val[6];

			  if (filt > 470.0f) {
				  filt = 470.0f; /* 远端上限：管长 500mm，球心最远约 470mm；
								  * 传感器实测 0.9 MCPS 下有效到 421 以上 */
			  } else if (filt < 21.0f) {
				  filt = 21.0f; /* 近端下限：避开传感器盲区 */
			  }
			  ball_pid.Target = (float) g_tof_filter.zero_mm - filt;
		  }
		  /* 第 8 个参数：积分偏置限幅（counts）。夹到 [10, 400]：
		   * 下限避免"限幅 0 = 积分失效"，上限避免大偏置把杆带飞。
		   * 改小限幅时立即把已积累的偏置缩回范围内 */
		  if (rx_cnt > 7) {
			  float m = rx_val[7];

			  if (m < 10.0f) {
				  m = 10.0f;
			  } else if (m > 400.0f) {
				  m = 400.0f;
			  }
			  s_i_bias_max = m;
			  if (s_i_bias > s_i_bias_max) {
				  s_i_bias = s_i_bias_max;
			  } else if (s_i_bias < -s_i_bias_max) {
				  s_i_bias = -s_i_bias_max;
			  }
		  }
	  }
	  if (tof_tick) {
		  tof_tick = 0;
		  Tof_Tick(&g_tof_filter);
	  }
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

	  /* 外环（100ms）：位置 PID → 内环位置目标 */

	  /* 编码器采样只在 TIM1 ISR 里做（20ms 一次，见 HAL_TIM_PeriodElapsedCallback）：
	   * Encoder_Tick 读 CNT→算差分→写 s_enc_last 不是原子操作，主循环会被 ToF 的
	   * I2C 阻塞拉长，若再在这里调一次，中断插进读写之间就会重复计数 */

	  /* VOFA 遥测（11 通道，顺序须与 assets/channels.json 一致）：
	   * ch0 enc_target  ch1 enc_actual  ch2 enc_out_hz  ch3 ball_target
	   * ch4 raw_mm      ch5 ema_mm      ch6 kf_pos      ch7 kf_vel
	   * ch8 kf_state    ch9 ball_out    ch10 range_status
	   * kf_state 位域：bit0=bad_streak≥3，bit1~4=bad_streak(0~15)，bit5~7=kf_miss(0~7)
	   * range_status：0=有效；>0 为 VL53L1X RangeStatus（1 sigma 超限 / 2 信号弱 /
	   *   7 绕回失败 等）；-1 等数据超时 / -2 读取失败 / -3 滤波器量程拒绝 */
	  if (vofa_tick) {
		  vofa_tick		   = 0;
		  int32_t vofa[11] = {(int32_t) encoder_pid.Target, (int32_t) encoder_pid.Actual, (int32_t) encoder_pid.Out,
							  (int32_t) ball_pid.Target,	   (int32_t) g_tof.raw_mm,		  (int32_t) g_tof.ema_mm,
							  (int32_t) g_tof.filt_mm,	   (int32_t) g_tof.kf_vel,		  (int32_t) g_tof.kf_state,
							  (int32_t) ball_pid.Out,	   (int32_t) g_tof.range_status};
		  VOFA_SendFrame(vofa, 11);
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
				  long v = strtol(p, &end, 10);
				  if (end == p) {
					  break;
				  }
				  rx_val[rx_cnt++] = (float) v;
				  p				   = end;
				  if (*p == ',') {
					  p++;
					  continue;
				  }
				  break;
			  }
			  uart1_last_ready = 0;
			  rx_valid		   = 1; /* 新参数行，待主循环接管写入一次 */
		  }
	  }

	  /* 5Hz：刷运行时页面 */
	  if (oled_tick) {
		  oled_tick = 0;

		  int16_t pos	  = g_tof.pos_cm_x10;
		  char	  sign	  = (pos < 0) ? '-' : ' ';
		  int32_t abs_pos = (pos < 0) ? -(int32_t) pos : (int32_t) pos;
		  uint8_t mot_en  = 0u;
		  char	  rx_str[64];
		  char	 *p = rx_str;
#if MOTOR_CONTROL_ENABLE
			mot_en = 1u;
#endif

			for (uint8_t i = 0; i < rx_cnt && i < RX_VAL_MAX; i++) {
				if (i && p < rx_str + sizeof(rx_str) - 1) *p++ = ',';
				p += snprintf(p, sizeof(rx_str) - (size_t) (p - rx_str), "%d", (int) rx_val[i]);
				if (p > rx_str + sizeof(rx_str) - 1) {
					p = rx_str + sizeof(rx_str) - 1; /* 截断时 snprintf 返回"本应长度"，会越界 */
				}
			}
			*p = '\0';

			OLED_ClearArea(0, 0, OLED_WIDTH, 64);
			OLED_Print(0, 0,  "RAW:%4d mm", (int) g_tof.raw_mm);
			OLED_Print(0, 8,  "POS:%c%ld.%ld cm", sign, (long) (abs_pos / 10), (long) (abs_pos % 10));
			OLED_Print(0, 16, "VAL:%3lu%%", (unsigned long) g_tof.valid_pct);
			OLED_Print(0, 24, "ENC:%6ld", (long) Encoder_Read());
			OLED_Print(0, 32, "M:%u R:%u", (unsigned) mot_en, (unsigned) runstate);
			OLED_Print(0, 48, "RX:%d %s", (int) rx_cnt, rx_str);
			/* TOFSTATE = g_tof.range_status（VL53L1X RangeStatus，见 vl53l1_def.h）：
			 *   0  有效
			 *   1  Sigma 超限          2  信号过弱
			 *   3  低于最小量程        4  相位越界
			 *   5  硬件自检失败        6  有效但未做绕回校验
			 *   7  绕回目标失败        8  内部算法溢出
			 *   9  串扰信号失败        10 背靠背首中断（丢弃）
			 *   11 多脉冲合并          12 有目标但信号不足
			 *   13 低于最小量程/ROI 裁剪
			 *   14 距离无效（负值）
			 *   -1 等数据就绪超时  -2 读取失败  -3 越出 [8,500]mm 量程门 */
			OLED_Print(0, 56, "TOFSTATE:%6ld", (long) g_tof.range_status);
	  }

	  /* 25ms：逐页回写一页，8 页整屏 200ms，单次 I2C 阻塞仅为整屏的 1/8 */
	  if (oled_page_tick) {
		  oled_page_tick = 0;
		  OLED_Tick();
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
	/* 编码器 50ms 1次 */
	if ((tick_ms % TOF_PERIOD_MS) == 0) {
		tof_tick = 1;
	}
	if ((tick_ms % ENCODER_PERIOD_MS) == 0) {
		Encoder_Tick();
	}

	if (++tick_ms >= 1000u) {
		tick_ms = 0;
		if (++ctrl_tick >= 1) {
			ctrl_tick = 0;
			// ball_pid.Out = -ball_pid.Out;
		}
	}
	if (tick_ms % Outter_PERIOD_MS == 0) {
		/* Target 由串口命令设定（第 7 个参数），此处不再周期覆盖；
		 * pos_cm_x10 取负号后作为反馈，故 Target>0 表示球向传感器方向移动 */
		float vel_kf;

		/* 外推位置：用 KF 速度把上一帧位置补到当前时刻，消除 0~34ms 采样滞后。
		 * 限幅与回退判断都在 TofFilter_GetCmX10At 内部，ISR 里只做一次调用 */
		ball_pid.Actual = -(float) TofFilter_GetCmX10At(&g_tof_filter, HAL_GetTick());

		/* D 项唯一速度源：Actual = -(pos)，故 de/dt = +kf_vel；外环 100ms，
		 * 每拍误差增量 = 0.1 × kf_vel（与 PID 内部差分同量纲）。
		 * KF 未就绪或 EMA 对照时无速度估计，D 置 0。 */
		vel_kf = KF_GetVel(&g_tof_filter.kf);
		if (!g_tof_filter.kf_ready || g_tof_filter.out_use_ema) {
			vel_kf = 0.0f;
		}
		ball_pid.Vel = 0.1f * vel_kf;
		/* 卡住判据：低通速度。慢爬行 ~8mm/s 必须算运动，不能被静止噪声掩盖 */
		s_vel_slow += 0.2f * (vel_kf - s_vel_slow);

		PID_Update(&ball_pid);
		/* 外环条件积分：运行、数据未过期、输出未饱和，且球处于"卡住"状态才累加。
		 * 数据过期时滤波器输出的是冻结值，P/D 已经恒定、杆自动保持；
		 * 唯一会继续累积的是积分，所以这里只挡积分，不外溢整套冻结逻辑。
		 * 球一动就停止积分，高速段还会按 τ≈1.4s 泄放，避免把动量积成过冲——
		 * 这是之前 Ki=0.02/cap=300 弛张振荡的直接原因。 */
		if (runstate && !TofFilter_IsStale(&g_tof_filter) &&
			ball_pid.Out > ball_pid.OutMin + 1.0f && ball_pid.Out < ball_pid.OutMax - 1.0f) {
			if (s_vel_slow < BALL_I_STUCK_MM_S && s_vel_slow > -BALL_I_STUCK_MM_S &&
				(ball_pid.Error0 > BALL_I_DEADBAND || ball_pid.Error0 < -BALL_I_DEADBAND)) {
				s_i_bias += s_ball_ki * ball_pid.Error0;
				if (s_i_bias > s_i_bias_max) {
					s_i_bias = s_i_bias_max;
				} else if (s_i_bias < -s_i_bias_max) {
					s_i_bias = -s_i_bias_max;
				}
			} else if (s_vel_slow > BALL_I_MOVE_MM_S || s_vel_slow < -BALL_I_MOVE_MM_S) {
				s_i_bias *= BALL_I_LEAK; /* 运动中泄放突破静摩擦时多积的偏置 */
			}
		}
		encoder_pid.Target = ball_pid.Out + s_i_bias + ZERO_THETA;
	}
	if ((tick_ms % Inner_PERIOD_MS) == 0) {
		inner_tick		   = 1;
		encoder_pid.Actual = (float) g_enc.total;
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
	if ((tick_ms % OLED_TICK_PERIOD_MS) == 0) {
		oled_page_tick = 1;
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
