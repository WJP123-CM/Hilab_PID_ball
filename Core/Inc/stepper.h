/**
 ******************************************************************************
 * @file    stepper.h
 * @brief   步进电机驱动（D36A 驱动板 + MS42CG 电机，带符号频率控制）
 *
 * 硬件接线（CubeMX 已配置）：
 *   - STEP 脉冲：PA3 = TIM2_CH4（PSC=71 → 计数时钟 1MHz，Period 仅作上电初值）
 *   - DIR  方向：PA2（Motor_direction_Pin，推挽输出）
 *   - EN   使能：PA1（Motor_enabled_Pin，推挽输出，D36A 高电平使能）
 *
 * D36A 控制时序（参考 WHEELTEC 官方 C8T6+D36A+MS42 例程）：
 *   - EN：高电平使能，低电平休眠（上电默认休眠，安全）
 *   - DIR：电平保持定向；切换方向后须 ≥1ms 再发 STEP 脉冲
 *          （本模块换向拍只切 DIR 不发脉冲，由 StepperSm 决策保证）
 *   - STEP：每个上升沿走一个细分步；D36A 出厂 16 细分，
 *           200 步/圈 × 16 = 3200 脉冲/圈（细分改变时须同步修改宏）
 *
 * 控制模型：Stepper_SetTargetFreq() 带符号目标频率（正=正转/负=反转/0=停），
 * 内部完成限幅、斜率限制、安全换向。核心决策 StepperSm_Update 为纯软件逻辑
 * （无硬件依赖），main.c 上电自检直接覆盖；频率换算：1kHz ≈ 18.75rpm
 ******************************************************************************
 */
#ifndef __STEPPER_H__
#define __STEPPER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h" /* HAL 类型、Motor_enabled/Motor_direction 引脚标签 */

#include <stdbool.h>
#include <stdint.h>

/* ============================ 参数宏 ====================================== */

/* D36A 出厂 16 细分：200 步/圈 × 16 = 3200 STEP 脉冲/圈；改 DIP 细分须同步改 */
#define STEPPER_PULSES_PER_REV 3200u

/* 编码器：MS42CG 标配 1000 线（按实物铭牌改），TIM3 TI12 模式 4 倍频
 * → 4000 计数/圈；与步进脉冲换算：1 STEP 脉冲 = 4000/3200 = 1.25 计数 */
#define STEPPER_ENC_PPR 1000u
#define STEPPER_ENC_CPR (STEPPER_ENC_PPR * 4u)

/* 频率限幅（Hz）：|freq| < MIN 视为停止（脉冲过疏只会引起抖动）；
 * MAX 按电机矩频特性约束（1kHz≈18.75rpm，2kHz≈37.5rpm） */
#define STEPPER_FREQ_MIN_HZ 30
#define STEPPER_FREQ_MAX_HZ 2000

/* 斜率限制：每拍频率变化量上限（Hz/拍）。50Hz 调用下 100Hz/拍 = 5kHz/s²，
 * 频率平滑渐变是防失步的核心——步进电机对频率阶跃敏感 */
#define STEPPER_SLEW_HZ_PER_STEP 100

/* 方向电平定义：CW/CCW 到 PA2 的实际电平。
 * 历史记录（2026-10-27）：多次对调后仍出现“固件未动、方向再次反转”的间歇现象，
 * 已怀疑 PA2 浮空 / 编码器 A/B 松动 / 电机相线接触不良，接线根因尚未闭环。
 * 当前工作区设置为 CW=1；H1 使能平台保持前必须先做小频率方向验证：
 * 若单向跑飞，先 H0，再查接线；不要只凭注释反复对调。
 * 注意：stepper.c 现在真正引用这两个宏，改值会改变 DIR 电平。 */
#define STEPPER_DIR_CW	1u /* 正转电平 */
#define STEPPER_DIR_CCW 0u /* 反转电平 */

/* TIM2 计数时钟分频校验值：PSC=71 → 1MHz（须与 CubeMX 中 TIM2.Prescaler 一致） */
#define STEPPER_TIM_PSC 71u

/* ==================== 核心决策状态机（纯软件，无硬件依赖） ================ */

typedef struct {
	int32_t cur_freq; /* 当前实际输出的带符号频率（Hz），0 = 脉冲停止 */
	bool	dir_cw;	  /* 当前 DIR 电平（true = 正转方向） */
} StepperSm_t;

typedef struct {
	int32_t out_freq;	 /* 本拍应输出的带符号频率（Hz） */
	bool	dir_cw;		 /* 本拍 DIR 电平 */
	bool	pulse_en;	 /* 本拍是否输出脉冲 */
	bool	dir_changed; /* 本拍发生换向（只建立 DIR 电平不发脉冲，
						  * 满足 D36A "换向后 ≥1ms 再发脉冲"时序） */
} StepperCmd_t;

/* 状态初始化（上电安全态：输出 0，方向默认正转） */
void StepperSm_Init(StepperSm_t *sm);

/* 核心决策：带符号目标频率 → 本拍输出决策。
 * 处理顺序：限幅(±MAX) → 死区(|f|<MIN 视为 0) → 斜率限制(每拍 ≤SLEW) →
 * 换向安全（方向不同时本拍输出 0、只切 DIR、dir_changed=true）。
 * 以固定周期调用（本工程 50Hz），斜率以"每拍最大变化量"表达 */
StepperCmd_t StepperSm_Update(StepperSm_t *sm, int32_t target_hz);

/* ============================ 驱动接口 ==================================== */

/* 初始化：校验 TIM2 分频配置、设定 STEP 占空比（约 50%）、启动编码器接口；
 * 不启动脉冲、不使能电机（上电安全态） */
void Stepper_Init(void);

/* 使能/休眠驱动板（高电平使能）；休眠时绕组断电，电机可手转 */
void Stepper_SetEnabled(bool enable);

/* 设定带符号目标频率（Hz），以固定周期调用（本工程 50Hz）：
 *   freq_hz > 0 → 正转；freq_hz < 0 → 反转；freq_hz = 0 → 停止脉冲
 * 脉冲启停、方向切换、ARR 更新均由本函数按 StepperSm 决策自动完成 */
void Stepper_SetTargetFreq(int32_t freq_hz);

/* 当前实际输出的带符号脉冲频率（Hz，0 = 停止）——经斜率限制后的真实输出，
 * 加减速/换向过程中与目标值不同，供显示与闭环观测 */
int32_t Stepper_GetPulseFreqHz(void);

/* 读取编码器累计位置（TIM3，32 位累计计数，内部已处理 16 位 CNT 回绕）；
 * 须在 Stepper_Init 后调用。方向约定：正转(CW)计数递增；
 * 若实测相反（闭环发散即为此现象），对调编码器 A/B 相接线或对调 DIR 宏 */
int32_t Stepper_ReadEncoder(void);

/* 编码器累计位置清零：当前时刻为新的 0 点（对位基准/测试归零用）。
 * 只清"相对"计数，不影响掉电保存使用的绝对计数 */
void Stepper_EncoderZero(void);

/* ==================== 绝对位置与掉电保存（Flash 末页） ====================
 * 掉电保存：程序用前 63 页，最后 1 页（0x0800FC00，链接脚本已减为 63K）
 * 专用于存绝对累计计数。前提：断电期间摆杆不动（有自锁/配重）。
 * Z 相（TIM3_CH3 / PB0）每圈一个脉冲，用于把绝对计数对齐到整圈零点。 */
#define STEPPER_ENC_STORE_ADDR	0x0800FC00u
#define STEPPER_ENC_STORE_MAGIC 0x454E4350u /* 'ENCP' */

/* 读绝对累计计数（不随 Stepper_EncoderZero 清零，上电由 Flash 恢复） */
int32_t Stepper_ReadEncoderAbs(void);

/* ==================== 编码器绝对位置软限幅 ==============================
 * 只在电机"朝超出限位区间外"的方向运动时切断 PWM；返回区间内的方向
 * 始终放行，避免顶死在限位后无法退回。
 * ENC_POS_DIR_SIGN：s_enc_abs 增大时对应 Stepper_SetTargetFreq 的符号。
 * 内部约定 freq>0 → enc 增大；依据是用户确认的"编码器示数与 ToF 测距示数
 * 正相关"，即 enc 增大同时 pos 增大，故该宏与 BALL_PID_SIGN 必同号。
 * 取 +1；仅当首测发现 freq>0 实际使 enc 减小时才改 -1。 */
#define ENC_POS_LIM_MIN	  4400   /* 允许的最小绝对计数 */
#define ENC_POS_LIM_MAX	  4800   /* 允许的最大绝对计数 */
#define ENC_POS_DIR_SIGN  (+1)

/* 已捕获的 Z 相脉冲总数（诊断用，上电从 0 开始） */
uint32_t Stepper_GetZCount(void);

/* Z 相捕获事件登记——仅由 TIM3 捕获 ISR 调用，只置标志不做事 */
void Stepper_ZIndexEvent(void);

/* 主循环调用：处理待定 Z 事件，把绝对计数对齐到最近整圈；返回是否校准 */
bool Stepper_ProcessZ(void);

/* 上电从 Flash 恢复绝对计数（magic+校验不符则返回 false，计数保持 0） */
bool Stepper_StoreRestore(void);

/* 标记位置有变化，需在主循环落盘（ISR 安全） */
void Stepper_StoreMarkDirty(void);

/* 是否有待落盘的位置（主循环用） */
bool Stepper_StorePending(void);

/* 把绝对计数写入 Flash 末页（仅主循环调用；擦写期间 CPU 取指停顿约 20ms，
 * 因此只在模式 0（控制已停、电机休眠）时执行） */
bool Stepper_StoreSave(void);

/* ==================== 编码器速度滤波（N 拍块差分 + 一阶低通对照） ========
 * 噪声模型（2026-10-27 开环定频采集实测）：低速段量化噪声主导（20ms 差分
 * 分辨率 ±40Hz 等效频率）；中高速段为真实转速纹波（开环步进力矩纹波，
 * std ≈ 7%×转速，正比于速度）。静止时数据精确为 0，无 EMI 幻计数。
 * 块差分：速度=(pos[n]−pos[n−N])×SCALE/N —— 量化噪声 ÷N、恒速零畸变、
 * 群延迟 (N−1)/2 拍（N=4 → 30ms）；一阶低通：y+=α(x−y)，延迟 ≈(1−α)/α 拍 */
#define STEPPER_SPDF_NMAX	8u
#define STEPPER_SPDF_SCALE	40 /* 计数差 → 等效脉冲Hz（50Hz 采样 × 3200/4000），
								* 须与 main.c 的 SPD_FB_SCALE 保持一致 */

typedef struct {
	int32_t pos[STEPPER_SPDF_NMAX]; /* 位置历史环形缓冲 */
	uint8_t n;		/* 有效窗口 N（1..NMAX） */
	uint8_t idx;	/* 最老样本下标 */
	uint8_t filled; /* 已填充样本数（未满窗返回 0） */
	float	lpf;	/* 一阶低通状态（等效Hz） */
	float	alpha;	/* 低通系数 (0,1] */
} StepperSpdFilt_t;

/* 初始化：n=窗口拍数（1=退化为原始差分），alpha=低通系数 */
void StepperSpdFilt_Init(StepperSpdFilt_t *f, uint8_t n, float alpha);

/* 每拍（20ms）喂入累计位置，返回 N 拍块差分速度（等效脉冲Hz，带符号） */
int32_t StepperSpdFilt_Update(StepperSpdFilt_t *f, int32_t enc_pos);

/* 每拍喂入原始速度（Hz），更新并返回一阶低通输出 */
float StepperSpdFilt_UpdateLpf(StepperSpdFilt_t *f, float raw_hz);

/* 复位：历史窗口与低通状态清零（对位归零/换段时用） */
void StepperSpdFilt_Reset(StepperSpdFilt_t *f);

#ifdef __cplusplus
}
#endif

#endif /* __STEPPER_H__ */
