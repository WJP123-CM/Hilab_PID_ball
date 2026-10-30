/**
 ******************************************************************************
 * @file    encoder.h
 * @brief   编码器（MS42CG 增量式）计数、绝对位置与掉电保存
 *
 * 硬件（CubeMX 已配置）：
 *   A/B 相 PA6/PA7 = TIM3_CH1/CH2（TI12 四倍频 + 输入滤波 15，硬件级毛刺抑制）
 *   Z   相 PB0     = TIM3_CH3（输入捕获中断，每圈一个脉冲）
 *
 * 本模块是该硬件的"基本处理"收口：16 位 CNT 回绕累计、方向修正、相对清零、
 * 绝对计数 + Z 整圈校准、Flash 末页掉电保存，以及绝对位置软限幅的区间常量。
 * 电机脉冲/方向/使能在 motor.c；本模块不碰 TIM2 与电机 GPIO。
 *
 * 方向符号链（改任一处必须核对全部四处，否则软限幅判向反、内外环成正反馈）：
 *   ENC_DIR_SIGN     : A/B 原始增量 → 计数增量的符号（2026-10-30 统一为 +1 负相关）
 *   ENC_POS_DIR_SIGN : 绝对计数增大 ↔ 摆杆哪一侧（motor.c 软限幅判向用，须与
 *                      ENC_DIR_SIGN 取反使 freq>0 时 pos_dir 指向 enc 实际运动方向）
 *   HOLD_PID_SIGN    : 内环 PID 输出 → 频率符号（main.c Ctrl_InnerTick，须使
 *                      target>fb 时 enc 朝 target 运动）
 *   BALL_PID_SIGN    : 外环输出 → 编码器目标偏移符号（main.c，须使 pos>0 时
 *                      s_ball_ofs_cnt 驱动 enc 朝使 pos 减小的方向运动）
 ******************************************************************************
 */
#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* 编码器线数（按实物铭牌改）；TIM3 TI12 四倍频 → 计数/圈 = 4×线数 */
#define ENC_PPR	1000u
#define ENC_CPR (ENC_PPR * 4u)

/* 计数方向修正：+1 = 计数增量与 Motor_SetTargetFreq 正号同向；
 *                         -1 = 计数增量与 freq 反向（负相关）。
 * 2026-10-30 统一改为负相关：freq>0 → Encoder_Read 返回值减小。
 * 依据：实测 freq>0 时原始 A/B CNT 减小（A/B 相接反或 DIR 极性相反），
 *       取 +1 后 d = +1 × (负数) = 负数 → enc 减小，与 freq 负相关。
 * 本宏统一作用于相对计数与绝对计数；改回接线修正后可恢复 -1。 */
#define ENC_DIR_SIGN (+1)

/* ==================== 绝对位置软限幅区间 ==============================
 * 只在电机"朝超出限位区间外"的方向运动时切断 PWM；返回区间内的方向始终
 * 放行，避免顶死在限位后无法退回。判向逻辑在 Motor_SetTargetFreq。
 * 注意：依赖 ENC_DIR_SIGN 与 ENC_POS_DIR_SIGN 一致——若 enc 符号反了，
 * 越界判断方向随之反，限幅会完全失效（已实测过）。 */
#define ENC_POS_LIM_MIN	  4400	 /* 允许的最小绝对计数 */
#define ENC_POS_LIM_MAX	  4800	 /* 允许的最大绝对计数 */
/* 2026-10-30：ENC_DIR_SIGN=+1 使 freq>0 → enc↓（绝对计数减小），
 * 故 freq>0 时 pos_dir 须为负（指向 MIN 方向）才能正确限幅。
 * 与 ENC_DIR_SIGN 取反：ENC_DIR_SIGN=+1 → ENC_POS_DIR_SIGN=-1。 */
#define ENC_POS_DIR_SIGN  (-1) /* 绝对计数增大 ↔ 正转方向 */

/* ==================== 掉电保存（Flash 末页） ==========================
 * 程序用前 63 页，最后 1 页（0x0800FC00，链接脚本已减为 63K）专存绝对计数。
 * 前提：断电期间摆杆不动（有自锁/配重）。Z 相每圈一个脉冲，用于把绝对计数
 * 对齐到整圈零点。 */
#define ENC_STORE_ADDR	0x0800FC00u
#define ENC_STORE_MAGIC 0x454E4350u /* 'ENCP' */

/**
 * @brief 启动 TIM3 编码器接口与 Z 相捕获中断，并以当前 CNT 为累计零点
 * @note  须在 MX_TIM3_Init 之后调用一次；不使能电机、不输出脉冲
 * @note  绝对计数先置 0，真实值由 Encoder_StoreRestore 恢复
 */
void Encoder_Init(void);

/**
 * @brief 读取相对累计位置（不随掉电保存变化）
 * @retval 32 位累计计数（已处理 16 位 CNT 回绕与 ENC_DIR_SIGN 方向修正）
 * @note  可在 50Hz ISR 与主循环交替调用：内部临界区保证差分不重算；
 *        要求两次调用间隔内计数变化 < 32768（50Hz 下转速上限远超工况）
 * @note  位置变化时置"待落盘"标志，ISR 安全
 */
int32_t Encoder_Read(void);

/**
 * @brief 相对计数清零：当前时刻为新的 0 点（对位基准/测试归零）
 * @note  只清相对计数，不影响掉电保存使用的绝对计数
 */
void Encoder_Zero(void);

/**
 * @brief 读取绝对累计计数（不随 Encoder_Zero 清零，上电由 Flash 恢复）
 * @retval 绝对计数（32 位读，单条指令原子）
 */
int32_t Encoder_ReadAbs(void);

/**
 * @brief 已捕获的 Z 相脉冲总数（诊断用，上电从 0 开始）
 */
uint32_t Encoder_GetZCount(void);

/**
 * @brief Z 相捕获事件登记——仅由 TIM3 捕获 ISR 调用
 * @note  ISR 内只置标志/计数，不做对齐计算、不写 Flash
 */
void Encoder_ZIndexEvent(void);

/**
 * @brief 处理待定 Z 事件，把绝对计数对齐到最近整圈
 * @retval true = 本次发生校准；false = 无待处理事件
 * @note  仅主循环调用（5Hz Enc_Service）
 */
bool Encoder_ProcessZ(void);

/**
 * @brief 标记绝对位置需落盘（ISR 安全，仅置标志）
 */
void Encoder_StoreMarkDirty(void);

/**
 * @brief 查询是否有待落盘的位置（主循环用）
 */
bool Encoder_StorePending(void);

/**
 * @brief 上电从 Flash 恢复绝对计数
 * @retval true = magic 与校验和均有效；false = 未写过/已擦除/校验失败（计数保持 0）
 * @note  须在 Encoder_Init 之后、控制环启动之前调用一次
 */
bool Encoder_StoreRestore(void);

/**
 * @brief 把绝对计数写入 Flash 末页
 * @retval true = 擦除与编程均成功；false = 失败（不重试，避免反复擦写阻塞）
 * @note  仅主循环调用，且只在模式 0（控制已停、电机休眠）时执行：
 *        擦写期间 CPU 从 Flash 取指停顿约 20~40ms，会打断 50Hz 控制环
 */
bool Encoder_StoreSave(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */
