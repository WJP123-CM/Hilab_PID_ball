/**
 ******************************************************************************
 * @file    encoder.c
 * @brief   编码器计数、绝对位置、Z 校准与掉电保存实现（详见 encoder.h）
 ******************************************************************************
 */
#include "encoder.h"

#include "main.h"					/* HAL 类型、CMSIS 临界区原语 */
#include "stm32f1xx_hal_flash_ex.h" /* Flash 页擦除（掉电位置保存） */
#include "tim.h"					/* htim3（编码器 + Z 捕获） */

/* ---- 编码器累计位置状态（TIM3 CNT 为 16 位，回绕由 Encoder_Read 差分累计） ---- */
static int32_t s_enc_total = 0;	  /* 32 位相对累计计数（Encoder_Zero 清零） */
static uint16_t s_enc_last = 0u;  /* 上一次读取的 CNT 原始值 */
static int32_t s_enc_abs = 0;	  /* 绝对累计计数（不随 Zero 清零，上电由 Flash 恢复） */
static volatile uint32_t s_z_cnt = 0u;	  /* Z 相脉冲总数（诊断） */
static volatile bool s_z_pending = false; /* 有未处理的 Z 事件 */
static volatile bool s_store_dirty = true; /* 需要落盘（上电未确认前为脏） */

/**
 * @brief 启动 TIM3 编码器接口与 Z 相捕获，并以当前 CNT 为相对零点
 * @note  初始化阶段调用一次；外设启动失败进 Error_Handler；
 *        绝对位置由上电 Encoder_StoreRestore 另行恢复
 */
void Encoder_Init(void) {
	/* 编码器接口启动（TIM3 CH1/CH2 = PA6/PA7，CubeMX 已配 TI12 四倍频 +
	 * 输入滤波 15）；随后以当前 CNT 为累计零点 */
	if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
		Error_Handler();
	}
	/* Z 相（TIM3_CH3 = PB0）输入捕获中断：每圈一个脉冲，ISR 只登记事件 */
	if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	s_enc_last	= (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
	s_enc_total = 0;
	s_enc_abs	= 0; /* 真实值由上电 Encoder_StoreRestore 恢复 */
}

/**
 * @brief 以当前 CNT 为相对零点（清 s_enc_total）
 * @note  模式切换时调用；不影响绝对计数 s_enc_abs（掉电保存仍有效）
 */
void Encoder_Zero(void) {
	s_enc_total = 0;
	s_enc_last	= (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
}

/**
 * @brief 读编码器相对累计计数（int16 差分处理 16 位回绕 + 方向修正）
 * @retval 32 位相对累计计数（Encoder_Zero 后从 0 起算）
 * @note  含 PRIMASK 临界区（仅几条指令），50Hz ISR 与 5Hz 主循环均可调用；
 *        安全条件：两次调用间隔内计数变化 < 32768（20ms @ 本机转速恒满足）
 */
int32_t Encoder_Read(void) {
	uint16_t now;
	int16_t	 d;
	uint32_t pm;

	/* 临界区：本函数既被 50Hz TIM1 ISR 调用（模式1/2），也被 5Hz 主循环
	 * Enc_Service 调用（模式0）。若主循环在执行中途被 ISR 抢占，两次调用会
	 * 出现差分重算，累计值被多计。临界区仅几条指令 */
	pm = __get_PRIMASK();
	__disable_irq();
	/* int16_t 差分天然处理 16 位回绕（正反向越界均正确）；
	 * 安全条件：两次调用间隔内计数变化 < 32768。50Hz 调用下对应转速上限
	 * ≈24600rpm，远超本电机工况，恒安全 */
	now			= (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
	d			= (int16_t) (now - s_enc_last);
	/* 统一方向：ENC_DIR_SIGN 修正 A/B 相接反，使计数增量与 freq 同向
	 * （否则内环成正反馈、软限幅判断方向也反） */
	d			= (int16_t) (ENC_DIR_SIGN * d);
	s_enc_total += d; /* 累计相对位置 */
	s_enc_abs   += d;
	s_enc_last	= now;
	__set_PRIMASK(pm);

	if (d != 0) {
		s_store_dirty = true; /* 位置变化，待主循环落盘 */
	}
	return s_enc_total;
}

/* ==================== 绝对位置 / Z 相 / 掉电保存 ======================== */

/* Flash 数据页布局（12 字节，WORD 编程） */
typedef struct {
	uint32_t magic;
	uint32_t enc_abs;
	uint32_t sum;
} EncStore_t;

/**
 * @brief 计算保存数据校验和（magic 与计数异或）
 * @param v 待校验的绝对位置
 * @retval ENC_STORE_MAGIC ^ v
 * @note  用于识别全 0xFF 未写入页与写入损坏
 */
static uint32_t enc_store_sum(uint32_t v) {
	return ENC_STORE_MAGIC ^ v;
}

/**
 * @brief 读绝对累计计数（不随 Encoder_Zero 清零）
 * @retval 32 位绝对计数（上电由 Encoder_StoreRestore 恢复）
 * @note  软限幅与掉电保存使用；32 位对齐变量，无临界区
 */
int32_t Encoder_ReadAbs(void) {
	return s_enc_abs;
}

/**
 * @brief 读 Z 相脉冲累计数
 * @retval 自复位以来的 Z 事件总数（诊断/OLED 显示）
 * @note  volatile 32 位读，与 ISR 并发写也原子
 */
uint32_t Encoder_GetZCount(void) {
	return s_z_cnt;
}

/**
 * @brief Z 相捕获事件登记（ISR 内只置标志与计数）
 * @note  仅 TIM3 捕获 ISR 调用；对齐与落盘由 5Hz Encoder_ProcessZ 处理
 */
void Encoder_ZIndexEvent(void) {
	s_z_cnt++;
	s_z_pending	  = true;
	s_store_dirty = true;
}

/**
 * @brief 若有待处理 Z 事件，把绝对计数对齐到最近整圈
 * @retval true = 本次处理了事件；false = 无待处理事件
 * @note  5Hz 主循环调用；整圈长度为 ENC_CPR，余数过半圈才进位
 */
bool Encoder_ProcessZ(void) {
	int32_t q;
	int32_t r;

	if (!s_z_pending) {
		return false;
	}
	s_z_pending = false;

	/* Z 为整圈零点：对齐到最近整圈（C 除法向零截断，余数过半圈才进位） */
	q = s_enc_abs / (int32_t) ENC_CPR;
	r = s_enc_abs % (int32_t) ENC_CPR;
	if (r > (int32_t) ENC_CPR / 2) {
		q++;
	} else if (r < -((int32_t) ENC_CPR / 2)) {
		q--;
	}
	s_enc_abs = q * (int32_t) ENC_CPR;
	return true;
}

/**
 * @brief 标记绝对位置待落盘
 * @note  停机/模式切换时调用；实际写入由 Encoder_StoreSave 在静止时执行
 */
void Encoder_StoreMarkDirty(void) {
	s_store_dirty = true;
}

/**
 * @brief 查询是否有待落盘数据
 * @retval true = 有脏数据待写；false = 已保存或无需保存
 */
bool Encoder_StorePending(void) {
	return s_store_dirty;
}

/**
 * @brief 上电从 Flash 恢复绝对位置
 * @retval true = magic 与校验和正确并已恢复；false = 未写过/已擦除/校验失败（保持 0）
 * @note  初始化阶段调用一次；成功同时清除脏标志
 */
bool Encoder_StoreRestore(void) {
	const EncStore_t *d = (const EncStore_t *) ENC_STORE_ADDR;

	if ((d->magic != ENC_STORE_MAGIC) || (d->sum != enc_store_sum(d->enc_abs))) {
		return false; /* 未写过/被擦除（全 0xFF）或校验失败 */
	}
	s_enc_abs	  = (int32_t) d->enc_abs;
	s_store_dirty = false;
	return true;
}

/**
 * @brief 擦除 Flash 数据页并写入当前绝对位置
 * @retval true = 擦除与全部字编程成功；false = 失败（不再重试）
 * @note  CPU 从 Flash 取指停顿 20~40ms，只在模式 0 且摆杆静止时由 5Hz 服务调用；
 *        失败也不重试，避免主循环被反复擦写阻塞
 */
bool Encoder_StoreSave(void) {
	EncStore_t			  d;
	FLASH_EraseInitTypeDef ei = {0};
	uint32_t			  page_err = 0u;
	const uint32_t		 *p;
	uint32_t			  addr;
	uint8_t				  i;
	HAL_StatusTypeDef	  ok;

	d.magic	  = ENC_STORE_MAGIC;
	d.enc_abs = (uint32_t) s_enc_abs;
	d.sum	  = enc_store_sum(d.enc_abs);

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
	ei.TypeErase   = FLASH_TYPEERASE_PAGES;
	ei.PageAddress = ENC_STORE_ADDR;
	ei.NbPages	   = 1u;
	ok			   = HAL_FLASHEx_Erase(&ei, &page_err);
	if (ok == HAL_OK) {
		/* 目标地址必须是 Flash（ENC_STORE_ADDR），源数据在栈上；
		 * 早期版本把栈地址当编程地址传入，导致擦除后写入静默丢失 */
		p	 = (const uint32_t *) &d;
		for (i = 0u; i < (sizeof(EncStore_t) / 4u); i++) {
			addr = ENC_STORE_ADDR + (4u * i);
			ok	 = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, p[i]);
			if (ok != HAL_OK) {
				break;
			}
		}
	}
	HAL_FLASH_Lock();
	s_store_dirty = false; /* 失败不重试：避免主循环被反复擦写阻塞 */
	return ok == HAL_OK;
}

/**
 * @brief 以当前点为编码器零点：相对与绝对计数同时清零，并立即写入 Flash
 * @retval true = Flash 写入成功；false = 擦写失败（RAM 零点仍生效）
 * @note  主循环调用（串口 C 命令）；Flash 擦写停顿 20~40ms，须在摆杆静止时执行
 */
bool Encoder_ZeroAbsAndSave(void) {
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	s_enc_total = 0;
	s_enc_abs	= 0;
	s_enc_last	= (uint16_t) __HAL_TIM_GET_COUNTER(&htim3);
	__set_PRIMASK(pm);
	s_store_dirty = true;
	return Encoder_StoreSave();
}
