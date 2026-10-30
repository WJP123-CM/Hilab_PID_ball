/**
 ******************************************************************************
 * @file    key.c
 * @brief   PA0 按键采样 + 15ms 消抖 + 事件队列（详见 key.h）
 ******************************************************************************
 */
#include "key.h"

#include "main.h" /* HAL_GPIO_ReadPin / GPIOA / GPIO_PIN_0 */

/* ---- 按键状态机（TIM1 ISR 写；队列由主循环读） ---- */
static volatile uint8_t s_key_q[KEY_QUEUE_LEN];
static volatile uint8_t s_key_q_head = 0; /* ISR 写 */
static volatile uint8_t s_key_q_tail = 0; /* 主循环写 */
static uint8_t	s_key_stable	 = 0; /* 已确认电平：1=按下 */
static uint8_t	s_key_diff_cnt	 = 0; /* 与稳态相反的连续采样数 */
static uint16_t s_key_hold_ms	 = 0; /* 稳态按下持续时长 */
static uint8_t	s_key_long_sent	 = 0; /* 本次按压已上报长按 */

/**
 * @brief 事件入队（队满丢弃最新事件，保证已入队事件不被覆盖）
 * @param evt KeyEvent_t 枚举值对应的整数（PRESS/RELEASE/CLICK/LONG）
 * @note  仅 TIM1 1ms ISR 调用（单生产者）；主循环单消费者只读，无锁
 */
static void Key_QueuePush(uint8_t evt) {
	uint8_t next = (uint8_t) ((s_key_q_head + 1u) % KEY_QUEUE_LEN);

	if (next != s_key_q_tail) {
		s_key_q[s_key_q_head] = evt;
		s_key_q_head		  = next;
	}
}

/**
 * @brief 1ms 节拍采样：消抖 → 确认稳态 → 生成 PRESS/RELEASE/CLICK/LONG 事件入队
 * @note  仅 TIM1 1ms ISR 调用；全程只读引脚与计数器，O(1) 无阻塞；
 *        长按每个按压缩周期只上报一次，单击在"未达长按即松开"时补发
 */
void Key_Tick1ms(void) {
	uint8_t raw = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) ? 1u : 0u;

	if (raw != s_key_stable) {
		/* 抖动中：未连续稳定 KEY_DEBOUNCE_MS 不认账，直接返回 */
		if (++s_key_diff_cnt < KEY_DEBOUNCE_MS) {
			return;
		}
		s_key_diff_cnt = 0;
		s_key_stable   = raw;

		if (raw != 0u) { /* 确认按下：清时长，每次按压只发一次 PRESS */
			s_key_hold_ms	= 0u;
			s_key_long_sent = 0u;
			Key_QueuePush(KEY_EVT_PRESS);
		} else { /* 确认松开：先判是否已达长按，再决定是否补发单击 */
			uint8_t was_long = s_key_long_sent;

			s_key_hold_ms = 0u;
			Key_QueuePush(KEY_EVT_RELEASE);
			if (was_long == 0u) {
				Key_QueuePush(KEY_EVT_CLICK);
			}
		}
		return;
	}

	s_key_diff_cnt = 0;
	if ((s_key_stable != 0u) && (s_key_long_sent == 0u)) {
		if (++s_key_hold_ms >= KEY_LONG_MS) {
			s_key_long_sent = 1u; /* 长按只上报一次，持续按住不重复触发 */
			Key_QueuePush(KEY_EVT_LONG);
		}
	}
}

/**
 * @brief 主循环取出一个最早入队的事件（非阻塞）
 * @retval 事件枚举值；队列为空时返回 KEY_EVT_NONE
 * @note  仅主循环调用（单消费者）；10~20ms 轮询一次即可覆盖人手操作
 */
KeyEvent_t Key_PollEvent(void) {
	uint8_t tail = s_key_q_tail;
	uint8_t evt;

	if (tail == s_key_q_head) {
		return KEY_EVT_NONE;
	}
	evt			 = s_key_q[tail];
	s_key_q_tail = (uint8_t) ((tail + 1u) % KEY_QUEUE_LEN);

	return (KeyEvent_t) evt;
}
