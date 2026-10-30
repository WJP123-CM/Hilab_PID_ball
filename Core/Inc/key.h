/**
 ******************************************************************************
 * @file    key.h
 * @brief   PA0 按键采样、消抖与事件队列（单一职责：只做"电平 → 事件"）
 *
 * 硬件：PA0 输入 + 内部下拉，按下 = 高电平（CubeMX 已配置）。
 * 采样由 TIM1 1ms 回调调用 Key_Tick1ms()，只读 IDR 与几个计数器，O(1) 不阻塞。
 * 消抖：电平需连续 KEY_DEBOUNCE_MS(15ms) 保持不变才确认新稳态。
 * 事件经 4 深环形队列交付主循环（ISR 单生产者 / 主循环单消费者），无锁。
 * 注意：本模块不含任何业务动作（急停/模式切换由 main.c 消费事件后处理）。
 ******************************************************************************
 */
#ifndef __KEY_H__
#define __KEY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 消抖与长按判定参数 */
#define KEY_DEBOUNCE_MS 15u	  /* 10~20ms 区间取中值 */
#define KEY_LONG_MS		1000u /* 按住满 1s 判定长按 */
#define KEY_QUEUE_LEN	4u

typedef enum {
	KEY_EVT_NONE = 0,
	KEY_EVT_PRESS,	 /* 确认按下（高电平稳态建立） */
	KEY_EVT_RELEASE, /* 确认松开 */
	KEY_EVT_CLICK,	 /* 单击：按下后未达长按阈值即松开 */
	KEY_EVT_LONG,	 /* 长按：按住达到 KEY_LONG_MS（每次按压仅一次） */
	KEY_EVT_CNT		 /* 事件种类计数（数组长度用） */
} KeyEvent_t;

/** @brief 1ms 节拍采样 + 消抖 + 入队（仅 TIM1 1ms ISR 调用） */
void Key_Tick1ms(void);

/** @brief 主循环取事件（非阻塞）：无事件立即返回 KEY_EVT_NONE */
KeyEvent_t Key_PollEvent(void);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_H__ */
