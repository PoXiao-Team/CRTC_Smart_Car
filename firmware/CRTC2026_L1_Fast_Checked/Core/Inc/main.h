#ifndef MAIN_H
#define MAIN_H
#include "stm32f1xx_hal.h"
void Error_Handler(void);
/* 调试只读：当前启动时的复位标志、主循环活跃计数、故障编号。
 * 变量保存在RAM，不保证跨掉电或复位保留；g_reset_flags为本次启动重新采集。
 */
extern volatile uint32_t g_reset_flags;
extern volatile uint32_t g_loop_heartbeat;
extern volatile uint32_t g_fault_tag;
#endif
