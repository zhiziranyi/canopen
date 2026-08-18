/**
 * @file    cia402.h
 * @brief   CiA 402 驱动状态机（电源状态机 + 轮廓位置/速度/回零模式）
 */
#ifndef CIA402_H
#define CIA402_H

#include <stdint.h>

void cia402_init(void);
void cia402_process(void);   /* 主循环周期调用 */

/* 由外部故障源调用（过流等） */
void cia402_raise_fault(uint16_t error_code);

#endif /* CIA402_H */
