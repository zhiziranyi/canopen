/**
 * @file    canopen_app.h
 * @brief   CANopen 应用层入口（CANopenNode 初始化 + 应用对象字典扩展）
 */
#ifndef CANOPEN_APP_H
#define CANOPEN_APP_H

#include <stdint.h>

void app_canopen_init(void);
uint32_t app_canopen_error_status(void);

#endif /* CANOPEN_APP_H */
