#include "canopen_app.h"
#include "CO_app_STM32.h"
#include "CANopen.h"
#include "OD.h"
#include "board.h"

static CANopenNodeSTM32 s_canopen;

/* CO_app_STM32.c 提供的入口 */
int canopen_app_init(CANopenNodeSTM32* canopenSTM32);
void canopen_app_process(void);
void canopen_app_interrupt(void);

void app_canopen_init(void)
{
    /* 协议栈功能裁剪见 lib/CO_STM32/CO_driver_target.h */
    s_canopen.CANHandle = &hcan1;
    s_canopen.HWInitFunction = MX_CAN_Init;
    s_canopen.timerHandle = &htim6;
    s_canopen.desiredNodeID = 0x01;
    s_canopen.baudrate = 1000;   /* kbps，仅用于 LSS（本项目禁用） */

    if (canopen_app_init(&s_canopen) != 0) {
        Error_Handler();
    }
}

/* 1ms 定时器钩子（board.c 调度） */
void canopen_timer_isr(void)
{
    canopen_app_interrupt();
}

uint32_t app_canopen_error_status(void)
{
    if (CO == NULL || CO->CANmodule == NULL) {
        return 0u;
    }
    return CO->CANmodule->CANerrorStatus;
}
