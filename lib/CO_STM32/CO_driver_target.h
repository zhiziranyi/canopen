/*
 * 设备与应用相关定义 (STM32F407 bxCAN)
 *
 * 基于 CANopenNode v4 CO_driver_target.h 模板改写。
 */
#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- 基本定义 ---------- */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x

typedef uint_fast8_t bool_t;
typedef float float32_t;
typedef double float64_t;

/* ---------- 协议栈功能裁剪 ----------
 * 仅启用本项目需要的模块：
 *   NMT / Heartbeat / EMCY / SYNC / PDO / SDO Server / LED
 * 关闭：LSS、GFC、SRDO、Gateway、CRC16、FIFO、Trace、SDO Client、
 *       HB Consumer、TIME、Node Guarding、Storage（无非易失存储）
 */
#undef  CO_CONFIG_LSS
#define CO_CONFIG_LSS 0
#undef  CO_CONFIG_GFC
#define CO_CONFIG_GFC 0
#undef  CO_CONFIG_SRDO
#define CO_CONFIG_SRDO 0
#undef  CO_CONFIG_GTW
#define CO_CONFIG_GTW 0
#undef  CO_CONFIG_CRC16
#define CO_CONFIG_CRC16 0
#undef  CO_CONFIG_FIFO
#define CO_CONFIG_FIFO 0
#undef  CO_CONFIG_TRACE
#define CO_CONFIG_TRACE 0
#undef  CO_CONFIG_SDO_CLI
#define CO_CONFIG_SDO_CLI 0
#undef  CO_CONFIG_HB_CONS
#define CO_CONFIG_HB_CONS 0
#undef  CO_CONFIG_TIME
#define CO_CONFIG_TIME 0
#undef  CO_CONFIG_NODE_GUARDING
#define CO_CONFIG_NODE_GUARDING 0
#undef  CO_CONFIG_STORAGE
#define CO_CONFIG_STORAGE 0

/* ---------- CAN 消息结构 ---------- */
typedef struct {
    uint32_t ident;  /* 标准标识符 (11bit) + RTR 标志(bit15) */
    uint8_t dlc;
    uint8_t data[8];
} CO_CANrxMsg_t;

#define CO_CANrxMsg_readIdent(msg) ((uint16_t)(((CO_CANrxMsg_t*)(msg))->ident))
#define CO_CANrxMsg_readDLC(msg)   ((uint8_t)(((CO_CANrxMsg_t*)(msg))->dlc))
#define CO_CANrxMsg_readData(msg)  ((uint8_t*)(((CO_CANrxMsg_t*)(msg)))->data)

/* 接收消息对象 */
typedef struct {
    uint16_t ident;
    uint16_t mask;
    void* object;
    void (*CANrx_callback)(void* object, void* message);
} CO_CANrx_t;

/* 发送消息对象 */
typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
    volatile bool_t bufferFull;
    volatile bool_t syncFlag;
} CO_CANtx_t;

/* CAN 模块对象 */
typedef struct {
    void* CANptr;
    CO_CANrx_t* rxArray;
    uint16_t rxSize;
    CO_CANtx_t* txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile bool_t useCANrxFilters;
    volatile bool_t bufferInhibitFlag;
    volatile bool_t firstCANtxMessage;
    volatile uint16_t CANtxCount;
    uint32_t errOld;

    /* STM32 专用 */
    uint32_t primask_send;
    uint32_t primask_emcy;
    uint32_t primask_od;
} CO_CANmodule_t;

/* 数据存储条目（本项目未启用 Storage，保留类型定义以兼容编译） */
typedef struct {
    void* addr;
    size_t len;
    uint8_t subIndexOD;
    uint8_t attr;
    void* addrNV;
} CO_storage_entry_t;

/* ---------- 临界区 ---------- */
#define CO_LOCK_CAN_SEND(CAN_MODULE)                                                               \
    do {                                                                                           \
        (CAN_MODULE)->primask_send = __get_PRIMASK();                                              \
        __disable_irq();                                                                           \
    } while (0)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE) __set_PRIMASK((CAN_MODULE)->primask_send)

#define CO_LOCK_EMCY(CAN_MODULE)                                                                   \
    do {                                                                                           \
        (CAN_MODULE)->primask_emcy = __get_PRIMASK();                                              \
        __disable_irq();                                                                           \
    } while (0)
#define CO_UNLOCK_EMCY(CAN_MODULE) __set_PRIMASK((CAN_MODULE)->primask_emcy)

#define CO_LOCK_OD(CAN_MODULE)                                                                     \
    do {                                                                                           \
        (CAN_MODULE)->primask_od = __get_PRIMASK();                                                \
        __disable_irq();                                                                           \
    } while (0)
#define CO_UNLOCK_OD(CAN_MODULE) __set_PRIMASK((CAN_MODULE)->primask_od)

/* 接收线程与处理线程之间的同步 */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)                                                                         \
    do {                                                                                           \
        CO_MemoryBarrier();                                                                        \
        rxNew = (void*)1L;                                                                         \
    } while (0)
#define CO_FLAG_CLEAR(rxNew)                                                                       \
    do {                                                                                           \
        CO_MemoryBarrier();                                                                        \
        rxNew = NULL;                                                                              \
    } while (0)

#ifdef __cplusplus
}
#endif

#endif /* CO_DRIVER_TARGET_H */
