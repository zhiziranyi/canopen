/**
 * @file    board.h
 * @brief   板级定义：STM32F407ZGT6 + SimpleFOC Mini + AS5600 + INA240 + TJA1050
 *
 * 硬件接线（详见 开发参考方案.md）：
 *   PE9/PE11/PE13 - TIM1_CH1/2/3 -> SimpleFOC IN1/IN2/IN3 (U/V/W)
 *   PE14          - EN 使能
 *   PC4           - NSP (nSLEEP)，必须拉高
 *   PA3           - ADC1_IN3 <- INA240 OUT (U 相电流)
 *   PB6/PB7       - I2C1 -> AS5600 (SCL/SDA)
 *   PD0/PD1       - CAN1_RX/CAN1_TX -> TJA1050
 *   PA9/PA10      - USART1 调试串口 115200
 */
#ifndef BOARD_H
#define BOARD_H

#include "stm32f4xx_hal.h"

/* ---------------- 引脚定义 ---------------- */
/* TIM1 三路 PWM */
#define PIN_PWM_U_GPIO    GPIOE
#define PIN_PWM_U_PIN     GPIO_PIN_9
#define PIN_PWM_V_GPIO    GPIOE
#define PIN_PWM_V_PIN     GPIO_PIN_11
#define PIN_PWM_W_GPIO    GPIOE
#define PIN_PWM_W_PIN     GPIO_PIN_13
#define PIN_DRV_EN_GPIO   GPIOE
#define PIN_DRV_EN_PIN    GPIO_PIN_14
#define PIN_DRV_NSLEEP_GPIO GPIOC
#define PIN_DRV_NSLEEP_PIN  GPIO_PIN_4

/* INA240 电流采样 */
#define PIN_CURRENT_GPIO  GPIOA
#define PIN_CURRENT_PIN   GPIO_PIN_3
#define ADC_CURRENT_CH    ADC_CHANNEL_3

/* AS5600 I2C */
#define PIN_ENC_SCL_GPIO  GPIOB
#define PIN_ENC_SCL_PIN   GPIO_PIN_6
#define PIN_ENC_SDA_GPIO  GPIOB
#define PIN_ENC_SDA_PIN   GPIO_PIN_7

/* CAN1 */
#define PIN_CAN_RX_GPIO   GPIOD
#define PIN_CAN_RX_PIN    GPIO_PIN_0
#define PIN_CAN_TX_GPIO   GPIOD
#define PIN_CAN_TX_PIN    GPIO_PIN_1

/* USART1 调试串口 */
#define PIN_DBG_TX_GPIO   GPIOA
#define PIN_DBG_TX_PIN    GPIO_PIN_9
#define PIN_DBG_RX_GPIO   GPIOA
#define PIN_DBG_RX_PIN    GPIO_PIN_10

/* 板载 LED（不同开发板差异大，默认 PB0/PB1，可按需修改） */
#define PIN_LED0_GPIO     GPIOB
#define PIN_LED0_PIN      GPIO_PIN_0
#define PIN_LED1_GPIO     GPIOB
#define PIN_LED1_PIN      GPIO_PIN_1

/* ---------------- 外设句柄 ---------------- */
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;     /* PWM 20kHz + 电流环 */
extern TIM_HandleTypeDef htim6;     /* CANopen 1ms */
extern TIM_HandleTypeDef htim7;     /* 速度/位置环 1kHz */
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

/* ---------------- 接口 ---------------- */
void SystemClock_Config(void);
void board_init(void);
void MX_CAN_Init(void);
void Error_Handler(void);

/* 调试打印 */
void dbg_printf(const char* fmt, ...);

/* RTC 备份寄存器（复位/掉电均保留，用于崩溃诊断） */
uint32_t bkp_read(uint8_t idx);
void bkp_write(uint8_t idx, uint32_t val);

/* 中断钩子（由 board.c 的 HAL 回调分发） */
void motor_current_loop_isr(void);    /* TIM1 更新 20kHz */
void motor_velocity_loop_isr(void);   /* TIM7 1kHz */
void motor_adc_complete_isr(void);    /* ADC1 注入转换完成 */
void canopen_timer_isr(void);         /* TIM6 1ms */

#endif /* BOARD_H */
