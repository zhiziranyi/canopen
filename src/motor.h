/**
 * @file    motor.h
 * @brief   电机驱动：TIM1 三路 PWM + INA240 电流采样 + 电流/速度/位置环
 *
 * 控制结构：
 *   TIM1 更新中断 (20kHz)  -> 电流环 (FOC 步进)
 *   TIM7 中断 (1kHz)      -> 编码器更新 + 速度环 + 位置环
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_init(void);
void motor_align_foc(void);   /* 上电 FOC 对齐校准（必做） */
void motor_enable(void);
void motor_disable(void);
int  motor_is_enabled(void);

void motor_set_velocity_target(float cps);    /* counts/s */
void motor_set_position_target(int32_t counts);
void motor_set_torque_voltage(float vq);      /* 电压型直接力矩指令 (V) */
void motor_stop(void);

/* 运动配置（由 CiA402 层写入） */
void motor_set_profile(uint32_t vel_cps, uint32_t acc_cps2, uint32_t dec_cps2);
void motor_set_position_p(float kp);
void motor_set_velocity_pi(float kp, float ki);
void motor_set_current_pi(float kp, float ki);

/* 状态读取 */
float   motor_get_current_u(void);
float   motor_get_current_iq(void);
float   motor_get_current_id(void);
float   motor_get_voltage_cmd(void);
int32_t motor_get_position(void);
float   motor_get_velocity(void);
float   motor_get_velocity_cmd(void);
int     motor_get_fault(void);
void    motor_clear_fault(void);

/* 中断钩子（board.c 调度） */
void motor_current_loop_isr(void);   /* 20kHz */
void motor_velocity_loop_isr(void);  /* 1kHz */
void motor_adc_complete_isr(void);   /* ADC 注入转换完成 */

#endif /* MOTOR_H */
