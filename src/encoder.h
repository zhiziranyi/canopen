/**
 * @file    encoder.h
 * @brief   AS5600 12bit 磁编码器 (I2C1)
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#define ENCODER_CPR        4096
#define ENCODER_AS5600_ADDR 0x36  /* 7bit */

int     encoder_init(void);
uint16_t encoder_read_raw(void);          /* 最近一次有效的 0..4095 原始角度 */
int     encoder_start_sample(void);       /* 启动非阻塞 I2C 采样 */
void    encoder_sample_complete_isr(void);
void    encoder_sample_error_isr(void);
int     encoder_is_healthy(void);
uint32_t encoder_get_error_count(void);
uint32_t encoder_get_sample_count(void);
float   encoder_get_elec_angle_rad(void); /* 电角度（含极对数） */
float   encoder_get_mech_angle_rad(void); /* 机械角 0..2π */
float   encoder_get_velocity(void);       /* counts/s，滤波后 */
int32_t encoder_get_position(void);       /* counts，含原点偏移 */
void    encoder_set_position_zero(void);  /* 当前位置设为 0 */
void    encoder_set_align(float mech_rad_at_zero, int8_t direction); /* FOC 校准结果 */
void    encoder_update_1k(void);          /* 1kHz 周期更新：读角/差分测速 */

#endif /* ENCODER_H */
