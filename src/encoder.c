#include "encoder.h"
#include "board.h"
#include "config.h"
#include <math.h>

#define AS5600_REG_ANGLE 0x0Eu   /* 12bit 角度 */

static volatile uint16_t s_raw_angle = 0;
static volatile float s_velocity = 0.0f;      /* counts/s */
static volatile int32_t s_position = 0;       /* 累计位置 counts */
static volatile int32_t s_last_raw = 0;
static volatile uint16_t s_prev_angle = 0;
static float s_zero_mech = 0.0f;      /* FOC 零点：机械角 (rad) */
static int8_t s_dir = 1;              /* 编码器方向 */

int encoder_init(void)
{
    uint8_t buf[2];
    /* 读一次确认 I2C 通 */
    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(ENCODER_AS5600_ADDR << 1),
                         AS5600_REG_ANGLE, I2C_MEMADD_SIZE_8BIT, buf, 2, 100) != HAL_OK) {
        return -1;
    }
    s_raw_angle = ((uint16_t)buf[0] << 8) | buf[1];
    s_prev_angle = s_raw_angle;
    s_last_raw = (int32_t)s_raw_angle;
    return 0;
}

uint16_t encoder_read_raw(void)
{
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(ENCODER_AS5600_ADDR << 1),
                         AS5600_REG_ANGLE, I2C_MEMADD_SIZE_8BIT, buf, 2, 50) == HAL_OK) {
        s_raw_angle = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return s_raw_angle;
}

/* 1kHz：读角 + 差分测速 + 累计位置 */
void encoder_update_1k(void)
{
    uint16_t raw = encoder_read_raw();
    int32_t delta = (int32_t)raw - (int32_t)s_prev_angle;

    /* 处理 0/4095 回绕 */
    if (delta > (int32_t)(ENCODER_CPR / 2)) {
        delta -= ENCODER_CPR;
    } else if (delta < -(int32_t)(ENCODER_CPR / 2)) {
        delta += ENCODER_CPR;
    }
    s_prev_angle = raw;

    s_position += delta;
    s_last_raw = (int32_t)raw;

    /* 一阶低通滤波：fc ~ 30Hz */
    float inst_vel = (float)delta * 1000.0f;   /* counts/s */
    s_velocity = s_velocity * 0.9f + inst_vel * 0.1f;

    /* 静止死区：消除编码器 1 LSB 抖动 */
    if (s_velocity < 1.0f && s_velocity > -1.0f) {
        s_velocity = 0.0f;
    }
}

float encoder_get_elec_angle_rad(void)
{
    float mech = encoder_get_mech_angle_rad();
    float d = mech - s_zero_mech;
    if (d < 0.0f) d += 6.2831853f;
    /* 电角度 = 方向 * 相对零点机械角 * 极对数 */
    float elec = (float)s_dir * d * (float)MOTOR_POLE_PAIRS;
    /* 归一化到 0..2pi */
    elec = elec - (float)((int)(elec / 6.2831853f)) * 6.2831853f;
    if (elec < 0.0f) elec += 6.2831853f;
    return elec;
}

float encoder_get_mech_angle_rad(void)
{
    return ((float)(s_raw_angle & 0x0FFF) / (float)ENCODER_CPR) * 6.2831853f;
}

void encoder_set_align(float mech_rad_at_zero, int8_t direction)
{
    s_zero_mech = mech_rad_at_zero;
    s_dir = (direction < 0) ? -1 : 1;
}

float encoder_get_velocity(void)
{
    return s_velocity;
}

int32_t encoder_get_position(void)
{
    return s_position;
}

void encoder_set_position_zero(void)
{
    s_position = 0;
}
