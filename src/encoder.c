#include "encoder.h"

#include "board.h"
#include "config.h"
#include "encoder_math.h"

#define AS5600_REG_ANGLE 0x0Eu
#define ENCODER_STALE_LIMIT_TICKS 20u
#define TWO_PI_F 6.2831853f

static volatile uint16_t s_raw_angle = 0u;
static volatile uint8_t s_sample_pending = 0u;
static volatile uint32_t s_sample_sequence = 0u;
static volatile uint32_t s_processed_sequence = 0u;
static volatile uint32_t s_error_count = 0u;
static volatile uint32_t s_stale_ticks = 0u;
static volatile uint8_t s_healthy = 0u;
static uint8_t s_sample_data[2];
static encoder_math_t s_math;
static float s_zero_mech = 0.0f;
static int8_t s_dir = 1;

static uint16_t decode_angle(const uint8_t data[2])
{
    return (uint16_t)((((uint16_t)data[0] << 8) | data[1]) & 0x0FFFu);
}

int encoder_init(void)
{
    uint8_t data[2];

    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(ENCODER_AS5600_ADDR << 1),
                         AS5600_REG_ANGLE, I2C_MEMADD_SIZE_8BIT,
                         data, 2u, 100u) != HAL_OK) {
        s_healthy = 0u;
        s_error_count++;
        return -1;
    }

    s_raw_angle = decode_angle(data);
    encoder_math_init(&s_math, s_raw_angle);
    s_sample_pending = 0u;
    s_sample_sequence = 1u;
    s_processed_sequence = 1u;
    s_error_count = 0u;
    s_stale_ticks = 0u;
    s_healthy = 1u;
    return 0;
}

uint16_t encoder_read_raw(void)
{
    return s_raw_angle;
}

int encoder_start_sample(void)
{
    HAL_StatusTypeDef status;

    if (s_sample_pending != 0u) {
        return 1;
    }

    s_sample_pending = 1u;
    status = HAL_I2C_Mem_Read_IT(&hi2c1, (uint16_t)(ENCODER_AS5600_ADDR << 1),
                                 AS5600_REG_ANGLE, I2C_MEMADD_SIZE_8BIT,
                                 s_sample_data, 2u);
    if (status != HAL_OK) {
        s_sample_pending = 0u;
        s_error_count++;
        return -1;
    }

    return 0;
}

void encoder_sample_complete_isr(void)
{
    s_raw_angle = decode_angle(s_sample_data);
    s_sample_sequence++;
    s_sample_pending = 0u;
}

void encoder_sample_error_isr(void)
{
    s_error_count++;
    s_sample_pending = 0u;
}

void encoder_update_1k(void)
{
    uint32_t sequence = s_sample_sequence;

    if (sequence != s_processed_sequence) {
        encoder_math_update(&s_math, s_raw_angle, 0.001f);
        s_processed_sequence = sequence;
        s_stale_ticks = 0u;
        s_healthy = 1u;
    } else {
        if (s_stale_ticks < UINT32_MAX) {
            s_stale_ticks++;
        }
        if (s_stale_ticks >= ENCODER_STALE_LIMIT_TICKS) {
            s_healthy = 0u;
        }
    }
}

int encoder_is_healthy(void)
{
    return s_healthy != 0u;
}

uint32_t encoder_get_error_count(void)
{
    return s_error_count;
}

uint32_t encoder_get_sample_count(void)
{
    return s_sample_sequence;
}

float encoder_get_elec_angle_rad(void)
{
    float mech = encoder_get_mech_angle_rad();
    float relative = mech - s_zero_mech;
    float electrical;

    if (relative < 0.0f) {
        relative += TWO_PI_F;
    }
    electrical = (float)s_dir * relative * (float)MOTOR_POLE_PAIRS;
    electrical -= (float)((int32_t)(electrical / TWO_PI_F)) * TWO_PI_F;
    if (electrical < 0.0f) {
        electrical += TWO_PI_F;
    }
    return electrical;
}

float encoder_get_mech_angle_rad(void)
{
    return ((float)s_raw_angle / (float)ENCODER_CPR) * TWO_PI_F;
}

void encoder_set_align(float mech_rad_at_zero, int8_t direction)
{
    s_zero_mech = mech_rad_at_zero;
    s_dir = (direction < 0) ? -1 : 1;
}

float encoder_get_velocity(void)
{
    uint32_t primask = __get_PRIMASK();
    float velocity;

    __disable_irq();
    velocity = encoder_math_velocity(&s_math);
    __set_PRIMASK(primask);
    return velocity;
}

int32_t encoder_get_position(void)
{
    uint32_t primask = __get_PRIMASK();
    int32_t position;

    __disable_irq();
    position = encoder_math_position(&s_math);
    __set_PRIMASK(primask);
    return position;
}

void encoder_set_position_zero(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    encoder_math_zero_position(&s_math);
    __set_PRIMASK(primask);
}
