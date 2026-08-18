#include "motor.h"

#include "board.h"
#include "config.h"
#include "encoder.h"
#include "foc.h"
#include "motion_profile.h"
#include "pid.h"

#include <math.h>

#define MOTOR_ALIGN_TEST_ANGLE_RAD 1.04719755f
#define MOTOR_TWO_PI_RAD 6.28318531f
#define MOTOR_CONTROL_START_MIN_UPDATES 100u

static volatile int s_enabled = 0;
static volatile motor_fault_t s_fault = MOTOR_FAULT_NONE;
static volatile float s_torque_voltage = 0.0f;
static volatile float s_current_target = 0.0f;
static volatile float s_velocity_cmd = 0.0f;
static volatile int32_t s_position_target = 0;
static volatile int s_position_mode = 0;
static volatile float s_iu = 0.0f;
static volatile float s_id = 0.0f;
static volatile float s_iq = 0.0f;
static volatile float s_vcmd = 0.0f;
static volatile uint8_t s_aligning = 0u;
static volatile float s_align_v = 0.0f;
static volatile float s_align_angle = 0.0f;
static volatile uint32_t s_control_update_count = 0u;
static volatile uint32_t s_velocity_tick_count = 0u;

static pid_t s_curr_id_pid;
static pid_t s_curr_iq_pid;
static pid_t s_vel_pid;
static motion_profile_t s_position_profile;
static float s_pos_kp = POS_P_DEFAULT;

static float clamp01(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

static void pwm_set_duty(float du, float dv, float dw)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t period_counts = arr + 1u;
    uint32_t compare_u = (uint32_t)(clamp01(du) * (float)period_counts);
    uint32_t compare_v = (uint32_t)(clamp01(dv) * (float)period_counts);
    uint32_t compare_w = (uint32_t)(clamp01(dw) * (float)period_counts);

    if (compare_u > arr) compare_u = arr;
    if (compare_v > arr) compare_v = arr;
    if (compare_w > arr) compare_w = arr;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare_u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, compare_v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, compare_w);
}

static void motor_latch_fault(motor_fault_t fault)
{
    if (fault == MOTOR_FAULT_NONE || s_fault != MOTOR_FAULT_NONE) {
        return;
    }
    s_fault = fault;
    s_enabled = 0;
    s_aligning = 0u;
    s_torque_voltage = 0.0f;
    s_current_target = 0.0f;
    s_velocity_cmd = 0.0f;
    pwm_set_duty(0.0f, 0.0f, 0.0f);
    HAL_GPIO_WritePin(PIN_DRV_EN_GPIO, PIN_DRV_EN_PIN, GPIO_PIN_RESET);
}

static void adc_start_sample(void)
{
    HAL_StatusTypeDef status = HAL_ADCEx_InjectedStart_IT(&hadc1);
    if (status != HAL_OK && status != HAL_BUSY) {
        motor_latch_fault(MOTOR_FAULT_INIT);
    }
}

static float wrap_mechanical_delta(float delta)
{
    if (delta > 3.14159265f) {
        delta -= MOTOR_TWO_PI_RAD;
    } else if (delta < -3.14159265f) {
        delta += MOTOR_TWO_PI_RAD;
    }
    return delta;
}

int motor_init(void)
{
    HAL_StatusTypeDef status;

    s_enabled = 0;
    s_aligning = 0u;
    s_fault = MOTOR_FAULT_NONE;
    HAL_GPIO_WritePin(PIN_DRV_EN_GPIO, PIN_DRV_EN_PIN, GPIO_PIN_RESET);
    pwm_set_duty(0.0f, 0.0f, 0.0f);

    pid_init(&s_curr_id_pid, CURR_P_DEFAULT, CURR_I_DEFAULT, 0.0f,
             -VOLTAGE_LIMIT_V, VOLTAGE_LIMIT_V, 1.0f / (float)FOC_LOOP_HZ);
    pid_init(&s_curr_iq_pid, CURR_P_DEFAULT, CURR_I_DEFAULT, 0.0f,
             -VOLTAGE_LIMIT_V, VOLTAGE_LIMIT_V, 1.0f / (float)FOC_LOOP_HZ);
    pid_init(&s_vel_pid, VEL_P_DEFAULT, VEL_I_DEFAULT, 0.0f,
             -VOLTAGE_LIMIT_V, VOLTAGE_LIMIT_V, 1.0f / (float)VEL_LOOP_HZ);
    motion_profile_init(&s_position_profile, 20000.0f, 100000.0f, 100000.0f);

    status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    if (status == HAL_OK) status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    if (status == HAL_OK) status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    if (status == HAL_OK) status = HAL_TIM_Base_Start_IT(&htim1);
    if (status == HAL_OK) status = HAL_TIM_Base_Start_IT(&htim7);
    if (status != HAL_OK) {
        motor_latch_fault(MOTOR_FAULT_INIT);
        return -1;
    }

    adc_start_sample();
    (void)encoder_start_sample();
    if (!encoder_is_healthy()) {
        motor_latch_fault(MOTOR_FAULT_ENCODER);
        return -1;
    }
    return 0;
}

int motor_align_foc(void)
{
    uint32_t updates_before;
    float angle_zero;
    float angle_test;
    float movement;
    int8_t direction;

    if (s_fault != MOTOR_FAULT_NONE || !encoder_is_healthy()) {
        motor_latch_fault(MOTOR_FAULT_ENCODER);
        return -1;
    }

    updates_before = s_control_update_count;
    s_align_v = FOC_ALIGN_VOLTAGE;
    s_align_angle = 0.0f;
    s_aligning = 1u;
    s_enabled = 1;
    HAL_GPIO_WritePin(PIN_DRV_EN_GPIO, PIN_DRV_EN_PIN, GPIO_PIN_SET);
    HAL_Delay(FOC_ALIGN_TIME_MS);

    if ((s_control_update_count - updates_before) < MOTOR_CONTROL_START_MIN_UPDATES) {
        motor_latch_fault(MOTOR_FAULT_CONTROL_TIMING);
        return -1;
    }
    if (!encoder_is_healthy()) {
        motor_latch_fault(MOTOR_FAULT_ENCODER);
        return -1;
    }
    angle_zero = encoder_get_mech_angle_rad();

    s_align_angle = MOTOR_ALIGN_TEST_ANGLE_RAD;
    HAL_Delay(FOC_ALIGN_TIME_MS);
    angle_test = encoder_get_mech_angle_rad();
    movement = wrap_mechanical_delta(angle_test - angle_zero);
    if (!encoder_is_healthy() || fabsf(movement) < FOC_ALIGN_MIN_MOVE_RAD) {
        motor_latch_fault(MOTOR_FAULT_ALIGNMENT);
        return -1;
    }
    direction = (movement < 0.0f) ? -1 : 1;

    s_align_angle = 0.0f;
    HAL_Delay(FOC_ALIGN_TIME_MS);
    angle_zero = encoder_get_mech_angle_rad();
    encoder_set_align(angle_zero, direction);

    s_aligning = 0u;
    s_enabled = 0;
    s_align_v = 0.0f;
    pwm_set_duty(0.0f, 0.0f, 0.0f);
    HAL_GPIO_WritePin(PIN_DRV_EN_GPIO, PIN_DRV_EN_PIN, GPIO_PIN_RESET);
    dbg_printf("[FOC] align ok, dir=%d, offset=%.4f rad, move=%.4f rad\r\n",
               direction, (double)angle_zero, (double)movement);
    return 0;
}

void motor_enable(void)
{
    if (s_fault != MOTOR_FAULT_NONE || !encoder_is_healthy()) {
        return;
    }
    pid_reset(&s_curr_id_pid);
    pid_reset(&s_curr_iq_pid);
    pid_reset(&s_vel_pid);
    s_torque_voltage = 0.0f;
    s_current_target = 0.0f;
    s_velocity_cmd = 0.0f;
    HAL_GPIO_WritePin(PIN_DRV_EN_GPIO, PIN_DRV_EN_PIN, GPIO_PIN_SET);
    s_enabled = 1;
}

void motor_disable(void)
{
    s_enabled = 0;
    s_torque_voltage = 0.0f;
    s_current_target = 0.0f;
    s_velocity_cmd = 0.0f;
    s_position_mode = 0;
    motion_profile_stop(&s_position_profile);
    pwm_set_duty(0.0f, 0.0f, 0.0f);
    HAL_GPIO_WritePin(PIN_DRV_EN_GPIO, PIN_DRV_EN_PIN, GPIO_PIN_RESET);
}

int motor_is_enabled(void)
{
    return s_enabled;
}

void motor_set_velocity_target(float cps)
{
    s_position_mode = 0;
    if (cps > VELOCITY_LIMIT_CPS) cps = VELOCITY_LIMIT_CPS;
    if (cps < -VELOCITY_LIMIT_CPS) cps = -VELOCITY_LIMIT_CPS;
    s_velocity_cmd = cps;
}

void motor_set_position_target(int32_t counts)
{
    int32_t current = encoder_get_position();
    s_position_target = counts;
    motion_profile_set_target(&s_position_profile, current, counts);
    s_position_mode = 1;
}

void motor_set_torque_voltage(float vq)
{
    s_position_mode = 0;
    if (vq > VOLTAGE_LIMIT_V) vq = VOLTAGE_LIMIT_V;
    if (vq < -VOLTAGE_LIMIT_V) vq = -VOLTAGE_LIMIT_V;
    s_torque_voltage = vq;
}

void motor_stop(void)
{
    s_position_mode = 0;
    s_velocity_cmd = 0.0f;
    s_torque_voltage = 0.0f;
    s_current_target = 0.0f;
    motion_profile_stop(&s_position_profile);
    pid_reset(&s_vel_pid);
}

void motor_set_profile(uint32_t vel_cps, uint32_t acc_cps2, uint32_t dec_cps2)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    motion_profile_set_limits(&s_position_profile, (float)vel_cps,
                              (float)acc_cps2, (float)dec_cps2);
    __set_PRIMASK(primask);
}

void motor_set_position_p(float kp)
{
    s_pos_kp = kp;
}

void motor_set_velocity_pi(float kp, float ki)
{
    pid_set_gains(&s_vel_pid, kp, ki, 0.0f);
}

void motor_set_current_pi(float kp, float ki)
{
    pid_set_gains(&s_curr_id_pid, kp, ki, 0.0f);
    pid_set_gains(&s_curr_iq_pid, kp, ki, 0.0f);
}

float motor_get_current_u(void) { return s_iu; }
float motor_get_current_iq(void) { return s_iq; }
float motor_get_current_id(void) { return s_id; }
float motor_get_voltage_cmd(void) { return s_vcmd; }
int32_t motor_get_position(void) { return encoder_get_position(); }
float motor_get_velocity(void) { return encoder_get_velocity(); }
float motor_get_velocity_cmd(void) { return s_velocity_cmd; }
motor_fault_t motor_get_fault(void) { return s_fault; }
uint32_t motor_get_control_update_count(void) { return s_control_update_count; }
uint32_t motor_get_velocity_tick_count(void) { return s_velocity_tick_count; }

const char* motor_fault_name(motor_fault_t fault)
{
    switch (fault) {
        case MOTOR_FAULT_NONE: return "none";
        case MOTOR_FAULT_INIT: return "init";
        case MOTOR_FAULT_CONTROL_TIMING: return "control-timing";
        case MOTOR_FAULT_ALIGNMENT: return "alignment";
        case MOTOR_FAULT_ENCODER: return "encoder";
        case MOTOR_FAULT_OVERCURRENT: return "overcurrent";
        default: return "unknown";
    }
}

void motor_clear_fault(void)
{
    if (s_fault == MOTOR_FAULT_ENCODER && !encoder_is_healthy()) {
        return;
    }
    s_fault = MOTOR_FAULT_NONE;
}

void motor_current_loop_isr(void)
{
    float ia;
    float ib;
    float ic;
    float electrical_angle;
    float sin_e;
    float cos_e;
    float id_value;
    float iq_value;
    float du;
    float dv;
    float dw;
    float vd = 0.0f;
    float vq = 0.0f;

    s_control_update_count++;

    if (s_aligning != 0u) {
        foc_inverse_park_svpwm(s_align_v, 0.0f,
                               sinf(s_align_angle), cosf(s_align_angle),
                               MOTOR_BUS_VOLTAGE, &du, &dv, &dw);
        pwm_set_duty(du, dv, dw);
        adc_start_sample();
        return;
    }

    if (!s_enabled || s_fault != MOTOR_FAULT_NONE) {
        pwm_set_duty(0.0f, 0.0f, 0.0f);
        adc_start_sample();
        return;
    }

    ia = s_iu;
    if (fabsf(ia) > OVERCURRENT_TRIP_A) {
        motor_latch_fault(MOTOR_FAULT_OVERCURRENT);
        adc_start_sample();
        return;
    }

    du = (float)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1)
       / (float)(__HAL_TIM_GET_AUTORELOAD(&htim1) + 1u);
    dv = (float)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2)
       / (float)(__HAL_TIM_GET_AUTORELOAD(&htim1) + 1u);
    dw = (float)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3)
       / (float)(__HAL_TIM_GET_AUTORELOAD(&htim1) + 1u);
    foc_reconstruct_currents(ia, du, dv, dw, &ib, &ic);

    electrical_angle = encoder_get_elec_angle_rad();
    sin_e = sinf(electrical_angle);
    cos_e = cosf(electrical_angle);
    foc_clarke_park(ia, ib, ic, sin_e, cos_e, &id_value, &iq_value);
    s_id = id_value;
    s_iq = iq_value;

#if FOC_CURRENT_LOOP_ENABLE
    vd = pid_update(&s_curr_id_pid, -id_value);
    vq = pid_update(&s_curr_iq_pid, s_current_target - iq_value);
#else
    vq = s_torque_voltage;
#endif
    s_vcmd = vq;
    foc_inverse_park_svpwm(vd, vq, sin_e, cos_e,
                           MOTOR_BUS_VOLTAGE, &du, &dv, &dw);
    pwm_set_duty(du, dv, dw);
    adc_start_sample();
}

void motor_velocity_loop_isr(void)
{
    float velocity_actual;
    float velocity_error;

    s_velocity_tick_count++;
    encoder_update_1k();
    (void)encoder_start_sample();

    if (!encoder_is_healthy()) {
        motor_latch_fault(MOTOR_FAULT_ENCODER);
        return;
    }
    if (!s_enabled || s_fault != MOTOR_FAULT_NONE) {
        pid_reset(&s_vel_pid);
        return;
    }

    if (s_position_mode) {
        s_velocity_cmd = motion_profile_step(&s_position_profile,
                                             encoder_get_position(),
                                             1.0f / (float)VEL_LOOP_HZ);
    }

    velocity_actual = encoder_get_velocity();
    velocity_error = s_velocity_cmd - velocity_actual;
#if FOC_CURRENT_LOOP_ENABLE
    s_current_target = pid_update(&s_vel_pid, velocity_error);
    if (s_current_target > CURRENT_LIMIT_A) s_current_target = CURRENT_LIMIT_A;
    if (s_current_target < -CURRENT_LIMIT_A) s_current_target = -CURRENT_LIMIT_A;
#else
    s_torque_voltage = pid_update(&s_vel_pid, velocity_error);
#endif
}

void motor_adc_complete_isr(void)
{
    uint32_t raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    float voltage = (float)raw * CURRENT_SENSE_SCALE;
    s_iu = (voltage - CURRENT_SENSE_OFFSET_V) / CURRENT_SENSE_GAIN;
}
