#include "motor.h"
#include "board.h"
#include "config.h"
#include "encoder.h"
#include "foc.h"
#include "pid.h"
#include <math.h>

/* ---------------- 内部状态 ---------------- */
static volatile int s_enabled = 0;
static volatile int s_fault = 0;

static volatile float s_torque_voltage = 0.0f;   /* 电压型 Vq 指令 (V) */
static volatile float s_current_target = 0.0f;   /* 电流型 Iq 指令 (A) */
static volatile float s_velocity_cmd = 0.0f;     /* counts/s */
static volatile int32_t s_position_target = 0;
static volatile int s_position_mode = 0;         /* 1 = 轮廓位置模式 */

static volatile float s_iu = 0.0f;
static volatile float s_id = 0.0f;
static volatile float s_iq = 0.0f;
static volatile float s_vcmd = 0.0f;
static volatile uint8_t s_aligning = 0;
static volatile float s_align_v = 0.0f;
static volatile float s_align_angle = 0.0f;

static pid_t s_curr_id_pid;
static pid_t s_curr_iq_pid;
static pid_t s_vel_pid;
static float s_pos_kp = POS_P_DEFAULT;

static uint32_t s_profile_vel = 20000;    /* counts/s */
static uint32_t s_profile_acc = 100000;   /* counts/s^2 */
static uint32_t s_profile_dec = 100000;

/* ---------------- 内部函数 ---------------- */
static void pwm_set_duty(float du, float dv, float dw)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(du * (float)arr));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(dv * (float)arr));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dw * (float)arr));
}

static void adc_start_sample(void)
{
    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

/* ========================================================================
 * 初始化
 * ===================================================================== */
void motor_init(void)
{
    pid_init(&s_curr_id_pid, CURR_P_DEFAULT, CURR_I_DEFAULT, 0.0f,
             -VOLTAGE_LIMIT_V, VOLTAGE_LIMIT_V, 1.0f / FOC_LOOP_HZ);
    pid_init(&s_curr_iq_pid, CURR_P_DEFAULT, CURR_I_DEFAULT, 0.0f,
             -VOLTAGE_LIMIT_V, VOLTAGE_LIMIT_V, 1.0f / FOC_LOOP_HZ);
    pid_init(&s_vel_pid, VEL_P_DEFAULT, VEL_I_DEFAULT, 0.0f,
             -VOLTAGE_LIMIT_V, VOLTAGE_LIMIT_V, 1.0f / VEL_LOOP_HZ);

    /* 启动三路 PWM + 更新中断（20kHz 电流环），EN 保持低 */
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
    pwm_set_duty(0.0f, 0.0f, 0.0f);

    /* 启动 1kHz 速度/位置环 */
    HAL_TIM_Base_Start_IT(&htim7);

    /* 启动 ADC 注入采样 */
    adc_start_sample();
}

/* ========================================================================
 * FOC 上电对齐校准：
 *   1. 施加固定电压矢量(电角度 0)，转子对齐到 A 相磁场
 *   2. 电角度 +60°，转子跟随转动，据此判断编码器方向
 *   3. 记录零点偏移与方向，之后正常 FOC 即可正确换相
 * ===================================================================== */
void motor_align_foc(void)
{
    float ma, mb, d;
    int8_t dir = 1;

    HAL_GPIO_WritePin(PIN_DRV_EN_GPIO, PIN_DRV_EN_PIN, GPIO_PIN_SET);
    s_aligning = 1;
    s_enabled = 1;
    s_fault = 0;
    s_align_v = FOC_ALIGN_VOLTAGE;

    s_align_angle = 0.0f;
    HAL_Delay(FOC_ALIGN_TIME_MS);
    ma = encoder_get_mech_angle_rad();

    s_align_angle = 1.0472f;   /* +60° 电角度 */
    HAL_Delay(FOC_ALIGN_TIME_MS);
    mb = encoder_get_mech_angle_rad();

    d = mb - ma;
    if (d > 3.14159f) d -= 6.28318f;
    else if (d < -3.14159f) d += 6.28318f;
    if (d < 0.0f) dir = -1;

    encoder_set_align(ma, dir);

    s_aligning = 0;
    s_enabled = 0;
    pwm_set_duty(0.0f, 0.0f, 0.0f);
    HAL_GPIO_WritePin(PIN_DRV_EN_GPIO, PIN_DRV_EN_PIN, GPIO_PIN_RESET);

    dbg_printf("[FOC] align ok, dir=%d, offset=%.4f rad\r\n",
               dir, (double)ma);
}

void motor_enable(void)
{
    if (s_fault) return;
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
    pwm_set_duty(0.0f, 0.0f, 0.0f);
    HAL_GPIO_WritePin(PIN_DRV_EN_GPIO, PIN_DRV_EN_PIN, GPIO_PIN_RESET);
}

int motor_is_enabled(void)
{
    return s_enabled;
}

/* ========================================================================
 * 指令接口
 * ===================================================================== */
void motor_set_velocity_target(float cps)
{
    s_position_mode = 0;
    s_velocity_cmd = cps;
    if (s_velocity_cmd > VELOCITY_LIMIT_CPS) s_velocity_cmd = VELOCITY_LIMIT_CPS;
    if (s_velocity_cmd < -VELOCITY_LIMIT_CPS) s_velocity_cmd = -VELOCITY_LIMIT_CPS;
}

void motor_set_position_target(int32_t counts)
{
    s_position_mode = 1;
    s_position_target = counts;
}

void motor_set_torque_voltage(float vq)
{
    s_position_mode = 0;
    s_torque_voltage = vq;
    if (s_torque_voltage > VOLTAGE_LIMIT_V) s_torque_voltage = VOLTAGE_LIMIT_V;
    if (s_torque_voltage < -VOLTAGE_LIMIT_V) s_torque_voltage = -VOLTAGE_LIMIT_V;
}

void motor_stop(void)
{
    s_position_mode = 0;
    s_velocity_cmd = 0.0f;
    s_torque_voltage = 0.0f;
    s_current_target = 0.0f;
    pid_reset(&s_vel_pid);
}

void motor_set_profile(uint32_t vel_cps, uint32_t acc_cps2, uint32_t dec_cps2)
{
    s_profile_vel = vel_cps;
    s_profile_acc = acc_cps2;
    s_profile_dec = dec_cps2;
}

void motor_set_position_p(float kp) { s_pos_kp = kp; }
void motor_set_velocity_pi(float kp, float ki) { pid_set_gains(&s_vel_pid, kp, ki, 0.0f); }
void motor_set_current_pi(float kp, float ki)
{
    pid_set_gains(&s_curr_id_pid, kp, ki, 0.0f);
    pid_set_gains(&s_curr_iq_pid, kp, ki, 0.0f);
}

/* ========================================================================
 * 状态读取
 * ===================================================================== */
float motor_get_current_u(void) { return s_iu; }
float motor_get_current_iq(void) { return s_iq; }
float motor_get_current_id(void) { return s_id; }
float motor_get_voltage_cmd(void) { return s_vcmd; }
int32_t motor_get_position(void) { return encoder_get_position(); }
float motor_get_velocity(void) { return encoder_get_velocity(); }
float motor_get_velocity_cmd(void) { return s_velocity_cmd; }
int motor_get_fault(void) { return s_fault; }
void motor_clear_fault(void) { s_fault = 0; }

/* ========================================================================
 * 20kHz 电流环（TIM1 更新中断）
 * ===================================================================== */
void motor_current_loop_isr(void)
{
    float ia, ib, ic, sin_e, cos_e;
    float id_cmd = 0.0f;
    float du, dv, dw;
    float vd = 0.0f, vq = 0.0f;

    /* 校准模式：固定电压矢量，不使用编码器反馈 */
    if (s_aligning) {
        float du, dv, dw;
        foc_inverse_park_svpwm(0.0f, s_align_v,
                               sinf(s_align_angle), cosf(s_align_angle),
                               MOTOR_BUS_VOLTAGE, &du, &dv, &dw);
        pwm_set_duty(du, dv, dw);
        adc_start_sample();
        return;
    }

    if (!s_enabled || s_fault) {
        pwm_set_duty(0.0f, 0.0f, 0.0f);
        adc_start_sample();
        return;
    }

    ia = s_iu;

    /* 过流保护 */
    if (fabsf(ia) > OVERCURRENT_TRIP_A) {
        s_fault = 1;
        motor_disable();
        adc_start_sample();
        return;
    }

    du = (float)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1) /
         (float)(__HAL_TIM_GET_AUTORELOAD(&htim1) + 1);
    dv = (float)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2) /
         (float)(__HAL_TIM_GET_AUTORELOAD(&htim1) + 1);
    dw = (float)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3) /
         (float)(__HAL_TIM_GET_AUTORELOAD(&htim1) + 1);

    float id_val, iq_val;
    foc_reconstruct_currents(ia, du, dv, dw, &ib, &ic);
    sin_e = sinf(encoder_get_elec_angle_rad());
    cos_e = cosf(encoder_get_elec_angle_rad());
    foc_clarke_park(ia, ib, ic, sin_e, cos_e, &id_val, &iq_val);
    s_id = id_val;
    s_iq = iq_val;

#if FOC_CURRENT_LOOP_ENABLE
    vd = pid_update(&s_curr_id_pid, id_cmd - id_val);
    vq = pid_update(&s_curr_iq_pid, s_current_target - iq_val);
#else
    /* 电压型：Vq 直接来自速度环/力矩指令，Vd = 0 */
    (void)id_cmd;
    vq = s_torque_voltage;
#endif
    s_vcmd = vq;

    foc_inverse_park_svpwm(vd, vq, sin_e, cos_e, MOTOR_BUS_VOLTAGE, &du, &dv, &dw);
    pwm_set_duty(du, dv, dw);

    /* 触发下一次 ADC 采样（本周期结束时结果就绪） */
    adc_start_sample();
}

/* ========================================================================
 * 1kHz 速度/位置环（TIM7 中断）
 * ===================================================================== */
void motor_velocity_loop_isr(void)
{
    static uint32_t profile_tick = 0;
    static uint32_t profile_total_ticks = 0;
    static int32_t profile_dist = 0;

    encoder_update_1k();

    if (!s_enabled || s_fault) {
        pid_reset(&s_vel_pid);
        return;
    }

    /* ---- 轮廓位置模式：梯形速度规划 ---- */
    if (s_position_mode) {
        int32_t cur = encoder_get_position();
        int32_t dist = s_position_target - cur;
        if (dist != profile_dist) {
            /* 新目标：重新规划 */
            profile_dist = dist;
            profile_tick = 0;
            /* 加速段 + 减速段总时间（梯形，初末速 0） */
            int32_t ad = (dist >= 0) ? 1 : -1;
            uint32_t d = (uint32_t)(ad * dist);
            uint32_t t1 = (s_profile_vel > 0) ? (s_profile_acc > 0 ? s_profile_vel / s_profile_acc : 1) : 1;
            uint32_t t2 = (s_profile_vel > 0) ? (s_profile_dec > 0 ? s_profile_vel / s_profile_dec : 1) : 1;
            uint32_t dist_at_vel = d - (s_profile_vel * t1 / 2) - (s_profile_vel * t2 / 2);
            uint32_t t3 = (s_profile_vel > 0) ? (dist_at_vel + s_profile_vel - 1) / s_profile_vel : 1;
            if ((int32_t)dist_at_vel <= 0) {
                /* 三角型速度曲线 */
                t3 = 0;
                uint32_t t_tri = 1;
                float vp = sqrtf(2.0f * (float)d * (float)s_profile_acc * (float)s_profile_dec /
                                 ((float)s_profile_acc + (float)s_profile_dec));
                if (vp < (float)s_profile_vel) {
                    t1 = (uint32_t)(vp / (float)s_profile_acc) + 1;
                    t2 = (uint32_t)(vp / (float)s_profile_dec) + 1;
                    t_tri = t1 + t2;
                } else {
                    t_tri = t1 + t2;
                }
                profile_total_ticks = t_tri;
            } else {
                profile_total_ticks = t1 + t2 + t3;
            }
        }

        profile_tick++;
        if (profile_tick >= profile_total_ticks) {
            profile_tick = profile_total_ticks;
            s_velocity_cmd = 0.0f;
            motor_set_velocity_target(0.0f);
            s_position_mode = 0;
        } else {
            /* 速度指令：到达剩余距离的一半时开始减速（简化为分段梯形） */
            int32_t remain = s_position_target - encoder_get_position();
            int32_t ad = (profile_dist >= 0) ? 1 : -1;
            float v_rem = (float)remain * (float)ad;
            float v_lim = sqrtf(2.0f * v_rem * (float)s_profile_dec);
            float v_max = (float)s_profile_vel;
            float v_cmd = v_max;
            if (v_lim < v_cmd) v_cmd = v_lim;
            if (v_cmd > v_max) v_cmd = v_max;
            s_velocity_cmd = (float)ad * v_cmd;
        }
    }

    /* ---- 速度环：输出 Vq（电压型）或 Iq（电流型） ---- */
    float vel_act = encoder_get_velocity();
    float vel_err = s_velocity_cmd - vel_act;
#if FOC_CURRENT_LOOP_ENABLE
    s_current_target = pid_update(&s_vel_pid, vel_err);
    if (s_current_target > CURRENT_LIMIT_A) s_current_target = CURRENT_LIMIT_A;
    if (s_current_target < -CURRENT_LIMIT_A) s_current_target = -CURRENT_LIMIT_A;
#else
    s_torque_voltage = pid_update(&s_vel_pid, vel_err);
#endif
}

/* ========================================================================
 * ADC 注入转换完成：读取 U 相电流
 * ===================================================================== */
void motor_adc_complete_isr(void)
{
    uint32_t raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    float v = (float)raw * CURRENT_SENSE_SCALE;
    /* INA240: Vout = 1.65V + 2.0 * I */
    s_iu = (v - CURRENT_SENSE_OFFSET_V) / CURRENT_SENSE_GAIN;
}
