/**
 * @file    config.h
 * @brief   电机与 FOC 参数配置
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ---------------- 电机参数 (2804 云台电机) ---------------- */
#define MOTOR_POLE_PAIRS   7
#define MOTOR_PHASE_R      8.5f    /* Ω */
#define MOTOR_PHASE_L      0.0015f /* H */
#define MOTOR_KV           100.0f
#define MOTOR_BUS_VOLTAGE  12.0f
#define MOTOR_RATED_CURRENT 0.5f

/* ---------------- 可选电流采样 (INA240A1, R100=0.1Ω, 20V/V) ---------------- */
#ifndef CURRENT_SENSE_PRESENT
#define CURRENT_SENSE_PRESENT 0
#endif

#define CURRENT_SENSE_OFFSET_V  1.65f
#define CURRENT_SENSE_GAIN      (20.0f * 0.1f)   /* V/A = 2.0 */
#define CURRENT_SENSE_SCALE     (3.3f / 4096.0f) /* 12bit ADC -> V */

/* 电流环使能：0 = 电压型 FOC（默认，安全），1 = 电流型 FOC */
#ifndef FOC_CURRENT_LOOP_ENABLE
#define FOC_CURRENT_LOOP_ENABLE 0
#endif

#if FOC_CURRENT_LOOP_ENABLE && !CURRENT_SENSE_PRESENT
#error "FOC current loop requires CURRENT_SENSE_PRESENT=1"
#endif

/* ---------------- 控制环频率 ---------------- */
#define FOC_LOOP_HZ   20000
#define VEL_LOOP_HZ   1000

/* ---------------- 默认 PID 参数（保守值，需现场整定） ---------------- */
#define POS_P_DEFAULT       4.0f
#define VEL_P_DEFAULT       0.02f
#define VEL_I_DEFAULT       0.05f
#define CURR_P_DEFAULT      0.5f
#define CURR_I_DEFAULT      2.0f

/* ---------------- 限制与保护 ---------------- */
#if CURRENT_SENSE_PRESENT
#define VOLTAGE_LIMIT_V     3.0f       /* Validated INA240 build limit. */
#define FOC_ALIGN_VOLTAGE   1.0f
#define FOC_ALIGN_TIME_MS   1500u
#else
#define VOLTAGE_LIMIT_V     1.0f       /* No current measurement: safe first-spin limit. */
#define FOC_ALIGN_VOLTAGE   0.5f
#define FOC_ALIGN_TIME_MS   250u
#endif
#define CURRENT_LIMIT_A     0.35f       /* Current-loop target; leaves margin to 0.8 A hard OCP. */
#define CURRENT_SOFT_LIMIT_A 0.60f      /* Proactively reduce Vq before 0.8 A hard OCP. */
#define VOLTAGE_RISE_V_PER_S 8.0f
#define VOLTAGE_FALL_V_PER_S 100.0f
#define VELOCITY_LIMIT_CPS  50000.0f   /* counts/s，约 12 rev/s */
#define OVERCURRENT_TRIP_A  0.8f
#define OVERCURRENT_CONFIRM_SAMPLES 8u

/* 无电流采样时，速度指令未建立反馈速度则立即断使能，防止热堵转。 */
#define STALL_GUARD_MIN_COMMAND_CPS 250.0f
#define STALL_GUARD_MIN_VOLTAGE_V   0.5f
#define STALL_GUARD_MIN_SPEED_CPS   50.0f
#define STALL_GUARD_TIMEOUT_S       0.75f

/* FOC 上电对齐校准 */
#define FOC_ALIGN_MIN_MOVE_RAD 0.02f /* 方向检测所需的最小机械角变化 */

#endif /* APP_CONFIG_H */
