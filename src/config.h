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

/* ---------------- 电流采样 (INA240A1, R100=0.1Ω, 20V/V) ---------------- */
#define CURRENT_SENSE_OFFSET_V  1.65f
#define CURRENT_SENSE_GAIN      (20.0f * 0.1f)   /* V/A = 2.0 */
#define CURRENT_SENSE_SCALE     (3.3f / 4096.0f) /* 12bit ADC -> V */

/* 电流环使能：0 = 电压型 FOC（默认，安全），1 = 电流型 FOC */
#ifndef FOC_CURRENT_LOOP_ENABLE
#define FOC_CURRENT_LOOP_ENABLE 0
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
#define VOLTAGE_LIMIT_V     3.0f       /* 8.5 ohm phase: <=0.35 A before OCP */
#define CURRENT_LIMIT_A     0.6f
#define VELOCITY_LIMIT_CPS  50000.0f   /* counts/s，约 12 rev/s */
#define OVERCURRENT_TRIP_A  0.8f

/* FOC 上电对齐校准 */
#define FOC_ALIGN_VOLTAGE   1.0f     /* 校准电压 (V) */
#define FOC_ALIGN_TIME_MS   1500     /* 每步保持时间 (ms) */
#define FOC_ALIGN_MIN_MOVE_RAD 0.02f /* 方向检测所需的最小机械角变化 */

#endif /* APP_CONFIG_H */
