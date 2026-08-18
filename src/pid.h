/**
 * @file    pid.h
 * @brief   通用浮点 PID 控制器（带积分限幅与输出限幅）
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float out_min;
    float out_max;
    float integral;
    float prev_error;
    float dt;
} pid_t;

void pid_init(pid_t* pid, float kp, float ki, float kd, float out_min, float out_max, float dt);
void pid_reset(pid_t* pid);
float pid_update(pid_t* pid, float error);
void pid_set_gains(pid_t* pid, float kp, float ki, float kd);

#endif /* PID_H */
