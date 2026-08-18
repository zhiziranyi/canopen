#include "pid.h"

void pid_init(pid_t* pid, float kp, float ki, float kd, float out_min, float out_max, float dt)
{
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->dt = (dt > 0.0f) ? dt : 0.001f;
}

void pid_reset(pid_t* pid)
{
    if (pid == NULL) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

void pid_set_gains(pid_t* pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float pid_update(pid_t* pid, float error)
{
    float p_term, i_term, d_term, out;
    if (pid == NULL) return 0.0f;

    p_term = pid->kp * error;

    /* 积分带积分限幅（抗饱和） */
    pid->integral += error * pid->dt;
    if (pid->integral > pid->out_max / (pid->ki + 1e-9f)) {
        pid->integral = pid->out_max / (pid->ki + 1e-9f);
    } else if (pid->integral < pid->out_min / (pid->ki + 1e-9f)) {
        pid->integral = pid->out_min / (pid->ki + 1e-9f);
    }
    i_term = pid->ki * pid->integral;

    d_term = pid->kd * (error - pid->prev_error) / pid->dt;
    pid->prev_error = error;

    out = p_term + i_term + d_term;
    if (out > pid->out_max) out = pid->out_max;
    else if (out < pid->out_min) out = pid->out_min;
    return out;
}
